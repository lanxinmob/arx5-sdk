"""SpaceMouse end-effector jogging through the existing joint-position server."""

import argparse
import ctypes
import os
from pathlib import Path
import select
import signal
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
import spnav

from vr_joint_bridge import JointClient, vector

ROOT = Path(__file__).resolve().parents[1]
DT = 0.02
AXES = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]])


class Mouse:
    def __init__(self, deadzone):
        spnav.spnav_open()
        self.deadzone = deadzone
        self.motion = np.zeros(6)
        self.buttons = np.zeros(2, dtype=bool)
        self.received = 0.0
        self.poller = select.poll()
        self.poller.register(spnav.spnav_fd(), select.POLLHUP | select.POLLERR)

    def read(self):
        if self.poller.poll(0):
            raise RuntimeError("SpaceMouse daemon disconnected")
        event = spnav.spnav_event()
        # The installed Python wrapper predates libspnav device-change events.
        for _ in range(1024):
            kind = spnav.libspnav.spnav_poll_event(ctypes.byref(event))
            if kind < 0:
                raise RuntimeError("SpaceMouse read failed")
            if kind == 0:
                break
            if kind == 1:
                m = event.motion
                self.motion = np.r_[AXES @ [m.x, m.y, m.z],
                                    AXES @ [m.rx, m.ry, m.rz]] / 350.0
                self.received = time.monotonic()
            elif kind == 2 and 0 <= event.button.bnum < 2:
                self.buttons[event.button.bnum] = bool(event.button.press)
        name = ctypes.create_string_buffer(256)
        if spnav.libspnav.spnav_dev_name(name, len(name)) <= 0:
            raise RuntimeError("SpaceMouse USB device disconnected")
        motion = np.clip(self.motion, -1, 1)
        if time.monotonic() - self.received > 0.25:
            motion[:] = 0
        motion = np.sign(motion) * np.maximum(np.abs(motion) - self.deadzone, 0) / (1 - self.deadzone)
        for start in (0, 3):
            motion[start:start + 3] /= max(1, np.linalg.norm(motion[start:start + 3]))
        return motion, self.buttons.copy()

    def close(self):
        spnav.spnav_close()


class Jogger:
    def __init__(self, robot, solver, q, gripper, args):
        self.robot, self.solver, self.args = robot, solver, args
        self.q = vector(q, robot.joint_dof)
        self.gripper = float(gripper)
        if not np.isfinite(self.gripper) or not -0.005 <= self.gripper <= robot.gripper_width + 0.005:
            raise ValueError("Invalid measured gripper position")
        self.gripper = float(np.clip(self.gripper, 0, robot.gripper_width))
        if np.any(self.q < robot.joint_pos_min - 0.1) or np.any(self.q > robot.joint_pos_max + 0.1):
            raise ValueError("Measured joints outside SDK limits")
        self.q = np.clip(self.q, robot.joint_pos_min, robot.joint_pos_max)

    def step(self, motion, buttons, dt):
        pose = vector(self.solver.forward_kinematics(self.q), 6)
        desired = pose.copy()
        desired[:3] += motion[:3] * self.args.translation_speed * dt
        turn = Rotation.from_rotvec(motion[3:] * self.args.rotation_speed * dt)
        desired[3:] = (turn * Rotation.from_euler("xyz", pose[3:])).as_euler("xyz")
        status = "holding"
        if np.any(motion):
            status = "IK boundary"
            for fraction in (1, 0.5, 0.25, 0.125):
                target = pose.copy()
                target[:3] += (desired[:3] - pose[:3]) * fraction
                target[3:] = (Rotation.from_rotvec(motion[3:] * self.args.rotation_speed * dt * fraction)
                              * Rotation.from_euler("xyz", pose[3:])).as_euler("xyz")
                result, q = self.solver.inverse_kinematics(target, self.q)
                q = np.asarray(q, dtype=float)
                if (result == 0 and q.shape == self.q.shape and np.isfinite(q).all()
                        and np.all(q >= self.robot.joint_pos_min)
                        and np.all(q <= self.robot.joint_pos_max)):
                    limit = np.minimum(self.robot.joint_vel_max, self.args.joint_speed) * dt
                    delta = np.clip(q - self.q, -limit, limit)
                    for _ in range(10):
                        candidate = self.q + delta
                        moved = vector(self.solver.forward_kinematics(candidate), 6)
                        distance = np.linalg.norm(moved[:3] - pose[:3])
                        angle = (Rotation.from_euler("xyz", moved[3:])
                                 * Rotation.from_euler("xyz", pose[3:]).inv()).magnitude()
                        if (distance <= self.args.translation_speed * dt + 1e-7
                                and angle <= self.args.rotation_speed * dt + 1e-7):
                            self.q = candidate
                            break
                        delta *= 0.5
                    status = "tracking"
                    break
        direction = int(buttons[0]) - int(buttons[1])
        self.gripper = float(np.clip(self.gripper + direction * self.args.gripper_speed * dt,
                                     0, self.robot.gripper_width))
        return {"joint_pos": self.q.copy(), "gripper_pos": self.gripper}, status


def check_state(robot, solver, state, armed=False):
    if state.get("emergency") is not False:
        raise ValueError("Joint server emergency")
    if armed and state.get("armed") is not True:
        raise ValueError("Joint server disarmed")
    q = vector(state["joint_pos"], robot.joint_dof)
    actual = vector(state["ee_pose"], 6)
    calculated = vector(solver.forward_kinematics(q), 6)
    error = (Rotation.from_euler("xyz", actual[3:])
             * Rotation.from_euler("xyz", calculated[3:]).inv()).magnitude()
    if np.linalg.norm(actual[:3] - calculated[:3]) > 0.005 or error > 0.05:
        raise ValueError("Server robot model does not match client model")
    return q


def run(args):
    os.chdir(ROOT)
    sys.path.insert(0, str(ROOT))
    import arx5_interface as sdk

    robot = sdk.RobotConfigFactory.get_instance().get_config(args.model)
    solver = sdk.Arx5Solver(robot.urdf_path, robot.joint_dof, robot.joint_pos_min,
                           robot.joint_pos_max, robot.base_link_name,
                           robot.eef_link_name, robot.gravity_vector)
    client = mouse = None
    stop_needed = False
    try:
        mouse = Mouse(args.deadzone)
        motion, buttons = mouse.read()
        if args.dry_run:
            jogger = Jogger(robot, solver, np.zeros(robot.joint_dof), robot.gripper_width, args)
            print("Dry run: SpaceMouse + IK only; no robot connection", flush=True)
        else:
            client = JointClient(args.host, args.port)
            state = client.request("GET_STATE")
            check_state(robot, solver, state)
            if args.check:
                print(f"Connected: {args.model}, SpaceMouse, {client.endpoint}; armed={state['armed']}", flush=True)
                return
            if state.get("armed") is not False:
                raise ValueError("Server already armed; stop the other control client first")
            jogger = None
            print("Connected. Hold both buttons for 1 s with cap neutral to home and enable.", flush=True)
        start = previous = last_print = time.monotonic()
        held_since = None
        released = False
        while not args.dry_run or time.monotonic() - start < args.seconds:
            tick = time.monotonic()
            motion, buttons = mouse.read()
            elapsed = tick - previous
            if client is not None and jogger is not None and elapsed > 0.2:
                raise RuntimeError("Control loop stalled")
            dt = min(DT, max(0, elapsed))
            previous = tick
            if jogger is None:
                if buttons.all() and not np.any(motion):
                    held_since = tick if held_since is None else held_since
                    if tick - held_since >= 1.0:
                        if client.request("GET_STATE").get("armed") is not False:
                            raise ValueError("Server was armed by another client")
                        stop_needed = True
                        print("RESET_TO_HOME: keep the arm's homing path clear", flush=True)
                        client.request("RESET_TO_HOME", timeout=30000)
                        state = client.request("GET_STATE")
                        q = check_state(robot, solver, state, armed=True)
                        jogger = Jogger(robot, solver, q, state["gripper_pos"], args)
                        previous = time.monotonic()
                        print("Enabled. Release both buttons; both buttons again stop.", flush=True)
                else:
                    held_since = None
            else:
                if not args.dry_run and released and buttons.all():
                    print("Both buttons: stopping", flush=True)
                    break
                if not buttons.any():
                    released = True
                control = motion if args.dry_run or released else np.zeros(6)
                grip_buttons = buttons if args.dry_run or released else np.zeros(2, dtype=bool)
                command, status = jogger.step(control, grip_buttons, dt)
                if client is not None:
                    state = client.request("SET_JOINT_POS", command, timeout=200)
                    actual = check_state(robot, solver, state, armed=True)
                    if np.max(np.abs(actual - jogger.q)) > 0.25:
                        raise RuntimeError("Joint tracking error exceeds 0.25 rad")
                if tick - last_print >= 0.5:
                    print(f"{status}: input={np.round(motion, 2)} gripper={jogger.gripper:.3f} m", flush=True)
                    last_print = tick
            time.sleep(max(0, DT - (time.monotonic() - tick)))
    finally:
        if client is not None:
            if stop_needed:
                try:
                    # An interrupted REQ receive cannot send another request.
                    client.socket.close()
                    client.open_socket()
                    client.request("SET_TO_DAMPING", timeout=500)
                    print("Joint server disarmed (damping)", flush=True)
                except Exception as error:
                    print(f"Stop request failed: {error}; server timeout policy remains active", file=sys.stderr)
            client.close()
        if mouse is not None:
            mouse.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="X5_umi")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--translation-speed", type=float, default=0.04, help="m/s")
    parser.add_argument("--rotation-speed", type=float, default=0.3, help="rad/s")
    parser.add_argument("--joint-speed", type=float, default=0.3, help="rad/s per joint")
    parser.add_argument("--gripper-speed", type=float, default=0.02, help="m/s")
    parser.add_argument("--deadzone", type=float, default=0.1)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="Read mouse and solve IK without connecting to robot")
    mode.add_argument("--check", action="store_true", help="Read GET_STATE and verify model without moving")
    parser.add_argument("--seconds", type=float, default=10, help="Dry-run duration")
    args = parser.parse_args()
    values = (args.translation_speed, args.rotation_speed, args.joint_speed,
              args.gripper_speed, args.seconds)
    if not np.isfinite(values).all() or min(values) <= 0:
        parser.error("Speeds and duration must be positive and finite")
    if not np.isfinite(args.deadzone) or not 0 <= args.deadzone < 1 or not 1 <= args.port <= 65535:
        parser.error("Invalid deadzone or port")

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        run(args)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"SpaceMouse bridge stopped: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
