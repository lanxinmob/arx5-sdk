"""Relative Quest teleoperation through the existing ZMQ joint servers."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import signal
import socket
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import zmq

from vr_receiver import decode_pos_cmd, inspect_connection

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
DT = 0.02
POSE_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {size} finite values")
    return result.copy()


def limited(delta, limit):
    return delta * min(1.0, limit / max(float(np.linalg.norm(delta)), 1e-12))


def rotation(pose):
    return Rotation.from_euler("xyz", pose[3:])


class JointClient:
    """Each worker owns its REQ socket; failed requests are never retried."""

    def __init__(self, host, port):
        self.endpoint = f"tcp://{host}:{port}"
        self.context = zmq.Context()
        self.socket = None

    def request(self, command, data=None, timeout=500):
        if self.socket is None:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.setsockopt(zmq.IMMEDIATE, 1)
            self.socket.connect(self.endpoint)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        try:
            self.socket.send_pyobj({"cmd": command, "data": data})
            reply = self.socket.recv_pyobj()
            if not isinstance(reply, dict) or reply.get("cmd") != command:
                raise ValueError(f"Invalid reply to {command}")
            result = reply.get("data")
            if isinstance(result, str) and result.startswith("ERROR"):
                raise ValueError(result)
            if command in ("GET_STATE", "SET_JOINT_POS"):
                if not isinstance(result, dict):
                    raise ValueError(f"Missing state in {command} reply")
            elif result != "OK":
                raise ValueError(f"Unexpected {command} reply: {result!r}")
            return result
        except BaseException:
            self.socket.close()
            self.socket = None
            raise

    def close(self):
        if self.socket is not None:
            self.socket.close()
        self.context.term()


@dataclass(frozen=True)
class Sample:
    pose: np.ndarray
    gripper: float
    counter: int
    received: float
    changed: float


class Inputs:
    def __init__(self, protocol, timeout):
        self.protocol, self.timeout = protocol, timeout
        self.lock = threading.Lock()
        self.samples = {}
        self.waiting_registration = set()

    def receive(self, topic, message_type, payload):
        side = {"ARX_VR_R": "right", "ARX_VR_L": "left"}.get(topic.lstrip("/"))
        if side is None:
            return
        # Quest can queue pose frames ahead of __publish when reconnecting.
        if message_type is None:
            if side not in self.waiting_registration:
                print(f"{side}: waiting for publisher registration; discarding early frames", flush=True)
                self.waiting_registration.add(side)
            return
        if message_type != "arm_control/PosCmd":
            raise ValueError(f"{side}: expected arm_control/PosCmd, got {message_type!r}")
        if side in self.waiting_registration:
            print(f"{side}: publisher registered as {message_type}", flush=True)
            self.waiting_registration.remove(side)
        data = decode_pos_cmd(payload, self.protocol)
        pose = vector([data[key] for key in POSE_FIELDS], 6)
        gripper, counter = data["gripper"], data["time_count"]
        if not 0 <= gripper <= 5:
            raise ValueError(f"{side}: VR gripper outside [0, 5]")
        now = time.monotonic()
        with self.lock:
            previous = self.samples.get(side)
            changed = now
            if previous is not None:
                if counter < previous.counter:
                    raise ValueError(f"{side}: VR counter reset; restart and realign")
                if counter == previous.counter:
                    changed = previous.changed
            self.samples[side] = Sample(pose, gripper, counter, now, changed)

    def get(self, side):
        with self.lock:
            sample = self.samples.get(side)
        if sample is None or time.monotonic() - sample.received > self.timeout:
            raise ValueError(f"{side}: no fresh VR packets")
        return sample


class Planner:
    def __init__(self, robot, solver, state, sample, frame, args):
        self.robot, self.solver, self.args = robot, solver, args
        self.frame = Rotation.from_euler("xyz", frame, degrees=True)
        self.q, self.gripper = self.check_state(state)
        self.pose = vector(solver.forward_kinematics(self.q), 6)
        self.origin = self.pose[:3].copy()
        self.anchor_pose, self.anchor_vr = self.pose.copy(), sample.pose.copy()
        self.previous = sample
        self.holding = True
        self.status = "aligned"

    def check_state(self, state):
        if state.get("emergency") is not False or not isinstance(state.get("armed"), bool):
            raise ValueError("Joint server emergency or incompatible state reply")
        q = vector(state["joint_pos"], self.robot.joint_dof)
        gripper = float(state["gripper_pos"])
        if not np.isfinite(gripper) or not 0 <= gripper <= self.robot.gripper_width:
            raise ValueError("Invalid measured gripper width")
        if np.any(q < self.robot.joint_pos_min) or np.any(q > self.robot.joint_pos_max):
            raise ValueError("Measured joints outside model limits")
        reported = vector(state["ee_pose"], 6)
        calculated = vector(self.solver.forward_kinematics(q), 6)
        if (np.linalg.norm(reported[:3] - calculated[:3]) > 0.005
                or (rotation(reported) * rotation(calculated).inv()).magnitude() > 0.05):
            raise ValueError("Server pose does not match selected model FK")
        return q, gripper

    def step(self, sample, state=None):
        if state is not None:
            actual, grip = self.check_state(state)
            if not state["armed"]:
                raise ValueError("Joint server disarmed; restart bridge explicitly")
            if np.max(np.abs(actual - self.q)) > 0.07 or abs(grip - self.gripper) > 0.004:
                raise ValueError("Robot tracking error exceeded joint/gripper limits")
        now = time.monotonic()
        if now - sample.changed > self.args.hold_timeout:
            self.holding, self.status = True, "holding (VR counter unchanged)"
            return self.q.copy(), self.gripper
        if self.holding:
            self.anchor_pose, self.anchor_vr = self.pose.copy(), sample.pose.copy()
            self.holding = False
        elif (np.linalg.norm(sample.pose[:3] - self.previous.pose[:3]) > 0.15
              or (rotation(sample.pose) * rotation(self.previous.pose).inv()).magnitude() > 0.8):
            raise ValueError("VR pose jumped; restart bridge to realign")
        self.previous = sample
        target = self.anchor_pose.copy()
        target[:3] += self.args.scale * self.frame.apply(sample.pose[:3] - self.anchor_vr[:3])
        relative = rotation(sample.pose) * rotation(self.anchor_vr).inv()
        relative = self.frame * relative * self.frame.inv()
        desired_rotation = (Rotation.from_rotvec(relative.as_rotvec() * self.args.rotation_scale)
                            * rotation(self.anchor_pose))
        if np.linalg.norm(target[:3] - self.origin) > self.args.radius:
            raise ValueError("Target exceeds startup workspace radius; restart to realign")
        target[:3] = self.pose[:3] + limited(target[:3] - self.pose[:3], self.args.speed * DT)
        delta_rot = (desired_rotation * rotation(self.pose).inv()).as_rotvec()
        target[3:] = (Rotation.from_rotvec(limited(delta_rot, 0.2 * DT))
                      * rotation(self.pose)).as_euler("xyz")
        candidate = self.q.copy()
        self.status = "tracking"
        if (np.linalg.norm(target[:3] - self.pose[:3]) > 1e-7
                or (rotation(target) * rotation(self.pose).inv()).magnitude() > 1e-6):
            status, solution = self.solver.inverse_kinematics(target, self.q)
            if status != 0:
                self.status = f"IK rejected: {self.solver.get_ik_status_name(status)}"
                return self.q.copy(), self.gripper
            solution = vector(solution, self.robot.joint_dof)
            if (np.any(solution < self.robot.joint_pos_min)
                    or np.any(solution > self.robot.joint_pos_max)):
                raise ValueError("IK solution outside joint limits")
            solved_pose = vector(self.solver.forward_kinematics(solution), 6)
            if (np.linalg.norm(solved_pose[:3] - target[:3]) > 0.002
                    or (rotation(solved_pose) * rotation(target).inv()).magnitude() > 0.02):
                raise ValueError("IK solution failed FK verification")
            delta = solution - self.q
            speed = np.minimum(self.args.joint_speed, self.robot.joint_vel_max)
            ratio = max(1.0, float(np.max(np.abs(delta) / (speed * DT))))
            candidate += delta / ratio
        pose = vector(self.solver.forward_kinematics(candidate), 6)
        # Joint interpolation can change Cartesian speed; check the resulting FK too.
        if (np.linalg.norm(pose[:3] - self.pose[:3]) > self.args.speed * DT * 1.1 + 1e-6
                or (rotation(pose) * rotation(self.pose).inv()).magnitude() > 0.2 * DT * 1.1 + 1e-6):
            self.status = "holding (FK speed limit)"
            return self.q.copy(), self.gripper
        if np.linalg.norm(pose[:3] - self.origin) > self.args.radius:
            raise ValueError("FK target outside startup workspace radius")
        desired_gripper = sample.gripper / 5 * self.robot.gripper_width
        gripper = self.gripper + float(np.clip(desired_gripper - self.gripper, -0.01 * DT, 0.01 * DT))
        self.q, self.pose, self.gripper = candidate, pose, gripper
        return candidate.copy(), gripper


def arm_worker(side, robot, solver, args, inputs, stop, start, ready, failed):
    client = JointClient(args.robot_host, getattr(args, side + "_port"))
    engaged = False
    try:
        inputs.get(side)
        state = client.request("GET_STATE")
        planner = Planner(robot, solver, state, inputs.get(side),
                          getattr(args, side + "_frame_rpy"), args)
        if args.execute:
            if stop.is_set():
                return
            engaged = True
            print(f"{side}: RESET_TO_HOME (arm moves and gripper opens)", flush=True)
            client.request("RESET_TO_HOME", timeout=30000)
            state = client.request("GET_STATE")
            planner = Planner(robot, solver, state, inputs.get(side),
                              getattr(args, side + "_frame_rpy"), args)
        ready.set()
        last_print = 0.0
        aligned = False
        while not stop.is_set():
            tick = time.monotonic()
            sample = inputs.get(side)
            if start.is_set():
                if not aligned:
                    planner = Planner(robot, solver, state, sample,
                                      getattr(args, side + "_frame_rpy"), args)
                    aligned = True
                q, grip = planner.step(sample, state if args.execute else None)
            else:
                q, grip = planner.q.copy(), planner.gripper
            if stop.is_set():
                break
            # Recheck age after IK; an old sample never renews the robot watchdog.
            if time.monotonic() - sample.received > args.input_timeout:
                raise ValueError("VR sample expired during planning")
            if args.execute:
                state = client.request("SET_JOINT_POS", {"joint_pos": q, "gripper_pos": grip})
            if tick - last_print >= 1:
                last_print = tick
                print(json.dumps({"arm": side, "execute": args.execute,
                                  "status": planner.status,
                                  "target_pose": planner.pose.round(5).tolist(),
                                  "joint_pos": q.round(5).tolist(), "gripper_m": round(grip, 5)}), flush=True)
            stop.wait(max(0, DT - (time.monotonic() - tick)))
    except Exception as error:
        print(f"STOP {side}: {type(error).__name__}: {error}", flush=True)
        failed.set()
        stop.set()
    finally:
        if engaged:
            try:
                client.request("SET_TO_DAMPING")
                print(f"{side}: damping acknowledged", flush=True)
            except Exception as error:
                print(f"{side}: damping not acknowledged: {error}; "
                      "server watchdog remains the fallback", flush=True)
        client.close()


def receive_session(listener, inputs, stop, failed):
    try:
        while not stop.is_set():
            try:
                connection, address = listener.accept()
                break
            except socket.timeout:
                continue
        else:
            return
        with connection:
            connection.settimeout(inputs.timeout)
            print(f"Quest connected: {address}", flush=True)
            def receive(topic, message_type, payload):
                if stop.is_set():
                    raise EOFError("Session stopped")
                inputs.receive(topic, message_type, payload)
            inspect_connection(connection, inputs.protocol, receive, report=False, stop_event=stop)
    except Exception as error:
        if not stop.is_set():
            print(f"STOP VR: {type(error).__name__}: {error}", flush=True)
            failed.set()
        stop.set()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Move both arms home, then send joint targets.")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--protocol", choices=("ROS1", "ROS2"), default="ROS2")
    parser.add_argument("--robot-host", default="127.0.0.1")
    for side, port in (("right", 8765), ("left", 8764)):
        parser.add_argument(f"--{side}-port", type=int, default=port)
        parser.add_argument(f"--{side}-model", default="X5")
        parser.add_argument(f"--{side}-frame-rpy", nargs=3, type=float, default=(0, 0, 0),
                            metavar=("ROLL", "PITCH", "YAW"), help="VR-to-base rotation in degrees.")
    parser.add_argument("--scale", type=float, default=0.2, help="Translation scale, (0, 1].")
    parser.add_argument("--rotation-scale", type=float, default=0, help="Orientation scale, [0, 1]; 0 holds orientation.")
    parser.add_argument("--speed", type=float, default=0.02, help="Cartesian speed cap, m/s (max 0.05).")
    parser.add_argument("--joint-speed", type=float, default=0.15, help="Joint speed cap, rad/s (max 0.3).")
    parser.add_argument("--radius", type=float, default=0.08, help="Radius around startup FK position, meters (max 0.15).")
    parser.add_argument("--input-timeout", type=float, default=0.3)
    parser.add_argument("--hold-timeout", type=float, default=0.1)
    args = parser.parse_args()
    for name, upper in (("scale", 1), ("speed", 0.05), ("joint_speed", 0.3),
                        ("radius", 0.15), ("input_timeout", 0.5), ("hold_timeout", 0.2)):
        if not 0 < getattr(args, name) <= upper:
            parser.error(f"{name} must be in (0, {upper}]")
    if not 0 <= args.rotation_scale <= 1 or args.hold_timeout >= args.input_timeout:
        parser.error("Invalid rotation scale or hold/input timeout ordering")
    if args.right_port == args.left_port or any(not 1 <= p <= 65535 for p in (args.port, args.right_port, args.left_port)):
        parser.error("Use valid ports and distinct left/right robot ports")
    for side in ("right", "left"):
        if not np.isfinite(getattr(args, side + "_frame_rpy")).all():
            parser.error("Frame angles must be finite")
    return args


def main():
    args = parse_args()
    import arx5_interface as sdk

    inputs = Inputs(args.protocol, args.input_timeout)
    stop, start = threading.Event(), threading.Event()
    failed = threading.Event()
    workers = []
    def interrupt(signum, frame):
        stop.set()
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.bind, args.port))
        listener.listen(1)
        listener.settimeout(0.2)
        receiver = threading.Thread(target=receive_session, args=(listener, inputs, stop, failed))
        receiver.start()
        try:
            print(f"VR bridge at {args.bind}:{args.port}. "
                  + ("EXECUTE: both arms will HOME after fresh VR data."
                     if args.execute else "PREVIEW: GET_STATE only; targets are simulated, no robot motion."), flush=True)
            print("Waiting for both Quest topics; open APK. Ctrl+C stops the session.", flush=True)
            deadline = time.monotonic() + 60
            while not stop.is_set():
                try:
                    for side in ("right", "left"):
                        inputs.get(side)
                    break
                except ValueError:
                    if time.monotonic() > deadline:
                        raise ValueError("Both Quest topics not received within 60 seconds")
                    stop.wait(0.05)
            if stop.is_set():
                return int(failed.is_set())
            configs = []
            # Preflight both endpoints before either arm is allowed to home.
            for side in ("right", "left"):
                robot = sdk.RobotConfigFactory.get_instance().get_config(getattr(args, side + "_model"))
                solver = sdk.Arx5Solver(robot.urdf_path, robot.joint_dof,
                                       robot.joint_pos_min, robot.joint_pos_max,
                                       robot.base_link_name, robot.eef_link_name, robot.gravity_vector)
                client = JointClient(args.robot_host, getattr(args, side + "_port"))
                try:
                    state = client.request("GET_STATE")
                    Planner(robot, solver, state, inputs.get(side), getattr(args, side + "_frame_rpy"), args)
                finally:
                    client.close()
                configs.append((side, robot, solver))
            ready = [threading.Event(), threading.Event()]
            for config, event in zip(configs, ready):
                worker = threading.Thread(target=arm_worker, args=(*config, args, inputs, stop, start, event, failed))
                workers.append(worker)
                worker.start()
            while not stop.wait(0.02):
                for side in ("right", "left"):
                    inputs.get(side)
                if all(event.is_set() for event in ready):
                    start.set()
        except Exception as error:
            print(f"STOP: {type(error).__name__}: {error}", flush=True)
            failed.set()
        finally:
            stop.set()
            for worker in workers:
                worker.join()
            receiver.join()
    return int(failed.is_set())


if __name__ == "__main__":
    sys.exit(main())
