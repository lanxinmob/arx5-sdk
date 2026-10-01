"""Drive two joint servers from the Quest ROS-TCP controller stream."""

import argparse
from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
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
RESET_COUNTER = 1107
RESET_HOLD = 1.0
RIGHT_CAMERA = "/dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0"
LEFT_CAMERA = "/dev/v4l/by-id/usb-MACROSILICON_Vin_4_20250814-video-index0"


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {size} finite values")
    return result.copy()


def rotation(pose):
    return Rotation.from_euler("xyz", pose[3:])


def mapped_pose(anchor_eef, anchor_vr, current_vr, frame, scale, rotation_scale):
    vr_origin = rotation(anchor_vr)
    eef_axes = rotation(anchor_eef) * frame
    target = anchor_eef.copy()
    target[:3] += scale * eef_axes.apply(vr_origin.inv().apply(current_vr[:3] - anchor_vr[:3]))
    relative_rotation = vr_origin.inv() * rotation(current_vr)
    target[3:] = (eef_axes * Rotation.from_rotvec(
        relative_rotation.as_rotvec() * rotation_scale) * frame.inv()).as_euler("xyz")
    return target


class JointClient:
    def __init__(self, host, port):
        self.context = zmq.Context()
        self.endpoint = f"tcp://{host}:{port}"
        self.open_socket()

    def open_socket(self):
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.IMMEDIATE, 1)
        self.socket.connect(self.endpoint)

    def request(self, command, data=None, timeout=500):
        self.socket.setsockopt(zmq.SNDTIMEO, timeout)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        try:
            self.socket.send_pyobj({"cmd": command, "data": data})
            reply = self.socket.recv_pyobj()
        except zmq.ZMQError:
            self.socket.close()
            self.open_socket()
            raise
        if not isinstance(reply, dict) or reply.get("cmd") != command:
            raise ValueError(f"Invalid {command} reply")
        result = reply.get("data")
        if isinstance(result, str) and result.startswith("ERROR"):
            raise ValueError(result)
        if command in ("GET_STATE", "SET_JOINT_POS"):
            if not isinstance(result, dict):
                raise ValueError(f"Missing state in {command} reply")
        elif result != "OK":
            raise ValueError(f"Unexpected {command} reply: {result!r}")
        return result

    def close(self):
        self.socket.close()
        self.context.term()


@dataclass(frozen=True)
class Sample:
    pose: np.ndarray
    gripper: float
    counter: int
    received: float
    changed: float
    generation: int
    epoch: int


def home_request(sample):
    # Observed during the A/X trial; the Quest app does not document this pattern.
    return (sample.counter == RESET_COUNTER and sample.gripper < 0.01
            and np.max(np.abs(sample.pose[3:])) < 0.05)


class Inputs:
    def __init__(self, timeout):
        self.timeout = timeout
        self.lock = threading.Lock()
        self.samples = {}
        self.generation = 0
        self.epochs = {}

    def new_session(self):
        with self.lock:
            self.generation += 1
            self.samples.clear()
            self.epochs.clear()

    def receive(self, topic, message_type, payload):
        side = {"ARX_VR_R": "right", "ARX_VR_L": "left"}.get(topic.lstrip("/"))
        if side is None or message_type is None:
            return
        if message_type != "arm_control/PosCmd":
            raise ValueError(f"{side}: unexpected Quest message type {message_type!r}")
        data = decode_pos_cmd(payload)
        pose = vector([data[key] for key in POSE_FIELDS], 6)
        gripper, counter = float(data["gripper"]), int(data["time_count"])
        if not 0 <= gripper <= 5:
            raise ValueError(f"{side}: Quest gripper outside [0, 5]")
        now = time.monotonic()
        with self.lock:
            previous = self.samples.get(side)
            epoch = self.epochs.get(side, 0)
            if previous is not None and counter < previous.counter:
                epoch += 1
                self.epochs[side] = epoch
                print(f"{side}: Quest counter reset {previous.counter} -> {counter}; realigning", flush=True)
            changed = previous.changed if previous is not None and counter == previous.counter else now
            self.samples[side] = Sample(pose, gripper, counter, now, changed, self.generation, epoch)

    def get(self, side):
        with self.lock:
            sample = self.samples.get(side)
            generation = self.generation
        if (sample is None or sample.generation != generation
                or time.monotonic() - sample.received > self.timeout):
            raise ValueError(f"{side}: no fresh Quest packets")
        return sample


class Planner:
    def __init__(self, robot, solver, state, sample, frame_rpy, args):
        self.robot, self.solver, self.args = robot, solver, args
        self.frame = Rotation.from_euler("xyz", frame_rpy, degrees=True)
        self.q = self.measured(state)
        self.pose = vector(solver.forward_kinematics(self.q), 6)
        self.anchor_eef, self.anchor_vr = self.pose.copy(), sample.pose.copy()
        self.holding = True
        self.epoch = sample.epoch
        self.status = "aligned"
        self.last_step = time.monotonic()
        self.target_history = deque(maxlen=args.joint_average_window)
        self.reset_filter()

    def reset_filter(self):
        self.target_history.clear()
        self.target_history.extend(self.q.copy() for _ in range(self.args.joint_average_window))

    def measured(self, state):
        if state.get("emergency") is not False or state.get("armed") is not True:
            raise ValueError("Joint server is disarmed or in emergency")
        q = vector(state["joint_pos"], self.robot.joint_dof)
        if np.any(q < self.robot.joint_pos_min) or np.any(q > self.robot.joint_pos_max):
            raise ValueError("Measured joint outside SDK limits")
        return q

    def solve(self, desired, actual_pose, seed_q):
        delta_rotation = (rotation(desired) * rotation(actual_pose).inv()).as_rotvec()
        for fraction in (1, 0.5, 0.25, 0.125, 0.0625, 0.03125):
            pose = actual_pose.copy()
            pose[:3] += fraction * (desired[:3] - actual_pose[:3])
            pose[3:] = (Rotation.from_rotvec(delta_rotation * fraction)
                        * rotation(actual_pose)).as_euler("xyz")
            status, candidate = self.solver.inverse_kinematics(pose, seed_q)
            candidate = np.asarray(candidate, dtype=float)
            if (status == 0 and candidate.shape == seed_q.shape
                    and np.isfinite(candidate).all()
                    and np.all(candidate >= self.robot.joint_pos_min)
                    and np.all(candidate <= self.robot.joint_pos_max)):
                return candidate, fraction
        return None, 0

    def step(self, sample, state):
        actual_q = self.measured(state)
        actual_pose = vector(self.solver.forward_kinematics(actual_q), 6)
        gripper = self.robot.gripper_width * sample.gripper / 5
        now = time.monotonic()
        elapsed = min(0.05, max(DT, now - self.last_step))
        self.last_step = now
        if sample.epoch != self.epoch:
            self.holding = True
            self.epoch = sample.epoch
        if now - sample.changed > self.args.hold_timeout:
            self.holding, self.status = True, "holding (Quest pose unchanged)"
            return self.q.copy(), gripper
        if self.holding:
            self.anchor_eef, self.anchor_vr = actual_pose.copy(), sample.pose.copy()
            self.q, self.pose = actual_q.copy(), actual_pose.copy()
            self.reset_filter()
            self.holding, self.status = False, "aligned"
        desired = mapped_pose(self.anchor_eef, self.anchor_vr, sample.pose,
                              self.frame, self.args.scale, self.args.rotation_scale)
        if (np.linalg.norm(desired[:3] - actual_pose[:3]) < 1e-7
                and (rotation(desired) * rotation(actual_pose).inv()).magnitude() < 1e-6):
            self.status = "aligned"
            return self.q.copy(), gripper
        solution, fraction = self.solve(desired, actual_pose, self.q)
        if solution is None:
            self.holding, self.status = True, "holding (IK boundary)"
            return self.q.copy(), gripper
        self.target_history.append(solution.copy())
        target = np.mean(np.stack(self.target_history), axis=0)
        delta = np.clip(target - self.q,
                        -self.robot.joint_vel_max * elapsed,
                        self.robot.joint_vel_max * elapsed)
        self.q += delta
        self.pose = vector(self.solver.forward_kinematics(self.q), 6)
        self.status = "tracking" if fraction == 1 else "tracking (IK boundary)"
        return self.q.copy(), gripper


def arm_worker(side, robot, solver, args, inputs, stop, start, ready, failed):
    client = JointClient(args.robot_host, getattr(args, side + "_port"))
    try:
        inputs.get(side)
        print(f"{side}: RESET_TO_HOME", flush=True)
        client.request("RESET_TO_HOME", timeout=30000)
        state = client.request("GET_STATE")
        ready.set()
        planner = None
        reset_since = None
        reset_latched = True
        recovering = False
        last_report = 0
        other_side = "left" if side == "right" else "right"
        while not stop.is_set():
            tick = time.monotonic()
            try:
                sample = inputs.get(side)
            except ValueError:
                if not recovering:
                    raise
                sample = None
            if start.is_set():
                if sample is None or home_request(sample):
                    if sample is not None and not recovering:
                        if home_request(inputs.get(other_side)):
                            reset_since = None
                        elif not reset_latched:
                            if reset_since is None:
                                reset_since = tick
                            elif tick - reset_since >= RESET_HOLD:
                                reset_latched = True
                                reset_since = None
                                recovering = True
                                print(f"{side}: Quest long press, slow return to home", flush=True)
                                client.request("RETURN_TO_HOME", timeout=35000)
                                state = client.request("GET_STATE")
                                planner = None
                                print(f"{side}: home complete; waiting for hand tracking", flush=True)
                    q = vector(state["joint_pos"], robot.joint_dof)
                    gripper = float(np.clip(state["gripper_pos"], 0, robot.gripper_width))
                else:
                    if recovering:
                        recovering = False
                        print(f"{side}: hand tracking resumed", flush=True)
                    reset_since = None
                    reset_latched = False
                    if planner is None:
                        planner = Planner(robot, solver, state, sample,
                                          getattr(args, side + "_frame_rpy"), args)
                    q, gripper = planner.step(sample, state)
            else:
                q = vector(state["joint_pos"], robot.joint_dof)
                gripper = float(np.clip(state["gripper_pos"], 0, robot.gripper_width))
            if stop.is_set():
                break
            if not recovering and time.monotonic() - sample.received > args.input_timeout:
                raise ValueError(f"{side}: Quest packet expired during planning")
            state = client.request("SET_JOINT_POS", {"joint_pos": q, "gripper_pos": gripper})
            if planner is not None and tick - last_report >= 1:
                last_report = tick
                print(json.dumps({"arm": side, "status": planner.status,
                                  "vr_counter": sample.counter,
                                  "target_pose": planner.pose.round(4).tolist(),
                                  "actual_joint_pos": vector(state["joint_pos"], robot.joint_dof).round(4).tolist(),
                                  "target_joint_pos": q.round(4).tolist(),
                                  "gripper_m": round(gripper, 4)}), flush=True)
            stop.wait(max(0, DT - (time.monotonic() - tick)))
    except Exception as error:
        print(f"STOP {side}: {type(error).__name__}: {error}", flush=True)
        failed.set()
        stop.set()
    finally:
        try:
            if ready.is_set():
                print(f"{side}: slow return to home", flush=True)
                client.request("RETURN_TO_HOME", timeout=35000)
        except Exception as error:
            print(f"{side}: return to home failed: {error}", flush=True)
        try:
            client.request("SET_TO_DAMPING", timeout=2000)
        except Exception as error:
            print(f"{side}: damping request failed: {error}", flush=True)
        client.close()


def receive_session(listener, inputs, stop, active, failed):
    while not stop.is_set():
        try:
            connection, address = listener.accept()
        except socket.timeout:
            continue
        except OSError:
            if not stop.is_set():
                failed.set()
                stop.set()
            return
        inputs.new_session()
        with connection:
            connection.settimeout(0.2)
            print(f"Quest connected: {address}", flush=True)
            try:
                inspect_connection(connection, inputs.receive, stop)
            except (OSError, EOFError, ValueError) as error:
                if stop.is_set():
                    return
                print(f"Quest disconnected: {error}", flush=True)
        if active.is_set():
            failed.set()
            stop.set()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--right-model", default="X5_umi")
    parser.add_argument("--left-model", default="X5")
    parser.add_argument("--right-port", type=int, default=8765)
    parser.add_argument("--left-port", type=int, default=8764)
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--rotation-scale", type=float, default=1.0)
    parser.add_argument("--joint-average-window", type=int, default=5,
                        help="IK joint targets to average (1 disables filtering)")
    parser.add_argument("--right-camera", default=RIGHT_CAMERA)
    parser.add_argument("--left-camera", default=LEFT_CAMERA)
    parser.add_argument("--no-preview", action="store_true")
    for side in ("right", "left"):
        parser.add_argument(f"--{side}-frame-rpy", nargs=3, type=float,
                            default=(0, 0, 0), metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--input-timeout", type=float, default=0.6)
    parser.add_argument("--hold-timeout", type=float, default=0.3)
    args = parser.parse_args()
    values = (args.scale, args.rotation_scale, args.input_timeout, args.hold_timeout,
              *args.right_frame_rpy, *args.left_frame_rpy)
    if not np.isfinite(values).all() or args.scale <= 0 or args.rotation_scale < 0:
        parser.error("Scale and frame values must be finite; scale must be positive")
    if not 0 < args.hold_timeout < args.input_timeout:
        parser.error("Use 0 < hold-timeout < input-timeout")
    if args.joint_average_window < 1:
        parser.error("Use joint-average-window >= 1")
    if not args.no_preview and Path(args.right_camera).resolve() == Path(args.left_camera).resolve():
        parser.error("Wrist cameras must use different devices")
    if (args.right_port == args.left_port or any(
            not 1 <= port <= 65535 for port in (args.port, args.right_port, args.left_port))):
        parser.error("Use valid ports and distinct joint server ports")
    return args


def main():
    args = parse_args()
    os.chdir(ROOT)
    import arx5_interface as sdk

    inputs = Inputs(args.input_timeout)
    stop, active, start, failed = (threading.Event() for _ in range(4))
    workers = []
    preview = None

    def interrupt(signum, frame):
        stop.set()

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.bind, args.port))
        listener.listen(1)
        listener.settimeout(0.2)
        receiver = threading.Thread(target=receive_session,
                                    args=(listener, inputs, stop, active, failed))
        receiver.start()
        try:
            if not args.no_preview:
                if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
                    try:
                        preview = subprocess.Popen([
                            sys.executable, "-u", str(Path(__file__).with_name("wrist_camera_preview.py")),
                            args.right_camera, args.left_camera,
                        ], start_new_session=True)
                    except OSError as error:
                        print(f"Wrist camera preview unavailable: {error}", flush=True)
                else:
                    print("Wrist camera preview unavailable: no desktop display", flush=True)
            print(f"Quest bridge listening on {args.bind}:{args.port}; waiting for both hands", flush=True)
            while not stop.is_set():
                try:
                    for side in ("right", "left"):
                        inputs.get(side)
                    break
                except ValueError:
                    stop.wait(0.05)
            if stop.is_set():
                return int(failed.is_set())
            configs = []
            for side in ("right", "left"):
                robot = sdk.RobotConfigFactory.get_instance().get_config(getattr(args, side + "_model"))
                solver = sdk.Arx5Solver(robot.urdf_path, robot.joint_dof,
                                       robot.joint_pos_min, robot.joint_pos_max,
                                       robot.base_link_name, robot.eef_link_name, robot.gravity_vector)
                client = JointClient(args.robot_host, getattr(args, side + "_port"))
                try:
                    state = client.request("GET_STATE")
                    if state.get("emergency") is not False:
                        raise ValueError(f"{side}: joint server emergency")
                    q = vector(state["joint_pos"], robot.joint_dof)
                    reported = vector(state["ee_pose"], 6)
                    calculated = vector(solver.forward_kinematics(q), 6)
                    if (np.linalg.norm(reported[:3] - calculated[:3]) > 0.005
                            or (rotation(reported) * rotation(calculated).inv()).magnitude() > 0.05):
                        raise ValueError(f"{side}: server model does not match {getattr(args, side + '_model')}")
                finally:
                    client.close()
                configs.append((side, robot, solver))
            ready = [threading.Event(), threading.Event()]
            active.set()
            for config, event in zip(configs, ready):
                worker = threading.Thread(target=arm_worker,
                                          args=(*config, args, inputs, stop, start, event, failed))
                workers.append(worker)
                worker.start()
            while not stop.wait(0.02):
                if all(event.is_set() for event in ready):
                    start.set()
                if preview is not None and preview.poll() is not None:
                    print(f"Wrist camera preview exited ({preview.returncode}); teleoperation continues", flush=True)
                    preview = None
        except Exception as error:
            print(f"STOP: {type(error).__name__}: {error}", flush=True)
            failed.set()
        finally:
            stop.set()
            for worker in workers:
                worker.join()
            receiver.join()
            if preview is not None and preview.poll() is None:
                preview.terminate()
                try:
                    preview.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    preview.kill()
                    preview.wait()
    return int(failed.is_set())


if __name__ == "__main__":
    sys.exit(main())
