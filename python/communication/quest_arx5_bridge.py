"""Preview Quest targets or send bounded XYZ targets to a guarded ARX5 server."""

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
import time

import numpy as np


@dataclass(frozen=True)
class QuestSample:
    pose: np.ndarray
    gripper: float
    mode1: int
    mode2: int
    counter: int
    received: float


def sample_from_msg(msg, now, previous=None):
    pose = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw], dtype=float)
    gripper = float(msg.gripper)
    if not np.isfinite(pose).all() or not math.isfinite(gripper):
        raise ValueError("Quest pose or gripper contains a nonfinite value")
    received = now
    if previous is not None and msg.time_count != 0 and msg.time_count == previous.counter:
        # The Wi-Fi app can repeat its last pose while this counter is frozen.
        received = previous.received
    return QuestSample(pose, gripper, msg.mode1, msg.mode2, msg.time_count, received)


def offline_home(model):
    import arx5_interface as arx5

    config = arx5.RobotConfigFactory.get_instance().get_config(model)
    solver = arx5.Arx5Solver(
        config.urdf_path, config.joint_dof, config.joint_pos_min,
        config.joint_pos_max, config.base_link_name, config.eef_link_name,
        config.gravity_vector,
    )
    return np.asarray(solver.forward_kinematics(np.zeros(config.joint_dof)), dtype=float), config.gripper_width


class CartesianPreview:
    def __init__(self, home_pose, scale, input_meters_per_unit, speed, radius, timeout,
                 xyz_map=("x", "y", "z"), input_jump_limit=0.1,
                 workspace_min=None, workspace_max=None):
        self.home = np.asarray(home_pose, dtype=float).copy()
        if self.home.shape != (6,) or not np.isfinite(self.home).all():
            raise ValueError("Home pose must have six finite values")
        if (workspace_min is None) != (workspace_max is None):
            raise ValueError("Workspace min and max must be provided together")
        self.workspace_min = None
        self.workspace_max = None
        if workspace_min is not None:
            self.workspace_min = np.asarray(workspace_min, dtype=float).copy()
            self.workspace_max = np.asarray(workspace_max, dtype=float).copy()
            if (self.workspace_min.shape != (3,) or self.workspace_max.shape != (3,) or
                    not np.isfinite(self.workspace_min).all() or
                    not np.isfinite(self.workspace_max).all() or
                    np.any(self.workspace_min > self.home[:3]) or
                    np.any(self.workspace_max < self.home[:3])):
                raise ValueError("Workspace bounds must contain the finite home XYZ")
        self.target = self.home.copy()
        self.scale = scale * input_meters_per_unit
        self.input_meters_per_unit = input_meters_per_unit
        self.input_jump_limit = input_jump_limit
        self.xyz_matrix = np.zeros((3, 3), dtype=float)
        for robot_axis, source_axis in enumerate(xyz_map):
            if source_axis != "0":
                sign = -1 if source_axis.startswith("-") else 1
                source_index = "xyz".index(source_axis.lstrip("+-"))
                self.xyz_matrix[robot_axis, source_index] = sign
        self.speed = speed
        self.radius = radius
        self.timeout = timeout
        self.anchor = None
        self.anchor_target = None
        self.previous = None
        self.last_step = None

    def step(self, sample, now):
        if sample is None or now - sample.received > self.timeout:
            self.anchor = None
            self.previous = None
            self.last_step = None
            return self.target.copy(), "waiting for fresh Quest data"

        if self.anchor is None:
            self.anchor = sample.pose[:3].copy()
            self.anchor_target = self.target[:3].copy()
            self.previous = sample.pose[:3].copy()
            self.last_step = now
            return self.target.copy(), "aligned"

        delta_input = sample.pose[:3] - self.previous
        self.previous = sample.pose[:3].copy()
        input_step = float(np.linalg.norm(delta_input))
        if (input_step * self.input_meters_per_unit > self.input_jump_limit or
                input_step * self.scale > 0.03):
            self.anchor = None
            self.last_step = None
            return self.target.copy(), "holding: Quest position jumped"

        displacement = self.xyz_matrix @ (sample.pose[:3] - self.anchor)
        desired = self.anchor_target + displacement * self.scale
        relative = desired - self.home[:3]
        distance = float(np.linalg.norm(relative))
        clamped = distance > self.radius
        if clamped:
            desired = self.home[:3] + relative * self.radius / distance
        if self.workspace_min is not None:
            bounded = np.clip(desired, self.workspace_min, self.workspace_max)
            clamped = clamped or not np.array_equal(bounded, desired)
            desired = bounded

        dt = min(max(now - self.last_step, 0.0), 0.1)
        self.last_step = now
        delta = desired - self.target[:3]
        travel = float(np.linalg.norm(delta))
        if travel > self.speed * dt and travel > 0:
            delta *= self.speed * dt / travel
        self.target[:3] += delta
        return self.target.copy(), "workspace clamped" if clamped else "tracking"


def positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("Expected a positive finite number")
    return result


def xyz_map(value):
    axes = tuple(axis.strip().lower() for axis in value.split(","))
    valid = {"x", "+x", "-x", "y", "+y", "-y", "z", "+z", "-z", "0"}
    if len(axes) != 3 or any(axis not in valid for axis in axes):
        raise argparse.ArgumentTypeError(
            "Expected three source axes, e.g. y,-x,z or 0,-x,0")
    nonzero = [axis.lstrip("+-") for axis in axes if axis != "0"]
    if len(nonzero) != len(set(nonzero)):
        raise argparse.ArgumentTypeError("Each Quest source axis may be used at most once")
    return axes


class GuardedCartesianClient:
    def __init__(self, port):
        import zmq

        self.zmq = zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDTIMEO, 1000)
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)
        self.socket.connect(f"tcp://127.0.0.1:{port}")

    def request(self, command, data=None, timeout_ms=1000):
        self.socket.setsockopt(self.zmq.RCVTIMEO, timeout_ms)
        try:
            self.socket.send_pyobj({"cmd": command, "data": data})
            reply = self.socket.recv_pyobj()
        except self.zmq.ZMQError as error:
            raise RuntimeError(f"Guarded server {command} failed: {error}") from error
        finally:
            self.socket.setsockopt(self.zmq.RCVTIMEO, 1000)
        if not isinstance(reply, dict) or reply.get("cmd") != command:
            raise RuntimeError(f"Invalid guarded server reply to {command}: {reply!r}")
        result = reply.get("data")
        if isinstance(result, str) and result.startswith("ERROR:"):
            raise RuntimeError(f"Guarded server rejected {command}: {result}")
        return result

    def close(self):
        self.socket.close(0)
        self.context.term()


def guarded_start(client, model, interface, radius, max_tracking_error):
    info = client.request("GET_SERVER_INFO")
    if not isinstance(info, dict) or info.get("name") != "arx5_agent_safe":
        raise RuntimeError("Expected the guarded Cartesian server")
    if info.get("model") != model or info.get("interface") != interface:
        raise RuntimeError(
            f"Guarded server is {info.get('model')}/{info.get('interface')}, "
            f"expected {model}/{interface}")
    limits = info.get("limits", {})
    if limits.get("workspace_min") is None or limits.get("workspace_max") is None:
        raise RuntimeError("Guarded server needs measured workspace min/max bounds")
    server_radius = float(limits["max_home_translation_radius"])
    if not math.isfinite(server_radius) or server_radius > radius:
        raise RuntimeError("Guarded server home radius exceeds requested Quest radius")
    server_step = float(limits["max_position_delta"])
    if not math.isfinite(server_step) or server_step > 0.002:
        raise RuntimeError("Guarded server position step must be at most 0.002 m")
    if info.get("armed"):
        raise RuntimeError("Guarded server is already armed; restart it before Quest control")
    home = np.asarray(info["home_pose"], dtype=float)
    workspace_min = np.asarray(limits["workspace_min"], dtype=float)
    workspace_max = np.asarray(limits["workspace_max"], dtype=float)
    if (home.shape != (6,) or not np.isfinite(home).all() or
            workspace_min.shape != (3,) or workspace_max.shape != (3,) or
            not np.isfinite(workspace_min).all() or not np.isfinite(workspace_max).all() or
            np.any(home[:3] < workspace_min) or np.any(home[:3] > workspace_max)):
        raise RuntimeError("Guarded server home pose is outside its finite workspace")
    client.request("RESET_TO_HOME", timeout_ms=30000)
    state = client.request("GET_STATE")
    if not isinstance(state, dict) or state.get("emergency"):
        raise RuntimeError("Guarded server reported an emergency or invalid state")
    command_pose = np.asarray(state["ee_cmd_pose"], dtype=float)
    measured_pose = np.asarray(state["ee_pose"], dtype=float)
    gripper = float(state["gripper_cmd_pos"])
    if (command_pose.shape != (6,) or not np.isfinite(command_pose).all() or
            measured_pose.shape != (6,) or not np.isfinite(measured_pose).all() or
            not math.isfinite(gripper) or
            np.linalg.norm(command_pose[:3] - home[:3]) > 0.0001 or
            np.linalg.norm(measured_pose[:3] - command_pose[:3]) > max_tracking_error):
        raise RuntimeError("Guarded server did not settle near a valid home command")
    if np.any(command_pose[:3] < workspace_min) or np.any(command_pose[:3] > workspace_max):
        raise RuntimeError("Guarded server command is outside its workspace")
    return command_pose, gripper, workspace_min, workspace_max, server_radius


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--mode", choices=("raw", "target", "control"), default="raw")
    parser.add_argument("--model", help="SDK model; defaults to X5_umi on right, X5 on left")
    parser.add_argument("--interface", help="Expected CAN interface of the guarded server")
    parser.add_argument("--server-port", type=int, default=8765)
    parser.add_argument("--reset-to-home", action="store_true",
                        help="Allow control mode to reset the arm before sending Quest targets")
    parser.add_argument("--scale", type=positive, help="Quest translation to robot translation ratio")
    parser.add_argument("--input-meters-per-unit", type=positive, default=1.0,
                        help="Use 0.001 if the official topic reports millimeters")
    parser.add_argument("--xyz-map", type=xyz_map,
                        help="Robot X,Y,Z source axes; 0 disables an axis (e.g. 0,-x,0)")
    parser.add_argument("--speed", type=positive, help="Target speed cap in m/s")
    parser.add_argument("--radius", type=positive, help="Target radius around SDK home in meters")
    parser.add_argument("--input-jump-limit", type=positive, default=0.1,
                        help="Reanchor after a larger Quest position step in meters")
    parser.add_argument("--max-tracking-error", type=positive, default=0.006,
                        help="Stop control if measured XYZ error exceeds this value in meters")
    parser.add_argument("--timeout", type=positive, default=0.3, help="Maximum Quest message age in seconds")
    parser.add_argument("--duration", type=positive, help="Exit after this many seconds")
    args = parser.parse_args()
    if not 1 <= args.server_port <= 65535:
        parser.error("--server-port must be between 1 and 65535")
    if args.mode == "control":
        missing = [name for name in ("interface", "scale", "xyz_map", "speed", "radius")
                   if getattr(args, name) is None]
        if missing or not args.reset_to_home:
            parser.error("Control requires --interface --scale --xyz-map --speed "
                         "--radius --reset-to-home")
        if args.speed > 0.01 or args.radius > 0.02:
            parser.error("Initial control is limited to 0.01 m/s and a 0.02 m home radius")
    elif args.reset_to_home:
        parser.error("--reset-to-home is only valid with --mode control")
    if args.scale is None:
        args.scale = 0.2
    if args.xyz_map is None:
        args.xyz_map = ("x", "y", "z")
    if args.speed is None:
        args.speed = 0.02
    if args.radius is None:
        args.radius = 0.08
    return args


def main():
    args = parse_args()
    import rclpy
    from rclpy.node import Node
    from arm_control.msg import PosCmd

    topic = "/ARX_VR_R" if args.side == "right" else "/ARX_VR_L"
    preview = None
    gripper_hold = None
    model = args.model or ("X5_umi" if args.side == "right" else "X5")
    if args.mode == "target":
        home, gripper_hold = offline_home(model)
        preview = CartesianPreview(home, args.scale, args.input_meters_per_unit,
                                   args.speed, args.radius, args.timeout, args.xyz_map,
                                   args.input_jump_limit)
        print(json.dumps({"mode": "target", "model": model, "home_pose": home.tolist(),
                          "gripper_hold_m": gripper_hold, "xyz_map": args.xyz_map,
                          "can_connected": False}), flush=True)

    class QuestNode(Node):
        def __init__(self):
            super().__init__(f"quest_arx5_{args.side}_{args.mode}")
            self.sample = None
            self.count = 0
            self.last_print = 0.0
            self.dynamic_poses = deque(maxlen=128)
            self.preview = preview
            self.gripper_hold = gripper_hold
            self.robot = None
            self.create_subscription(PosCmd, topic, self.on_message, 10)
            self.create_timer(0.1, self.report)

        def on_message(self, msg):
            try:
                previous = self.sample
                self.sample = sample_from_msg(msg, time.monotonic(), previous)
            except ValueError as error:
                self.get_logger().warning(str(error))
                return
            self.count += 1
            if previous is not None and self.sample.counter < previous.counter:
                self.dynamic_poses.clear()
            elif (previous is not None and self.sample.counter != 0 and
                  self.sample.counter > previous.counter):
                step = np.linalg.norm(self.sample.pose[:3] - previous.pose[:3])
                if step * args.input_meters_per_unit > args.input_jump_limit:
                    self.dynamic_poses.clear()
                self.dynamic_poses.append((self.sample.received, self.sample.pose[:3].copy()))

        def source_ready(self, now):
            if self.sample is None or now - self.sample.received > args.timeout:
                return False
            recent = [pose for received, pose in self.dynamic_poses
                      if now - received <= 1.0]
            if len(recent) < 3:
                return False
            travel = max(float(np.linalg.norm(pose - recent[0])) for pose in recent[1:])
            return travel * args.input_meters_per_unit >= 0.005

        def report(self):
            now = time.monotonic()
            sample = self.sample
            if self.preview is not None:
                target, status = self.preview.step(sample, now)
                if self.robot is not None:
                    state = self.robot.request("SET_EE_POSE", {
                        "ee_pose": target, "gripper_pos": self.gripper_hold})
                    if not isinstance(state, dict) or state.get("emergency"):
                        raise RuntimeError("Guarded server reported an emergency or invalid state")
                    command_pose = np.asarray(state["ee_cmd_pose"], dtype=float)
                    if command_pose.shape != (6,) or not np.allclose(command_pose, target, atol=1e-6):
                        raise RuntimeError("Guarded server command differs from Quest target")
                    measured = np.asarray(state["ee_pose"], dtype=float)
                    if (measured.shape != (6,) or not np.isfinite(measured).all() or
                            np.linalg.norm(measured[:3] - target[:3]) > args.max_tracking_error):
                        raise RuntimeError("Guarded server XYZ tracking error exceeded limit")
            if now - self.last_print < 0.2:
                return
            self.last_print = now
            output = {"topic": topic, "count": self.count}
            if sample is not None:
                output.update(vr_pose=sample.pose.tolist(), gripper_raw=sample.gripper,
                              mode1=sample.mode1, mode2=sample.mode2,
                              time_count=sample.counter,
                              age_s=round(now - sample.received, 3))
            if self.preview is not None:
                output.update(target_pose=target.round(6).tolist(),
                              target_gripper_m=self.gripper_hold, status=status)
                if self.robot is not None:
                    output.update(measured_pose=measured.round(6).tolist(),
                                  position_error_m=round(float(np.linalg.norm(
                                      measured[:3] - target[:3])), 6), can_connected=True)
            elif sample is None:
                output["status"] = "waiting for Quest data"
            elif now - sample.received > args.timeout:
                output["status"] = "Quest data stale"
            elif args.mode == "control":
                output["status"] = "waiting for moving Quest poses"
            print(json.dumps(output), flush=True)

    rclpy.init()
    node = QuestNode()
    robot = None
    started = time.monotonic()
    try:
        while rclpy.ok() and (args.duration is None or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.mode == "control" and robot is None and node.source_ready(time.monotonic()):
                robot = GuardedCartesianClient(args.server_port)
                home, gripper_hold, workspace_min, workspace_max, server_radius = guarded_start(
                    robot, model, args.interface, args.radius, args.max_tracking_error)
                node.preview = CartesianPreview(home, args.scale, args.input_meters_per_unit,
                                                args.speed, server_radius, args.timeout, args.xyz_map,
                                                args.input_jump_limit, workspace_min,
                                                workspace_max)
                node.gripper_hold = gripper_hold
                node.robot = robot
                print(json.dumps({"mode": "control", "model": model,
                                  "interface": args.interface, "home_pose": home.tolist(),
                                  "gripper_hold_m": gripper_hold, "xyz_map": args.xyz_map,
                                  "speed_m_s": args.speed, "radius_m": server_radius,
                                  "workspace_min": workspace_min.tolist(),
                                  "workspace_max": workspace_max.tolist(),
                                  "can_connected": True}), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if robot is not None:
            robot.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
