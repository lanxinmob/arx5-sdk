"""Inspect ARX VR ROS 2 input and preview bounded Cartesian targets without CAN."""

import argparse
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


def sample_from_msg(msg, now):
    pose = np.array([msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw], dtype=float)
    gripper = float(msg.gripper)
    if not np.isfinite(pose).all() or not math.isfinite(gripper):
        raise ValueError("Quest pose or gripper contains a nonfinite value")
    return QuestSample(pose, gripper, msg.mode1, msg.mode2, msg.time_count, now)


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
    def __init__(self, home_pose, scale, input_meters_per_unit, speed, radius, timeout):
        self.home = np.asarray(home_pose, dtype=float).copy()
        if self.home.shape != (6,) or not np.isfinite(self.home).all():
            raise ValueError("Home pose must have six finite values")
        self.target = self.home.copy()
        self.scale = scale * input_meters_per_unit
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
        if np.linalg.norm(delta_input) * self.scale > 0.03:
            self.anchor = None
            self.last_step = None
            return self.target.copy(), "holding: Quest position jumped"

        desired = self.anchor_target + (sample.pose[:3] - self.anchor) * self.scale
        relative = desired - self.home[:3]
        distance = float(np.linalg.norm(relative))
        clamped = distance > self.radius
        if clamped:
            desired = self.home[:3] + relative * self.radius / distance

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


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--mode", choices=("raw", "target"), default="raw")
    parser.add_argument("--model", help="SDK model for target preview; defaults to X5_umi on right, X5 on left")
    parser.add_argument("--scale", type=positive, default=0.2, help="Quest translation to robot translation ratio")
    parser.add_argument("--input-meters-per-unit", type=positive, default=1.0,
                        help="Use 0.001 if the official topic reports millimeters")
    parser.add_argument("--speed", type=positive, default=0.02, help="Preview target speed cap in m/s")
    parser.add_argument("--radius", type=positive, default=0.08, help="Preview radius around SDK home in meters")
    parser.add_argument("--timeout", type=positive, default=0.3, help="Maximum Quest message age in seconds")
    parser.add_argument("--duration", type=positive, help="Exit after this many seconds")
    return parser.parse_args()


def main():
    args = parse_args()
    import rclpy
    from rclpy.node import Node
    from arm_control.msg import PosCmd

    topic = "/ARX_VR_R" if args.side == "right" else "/ARX_VR_L"
    preview = None
    gripper_hold = None
    if args.mode == "target":
        model = args.model or ("X5_umi" if args.side == "right" else "X5")
        home, gripper_hold = offline_home(model)
        preview = CartesianPreview(home, args.scale, args.input_meters_per_unit,
                                   args.speed, args.radius, args.timeout)
        print(json.dumps({"mode": "target", "model": model, "home_pose": home.tolist(),
                          "gripper_hold_m": gripper_hold, "can_connected": False}), flush=True)

    class QuestNode(Node):
        def __init__(self):
            super().__init__(f"quest_arx5_{args.side}_preview")
            self.sample = None
            self.count = 0
            self.last_print = 0.0
            self.create_subscription(PosCmd, topic, self.on_message, 10)
            self.create_timer(0.1, self.report)

        def on_message(self, msg):
            try:
                self.sample = sample_from_msg(msg, time.monotonic())
            except ValueError as error:
                self.get_logger().warning(str(error))
                return
            self.count += 1

        def report(self):
            now = time.monotonic()
            sample = self.sample
            if preview is not None:
                target, status = preview.step(sample, now)
            if now - self.last_print < 0.2:
                return
            self.last_print = now
            output = {"topic": topic, "count": self.count}
            if sample is not None:
                output.update(vr_pose=sample.pose.tolist(), gripper_raw=sample.gripper,
                              mode1=sample.mode1, mode2=sample.mode2,
                              time_count=sample.counter,
                              age_s=round(now - sample.received, 3))
            if preview is not None:
                output.update(target_pose=target.round(6).tolist(),
                              target_gripper_m=gripper_hold, status=status)
            elif sample is None:
                output["status"] = "waiting for Quest data"
            elif now - sample.received > args.timeout:
                output["status"] = "Quest data stale"
            print(json.dumps(output), flush=True)

    rclpy.init()
    node = QuestNode()
    started = time.monotonic()
    try:
        while rclpy.ok() and (args.duration is None or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
