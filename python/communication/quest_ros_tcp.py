"""Publish Quest Unity ROS-TCP PosCmd packets as ROS 2 topics; no robot control."""

import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.serialization import deserialize_message
from arm_control.msg import PosCmd

from vr_receiver import inspect_connection


TOPICS = {"ARX_VR_R": "/ARX_VR_R", "ARX_VR_L": "/ARX_VR_L"}


class RosShutdown:
    def is_set(self):
        return not rclpy.ok()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("quest_ros_tcp")
    publishers = {name: node.create_publisher(PosCmd, topic, 10)
                  for name, topic in TOPICS.items()}
    counts = {name: 0 for name in TOPICS}
    last_report = {name: 0.0 for name in TOPICS}

    def publish(topic, message_type, payload):
        if not rclpy.ok():
            return
        name = topic.lstrip("/")
        if name not in publishers:
            return
        if message_type is None:
            return
        if message_type != "arm_control/PosCmd" or len(payload) != 240:
            raise ValueError(f"Unexpected {name} message: {message_type}, {len(payload)} bytes")
        msg = deserialize_message(payload, PosCmd)
        if not all(math.isfinite(value) for value in
                   (msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.gripper)):
            raise ValueError(f"Nonfinite {name} pose or gripper")
        publishers[name].publish(msg)
        counts[name] += 1
        now = time.monotonic()
        if now - last_report[name] >= 1.0:
            last_report[name] = now
            print(json.dumps({"topic": TOPICS[name], "count": counts[name],
                              "xyz": [msg.x, msg.y, msg.z],
                              "rpy": [msg.roll, msg.pitch, msg.yaw],
                              "gripper_raw": msg.gripper,
                              "time_count": msg.time_count}), flush=True)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((args.bind, args.port))
            listener.listen(1)
            listener.settimeout(1.0)
            print(f"Quest ROS-TCP listening on {args.bind}:{args.port}; no CAN access", flush=True)
            while rclpy.ok():
                try:
                    connection, address = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.settimeout(5.0)
                    print(f"Quest connected: {address}", flush=True)
                    try:
                        inspect_connection(connection, "ROS2", publish, report=False,
                                           stop_event=RosShutdown())
                    except (OSError, EOFError, ValueError) as error:
                        print(f"Quest disconnected: {error}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
