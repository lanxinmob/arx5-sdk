"""Decode Quest ROS-TCP controller messages for the dual-arm bridge."""

import json
import math
import socket
import struct


def decode_pos_cmd(payload):
    """Extract the pose, gripper, and counter from a ROS2 arm_control/PosCmd."""
    if len(payload) != 240:
        raise ValueError(f"Expected 240 PosCmd bytes, got {len(payload)}")
    if payload[:4] not in (b"\x00\x01\x00\x00", b"\x00\x00\x00\x00"):
        raise ValueError("Unsupported PosCmd CDR header")
    endian = "<" if payload[1] == 1 else ">"
    values = struct.unpack_from(endian + "25d9i", payload, 4)
    if not all(math.isfinite(value) for value in values[:25]):
        raise ValueError("Nonfinite PosCmd float field")
    result = dict(zip(("x", "y", "z", "roll", "pitch", "yaw", "gripper"), values[:7]))
    result["time_count"] = values[-1]
    return result


def receive_exact(connection, size, stop_event):
    data = bytearray()
    while len(data) < size:
        if stop_event.is_set():
            raise EOFError("Session stopped")
        try:
            chunk = connection.recv(size - len(data))
        except socket.timeout:
            continue
        if not chunk:
            raise EOFError("Peer disconnected")
        data.extend(chunk)
    return bytes(data)


def receive_field(connection, limit, stop_event):
    size = struct.unpack("<I", receive_exact(connection, 4, stop_event))[0]
    if size > limit:
        raise ValueError(f"Field size {size} exceeds limit {limit}")
    return receive_exact(connection, size, stop_event)


def send_command(connection, command, params):
    topic = command.encode("utf-8")
    payload = json.dumps(params).encode("utf-8")
    connection.sendall(struct.pack("<I", len(topic)) + topic
                       + struct.pack("<I", len(payload)) + payload)


def parse_command(topic, payload):
    # Unity's ROS2 WriteUnaligned includes a single NUL in the string length.
    data = payload[:-1] if payload.endswith(b"\0") else payload
    try:
        params = json.loads(data)
    except (ValueError, UnicodeError) as error:
        raise ValueError(
            f"Invalid JSON for {topic!r}: {error}; "
            f"bytes={len(payload)}, tail_hex={payload[-32:].hex()}"
        ) from error
    if not isinstance(params, dict):
        raise ValueError("System command must contain a JSON object")
    return params


def inspect_connection(connection, on_message, stop_event):
    # Unity ROS-TCP-Endpoint v0.7.0 uses a JSON string inside metadata.
    send_command(connection, "__handshake", {
        "version": "v0.7.0", "metadata": json.dumps({"protocol": "ROS2"}),
    })
    publishers = {}
    while not stop_event.is_set():
        topic = receive_field(connection, 4096, stop_event).decode("utf-8").rstrip("\0")
        payload = receive_field(connection, 1024 * 1024, stop_event)
        if not topic:
            continue
        if topic.startswith("__"):
            params = parse_command(topic, payload)
            if topic == "__publish":
                name, message = params.get("topic"), params.get("message_name")
                if not isinstance(name, str) or not isinstance(message, str):
                    raise ValueError("Invalid publisher registration")
                publishers[name] = message
            elif topic == "__topic_list":
                send_command(connection, "__topic_list", {"topics": [], "types": []})
            elif topic != "__subscribe":
                send_command(connection, "__error", {
                    "text": f"Quest bridge does not implement {topic}",
                })
            continue
        on_message(topic, publishers.get(topic), payload)
