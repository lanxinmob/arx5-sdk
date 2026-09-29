"""Inspect Unity ROS-TCP messages without connecting to robot hardware."""

import argparse
import json
import math
import socket
import struct
import time


def decode_pos_cmd(payload, protocol):
    """Decode ARX arm_control/PosCmd: 25 float64 values and nine int32s."""
    offset, endian = 0, "<"
    if protocol == "ROS2":
        if payload[:4] not in (b"\x00\x01\x00\x00", b"\x00\x00\x00\x00"):
            raise ValueError("Unsupported PosCmd CDR header")
        offset = 4
        endian = "<" if payload[1] == 1 else ">"
    if len(payload) != offset + 236:
        raise ValueError(f"Expected {offset + 236} PosCmd bytes, got {len(payload)}")
    values = struct.unpack_from(endian + "25d9i", payload, offset)
    if not all(math.isfinite(value) for value in values[:25]):
        raise ValueError("Nonfinite PosCmd float field")
    names = ("x y z roll pitch yaw gripper quater_x quater_y quater_z quater_w "
             "chx chy chz vel_l vel_r height head_pit head_yaw").split()
    result = dict(zip(names, values[:19]))
    result.update(temp_float_data=values[19:25], temp_int_data=values[25:31],
                  mode1=values[31], mode2=values[32], time_count=values[33])
    return result


def receive_exact(connection, size, stop_event=None, wait_on_timeout=False):
    data = bytearray()
    while len(data) < size:
        if stop_event is not None and stop_event.is_set():
            raise EOFError("Session stopped")
        try:
            chunk = connection.recv(size - len(data))
        except socket.timeout:
            if wait_on_timeout:
                continue
            raise
        if not chunk:
            raise EOFError("Peer disconnected")
        data.extend(chunk)
    return bytes(data)


def receive_field(connection, limit, stop_event=None, wait_on_timeout=False):
    size = struct.unpack("<I", receive_exact(connection, 4, stop_event, wait_on_timeout))[0]
    if size > limit:
        raise ValueError(f"Field size {size} exceeds limit {limit}")
    return receive_exact(connection, size, stop_event, wait_on_timeout)


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


def inspect_connection(connection, protocol, on_message=None, report=True, stop_event=None,
                       wait_on_timeout=False):
    # Unity ROS-TCP-Endpoint v0.7.0 uses a JSON string inside metadata.
    send_command(connection, "__handshake", {
        "version": "v0.7.0", "metadata": json.dumps({"protocol": protocol}),
    })
    publishers, counts, last_print = {}, {}, {}
    while stop_event is None or not stop_event.is_set():
        topic = receive_field(connection, 4096, stop_event, wait_on_timeout).decode("utf-8").rstrip("\0")
        payload = receive_field(connection, 1024 * 1024, stop_event, wait_on_timeout)
        if not topic:
            continue
        if topic.startswith("__"):
            params = parse_command(topic, payload)
            if report:
                print(f"{topic}: {params}", flush=True)
            if topic == "__publish":
                name, message = params.get("topic"), params.get("message_name")
                if not isinstance(name, str) or not isinstance(message, str):
                    raise ValueError("Invalid publisher registration")
                publishers[name] = message
            elif topic == "__topic_list":
                send_command(connection, "__topic_list", {"topics": [], "types": []})
            elif topic != "__subscribe":
                send_command(connection, "__error", {
                    "text": f"Diagnostic receiver does not implement {topic}",
                })
            continue
        if on_message is not None:
            on_message(topic, publishers.get(topic), payload)
        if not report:
            continue
        counts[topic] = counts.get(topic, 0) + 1
        now = time.monotonic()
        if now - last_print.get(topic, float("-inf")) >= 1:
            last_print[topic] = now
            report = {
                "topic": topic, "message_type": publishers.get(topic, "unregistered"),
                "count": counts[topic], "bytes": len(payload),
                "preview_hex": payload[:128].hex(),
            }
            if publishers.get(topic) == "arm_control/PosCmd":
                try:
                    report["pos_cmd"] = decode_pos_cmd(payload, protocol)
                except ValueError as error:
                    report["decode_error"] = str(error)
            print(json.dumps(report), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--protocol", choices=("ROS1", "ROS2"), default="ROS1")
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((args.bind, args.port))
        listener.listen(1)
        print(f"VR diagnostic receiver listening on {args.bind}:{args.port}; "
              "NO robot commands are sent. Ctrl+C to stop.", flush=True)
        while True:
            connection, address = listener.accept()
            with connection:
                connection.settimeout(15)
                print(f"Connected: {address}", flush=True)
                try:
                    inspect_connection(connection, args.protocol)
                except (OSError, EOFError, ValueError) as error:
                    print(f"Disconnected: {address}: {error}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
