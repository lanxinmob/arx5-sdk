"""Inspect Unity ROS-TCP messages without connecting to robot hardware."""

import argparse
import json
import socket
import struct
import time


def receive_exact(connection, size):
    data = bytearray()
    while len(data) < size:
        chunk = connection.recv(size - len(data))
        if not chunk:
            raise EOFError("Peer disconnected")
        data.extend(chunk)
    return bytes(data)


def receive_field(connection, limit):
    size = struct.unpack("<I", receive_exact(connection, 4))[0]
    if size > limit:
        raise ValueError(f"Field size {size} exceeds limit {limit}")
    return receive_exact(connection, size)


def send_command(connection, command, params):
    topic = command.encode("utf-8")
    payload = json.dumps(params).encode("utf-8")
    connection.sendall(struct.pack("<I", len(topic)) + topic
                       + struct.pack("<I", len(payload)) + payload)


def inspect_connection(connection, protocol):
    # Unity ROS-TCP-Endpoint v0.7.0 uses a JSON string inside metadata.
    send_command(connection, "__handshake", {
        "version": "v0.7.0", "metadata": json.dumps({"protocol": protocol}),
    })
    publishers, counts, last_print = {}, {}, {}
    while True:
        topic = receive_field(connection, 4096).decode("utf-8").rstrip("\0")
        payload = receive_field(connection, 1024 * 1024)
        if not topic:
            continue
        if topic.startswith("__"):
            params = json.loads(payload)
            if not isinstance(params, dict):
                raise ValueError("System command must contain a JSON object")
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
        counts[topic] = counts.get(topic, 0) + 1
        now = time.monotonic()
        if now - last_print.get(topic, float("-inf")) >= 1:
            last_print[topic] = now
            print(json.dumps({
                "topic": topic, "message_type": publishers.get(topic, "unregistered"),
                "count": counts[topic], "bytes": len(payload),
                "preview_hex": payload[:128].hex(),
            }), flush=True)


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
