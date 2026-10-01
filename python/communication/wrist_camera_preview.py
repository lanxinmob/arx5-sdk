"""Show a selected iPhUMI camera and two wrist cameras during VR control."""

import argparse
from datetime import datetime
import multiprocessing
import os
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np


HEADER_HEIGHT = 40
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = WINDOW_WIDTH * 3 // 4
MAIN_HEIGHT = 600
WINDOW = "ARX5 Cameras"
IPHUMI_PORTS = {"main": 5555, "ultrawide": 5556}


class CameraFeed:
    unavailable_status = "CAMERA UNAVAILABLE"

    def __init__(self, device):
        self.device = device
        self.frame = None
        self.received = 0.0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.thread.start()

    def capture(self):
        while not self.stop.is_set():
            camera = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if camera.isOpened():
                camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                camera.set(cv2.CAP_PROP_FPS, 30)
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                while not self.stop.is_set():
                    ok, frame = camera.read()
                    if not ok:
                        break
                    with self.lock:
                        self.frame = frame
                        self.received = time.monotonic()
            camera.release()
            with self.lock:
                self.frame = None
            self.stop.wait(1)

    def latest(self):
        with self.lock:
            return self.frame, self.received

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)


def receive_iphone_usb(sender, port):
    try:
        import pypeertalk
    except ImportError:
        print("iPHUMI USB preview requires pypeertalk==0.1.2", flush=True)
        sender.close()
        return
    try:
        last_error = 0.0
        while True:
            try:
                devices = pypeertalk.get_connected_devices()
                if devices:
                    sender.send_bytes(b"")
                    client = pypeertalk.PeerTalkClient(devices[0], port, 1)
                    while True:
                        message = client.get_latest_message(1000)
                        if not message:
                            break
                        sender.send_bytes(message)
            except Exception as error:
                now = time.monotonic()
                if now - last_error >= 10:
                    print(f"iPHUMI USB: {error}", flush=True)
                    last_error = now
            time.sleep(1)
    finally:
        sender.close()


class IPhoneUSBFeed:
    unavailable_status = "IPHUMI USB UNAVAILABLE"

    def __init__(self, stream):
        self.stream = stream
        self.context = multiprocessing.get_context("spawn")
        self.frame = None
        self.received = 0.0
        self.connected = 0.0
        self.start()

    def start(self):
        self.receiver, sender = self.context.Pipe(duplex=False)
        self.process = self.context.Process(target=receive_iphone_usb,
                                            args=(sender, IPHUMI_PORTS[self.stream]), daemon=True)
        try:
            self.process.start()
        finally:
            sender.close()
        self.started = time.monotonic()

    def latest(self):
        jpeg = None
        try:
            for _ in range(8):
                if not self.receiver.poll():
                    break
                message = self.receiver.recv_bytes()
                if message:
                    jpeg = message
                elif self.connected == 0:
                    self.connected = time.monotonic()
        except (EOFError, OSError):
            pass
        if jpeg:
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                if self.received == 0:
                    print(f"iPHUMI USB {self.stream}: {frame.shape[1]}x{frame.shape[0]}", flush=True)
                self.frame = frame
                self.received = time.monotonic()
        now = time.monotonic()
        if ((not self.process.is_alive() and now - self.started > 5)
                or (self.connected and now - max(self.received, self.connected) > 5)):
            self.close()
            self.frame = None
            self.received = 0.0
            self.connected = 0.0
            self.start()
        return self.frame, self.received

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join()
        self.receiver.close()


def draw_feed(canvas, bounds, label, feed):
    x, y, width, height = bounds
    cv2.putText(canvas, label, (x + 16, y + 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (230, 230, 230), 2, cv2.LINE_AA)
    frame, received = feed.latest()
    status = feed.unavailable_status
    if frame is not None:
        if time.monotonic() - received > 1:
            status = "STALE FRAME"
        elif (isinstance(feed, CameraFeed) and int(frame.max()) <= 32
              and int(frame.max()) - int(frame.min()) <= 2):
            status = "NO VIDEO SIGNAL"
        else:
            frame_height, frame_width = frame.shape[:2]
            image_height = height - HEADER_HEIGHT
            scale = min(width / frame_width, image_height / frame_height)
            size = (max(1, int(frame_width * scale)),
                    max(1, int(frame_height * scale)))
            image = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            left = x + (width - size[0]) // 2
            top = y + HEADER_HEIGHT + (image_height - size[1]) // 2
            canvas[top:top + size[1], left:left + size[0]] = image
            return
    text_width, text_height = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX,
                                               0.8, 2)[0]
    cv2.putText(canvas, status,
                (x + (width - text_width) // 2,
                 y + HEADER_HEIGHT + (height - HEADER_HEIGHT + text_height) // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (80, 160, 255), 2, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("right_camera")
    parser.add_argument("left_camera")
    parser.add_argument("--primary-camera", choices=tuple(IPHUMI_PORTS), default="main")
    parser.add_argument("--record-video", nargs="?", const="", metavar="MP4")
    args = parser.parse_args()
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("A desktop display is required for the camera preview")

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda signum, frame: stop.set())
    writer = None
    recording_path = None
    if args.record_video is not None:
        recording_path = (Path(args.record_video).expanduser() if args.record_video else
                          Path.home() / "Videos" / "arx5" /
                          f"teleop-{datetime.now():%Y%m%d-%H%M%S-%f}.mp4")
        if recording_path.suffix.lower() != ".mp4":
            parser.error("Recording path must end in .mp4")
        if recording_path.exists():
            parser.error(f"Recording already exists: {recording_path}")
        recording_path.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(recording_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                 30, (WINDOW_WIDTH, WINDOW_HEIGHT))
        if not writer.isOpened():
            writer.release()
            recording_path.unlink(missing_ok=True)
            parser.error(f"Could not start MP4 recording: {recording_path}")
        print(f"Recording three camera views to {recording_path}", flush=True)

    feeds = []
    try:
        feeds.append(IPhoneUSBFeed(args.primary_camera))
        feeds.append(CameraFeed(args.left_camera))
        feeds.append(CameraFeed(args.right_camera))
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, WINDOW_WIDTH, WINDOW_HEIGHT)
        while not stop.is_set():
            tick = time.monotonic()
            canvas = np.full((WINDOW_HEIGHT, WINDOW_WIDTH, 3),
                             (24, 27, 30), dtype=np.uint8)
            views = (
                ((0, 0, WINDOW_WIDTH, MAIN_HEIGHT),
                 f"IPHUMI {args.primary_camera.upper()}", feeds[0]),
                ((0, MAIN_HEIGHT, WINDOW_WIDTH // 2, WINDOW_HEIGHT - MAIN_HEIGHT),
                 "LEFT WRIST", feeds[1]),
                ((WINDOW_WIDTH // 2, MAIN_HEIGHT, WINDOW_WIDTH // 2,
                  WINDOW_HEIGHT - MAIN_HEIGHT), "RIGHT WRIST", feeds[2]),
            )
            for bounds, label, feed in views:
                draw_feed(canvas, bounds, label, feed)
            cv2.line(canvas, (0, MAIN_HEIGHT), (WINDOW_WIDTH, MAIN_HEIGHT),
                     (70, 70, 70), 1)
            cv2.line(canvas, (WINDOW_WIDTH // 2, MAIN_HEIGHT),
                     (WINDOW_WIDTH // 2, WINDOW_HEIGHT), (70, 70, 70), 1)
            cv2.imshow(WINDOW, canvas)
            if writer is not None:
                writer.write(canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
            time.sleep(max(0, 1 / 30 - (time.monotonic() - tick)))
    finally:
        if writer is not None:
            writer.release()
            print(f"Saved camera recording: {recording_path}", flush=True)
        for feed in feeds:
            feed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
