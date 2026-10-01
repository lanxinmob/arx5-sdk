"""Show the two wrist cameras without blocking the VR control loop."""

import argparse
import os
import threading
import time

import cv2
import numpy as np


TILE_WIDTH = 640
HEADER_HEIGHT = 40
WINDOW_WIDTH = TILE_WIDTH * 2
WINDOW_HEIGHT = WINDOW_WIDTH * 3 // 4
TILE_HEIGHT = WINDOW_HEIGHT - HEADER_HEIGHT
WINDOW = "ARX5 Wrist Cameras"


class CameraFeed:
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


def draw_feed(canvas, index, label, feed):
    x = index * TILE_WIDTH
    cv2.putText(canvas, label, (x + 16, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (230, 230, 230), 2, cv2.LINE_AA)
    frame, received = feed.latest()
    status = "CAMERA UNAVAILABLE"
    if frame is not None:
        if time.monotonic() - received > 1:
            status = "STALE FRAME"
        elif int(frame.max()) <= 32 and int(frame.max()) - int(frame.min()) <= 2:
            status = "NO VIDEO SIGNAL"
        else:
            height, width = frame.shape[:2]
            scale = min(TILE_WIDTH / width, TILE_HEIGHT / height)
            size = (max(1, int(width * scale)), max(1, int(height * scale)))
            image = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            left = x + (TILE_WIDTH - size[0]) // 2
            top = HEADER_HEIGHT + (TILE_HEIGHT - size[1]) // 2
            canvas[top:top + size[1], left:left + size[0]] = image
            return
    cv2.putText(canvas, status, (x + 170, 220), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (80, 160, 255), 2, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("right_camera")
    parser.add_argument("left_camera")
    args = parser.parse_args()
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("A desktop display is required for the wrist camera preview")

    feeds = [CameraFeed(args.left_camera), CameraFeed(args.right_camera)]
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, WINDOW_WIDTH, WINDOW_HEIGHT)
        while True:
            tick = time.monotonic()
            canvas = np.full((WINDOW_HEIGHT, WINDOW_WIDTH, 3),
                             (24, 27, 30), dtype=np.uint8)
            for index, (label, feed) in enumerate(zip(("LEFT WRIST", "RIGHT WRIST"), feeds)):
                draw_feed(canvas, index, label, feed)
            cv2.imshow(WINDOW, canvas)
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
        for feed in feeds:
            feed.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
