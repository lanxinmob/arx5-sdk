"""Desktop preview isolated from robot control and blocking terminal input."""

import multiprocessing as mp
import os
import queue
import signal
import threading
import time

import numpy as np

from recording.hdf5_recorder import camera_frame_problem


def compose_preview(frames, status, cv2):
    tiles = []
    for name, rgb, age in frames:
        image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        scale = min(640 / image.shape[1], 480 / image.shape[0])
        image = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
        tile = np.zeros((520, 640, 3), np.uint8)
        y, x = 40 + (480 - image.shape[0]) // 2, (640 - image.shape[1]) // 2
        tile[y:y + image.shape[0], x:x + image.shape[1]] = image
        cv2.putText(tile, f"{name} {rgb.shape[1]}x{rgb.shape[0]} age={age:.3f}s", (12, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (220, 220, 220), 1, cv2.LINE_AA)
        if camera_frame_problem(rgb):
            cv2.rectangle(tile, (16, 208), (624, 304), (20, 20, 20), -1)
            cv2.putText(tile, "NO CAMERA SIGNAL", (56, 246), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (40, 40, 255), 2, cv2.LINE_AA)
            cv2.putText(tile, "Check the input channel / --wrist-camera", (42, 283),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1, cv2.LINE_AA)
        tiles.append(tile)
    canvas = np.concatenate(tiles, axis=1)
    banner = np.zeros((48, canvas.shape[1], 3), np.uint8)
    cv2.putText(banner, status + " | Q/ESC: exit & home", (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 100), 1, cv2.LINE_AA)
    return np.concatenate([banner, canvas], axis=0)


def _window_loop(inbox, ready, quit_requested, stop):
    # Parent owns signal handling and homing, including Ctrl+C in its terminal.
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)
    import cv2
    title = "ARX5 Recorder"
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        waiting = np.zeros((568, 640, 3), np.uint8)
        cv2.putText(waiting, "Waiting for camera frames...", (40, 270), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.imshow(title, waiting)
        while not stop.is_set():
            rendered = False
            try:
                frames, status = inbox.get(timeout=0.02)
                # Only render the newest queued update. This also removes live
                # preview frames queued just before recording mode was entered.
                while True:
                    try:
                        frames, status = inbox.get_nowait()
                    except queue.Empty:
                        break
                canvas = compose_preview(frames, status, cv2)
                if not ready.is_set():
                    cv2.resizeWindow(title, canvas.shape[1], canvas.shape[0])
                cv2.imshow(title, canvas)
                rendered = True
            except queue.Empty:
                pass
            key = cv2.waitKey(1) & 0xFF
            if rendered:
                ready.set()  # A delivered frame was rendered, not merely an empty window created.
            if key in (ord("q"), ord("Q"), 27) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                quit_requested.set()
                return
    finally:
        cv2.destroyAllWindows()


class Preview:
    def __init__(self, cameras, fps=30):
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("Desktop display unavailable; run on the desktop or explicitly use --no-preview")
        self.cameras = cameras
        self.status = "IDLE / GRAVITY COMPENSATION"
        self.writer = None
        self.error = None
        self.fps = fps
        context = mp.get_context("spawn")  # Never fork CAN/OpenCV worker threads.
        self.inbox = context.Queue(maxsize=2)
        self.ready = context.Event()
        self.quit_requested = context.Event()
        self.stop = context.Event()
        self.process = context.Process(target=_window_loop,
                                       args=(self.inbox, self.ready, self.quit_requested, self.stop), daemon=True)
        self.process.start()
        self.thread = threading.Thread(target=self._feed, daemon=True, name="preview-feed")
        self.thread.start()
        try:
            deadline = time.monotonic() + 8
            while not self.ready.wait(0.05):
                self.check()
                if time.monotonic() > deadline:
                    raise RuntimeError("Preview window startup timed out")
        except BaseException:
            self.close()
            raise

    def _feed(self):
        try:
            while not self.stop.is_set():
                frames = []
                writer = self.writer
                status = self.status
                if writer is not None:
                    committed = writer.latest_written
                    if committed is None:
                        self.stop.wait(1 / self.fps)
                        continue
                    index, row = committed
                    for name in self.cameras:
                        frames.append((name, row[f"observations/images/{name}"],
                                       time.monotonic() - row[f"timestamps/cameras/{name}"]))
                    status = f"HDF5 READBACK FRAME {index} | {status}"
                else:
                    for name, camera in self.cameras.items():
                        rgb, received, _ = camera.latest()
                        frames.append((name, rgb, time.monotonic() - received))
                try:
                    self.inbox.put_nowait((frames, status))
                except queue.Full:
                    pass  # Only the preview can skip updates; recording has its own queue.
                self.stop.wait(1 / self.fps)
        except Exception as exc:
            self.error = exc

    def check(self):
        if self.quit_requested.is_set():
            raise KeyboardInterrupt("Preview closed")
        if self.error:
            raise RuntimeError(f"Preview camera error: {self.error}") from self.error
        if not self.process.is_alive():
            raise RuntimeError(f"Preview process exited ({self.process.exitcode})")

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.kill()  # Only the GUI child, never the robot process.
            self.process.join(timeout=1)
        self.inbox.cancel_join_thread()
        self.inbox.close()
