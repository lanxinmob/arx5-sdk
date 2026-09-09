"""Bounded asynchronous HDF5 recording for one leader/follower pair."""

import json
import math
import os
from pathlib import Path
import queue
import threading
import time

import h5py
import numpy as np


DEFAULT_MAX_CAMERA_AGE_SECONDS = 0.03


def get_image_transform(input_res=(1920, 1080), output_res=(256, 256), bgr_to_rgb=False):
    """Match UMI's resize-then-center-crop image transform."""
    import cv2

    iw, ih = input_res
    ow, oh = output_res
    if min(iw, ih, ow, oh) <= 0:
        raise ValueError("Image resolutions must be positive")

    interpolation = cv2.INTER_AREA
    if iw / ih >= ow / oh:
        # Input is wider: fit output height, then remove equal side margins.
        rh = oh
        rw = math.ceil(rh / ih * iw)
        if oh > ih:
            interpolation = cv2.INTER_LINEAR
    else:
        # Input is taller: fit output width, then remove equal top/bottom margins.
        rw = ow
        rh = math.ceil(rw / iw * ih)
        if ow > iw:
            interpolation = cv2.INTER_LINEAR

    x1 = (rw - ow) // 2
    y1 = (rh - oh) // 2
    channels = slice(None, None, -1) if bgr_to_rgb else slice(None)

    def transform(image):
        if image.shape != (ih, iw, 3):
            raise ValueError(f"Expected image shape {(ih, iw, 3)}, got {image.shape}")
        resized = cv2.resize(image, (rw, rh), interpolation=interpolation)
        return np.ascontiguousarray(resized[y1:y1 + oh, x1:x1 + ow, channels])

    return transform


class InstructionHistory:
    def __init__(self, directory):
        self.path = Path(directory) / "instructions.json"
        self.items = json.loads(self.path.read_text(encoding="utf-8")) if self.path.exists() else []
        if not isinstance(self.items, list) or any(not isinstance(x, str) for x in self.items):
            raise ValueError(f"Invalid instruction history: {self.path}")

    def add(self, instruction):
        instruction = instruction.strip()
        if not instruction:
            raise ValueError("Language instruction must not be empty")
        if instruction not in self.items:
            self.items.append(instruction)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(self.items, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            os.replace(temporary, self.path)
        return instruction

    def select(self, ask=input, output=print):
        output("\n历史语言指令：")
        for index, instruction in enumerate(self.items, 1):
            output(f"  {index}: {instruction}")
        while True:
            answer = ask("q 退出；编号复用；e 编号编辑；直接输入新指令（n 文本强制新建）：").strip()
            if answer.lower() == "q":
                return None
            if answer.startswith("n "):
                answer = answer[2:].strip()
            elif answer.isdigit() or answer.startswith("e "):
                editing = answer.startswith("e ")
                number = answer[2:].strip() if editing else answer
                if not number.isdigit() or not 1 <= int(number) <= len(self.items):
                    output("编号无效。")
                    continue
                answer = self.items[int(number) - 1]
                if editing:
                    # readline inserts the old text into the editable input buffer.
                    try:
                        import readline
                        readline.set_startup_hook(lambda: readline.insert_text(answer))
                        replacement = ask(f"修改指令（当前：{answer}；空输入保留）：")
                    finally:
                        readline.set_startup_hook(None)
                    answer = replacement.strip() or answer
            if answer:
                return self.add(answer)
            output("指令不能为空。")


def camera_frame_problem(rgb):
    """Recognize empty images and capture-card black/limited-range dark-gray output."""
    if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3
            or rgb.shape[2] != 3 or not rgb.size):
        return "invalid/empty RGB frame"
    darkest, brightest = int(rgb.min()), int(rgb.max())
    if brightest <= 2 or (brightest <= 32 and brightest - darkest <= 2):
        return f"near-black/no-signal frame (pixel range {darkest}..{brightest}); check camera input/channel"
    return None


def check_camera_streams(cameras, max_age=DEFAULT_MAX_CAMERA_AGE_SECONDS, timeout=2.0,
                         required_frames=3, check=lambda: None):
    """Require new, valid frames from every camera; a cached image cannot pass."""
    started = time.monotonic()
    previous = {}
    counts = {name: 0 for name in cameras}
    frame_errors = {}
    while True:
        check()
        for name, camera in cameras.items():
            rgb, received, sequence = camera.latest()
            problem = camera_frame_problem(rgb)
            if problem:
                # Capture cards may emit blank frames briefly while acquiring HDMI sync.
                # They must then deliver consecutive valid frames within the same timeout.
                frame_errors[name] = f"{getattr(camera, 'device', name)}: {problem}"
                counts[name] = 0
                previous.pop(name, None)
                continue
            frame_errors.pop(name, None)
            if not np.isfinite(received) or not 0 <= time.monotonic() - received <= max_age:
                raise RuntimeError(f"Camera {name}: stale/invalid frame timestamp")
            if name in previous:
                last_sequence, last_received = previous[name]
                if sequence < last_sequence or received < last_received:
                    raise RuntimeError(f"Camera {name}: frame sequence/timestamp moved backwards")
                if sequence > last_sequence and received > last_received:
                    counts[name] += 1
            previous[name] = (sequence, received)
        if all(count >= required_frames for count in counts.values()):
            return counts
        if time.monotonic() - started >= timeout:
            missing = [name for name, count in counts.items() if count < required_frames]
            detail = "; ".join(frame_errors[name] for name in missing if name in frame_errors)
            raise RuntimeError(f"Cameras not delivering new frames: {', '.join(missing)}. {detail}")
        time.sleep(0.01)


class Camera:
    """Continuously drain V4L2; timestamps are host receive times, not exposure times."""

    def __init__(self, device, width=1920, height=1080, fps=30, fourcc="MJPG",
                 output_res=(256, 256)):
        import cv2
        self.cv2 = cv2
        self.device = device
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.frame = None
        self.error = None
        self.output_res = tuple(output_res)
        self.image_transform = None
        self.transform_input_res = None
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f"Cannot open camera {device}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
        self.metadata = {
            "device": device, "requested_fps": fps,
            "reported_fps": self.cap.get(cv2.CAP_PROP_FPS),
            "requested_width": width, "requested_height": height,
            "reported_width": int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "reported_height": int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "output_width": self.output_res[0], "output_height": self.output_res[1],
            "image_transform": "UMI resize_preserve_aspect_then_center_crop",
            "fourcc": fourcc, "timestamp_source": "host_monotonic_after_read",
        }
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"camera-{device}")
        self.thread.start()
        if not self.ready.wait(5) or self.error:
            self.close()
            reason = str(self.error) if self.error else "no frame received within 5 seconds; check camera signal/device/format"
            raise RuntimeError(f"Camera {device} failed to deliver a frame: {reason}")
        self.metadata["shape"] = list(self.latest()[0].shape)

    def _run(self):
        sequence = 0
        try:
            while not self.stop_event.is_set():
                ok, bgr = self.cap.read()
                received = time.monotonic()
                if not ok:
                    raise RuntimeError(f"Camera disconnected/read failed: {self.device}")
                h, w = bgr.shape[:2]
                input_res = (w, h)
                if input_res != self.transform_input_res:
                    self.image_transform = get_image_transform(
                        input_res=input_res,
                        output_res=self.output_res,
                        bgr_to_rgb=True,
                    )
                    self.transform_input_res = input_res
                rgb = self.image_transform(bgr)
                with self.lock:
                    self.frame = (rgb, received, sequence)
                sequence += 1
                self.ready.set()
        except Exception as exc:
            self.error = exc
            self.ready.set()
        finally:
            self.cap.release()

    def latest(self):
        if self.error:
            raise RuntimeError(str(self.error)) from self.error
        with self.lock:
            if self.frame is None:
                raise RuntimeError(f"No frame from {self.device}")
            # Frames are replaced, never mutated; safe to share with writer.
            return self.frame

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        # The capture thread owns release(); concurrent release/read can crash OpenCV.


class SyntheticCamera:
    def __init__(self, width=640, height=480, fps=30, output_res=(256, 256)):
        self.fps = fps
        self.origin = time.monotonic()
        output_width, output_height = output_res
        self.rgb = np.zeros((output_height, output_width, 3), dtype=np.uint8)
        self.rgb[:, :, 0] = 180  # Distinct RGB channels make channel-order tests useful.
        self.metadata = {"device": "synthetic", "requested_fps": fps,
                         "requested_width": width, "requested_height": height,
                         "output_width": output_width, "output_height": output_height,
                         "image_transform": "UMI resize_preserve_aspect_then_center_crop",
                         "shape": list(self.rgb.shape), "timestamp_source": "synthetic_monotonic"}

    def latest(self):
        sequence = int((time.monotonic() - self.origin) * self.fps)
        return self.rgb, self.origin + sequence / self.fps, sequence

    def close(self):
        pass


def state_vector(state, field):
    gripper_field = {"pos": "gripper_pos", "vel": "gripper_vel", "torque": "gripper_torque"}[field]
    return np.append(np.asarray(getattr(state, field)(), dtype=np.float64), getattr(state, gripper_field))


class EpisodeBundle:
    """One instruction, multiple independently committed demos in one HDF5 file."""

    def __init__(self, directory, instruction):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        index = 0
        while True:
            stem = f"episode_{index:06d}"
            self.path = directory / f"{stem}.hdf5"
            self.partial = directory / f"{stem}.partial.hdf5"
            if any(directory.glob(f"{stem}.*")):
                index += 1
                continue
            try:
                file = h5py.File(self.partial, "x")
                break
            except FileExistsError:
                index += 1
        with file:
            file.attrs.update(schema_version="arx5_multi_demo_v1", language_instruction=instruction,
                              complete=False, num_demos=0, num_samples=0)
            file.create_dataset("language_instruction", data=instruction, dtype=h5py.string_dtype("utf-8"))
            for group in ("data", "_pending", "discarded", "interrupted"):
                file.create_group(group)
        self.instruction = instruction
        self.closed = False

    def close(self):
        if self.closed:
            return self.path if self.path.exists() else self.partial
        with h5py.File(self.partial, "r+") as file:
            complete = len(file["data"]) > 0
            file.attrs["complete"] = complete
        if complete:
            os.link(self.partial, self.path)
            self.partial.unlink()
        self.closed = True
        return self.path if complete else self.partial


class EpisodeWriter:
    """Single disk owner. Incomplete files never receive the final episode name."""

    def __init__(self, directory, instruction, metadata, queue_size=64, bundle=None):
        self.bundle = bundle
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        index = 0
        while bundle is None:
            stem = f"episode_{index:06d}"
            self.path = directory / f"{stem}.hdf5"
            self.partial = directory / f"{stem}.partial.hdf5"
            if any(directory.glob(f"{stem}.*")):
                index += 1
                continue
            try:
                self.file = h5py.File(self.partial, "x")
                break
            except FileExistsError:
                index += 1
        self.root = self.file if bundle is None else None
        if bundle is not None:
            if bundle.closed or instruction != bundle.instruction:
                raise ValueError("Closed bundle or instruction mismatch")
            self.path, self.partial = bundle.path, bundle.partial
            self.file = h5py.File(self.partial, "r+")
            self.file.attrs["sim"] = bool(metadata.get("sim", False))
            index = sum(len(self.file[group]) for group in ("data", "_pending", "discarded", "interrupted"))
            self.demo_name = f"demo_{index}"
            self.root = self.file["_pending"].create_group(self.demo_name)
        self.root.attrs.update({
            "schema_version": "arx5_aloha_v1", "sim": bool(metadata.get("sim", False)),
            "compress": False, "image_storage": "rgb_uint8_lzf", "color_order": "RGB",
            "language_instruction": instruction, "complete": False, "num_samples": 0,
            "action_representation": "absolute_joint_position_rad_and_gripper_width_m",
            "state_order": "joint_1..joint_6,gripper_width",
            "timestamp_clock": "host_monotonic_seconds",
            "sample_clock": metadata.get("sample_clock", "periodic"),
            "reference_camera": metadata.get("reference_camera", ""),
            "metadata_json": json.dumps(metadata, ensure_ascii=False),
        })
        self.root.create_dataset("language_instruction", data=instruction, dtype=h5py.string_dtype("utf-8"))
        self.queue = queue.Queue(maxsize=queue_size)
        self.error = None
        self.count = 0
        self.closed = False
        self.latest_written = None
        self.closing = False
        self._sentinel_sent = False
        self.thread = threading.Thread(target=self._run, daemon=True, name="hdf5-writer")
        self.thread.start()

    def check(self):
        if self.error:
            raise RuntimeError(f"HDF5 writer failed; partial file: {self.partial}: {self.error}") from self.error

    def submit(self, row):
        self.check()
        if self.closed or getattr(self, "closing", False):
            raise RuntimeError("Episode already closed")
        try:
            self.queue.put_nowait(row)
        except queue.Full as exc:
            raise RuntimeError("HDF5 queue full: disk cannot keep up; episode stopped without silent frame loss") from exc

    def _run(self):
        datasets = {}
        try:
            while True:
                row = self.queue.get()
                if row is None:
                    break
                if datasets and row.keys() != datasets.keys():
                    raise ValueError("Sample fields changed within episode")
                for name, value in row.items():
                    value = np.asarray(value)
                    if name not in datasets:
                        parent, _, leaf = name.rpartition("/")
                        group = self.root.require_group(parent) if parent else self.root
                        image = name.startswith("observations/images/")
                        datasets[name] = group.create_dataset(
                            leaf, shape=(0,) + value.shape, maxshape=(None,) + value.shape,
                            dtype=value.dtype, chunks=(1,) + value.shape if image else True,
                            compression="lzf" if image else None,
                        )
                    dataset = datasets[name]
                    if value.shape != dataset.shape[1:]:
                        raise ValueError(f"Shape changed for {name}: {value.shape}")
                    dataset.resize(self.count + 1, axis=0)
                    dataset[self.count] = value
                written_index = self.count
                self.count += 1
                self.root.attrs.modify("num_samples", self.count)
                # Read the preview back through the HDF5 datasets in this writer
                # thread. The UI therefore cannot accidentally display a newer
                # live camera frame or the original in-memory sample buffer.
                readback = {}
                for name, dataset in datasets.items():
                    if (name.startswith("observations/images/")
                            or name.startswith("timestamps/cameras/")):
                        readback[name] = np.asarray(dataset[written_index]).copy()
                self.latest_written = (written_index, readback)
                if self.count % 30 == 0:
                    self.file.flush()
        except Exception as exc:
            self.error = exc

    def finish(self, status="saved", success=None, reason=""):
        if self.closed:
            raise RuntimeError("Episode already closed")
        self.closing = True
        while not self._sentinel_sent and self.thread.is_alive():
            try:
                self.queue.put(None, timeout=0.1)
                self._sentinel_sent = True
                break
            except queue.Full:
                continue
        self.thread.join()
        # A failed row may have partially resized datasets: retain only committed rows.
        def truncate(_, obj):
            if isinstance(obj, h5py.Dataset) and obj.ndim and obj.maxshape[0] is None:
                obj.resize(self.count, axis=0)
        try:
            self.root.visititems(truncate)
            complete = status == "saved" and self.error is None and self.count > 0
            self.root.attrs.update({"complete": complete, "status": status,
                                    "success": -1 if success is None else int(success),
                                    "stop_reason": str(self.error or reason), "num_samples": self.count})
            if self.count > 1:
                timestamps = self.root["timestamps/observation"]
                self.root.attrs["actual_fps"] = (self.count - 1) / (timestamps[-1] - timestamps[0])
            if self.bundle is not None:
                destination = "data" if complete else "discarded" if status == "discarded" else "interrupted"
                self.file.move(self.root.name, f"/{destination}/{self.demo_name}")
                self.file.attrs["num_demos"] = len(self.file["data"])
                self.file.attrs["num_samples"] = sum(int(d.attrs["num_samples"]) for d in self.file["data"].values())
            self.file.flush()
        finally:
            self.file.close()
            self.closed = True
        if self.bundle is not None:
            self.check()
            return self.partial
        if complete:
            # link() fails if a destination exists, so an earlier episode cannot be overwritten.
            os.link(self.partial, self.path)
            self.partial.unlink()
            result = self.path
        elif status == "discarded" and self.error is None:
            result = self.partial.with_name(self.partial.name.replace(".partial.", ".discarded."))
            os.link(self.partial, result)
            self.partial.unlink()
        else:
            result = self.partial
        self.check()
        return result


class EpisodeSampler:
    """Called in the teleop thread; only sample packing and nonblocking queue insertion."""

    def __init__(self, writer, cameras, fps=30, max_camera_age=DEFAULT_MAX_CAMERA_AGE_SECONDS,
                 seconds=60, forward_kinematics=None, sample_clock="periodic",
                 reference_camera=None):
        if sample_clock not in ("periodic", "camera"):
            raise ValueError(f"Unknown sample clock: {sample_clock}")
        if sample_clock == "camera":
            reference_camera = reference_camera or next(iter(cameras), None)
            if reference_camera not in cameras:
                raise ValueError("Camera sample clock requires a valid reference camera")
        self.writer = writer
        self.cameras = cameras
        self.period = 1 / fps
        self.max_camera_age = max_camera_age
        self.seconds = seconds
        self.forward_kinematics = forward_kinematics
        self.sample_clock = sample_clock
        self.reference_camera = reference_camera
        self.last_reference_sequence = None
        self.last_saved_reference_sequence = None
        self.dropped_camera_samples = 0
        self.last_camera_drop_warning = None
        self.origin = None
        self.next_time = None
        self.done = threading.Event()
        self.count = 0

    def __call__(self, leader, follower, command, observation_time, command_time):
        self.writer.check()
        if self.done.is_set():
            return
        if self.origin is None:
            self.origin = observation_time
            self.next_time = observation_time
        if self.seconds and observation_time - self.origin >= self.seconds:
            self.done.set()
            return
        frames = {}
        reference_sequence = None
        if self.sample_clock == "camera":
            frames = {name: camera.latest() for name, camera in self.cameras.items()}
            reference_sequence = frames[self.reference_camera][2]
            if self.last_reference_sequence is not None:
                if reference_sequence < self.last_reference_sequence:
                    raise RuntimeError(f"Camera {self.reference_camera} frame sequence moved backwards")
                if reference_sequence == self.last_reference_sequence:
                    return
            self.last_reference_sequence = reference_sequence
            if self.last_saved_reference_sequence is None:
                self.last_saved_reference_sequence = reference_sequence - 1
        if self.sample_clock == "periodic":
            if observation_time < self.next_time:
                return
            missed = int(max(0, (observation_time - self.next_time) / self.period))
        else:
            missed = max(0, int(reference_sequence - self.last_saved_reference_sequence - 1))
        if not frames:
            frames = {name: camera.latest() for name, camera in self.cameras.items()}
        now = time.monotonic()
        stale = []
        for name, (_, timestamp, _) in frames.items():
            age = now - timestamp
            if not np.isfinite(timestamp) or not 0 <= age <= self.max_camera_age:
                stale.append(f"{name} age={age * 1000:.1f}ms")
        if stale:
            self.dropped_camera_samples += 1
            if self.last_camera_drop_warning is None or now - self.last_camera_drop_warning >= 1:
                print(
                    f"[camera] 丢弃当前样本（累计 {self.dropped_camera_samples} 条）："
                    f"{', '.join(stale)}，上限={self.max_camera_age * 1000:.1f}ms",
                    flush=True,
                )
                self.last_camera_drop_warning = now
            if self.sample_clock == "periodic":
                self.next_time += (missed + 1) * self.period
            return
        row = {
            "observations/qpos": state_vector(follower, "pos"),
            "observations/qvel": state_vector(follower, "vel"),
            "observations/effort": state_vector(follower, "torque"),
            "leader/qpos": state_vector(leader, "pos"),
            "leader/qvel": state_vector(leader, "vel"),
            "action": state_vector(command, "pos"),
            "action_velocity": state_vector(command, "vel"),
            "timestamps/observation": observation_time,
            "timestamps/command": command_time,
            "timestamps/relative": observation_time - self.origin,
            "timestamps/leader_sdk": float(leader.timestamp),
            "timestamps/follower_sdk": float(follower.timestamp),
            "quality/missed_sample_periods": np.int64(missed),
        }
        if self.forward_kinematics:
            row["observations/ee_pose"] = np.asarray(self.forward_kinematics(follower.pos())).copy()
        for name, (rgb, timestamp, sequence) in frames.items():
            row[f"observations/images/{name}"] = rgb
            row[f"timestamps/cameras/{name}"] = timestamp
            row[f"quality/camera_sequence/{name}"] = np.int64(sequence)
        self.writer.submit(row)
        if self.sample_clock == "camera":
            self.last_saved_reference_sequence = reference_sequence
        self.count += 1
        if self.sample_clock == "periodic":
            self.next_time += (missed + 1) * self.period
