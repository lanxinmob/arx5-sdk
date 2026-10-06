"""Record language-conditioned ARX5 leader/follower demonstrations (see recording/README.md)."""

import argparse
import json
from pathlib import Path
import select
import queue
import signal
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from recording.hdf5_recorder import (
    DEFAULT_MAX_CAMERA_AGE_SECONDS, Camera, EpisodeSampler, EpisodeWriter, EpisodeBundle,
    InstructionHistory, SyntheticCamera, check_camera_streams,
)


class SimState:
    def __init__(self, phase):
        self.timestamp = time.monotonic()
        self.gripper_pos = 0.04 + 0.01 * np.sin(phase)
        self.gripper_vel = 0.01 * np.cos(phase)
        self.gripper_torque = 0.0
        self.phase = phase

    def pos(self):
        return np.full(6, 0.1 * np.sin(self.phase))

    def vel(self):
        return np.full(6, 0.1 * np.cos(self.phase))

    def torque(self):
        return np.zeros(6)


class SimTeleop:
    def __init__(self, hz):
        self.controller_dt = 1 / hz
        self.teleop_started = False

    def run(self, stop_event, on_step):
        origin = time.monotonic()
        while not stop_event.is_set():
            now = time.monotonic()
            if self.teleop_started:
                on_step(SimState(now - origin), SimState(now - origin - 0.05),
                        SimState(now - origin), now, time.monotonic())
            stop_event.wait(self.controller_dt)

    def go_home(self):
        self.teleop_started = False

    def enter_idle(self):
        self.teleop_started = False

    def begin_episode(self, check=lambda: None):
        check()

    def slow_home(self):
        self.teleop_started = False
        print("[simulate] 两臂缓慢回 home（不发送硬件指令）。", flush=True)


class Session:
    def __init__(self, teleop, preview=None):
        self.teleop = teleop
        self.preview = preview
        self.lock = threading.Lock()
        self.sampler = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = None

    def on_step(self, *args):
        with self.lock:
            if self.sampler is not None:
                self.sampler(*args)
                if self.sampler.done.is_set():
                    self.teleop.teleop_started = False

    def start_control(self):
        self.stop_event.clear()
        def run():
            try:
                self.teleop.run(self.stop_event, self.on_step)
            except Exception as exc:
                self.error = exc
                self.teleop.teleop_started = False
                self.stop_event.set()
        self.thread = threading.Thread(target=run, daemon=True, name="teleop")
        self.thread.start()

    def stop_control(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("Teleop thread did not stop; cannot safely issue a home command")

    def check(self):
        if self.preview:
            self.preview.check()
        if self.error:
            raise RuntimeError(f"Teleop/recording stopped: {self.error}") from self.error
        if self.sampler:
            self.sampler.writer.check()

    def attach(self, sampler):
        check_camera_streams(sampler.cameras, max_age=sampler.max_camera_age, check=self.check)
        self.stop_control()
        self.check()
        if self.preview:
            self.preview.status = "ALIGNING FOLLOWER"
        self.teleop.begin_episode(check=self.check)
        # Alignment can take time; require fresh frames again before enabling recording.
        check_camera_streams(sampler.cameras, max_age=sampler.max_camera_age, check=self.check)
        with self.lock:
            self.sampler = sampler
            self.teleop.teleop_started = True
        if self.preview:
            self.preview.writer = sampler.writer
            self.preview.status = f"RECORDING {sampler.writer.path.stem}"
        self.start_control()

    def detach(self, to_idle=True, keep_preview=False):
        self.stop_control()
        with self.lock:
            self.teleop.teleop_started = False
            self.sampler = None
        if to_idle and self.error is None:
            self.teleop.enter_idle()
            if self.preview:
                if not keep_preview:
                    self.preview.writer = None
                self.preview.status = "IDLE / GRAVITY COMPENSATION"
            self.start_control()

    def shutdown(self):
        self.detach(to_idle=False)
        if self.preview:
            self.preview.status = "RETURNING HOME"
        self.teleop.slow_home()

    def prompt(self, text):
        # Preview and fault handling continue even while readline edits an instruction.
        result = queue.Queue(maxsize=1)
        def read():
            try:
                result.put(input(text))
            except BaseException as exc:
                result.put(exc)
        threading.Thread(target=read, daemon=True, name="instruction-input").start()
        while True:
            self.check()
            try:
                answer = result.get(timeout=0.05)
            except queue.Empty:
                continue
            if isinstance(answer, BaseException):
                raise answer
            return answer


def command(session, cameras, sampler=None):
    """TTY line commands work over SSH; polling also surfaces background errors."""
    while True:
        session.check()
        for camera in cameras.values():
            _, timestamp, _ = camera.latest()
            if time.monotonic() - timestamp > 2:
                raise RuntimeError("Camera stopped delivering frames")
        if sampler and sampler.done.is_set():
            return "timeout"
        if select.select([sys.stdin], [], [], 0.05)[0]:
            line = sys.stdin.readline()
            if not line:
                raise EOFError("Terminal input closed")
            return line.strip().lower()


def positive(value):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("leader_model", nargs="?", default="X5")
    parser.add_argument("leader_interface", nargs="?", default="can1")
    parser.add_argument("follower_model", nargs="?", default="X5_umi")
    parser.add_argument("follower_interface", nargs="?", default="can0")
    parser.add_argument("--dataset-dir", type=Path, default=ROOT / "data" / "demonstrations")
    parser.add_argument("--wrist-camera", default="/dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0")
    parser.add_argument("--main-camera", default=None, help="Optional additional main-view camera")
    parser.add_argument("--fps", type=positive, default=60, help="Dataset sampling Hz")
    parser.add_argument("--camera-fps", type=positive, default=60)
    parser.add_argument("--sample-clock", choices=("camera", "periodic"), default="camera",help="Drive samples from new wrist-camera frames or a periodic timer")
    parser.add_argument("--control-hz", type=positive, default=500, help="Python teleop loop target Hz; SDK servo rate unchanged")
    parser.add_argument("--diagnostics", action="store_true", help="Print actual gains, output torque and fault state once per second")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--image-size", type=int, default=256,help="Square RGB size saved after UMI resize + center crop")
    parser.add_argument("--fourcc", default="MJPG", help="V4L2 input format, e.g. MJPG or YUYV")
    parser.add_argument("--max-camera-age", type=positive, default=DEFAULT_MAX_CAMERA_AGE_SECONDS,help="Stop recording if camera receive time is older than this (s)")
    parser.add_argument("--episode-seconds", type=float, default=60, help="Automatic stop; 0 means unlimited")
    parser.add_argument("--queue-size", type=int, default=64, help="Maximum queued samples before abort")
    parser.add_argument("--instruction", help="Use this instruction each episode instead of interactive selection")
    parser.add_argument("--episodes", type=int, default=0, help="Number of saved demos then exit; 0 means continuous collection")
    parser.add_argument("--demos-per-instruction", type=int, default=1, help="1: one file per demo; >1: group this many demos under one instruction; 0: unlimited demos per instruction")
    parser.add_argument("--result-on-timeout", choices=("success", "failure", "discard"), help="Explicit label for automatic duration stop; otherwise prompt for result")
    parser.add_argument("--auto-start", action="store_true", help="Start after instruction selection without waiting for r")
    parser.add_argument("--simulate", action="store_true", help="Synthetic robot and images; never opens CAN or cameras")
    parser.add_argument("--check-cameras", action="store_true", help="Check camera frames for 2 seconds then exit; never opens CAN")
    
    preview_group = parser.add_mutually_exclusive_group()
    preview_group.add_argument("--preview", dest="preview", action="store_true", help="Show desktop preview (default on hardware)")
    preview_group.add_argument("--no-preview", dest="preview", action="store_false", help="Explicit headless mode")
    parser.set_defaults(preview=None)
    parser.add_argument("--preview-fps", type=positive, default=30)
    parser.add_argument("--home-seconds", type=positive, default=3, help="Target duration for simultaneous homing, including gain ramp and approach")
    parser.add_argument("--home-speed", type=positive, default=1.5, help="Maximum planned homing joint speed (rad/s); long moves extend duration")
    parser.add_argument("--follower-idle-kd-scale", type=positive, default=0.5, help="Follower idle damping as a fraction of SDK default kd")
    parser.add_argument("--follower-gravity-scale", type=positive, default=None, help="Follower gravity multiplier; default 1.05 for X5_umi, otherwise 1.0")
    parser.add_argument("--leader-gravity-scale", type=positive, default=1.0, help="Leader gravity multiplier (default 1.0)")
    parser.add_argument("--gripper-open-readout", "-g", type=float, default=-3.4)
    parser.add_argument("--leader-kp", type=float, default=0.0)
    parser.add_argument("--leader-kd-scale", type=float, default=0.05)
    parser.add_argument("--leader-gripper-kd", type=float, default=0.05)
    parser.add_argument("--vel-ratio", type=float, default=0.3)
    parser.add_argument("--force-scale", type=float, default=0.5)
    args = parser.parse_args()
    if args.fps > args.control_hz:
        parser.error("--fps must not exceed --control-hz")
    if args.sample_clock == "camera" and not np.isclose(args.fps, args.camera_fps):
        parser.error("camera sample clock requires --fps and --camera-fps to match")
    if (args.width <= 0 or args.height <= 0 or args.image_size <= 0 or args.queue_size <= 0
            or args.episodes < 0 or args.demos_per_instruction < 0):
        parser.error("width/height/image-size/queue-size must be positive; episodes must be nonnegative")
    if not np.isfinite(args.episode_seconds) or args.episode_seconds < 0:
        parser.error("--episode-seconds must be finite and nonnegative")
    if len(args.fourcc) != 4:
        parser.error("--fourcc must contain exactly 4 characters")
    if args.instruction is not None and not args.instruction.strip():
        parser.error("--instruction must not be empty")
    if args.leader_interface == args.follower_interface:
        parser.error("Leader and follower must use different CAN interfaces")
    if args.main_camera and Path(args.main_camera).resolve() == Path(args.wrist_camera).resolve():
        parser.error("Wrist and main cameras must use different devices")
    args.dataset_dir = args.dataset_dir.expanduser().resolve()
    if args.preview is None:
        args.preview = not args.simulate
    if args.follower_gravity_scale is None:
        args.follower_gravity_scale = 1.05 if args.follower_model == "X5_umi" else 1.0
    return args


def main():
    args = parse_args()
    history = InstructionHistory(args.dataset_dir)
    cameras = {}
    session = None
    writer = None
    preview = None
    saved = 0
    instruction = None
    bundle = None
    instruction_count = 0
    saved_by_instruction = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Received {signal.Signals(signum).name}")
    previous_signals = {sig: signal.signal(sig, interrupted)
                        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        # Validate every requested camera before constructing any hardware controllers.
        devices = {"cam_wrist": args.wrist_camera}
        if args.main_camera:
            devices["cam_high"] = args.main_camera
        for name, device in devices.items():
            output_res = (args.image_size, args.image_size)
            cameras[name] = (SyntheticCamera(args.width, args.height, args.camera_fps, output_res) if args.simulate
                             else Camera(device, args.width, args.height, args.camera_fps,
                                         args.fourcc, output_res))
            print(f"{name}: {json.dumps(cameras[name].metadata, ensure_ascii=False)}")
            shape = cameras[name].metadata['shape']
            print(f"{name} 保存图像尺寸：{shape[1]}×{shape[0]} RGB；记录 {args.fps:g} Hz，预览 {args.preview_fps:g} FPS")
        if args.preview:
            from recording.preview import Preview
            preview = Preview(cameras, args.preview_fps)
        print("检查相机：确认各视角连续输出新画面……", flush=True)
        check_camera_streams(cameras, max_age=args.max_camera_age,
                             check=preview.check if preview else lambda: None)
        print("相机画面检查通过。", flush=True)
        if args.check_cameras:
            initial = {name: camera.latest()[2] for name, camera in cameras.items()}
            start = time.monotonic()
            while time.monotonic() - start < 2:
                if preview:
                    preview.check()
                for name, camera in cameras.items():
                    if time.monotonic() - camera.latest()[1] > args.max_camera_age:
                        raise RuntimeError(f"Camera {name} frame stale during check")
                time.sleep(0.02)
            for name, camera in cameras.items():
                measured = (camera.latest()[2] - initial[name]) / (time.monotonic() - start)
                print(f"{name}: 实测接收 {measured:.1f} FPS")
            return
        metadata = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
        metadata.update({"sim": args.simulate, "cameras": {k: v.metadata for k, v in cameras.items()},
                         "reference_camera": "cam_wrist",
                         "joint_units": "rad", "gripper_units": "m", "effort_units": "Nm (including gripper motor)",
                         "ee_pose_order": "x,y,z,roll,pitch,yaw (m,rad); follower base frame",
                         "action_alignment": "pre-command follower observation, then same-cycle submitted target; SDK may interpolate/limit",
                         "camera_alignment": (
                             "new reference-camera frame drives sample packing; robot state is read in the same "
                             "control callback; timestamps are host receive times, not exposure times"
                             if args.sample_clock == "camera" else
                             "periodic sample packing uses latest host-received RGB; not hardware synchronized"
                         )})
        fk = None
        if args.simulate:
            teleop = SimTeleop(args.control_hz)
        else:
            from recording.robot_lifecycle import RecordingTeleop
            import arx5_interface as arx5
            print(f"SDK: {arx5.__file__}", flush=True)
            teleop = RecordingTeleop(
                args.leader_model, args.leader_interface, args.follower_model, args.follower_interface,
                args.gripper_open_readout, args.leader_kp, args.leader_kd_scale,
                args.leader_gripper_kd, args.vel_ratio, args.force_scale,
                follower_gravity_scale=args.follower_gravity_scale,
                leader_gravity_scale=args.leader_gravity_scale,
            )
            session = Session(teleop, preview)
            teleop.diagnostics = args.diagnostics
            teleop.home_seconds = args.home_seconds
            teleop.home_speed = args.home_speed
            teleop.follower_idle_kd_scale = args.follower_idle_kd_scale
            print(f"从臂待机 kd 倍率={args.follower_idle_kd_scale:g}，重力补偿倍率={args.follower_gravity_scale:g}")
            config = teleop.follower.get_robot_config()
            solver = arx5.Arx5Solver(config.urdf_path, config.joint_dof, config.joint_pos_min,
                                    config.joint_pos_max, config.base_link_name, config.eef_link_name,
                                    config.gravity_vector)
            fk = solver.forward_kinematics
            metadata["sdk_controller_dt"] = teleop.controller_dt
            metadata["follower_gripper_width"] = config.gripper_width
            metadata["follower_joint_pos_min"] = config.joint_pos_min.tolist()
            metadata["follower_joint_pos_max"] = config.joint_pos_max.tolist()
            metadata["follower_gripper_open_readout"] = config.gripper_open_readout
            metadata["follower_gravity_vector"] = config.gravity_vector.tolist()
            metadata["leader_gripper_open_readout"] = teleop.leader.get_robot_config().gripper_open_readout
            teleop.controller_dt = 1 / args.control_hz
        if session is None:
            session = Session(teleop, preview)
        if not args.simulate:
            print("初始化：两臂缓慢回到 home，然后均进入重力补偿拖动模式。")
            if preview:
                preview.status = "RETURNING HOME"
            teleop.slow_home()
        teleop.enter_idle()
        if preview:
            preview.status = "IDLE / GRAVITY COMPENSATION"
        session.start_control()
        while args.episodes == 0 or saved < args.episodes:
            if instruction is None or (args.demos_per_instruction and instruction_count >= args.demos_per_instruction):
                if bundle is not None:
                    print(f"指令采集文件：{bundle.close()}")
                    bundle = None
                instruction = history.add(args.instruction) if args.instruction else history.select(ask=session.prompt)
                if instruction is None:  # q is available at the same level as language selection.
                    break
                instruction_count = 0
            session.check()
            instruction_saved = saved_by_instruction.get(instruction, 0)
            print(
                f"\n当前指令：{instruction}"
                f"（本指令已保存 {instruction_saved} 条 demo；本次采集共保存 {saved} 条 demo）"
            )
            if not args.auto_start:
                print("r 回车开始；n 更换指令；h 两臂回零；q 退出。")
                while True:
                    choice = command(session, cameras)
                    if choice in ("r", "q", "n"):
                        break
                    if choice == "h":
                        session.stop_control()
                        if preview:
                            preview.status = "RETURNING HOME"
                        teleop.slow_home()
                        teleop.enter_idle()
                        if preview:
                            preview.status = "IDLE / GRAVITY COMPENSATION"
                        session.start_control()
                        print("已回零，输入 r 回车开始。")
                if choice == "q":
                    break
                if choice == "n":
                    instruction = None
                    continue
            print("本轮开始前检查各相机新画面……", flush=True)
            check_camera_streams(cameras, max_age=args.max_camera_age, check=session.check)
            metadata["episode_start_unix"] = time.time()
            metadata["episode_start_monotonic"] = time.monotonic()
            if args.demos_per_instruction != 1 and bundle is None:
                bundle = EpisodeBundle(args.dataset_dir, instruction)
            writer = EpisodeWriter(args.dataset_dir, instruction, metadata, args.queue_size, bundle=bundle)
            sampler = EpisodeSampler(
                writer, cameras, args.fps, args.max_camera_age, args.episode_seconds, fk,
                sample_clock=args.sample_clock, reference_camera="cam_wrist",
            )
            session.attach(sampler)
            demo_label = f"/{writer.demo_name}" if bundle else ""
            print(f"正在采集 {writer.path.name}{demo_label}：ok/f 回车成功/失败保存；d 丢弃；q 中断本轮并退出。")
            last_report = 0
            while True:
                # Unattended simulation also works with stdin redirected from /dev/null.
                if args.auto_start and args.instruction and not sys.stdin.isatty():
                    session.check()
                    if sampler.done.wait(0.05):
                        choice = "timeout"
                    else:
                        continue
                else:
                    choice = command(session, cameras, sampler)
                if choice == "timeout":
                    session.detach(keep_preview=True)
                    writer.check()
                    print("已到时停止，两臂处于拖动模式。请选择：ok 成功保存；f 失败保存；d 丢弃；q 中断并退出。")
                    if args.result_on_timeout:
                        choice = {"success": "ok", "failure": "f", "discard": "d"}[args.result_on_timeout]
                    else:
                        if not sys.stdin.isatty() and args.auto_start:
                            raise RuntimeError("Unattended duration stop requires --result-on-timeout success/failure/discard")
                        while choice not in ("ok", "f", "d", "q"):
                            choice = command(session, cameras)
                if choice in ("ok", "f", "d", "q"):
                    break
                if sampler.count != last_report:
                    print(f"已采集 {sampler.count} 帧；输入 ok/f/d/q 后回车。")
                    last_report = sampler.count
            session.detach()
            session.check()
            result = writer.finish("discarded" if choice == "d" else "interrupted" if choice == "q" else "saved",
                                   choice == "ok" if choice in ("ok", "f") else None,
                                   "duration_limit" if sampler.done.is_set() else "operator")
            label = "已丢弃（保留文件可恢复）" if choice == "d" else "已保留中断数据" if choice == "q" else "已保存" if writer.count else "空轮次，仅保留 partial 文件"
            print(f"{label}：{result}{demo_label}，{writer.count} 帧")
            if choice in ("ok", "f") and writer.count:
                saved += 1
                instruction_count += 1
                saved_by_instruction[instruction] = saved_by_instruction.get(instruction, 0) + 1
            writer = None
            if choice == "q":
                break
    except (KeyboardInterrupt, EOFError):
        print("\n采集被中断；正在缓慢回 home，本轮保留为 partial 文件。", flush=True)
    finally:
        # Repeated Ctrl+C / service-stop signals must not cut short a recovery motion.
        for sig in previous_signals:
            signal.signal(sig, signal.SIG_IGN)
        reason = str(sys.exc_info()[1] or "interrupted")
        try:
            if session:
                # All exit paths (including camera/disk/control errors) use the same recovery.
                session.shutdown()
        finally:
            try:
                if writer and not writer.closed:
                    print(f"保留中断数据：{writer.finish('interrupted', reason=reason)}")
            finally:
                try:
                    if bundle is not None:
                        print(f"指令采集文件：{bundle.close()}")
                finally:
                    try:
                        if preview:
                            preview.close()
                        for camera in cameras.values():
                            camera.close()
                    finally:
                        for sig, handler in previous_signals.items():
                            signal.signal(sig, handler)


if __name__ == "__main__":
    main()
