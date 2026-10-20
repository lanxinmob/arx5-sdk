"""ZMQ absolute joint-position control using the local ARX5 SDK."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import click
import numpy as np
import zmq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
HOME_SPEED = 0.4  # Maximum planned joint speed in rad/s during return to home.
HOME_TOLERANCE = 0.05
# Give the arm time to settle after the operator releases it before full
# position gains are enabled.  A short ramp is perceptible as a kick at HOLD.
HOLD_GAIN_RAMP_S = 0.3
HOLD_GAIN_RAMP_STEPS = 5


class JointServer:
    def __init__(self, sdk, controller, robot, *, timeout=2.0, duration=0.05):
        for value in (timeout, duration):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Timeout and duration must be positive and finite")
        if duration >= timeout:
            raise ValueError("Duration must be shorter than timeout")
        self.sdk, self.controller, self.robot = sdk, controller, robot
        self.timeout, self.duration = timeout, duration
        self.armed = False
        self.mode = "damping"
        self.last_action = 0.0
        self.hold_gain = None
        self.drag_gain = None

    def refresh_gains(self):
        """Capture default gains after SDK reset has initialized them."""
        self.hold_gain = self.controller.get_gain()
        self.drag_gain = self.controller.get_gain()
        self.drag_gain.kp()[:] = 0.0
        self.drag_gain.kd()[:] = self.hold_gain.kd() * 0.05
        self.drag_gain.gripper_kp = 0.0
        self.drag_gain.gripper_kd = 0.05

    def state(self):
        joint = self.controller.get_joint_state()
        eef = self.controller.get_eef_state()
        return dict(
            timestamp=joint.timestamp, ee_pose=eef.pose_6d().copy(),
            joint_pos=joint.pos().copy(), joint_vel=joint.vel().copy(),
            joint_torque=joint.torque().copy(), gripper_pos=joint.gripper_pos,
            gripper_vel=joint.gripper_vel, gripper_torque=joint.gripper_torque,
            emergency=self.controller.is_emergency(), armed=self.armed,
            mode=self.mode,
        )

    def disarm(self):
        self.armed = False
        self.mode = "damping"
        self.controller.set_to_damping()

    def drag(self):
        """Enable low-gain physical dragging without accepting targets."""
        if self.controller.is_emergency():
            raise ValueError("SDK emergency is latched; resolve fault and restart controller")
        if self.drag_gain is None:
            self.refresh_gains()
        # A policy trajectory may still be installed in the SDK interpolator
        # when the client asks for human control.  Damping first replaces that
        # trajectory with the measured state, so switching to zero-kp drag
        # cannot expose an old policy/home target on the next controller tick.
        self.controller.set_to_damping()
        self.controller.set_gain(self.drag_gain)
        self.armed = False
        self.mode = "drag"
        return self.state()

    def hold(self):
        """Lock the current measured position and accept joint targets."""
        if self.controller.is_emergency():
            raise ValueError("SDK emergency is latched; resolve fault and restart controller")
        if self.hold_gain is None:
            self.refresh_gains()
        # Drag mode leaves the previous position command in the SDK
        # interpolator.  Calling set_joint_cmd() here can briefly expose that
        # stale command when the high position gain is enabled.  Damping first
        # pins the interpolator to the measured state, so HOLD starts from the
        # pose at which the operator released the arm.
        self.controller.set_to_damping()
        damping_gain = self.controller.get_gain()
        for alpha in np.linspace(0.0, 1.0, HOLD_GAIN_RAMP_STEPS + 1)[1:]:
            self.controller.set_gain(
                damping_gain * (1.0 - float(alpha))
                + self.hold_gain * float(alpha)
            )
            time.sleep(HOLD_GAIN_RAMP_S / HOLD_GAIN_RAMP_STEPS)
        self.armed = True
        self.mode = "hold"
        self.last_action = time.monotonic()
        return self.state()

    def check_timeout(self):
        if not self.armed:
            return
        if self.controller.is_emergency():
            self.disarm()
        elif time.monotonic() - self.last_action >= self.timeout:
            self.safe_return_home("command timeout")

    def return_home(self):
        if not self.armed or self.controller.is_emergency():
            raise ValueError("Joint server is disarmed or in emergency")
        joint = self.controller.get_joint_state()
        start = np.asarray(joint.pos(), dtype=float).copy()
        if start.shape != (self.robot.joint_dof,) or not np.isfinite(start).all():
            raise ValueError("Invalid measured joints during return to home")
        outside = np.maximum(self.robot.joint_pos_min - start,
                             start - self.robot.joint_pos_max)
        if np.max(outside) > 0.1:
            raise ValueError("Measured joint too far outside SDK limits for return to home")
        gripper = float(joint.gripper_pos)
        if not np.isfinite(gripper):
            raise ValueError("Invalid measured gripper during return to home")
        gripper = float(np.clip(gripper, 0, self.robot.gripper_width))

        home = np.zeros(self.robot.joint_dof)
        if np.max(np.abs(start)) <= HOME_TOLERANCE:
            return
        approach = home.copy()
        approach[2] = 0.03  # Match the SDK homing path's final joint-3 clearance.
        for target in (approach, home):
            measured = np.asarray(self.controller.get_joint_state().pos(), dtype=float).copy()
            duration = max(0.5, 1.5 * np.max(np.abs(target - measured)) / HOME_SPEED)
            command = self.sdk.JointState(self.robot.joint_dof)
            command.pos()[:] = target
            command.gripper_pos = gripper
            command.timestamp = self.controller.get_timestamp() + duration
            self.controller.set_joint_cmd(command)
            self.last_action = time.monotonic()
            deadline = self.last_action + duration + 2.0
            while time.monotonic() < deadline:
                if self.controller.is_emergency():
                    raise ValueError("SDK emergency during return to home")
                actual = np.asarray(self.controller.get_joint_state().pos(), dtype=float)
                if actual.shape != target.shape or not np.isfinite(actual).all():
                    raise ValueError("Invalid feedback during return to home")
                if (time.monotonic() >= self.last_action + duration
                        and np.max(np.abs(actual - target)) <= HOME_TOLERANCE):
                    break
                time.sleep(0.05)
            else:
                raise TimeoutError("Arm did not reach home")
        print("Joint server: slow return to home complete", flush=True)

    def safe_return_home(self, reason):
        try:
            if self.armed and not self.controller.is_emergency():
                print(f"Joint server: slow return to home ({reason})", flush=True)
                self.return_home()
        except Exception as error:
            print(f"Joint server: return to home failed ({reason}): {error}", flush=True)
        finally:
            try:
                self.disarm()
            except Exception as error:
                print(f"Joint server: damping failed ({reason}): {error}", flush=True)

    def set_position(self, data):
        if not self.armed:
            raise ValueError("RESET_TO_HOME is required before joint commands")
        if self.controller.is_emergency():
            raise ValueError("SDK emergency is latched; resolve fault and restart controller")
        if not isinstance(data, dict) or set(data) != {"joint_pos", "gripper_pos"}:
            raise ValueError("Expected joint_pos and gripper_pos only")
        position = np.asarray(data["joint_pos"], dtype=float)
        gripper = np.asarray(data["gripper_pos"], dtype=float)
        if position.shape != (self.robot.joint_dof,) or gripper.shape != ():
            raise ValueError("Invalid joint or gripper shape")
        if not np.isfinite(position).all() or not np.isfinite(gripper):
            raise ValueError("Positions must be finite")
        gripper = float(gripper)
        if np.any(position < self.robot.joint_pos_min) or np.any(position > self.robot.joint_pos_max):
            raise ValueError("Joint position outside SDK limits")
        if not 0 <= gripper <= self.robot.gripper_width:
            raise ValueError("Gripper outside SDK limits (meters)")
        command = self.sdk.JointState(self.robot.joint_dof)
        command.pos()[:] = position
        command.gripper_pos = gripper
        command.timestamp = self.controller.get_timestamp() + self.duration
        self.controller.set_joint_cmd(command)
        self.last_action = time.monotonic()
        return self.state()

    def set_trajectory(self, data):
        """Replace the SDK's future trajectory with a timed joint trajectory.

        ``time_from_now`` is expressed in seconds from receipt of this request.
        The SDK keeps the resulting trajectory and performs the interpolation at
        its own controller rate; callers do not need to stream single points.
        """
        if not self.armed:
            raise ValueError("RESET_TO_HOME is required before joint commands")
        if self.controller.is_emergency():
            raise ValueError("SDK emergency is latched; resolve fault and restart controller")
        if not isinstance(data, dict) or set(data) != {
            "joint_pos", "gripper_pos", "time_from_now"
        }:
            raise ValueError(
                "Expected joint_pos, gripper_pos, and time_from_now only"
            )

        positions = np.asarray(data["joint_pos"], dtype=float)
        grippers = np.asarray(data["gripper_pos"], dtype=float)
        offsets = np.asarray(data["time_from_now"], dtype=float)
        if positions.ndim != 2 or positions.shape[1] != self.robot.joint_dof:
            raise ValueError("Invalid joint trajectory shape")
        if grippers.shape != (positions.shape[0],) or offsets.shape != grippers.shape:
            raise ValueError("Trajectory fields must have matching lengths")
        if positions.shape[0] == 0:
            raise ValueError("Joint trajectory must contain at least one waypoint")
        if (
            not np.isfinite(positions).all()
            or not np.isfinite(grippers).all()
            or not np.isfinite(offsets).all()
        ):
            raise ValueError("Trajectory values must be finite")
        if np.any(np.diff(offsets) <= 0):
            raise ValueError("Trajectory times must be strictly ascending")
        if np.any(positions < self.robot.joint_pos_min) or np.any(
            positions > self.robot.joint_pos_max
        ):
            raise ValueError("Joint position outside SDK limits")
        if np.any(grippers < 0) or np.any(grippers > self.robot.gripper_width):
            raise ValueError("Gripper outside SDK limits (meters)")

        # The action clock is a wall-clock timestamp in the policy process.  A
        # waypoint that is already due is discarded here; the SDK still gets a
        # complete future trajectory in one call.
        future = offsets > 0
        if not np.any(future):
            raise ValueError("Joint trajectory has no future waypoints")
        positions = positions[future]
        grippers = grippers[future]
        offsets = offsets[future]
        start_time = self.controller.get_timestamp()
        trajectory = []
        for position, gripper, offset in zip(positions, grippers, offsets):
            command = self.sdk.JointState(self.robot.joint_dof)
            command.pos()[:] = position
            command.gripper_pos = float(gripper)
            command.timestamp = start_time + float(offset)
            trajectory.append(command)
        self.controller.set_joint_traj(trajectory)
        self.last_action = time.monotonic()
        return self.state()

    def handle(self, message):
        cmd = message.get("cmd", "UNKNOWN") if isinstance(message, dict) else "UNKNOWN"
        if not isinstance(cmd, str):
            cmd = "UNKNOWN"
        try:
            self.check_timeout()
            if not isinstance(message, dict):
                raise ValueError("Expected a command dictionary")
            if cmd == "GET_STATE":
                data = self.state()
            elif cmd == "RESET_TO_HOME":
                self.armed = False
                self.controller.reset_to_home()
                time.sleep(0.55)
                if self.controller.is_emergency():
                    raise ValueError("SDK emergency during reset")
                self.refresh_gains()
                self.last_action = time.monotonic()
                self.armed = True
                self.mode = "hold"
                self.controller.set_gain(self.hold_gain)
                data = "OK"
            elif cmd == "SET_TO_DAMPING":
                self.disarm()
                data = "OK"
            elif cmd == "DRAG":
                data = self.drag()
            elif cmd == "HOLD":
                data = self.hold()
            elif cmd == "RETURN_TO_HOME":
                self.return_home()
                data = "OK"
            elif cmd == "SET_JOINT_POS":
                data = self.set_position(message.get("data"))
            elif cmd == "SET_JOINT_TRAJ":
                data = self.set_trajectory(message.get("data"))
            else:
                raise ValueError(f"Unsupported command: {cmd}")
            return {"cmd": cmd, "data": data}
        except Exception as error:
            if cmd in ("RESET_TO_HOME", "RETURN_TO_HOME"):
                try:
                    self.disarm()
                except Exception as stop_error:
                    return {"cmd": cmd, "data": f"ERROR: {error}; damping failed: {stop_error}"}
            return {"cmd": cmd, "data": f"ERROR: {error}"}


def serve(model, interface, port, bind, timeout, duration, gravity_scale):
    # SDK URDF paths are relative to python/.
    os.chdir(ROOT)
    import arx5_interface as sdk

    if not hasattr(sdk.Arx5JointController, "is_emergency"):
        raise RuntimeError("Build the patched local SDK first (see README)")
    if not hasattr(sdk.Arx5JointController, "set_joint_traj"):
        raise RuntimeError("SDK binding lacks set_joint_traj; rebuild the local SDK")
    robot = sdk.RobotConfigFactory.get_instance().get_config(model)
    robot.gripper_open_readout = -3.37472  # Same calibration as zmq_server.py.
    if gravity_scale != 1.0:
        robot.gravity_vector = robot.gravity_vector * gravity_scale
    config = sdk.ControllerConfigFactory.get_instance().get_config("joint_controller", robot.joint_dof)
    # Match the teleop follower: position tracking with gravity feedforward.
    config.gravity_compensation = True
    controller = None
    server = None
    with zmq.Context() as context:
        with context.socket(zmq.REP) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.bind(f"tcp://{bind}:{port}")
            try:
                controller = sdk.Arx5JointController(robot, config, interface)
                server = JointServer(sdk, controller, robot, timeout=timeout, duration=duration)
                print(
                    f"Joint server: {model} {interface} at {bind}:{port}; "
                    f"gravity_scale={gravity_scale:g}; awaiting RESET_TO_HOME",
                    flush=True,
                )
                while True:
                    server.check_timeout()
                    if socket.poll(20):
                        try:
                            message = socket.recv_pyobj()
                        except Exception as error:
                            socket.send_pyobj({"cmd": "UNKNOWN", "data": f"ERROR: {error}"})
                            continue
                        socket.send_pyobj(server.handle(message))
            finally:
                if server is not None:
                    server.safe_return_home("server stopping")
                elif controller is not None:
                    controller.set_to_damping()


@click.command()
@click.argument("arms", nargs=-1, required=True, metavar="MODEL INTERFACE [MODEL INTERFACE]...")
@click.option("--port", "ports", multiple=True, type=click.IntRange(1, 65535))
@click.option("--bind", default="127.0.0.1", show_default=True)
@click.option("--timeout", default=2.0, type=float, show_default=True)
@click.option("--duration", default=0.05, type=float, show_default=True, help="SDK interpolation horizon, seconds.")
@click.option(
    "--gravity-scale",
    default=1.0,
    type=click.FloatRange(min=0.0, min_open=True),
    show_default=True,
    help=(
        "Scale gravity feedforward in DRAG mode. Lower this for an arm that "
        "tilts up/back; try 0.9, 0.85, or 0.8 after a guarded static check."
    ),
)
def main(arms, ports, bind, timeout, duration, gravity_scale):
    """Run one independent joint-position service per arm."""
    if len(arms) % 2:
        raise click.UsageError("Provide MODEL INTERFACE pairs")
    pairs = list(zip(arms[::2], arms[1::2]))
    if not ports and len(pairs) == 1:
        ports = (8765,)
    if len(ports) != len(pairs) or len(set(ports)) != len(ports):
        raise click.UsageError("Provide one unique --port per arm")
    if len(set(arms[1::2])) != len(pairs):
        raise click.UsageError("CAN interfaces must be unique")
    if any(not np.isfinite(v) or v <= 0 for v in (timeout, duration)) or duration >= timeout:
        raise click.UsageError("Timeout and duration must be positive and finite; duration must be less than timeout")

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    children = []
    try:
        if len(pairs) == 1:
            serve(*pairs[0], ports[0], bind, timeout, duration, gravity_scale)
        else:
            for (model, interface), port in zip(pairs, ports):
                children.append(subprocess.Popen([
                    sys.executable, "-u", str(Path(__file__).resolve()), model, interface,
                    "--port", str(port), "--bind", bind, "--timeout", str(timeout),
                    "--duration", str(duration),
                    "--gravity-scale", str(gravity_scale),
                ], start_new_session=True))
            while True:
                for child in children:
                    if child.poll() is not None:
                        raise click.ClickException(f"Arm service exited ({child.returncode}); stopping peers")
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=40)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == "__main__":
    main()
