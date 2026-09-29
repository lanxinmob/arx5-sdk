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


class JointServer:
    def __init__(self, sdk, controller, robot, *, timeout=2.0, max_step=0.1,
                 max_gripper_step=0.005, duration=0.1, timeout_action="damping"):
        for value in (timeout, max_step, max_gripper_step, duration):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Timeout, limits and duration must be positive and finite")
        if duration >= timeout:
            raise ValueError("Duration must be shorter than timeout")
        if timeout_action not in ("damping", "hold"):
            raise ValueError("Timeout action must be damping or hold")
        self.sdk, self.controller, self.robot = sdk, controller, robot
        self.timeout, self.max_step = timeout, max_step
        self.max_gripper_step, self.duration = max_gripper_step, duration
        self.timeout_action = timeout_action
        self.armed = False
        self.holding = False
        self.last_action = 0.0
        self.last_target = None

    def state(self):
        joint = self.controller.get_joint_state()
        eef = self.controller.get_eef_state()
        return dict(
            timestamp=joint.timestamp, ee_pose=eef.pose_6d().copy(),
            joint_pos=joint.pos().copy(), joint_vel=joint.vel().copy(),
            joint_torque=joint.torque().copy(), gripper_pos=joint.gripper_pos,
            gripper_vel=joint.gripper_vel, gripper_torque=joint.gripper_torque,
            emergency=self.controller.is_emergency(), armed=self.armed,
            holding=self.holding, timeout_action=self.timeout_action,
        )

    def disarm(self):
        self.armed = False
        self.holding = False
        self.last_target = None
        self.controller.set_to_damping()

    def hold_position(self):
        if self.controller.is_emergency():
            raise ValueError("Cannot hold an emergency controller")
        if self.holding:
            return
        if not self.armed:
            raise ValueError("Cannot hold an unarmed controller")
        state = self.controller.get_joint_state()
        position = np.asarray(state.pos(), dtype=float).copy()
        gripper = float(state.gripper_pos)
        if (position.shape != (self.robot.joint_dof,) or not np.isfinite(position).all()
                or np.any(position < self.robot.joint_pos_min)
                or np.any(position > self.robot.joint_pos_max)
                or not np.isfinite(gripper) or not 0 <= gripper <= self.robot.gripper_width):
            raise ValueError("Cannot hold invalid measured joint or gripper position")
        command = self.sdk.JointState(self.robot.joint_dof)
        command.pos()[:] = position
        command.gripper_pos = gripper
        command.timestamp = self.controller.get_timestamp() + self.duration
        self.controller.set_joint_cmd(command)
        self.armed = False
        self.holding = True
        self.last_target = None

    def check_timeout(self):
        if self.controller.is_emergency():
            if self.armed or self.holding:
                self.disarm()
        elif self.armed and time.monotonic() - self.last_action >= self.timeout:
            if self.timeout_action == "damping":
                self.disarm()
            else:
                try:
                    self.hold_position()
                except Exception as error:
                    self.disarm()
                    raise RuntimeError(f"Timeout position hold failed: {error}") from error

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
        state = self.controller.get_joint_state()
        actual = np.r_[state.pos(), state.gripper_pos]
        target = np.r_[position, gripper]
        if not np.isfinite(actual).all():
            raise ValueError("Nonfinite robot feedback")
        limits = np.r_[np.minimum(self.max_step, self.robot.joint_vel_max * self.duration),
                       min(self.max_gripper_step, self.robot.gripper_vel_max * self.duration)]
        for reference in (actual, self.last_target):
            if reference is not None and np.any(np.abs(target - reference) > limits + 1e-9):
                raise ValueError("Target jump exceeds step or velocity limit")
        command = self.sdk.JointState(self.robot.joint_dof)
        command.pos()[:] = position
        command.gripper_pos = gripper
        command.timestamp = self.controller.get_timestamp() + self.duration
        self.controller.set_joint_cmd(command)
        self.last_target = target.copy()
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
                self.holding = False
                self.last_target = None
                self.controller.reset_to_home()
                if self.controller.is_emergency():
                    raise ValueError("SDK emergency during reset")
                state = self.controller.get_joint_state()
                self.last_target = np.r_[state.pos(), state.gripper_pos]
                self.last_action = time.monotonic()
                self.armed = True
                data = "OK"
            elif cmd == "SET_TO_DAMPING":
                self.disarm()
                data = "OK"
            elif cmd == "HOLD_POSITION":
                try:
                    self.hold_position()
                except Exception:
                    if self.armed:
                        self.disarm()
                    raise
                data = "OK"
            elif cmd == "SET_JOINT_POS":
                data = self.set_position(message.get("data"))
            else:
                raise ValueError(f"Unsupported command: {cmd}")
            return {"cmd": cmd, "data": data}
        except Exception as error:
            if cmd == "RESET_TO_HOME":
                try:
                    self.disarm()
                except Exception as stop_error:
                    return {"cmd": cmd, "data": f"ERROR: {error}; damping failed: {stop_error}"}
            return {"cmd": cmd, "data": f"ERROR: {error}"}


def serve(model, interface, port, bind, timeout, max_step, max_gripper_step, duration, timeout_action):
    # SDK URDF paths are relative to python/.
    os.chdir(ROOT)
    import arx5_interface as sdk

    if not hasattr(sdk.Arx5JointController, "is_emergency"):
        raise RuntimeError("Build the patched local SDK first (see README)")
    robot = sdk.RobotConfigFactory.get_instance().get_config(model)
    robot.gripper_open_readout = -3.37472  # Same calibration as zmq_server.py.
    config = sdk.ControllerConfigFactory.get_instance().get_config("joint_controller", robot.joint_dof)
    # Match the teleop follower: position tracking with gravity feedforward.
    config.gravity_compensation = True
    controller = None
    with zmq.Context() as context:
        with context.socket(zmq.REP) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.bind(f"tcp://{bind}:{port}")
            try:
                controller = sdk.Arx5JointController(robot, config, interface)
                server = JointServer(sdk, controller, robot, timeout=timeout, max_step=max_step,
                                     max_gripper_step=max_gripper_step, duration=duration,
                                     timeout_action=timeout_action)
                print(f"Joint server: {model} {interface} at {bind}:{port}; awaiting RESET_TO_HOME", flush=True)
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
                if controller is not None:
                    controller.set_to_damping()


@click.command()
@click.argument("arms", nargs=-1, required=True, metavar="MODEL INTERFACE [MODEL INTERFACE]...")
@click.option("--port", "ports", multiple=True, type=click.IntRange(1, 65535))
@click.option("--bind", default="127.0.0.1", show_default=True)
@click.option("--timeout", default=2.0, type=float, show_default=True)
@click.option("--timeout-action", type=click.Choice(("damping", "hold")),
              default="damping", show_default=True,
              help="Action after accepted joint commands stop arriving.")
@click.option("--max-step", default=0.1, type=float, show_default=True, help="Maximum joint target delta, radians.")
@click.option("--max-gripper-step", default=0.005, type=float, show_default=True, help="Maximum gripper target delta, meters.")
@click.option("--duration", default=0.1, type=float, show_default=True, help="SDK interpolation horizon, seconds.")
def main(arms, ports, bind, timeout, timeout_action, max_step, max_gripper_step, duration):
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
    if any(not np.isfinite(v) or v <= 0 for v in (timeout, max_step, max_gripper_step, duration)) or duration >= timeout:
        raise click.UsageError("Limits must be positive and finite; duration must be less than timeout")

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    children = []
    try:
        if len(pairs) == 1:
            serve(*pairs[0], ports[0], bind, timeout, max_step, max_gripper_step, duration,
                  timeout_action)
        else:
            for (model, interface), port in zip(pairs, ports):
                children.append(subprocess.Popen([
                    sys.executable, "-u", str(Path(__file__).resolve()), model, interface,
                    "--port", str(port), "--bind", bind, "--timeout", str(timeout),
                    "--max-step", str(max_step), "--max-gripper-step", str(max_gripper_step),
                    "--duration", str(duration), "--timeout-action", timeout_action,
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
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == "__main__":
    main()
