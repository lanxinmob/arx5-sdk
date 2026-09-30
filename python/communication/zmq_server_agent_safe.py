import os
import sys

import numpy as np


ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, ROOT_DIR)
os.chdir(ROOT_DIR)

import time
import traceback
from typing import Any, Optional, Sequence, cast

import arx5_interface as arx5
import click
import zmq

from communication.agent_action_guard import (
    ActionGuardError,
    AgentActionGuard,
)


def _exception_text() -> str:
    return "".join(traceback.format_exception(*sys.exc_info()))


class Arx5AgentSafeServer:
    def __init__(
        self,
        *,
        zmq_ip: str,
        zmq_port: int,
        model: str,
        interface: str,
        no_cmd_timeout: float = 2.0,
        max_position_delta: float = 0.002,
        max_rpy_delta: float = 0.02,
        gripper_min: float = 0.0,
        gripper_max: float = 0.086,
        max_gripper_delta: float = 0.002,
        max_home_translation_radius: float = 0.45,
        workspace_min: Optional[Sequence[float]] = None,
        workspace_max: Optional[Sequence[float]] = None,
        reset_settle_time: float = 0.7,
    ) -> None:
        if not np.isfinite(no_cmd_timeout) or no_cmd_timeout <= 0.0:
            raise ValueError("no_cmd_timeout must be a positive finite value")
        if not np.isfinite(reset_settle_time) or reset_settle_time < 0.0:
            raise ValueError("reset_settle_time must be a finite nonnegative value")

        self.model = model
        self.interface = interface
        if not hasattr(arx5.Arx5CartesianController, "is_emergency"):
            raise RuntimeError("Build the patched local SDK before starting the guarded server")
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(self.model)
        robot_config.gripper_open_readout = -3.37472
        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "cartesian_controller", robot_config.joint_dof
        )
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.controller = None
        try:
            self.socket.bind(f"tcp://{zmq_ip}:{zmq_port}")
            self.controller = arx5.Arx5CartesianController(
                robot_config, controller_config, interface
            )
            home_pose = self.controller.get_home_pose().copy()
            self.guard = AgentActionGuard(
                home_pose,
                max_position_delta=max_position_delta,
                max_rpy_delta=max_rpy_delta,
                gripper_min=gripper_min,
                gripper_max=gripper_max,
                max_gripper_delta=max_gripper_delta,
                max_home_translation_radius=max_home_translation_radius,
                workspace_min=workspace_min,
                workspace_max=workspace_max,
            )
            self.poller = zmq.Poller()
            self.poller.register(self.socket, zmq.POLLIN)
        except BaseException:
            if self.controller is not None:
                try:
                    self.controller.set_to_damping()
                except Exception:
                    print(_exception_text())
            self.socket.close()
            self.context.term()
            raise

        self.zmq_ip = zmq_ip
        self.zmq_port = zmq_port
        self.no_cmd_timeout = float(no_cmd_timeout)
        self.reset_settle_time = float(reset_settle_time)
        self.last_command_time = time.monotonic()
        self._closed = False
        print(f"Arx5AgentSafeServer initialized with {model} on {interface}")

    def _state_data(self) -> dict[str, Any]:
        eef_state = self.controller.get_eef_state()
        eef_cmd = self.controller.get_eef_cmd()
        joint_state = self.controller.get_joint_state()
        return {
            "emergency": self.controller.is_emergency(),
            "timestamp": eef_state.timestamp,
            "ee_pose": eef_state.pose_6d().copy(),
            "ee_cmd_pose": eef_cmd.pose_6d().copy(),
            "gripper_cmd_pos": eef_cmd.gripper_pos,
            "joint_pos": joint_state.pos().copy(),
            "joint_vel": joint_state.vel().copy(),
            "joint_torque": joint_state.torque().copy(),
            "gripper_pos": joint_state.gripper_pos,
            "gripper_vel": joint_state.gripper_vel,
            "gripper_torque": joint_state.gripper_torque,
        }

    def _send_error(self, cmd: str, message: str) -> None:
        self.socket.send_pyobj({"cmd": cmd, "data": f"ERROR: {message}"})

    def _handle_timeout(self) -> None:
        was_armed = self.guard.armed
        if was_armed:
            self.guard.disarm()
            print(
                f"Timeout: no accepted pose command for {self.no_cmd_timeout:.3f}s; "
                "setting damping and requiring RESET_TO_HOME"
            )
            self.controller.set_to_damping()
        self.last_command_time = time.monotonic()

    def _handle_set_ee_pose(self, data: Any) -> None:
        try:
            action = self.guard.validate_payload(data)
        except ActionGuardError as exc:
            self._send_error("SET_EE_POSE", str(exc))
            return

        try:
            self.controller.set_eef_cmd(
                arx5.EEFState(action.ee_pose.copy(), action.gripper_pos)
            )
        except Exception:
            print(_exception_text())
            self._send_error("SET_EE_POSE", "robot SDK rejected the validated action")
            return

        self.guard.accept(action)
        self.last_command_time = time.monotonic()
        self.socket.send_pyobj({"cmd": "SET_EE_POSE", "data": self._state_data()})

    def _handle_reset_to_home(self) -> None:
        self.guard.disarm()
        try:
            self.controller.reset_to_home()
            if self.reset_settle_time:
                time.sleep(self.reset_settle_time)
            if self.controller.is_emergency():
                raise RuntimeError("SDK emergency during reset")
            reset_cmd = self.controller.get_eef_cmd()
            self.guard.arm(reset_cmd.pose_6d().copy(), reset_cmd.gripper_pos)
            self.last_command_time = time.monotonic()
            self.socket.send_pyobj({"cmd": "RESET_TO_HOME", "data": "OK"})
        except Exception:
            self.guard.disarm()
            try:
                self.controller.set_to_damping()
            except Exception:
                print(_exception_text())
            raise

    def _validate_gain_vector(self, value: Any, name: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        joint_dof = int(self.controller.get_robot_config().joint_dof)
        if array.shape != (joint_dof,):
            raise ValueError(f"{name} must have shape ({joint_dof},)")
        if not np.all(np.isfinite(array)) or np.any(array < 0.0):
            raise ValueError(f"{name} must contain finite nonnegative values")
        return array

    @staticmethod
    def _validate_gain_scalar(value: Any, name: str) -> float:
        if isinstance(value, (bool, np.bool_)):
            raise ValueError(f"{name} must be a finite nonnegative scalar")
        array = np.asarray(value, dtype=np.float64)
        if array.shape != () or not np.isfinite(array.item()) or array.item() < 0.0:
            raise ValueError(f"{name} must be a finite nonnegative scalar")
        return float(array.item())

    def _handle_message(self, message: Any) -> None:
        if self.controller.is_emergency():
            self.guard.disarm()
        if not isinstance(message, dict):
            self._send_error("UNKNOWN", "message must be a dict")
            return
        cmd_value = message.get("cmd")
        if not isinstance(cmd_value, str):
            self._send_error("UNKNOWN", "message cmd must be a string")
            return
        cmd = cmd_value

        if cmd == "GET_SERVER_INFO":
            self.socket.send_pyobj(
                {
                    "cmd": cmd,
                    "data": {
                        "name": "arx5_agent_safe",
                        "protocol_version": 1,
                        "model": self.model,
                        "interface": self.interface,
                        "armed": self.guard.armed,
                        "home_pose": self.guard.home_pose,
                        "limits": {
                            "max_position_delta": self.guard.max_position_delta,
                            "max_rpy_delta": self.guard.max_rpy_delta,
                            "gripper_min": self.guard.gripper_min,
                            "gripper_max": self.guard.gripper_max,
                            "max_gripper_delta": self.guard.max_gripper_delta,
                            "max_home_translation_radius": (
                                self.guard.max_home_translation_radius
                            ),
                            "workspace_min": (
                                None if self.guard.workspace_min is None
                                else self.guard.workspace_min.copy()
                            ),
                            "workspace_max": (
                                None if self.guard.workspace_max is None
                                else self.guard.workspace_max.copy()
                            ),
                        },
                    },
                }
            )
        elif cmd == "GET_STATE":
            self.socket.send_pyobj({"cmd": cmd, "data": self._state_data()})
        elif cmd == "SET_EE_POSE":
            self._handle_set_ee_pose(message.get("data"))
        elif cmd == "RESET_TO_HOME":
            self._handle_reset_to_home()
        elif cmd == "SET_TO_DAMPING":
            self.guard.disarm()
            self.controller.set_to_damping()
            self.socket.send_pyobj({"cmd": cmd, "data": "OK"})
        elif cmd == "GET_GAIN":
            gain = self.controller.get_gain()
            self.socket.send_pyobj(
                {
                    "cmd": cmd,
                    "data": {
                        "kp": gain.kp().copy(),
                        "kd": gain.kd().copy(),
                        "gripper_kp": gain.gripper_kp,
                        "gripper_kd": gain.gripper_kd,
                    },
                }
            )
        elif cmd == "SET_GAIN":
            data = message.get("data")
            if not isinstance(data, dict):
                raise ValueError("SET_GAIN data must be a dict")
            kp = self._validate_gain_vector(data["kp"], "kp")
            kd = self._validate_gain_vector(data["kd"], "kd")
            gripper_kp = self._validate_gain_scalar(
                data["gripper_kp"], "gripper_kp"
            )
            gripper_kd = self._validate_gain_scalar(
                data["gripper_kd"], "gripper_kd"
            )
            self.controller.set_gain(arx5.Gain(kp, kd, gripper_kp, gripper_kd))
            self.socket.send_pyobj({"cmd": cmd, "data": "OK"})
        else:
            raise ValueError(f"unknown message type: {cmd}")

    def run(self) -> None:
        print(f"Arx5AgentSafeServer running on {self.zmq_ip}:{self.zmq_port}")
        try:
            while True:
                elapsed = time.monotonic() - self.last_command_time
                remaining = self.no_cmd_timeout - elapsed
                if remaining <= 0:
                    self._handle_timeout()
                    continue
                events = dict(self.poller.poll(max(1, int(remaining * 1000))))
                if self.socket not in events:
                    continue

                message = self.socket.recv_pyobj()
                try:
                    self._handle_message(message)
                except Exception as exc:
                    cmd = (
                        cast(str, message.get("cmd"))
                        if isinstance(message, dict)
                        and isinstance(message.get("cmd"), str)
                        else "UNKNOWN"
                    )
                    print(_exception_text())
                    self._send_error(cmd, str(exc))
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.guard.disarm()
        try:
            self.controller.set_to_damping()
        except Exception:
            print(_exception_text())
        self.poller.unregister(self.socket)
        self.socket.close()
        self.context.term()
        print("Arx5AgentSafeServer terminated")


@click.command()
@click.argument("model")
@click.argument("interface")
@click.option("--bind", "zmq_ip", default="127.0.0.1", show_default=True)
@click.option("--port", "zmq_port", default=8765, type=click.IntRange(1, 65535))
@click.option("--timeout", "no_cmd_timeout", default=2.0, type=float, show_default=True)
@click.option("--max-position-delta", default=0.002, type=float, show_default=True)
@click.option("--max-rpy-delta", default=0.02, type=float, show_default=True)
@click.option("--gripper-min", default=0.0, type=float, show_default=True)
@click.option("--gripper-max", default=0.086, type=float, show_default=True)
@click.option("--max-gripper-delta", default=0.002, type=float, show_default=True)
@click.option(
    "--max-home-translation-radius", default=0.45, type=float, show_default=True
)
@click.option("--workspace-min", type=(float, float, float), default=None)
@click.option("--workspace-max", type=(float, float, float), default=None)
@click.option("--reset-settle-time", default=0.7, type=float, show_default=True)
def main(
    model: str,
    interface: str,
    zmq_ip: str,
    zmq_port: int,
    no_cmd_timeout: float,
    max_position_delta: float,
    max_rpy_delta: float,
    gripper_min: float,
    gripper_max: float,
    max_gripper_delta: float,
    max_home_translation_radius: float,
    workspace_min: Optional[tuple[float, float, float]],
    workspace_max: Optional[tuple[float, float, float]],
    reset_settle_time: float,
) -> None:
    try:
        server = Arx5AgentSafeServer(
            model=model,
            interface=interface,
            zmq_ip=zmq_ip,
            zmq_port=zmq_port,
            no_cmd_timeout=no_cmd_timeout,
            max_position_delta=max_position_delta,
            max_rpy_delta=max_rpy_delta,
            gripper_min=gripper_min,
            gripper_max=gripper_max,
            max_gripper_delta=max_gripper_delta,
            max_home_translation_radius=max_home_translation_radius,
            workspace_min=workspace_min,
            workspace_max=workspace_max,
            reset_settle_time=reset_settle_time,
        )
    except (ActionGuardError, ValueError) as exc:
        raise click.ClickException(str(exc)) from exc
    server.run()


if __name__ == "__main__":
    main()
