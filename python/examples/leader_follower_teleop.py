"""
Leader-follower (master-slave) teleoperation for two ARX arms.

This example ports the leader-follower teleoperation feature from the ROS
version (ARX_X5/ROS/X5_ws, `remote_master_v1` + `remote_slave_v1`) to the
arx5-sdk python interface:

    ROS version                                this example
    ---------------------------------------------------------------
    remote_master: G_COMPENSATION       ->     leader arm runs with
        (gravity compensated, freely           gravity compensation
        draggable by hand)                     enabled, kp = 0 and low
                                               kd, so it can be dragged
    remote_slave: FollowCallbackV1      ->     follower runs position
        (POSITION_CONTROL + setCatch)          control, mirroring the
                                               leader's joint positions
                                               and gripper
    joystick button: GO_HOME            ->     press "h"
    joystick button: start/stop         ->     press "space"
    exit to PROTECT                     ->     press "q" or Ctrl+C

On top of the ROS version, this example adds gripper force feedback: when
the follower's gripper is blocked by an object (its measured gripper torque
rises), a proportional torque is applied to the leader's gripper motor,
pushing the operator's fingers apart, so it feels like actually holding
what the follower is holding. Tune/disable it with --force-scale.

Usage:
    python examples/leader_follower_teleop.py X5 can1 X5 can0
    # Newer X5 arms usually need a negative gripper readout (calibrate with
    # examples/calibrate.py to get the exact value for your arm):
    python examples/leader_follower_teleop.py X5 can1 X5 can0 -g -3.4
"""

import threading
import time

import os
import sys

import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5
import click


class LeaderFollowerTeleop:
    def __init__(
        self,
        leader_model: str,
        leader_interface: str,
        follower_model: str,
        follower_interface: str,
        gripper_open_readout: float,
        leader_kp: float,
        leader_kd_scale: float,
        leader_gripper_kd: float,
        vel_ratio: float,
        force_scale: float,
        follower_gravity_scale: float = 1.0,
        leader_gravity_scale: float = 1.2,
        follower_gripper_open_readout: float = -3.45369,
        gripper_close_threshold: float = 0.001,
        follower_gripper_kp: float = None,
    ):
        assert leader_interface != follower_interface
        if not np.isfinite(gripper_close_threshold) or not 0 <= gripper_close_threshold <= 0.005:
            raise ValueError("Gripper close threshold must be between 0 and 0.005 m")
        if follower_gripper_kp is not None and (not np.isfinite(follower_gripper_kp) or not 0 < follower_gripper_kp <= 10):
            raise ValueError("Follower gripper kp must be positive and at most 10")
        # Gripper force feedback gain: the leader's gripper motor is fed with
        # -force_scale * (follower's measured gripper torque). Per the SDK
        # convention, the follower's gripper torque is negative while it is
        # blocked while closing (i.e. grasping an object), so a positive
        # force_scale pushes the leader's gripper open, like holding an object.
        # Set it to 0 to disable; use a negative value to invert the feel.
        self.force_scale = force_scale
        self.force_torque_limit = 0.6  # N*m, safety clamp for the feedback torque

        self.drag_kp = leader_kp
        self.drag_kd_scale = leader_kd_scale
        self.drag_gripper_kd = leader_gripper_kd
        self.gripper_close_threshold = gripper_close_threshold
        # Velocity feedforward ratio: how much of the leader's joint velocity
        # is added to the follower command. Reduces tracking delay.
        self.vel_ratio = vel_ratio
        # Whether the control loop is allowed to send commands. It is disabled
        # while reset_to_home() is running, otherwise the streamed commands
        # would keep overriding the home trajectory.
        self.command_enabled = True
        self.teleop_started = False
        self.diagnostics = False

        # IMPORTANT: build and modify the configs BEFORE constructing the
        # controllers. get_robot_config() returns a copy, so changing it after
        # construction has no effect.
        leader_robot_config = arx5.RobotConfigFactory.get_instance().get_config(leader_model)
        leader_robot_config.gravity_vector = leader_robot_config.gravity_vector * leader_gravity_scale
        self.leader_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", leader_robot_config.joint_dof
        )
        # The leader must keep gravity compensation enabled so that the
        # feedforward torque cancels its own weight (the ROS "G_COMPENSATION"
        # mode). This is the default for the joint controller, but we make it
        # explicit here.
        self.leader_controller_config.gravity_compensation = True
        if gripper_open_readout != 0.0:
            leader_robot_config.gripper_open_readout = gripper_open_readout

        follower_robot_config = arx5.RobotConfigFactory.get_instance().get_config(follower_model)
        # Must be applied before controller/solver construction; getters return copies.
        follower_robot_config.gravity_vector = follower_robot_config.gravity_vector * follower_gravity_scale
        self.follower_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", follower_robot_config.joint_dof
        )
        self.follower_controller_config.gravity_compensation = True
        if follower_gripper_kp is not None:
            self.follower_controller_config.default_gripper_kp = follower_gripper_kp
        follower_readout = gripper_open_readout if follower_gripper_open_readout is None else follower_gripper_open_readout
        if follower_readout != 0.0:
            follower_robot_config.gripper_open_readout = follower_readout

        assert leader_robot_config.joint_dof == follower_robot_config.joint_dof
        self.dof = leader_robot_config.joint_dof

        self.leader = arx5.Arx5JointController(
            leader_robot_config, self.leader_controller_config, leader_interface
        )
        self.follower = arx5.Arx5JointController(
            follower_robot_config, self.follower_controller_config, follower_interface
        )
        self.controller_dt = self.leader_controller_config.controller_dt

        # Follower joint limits (for clipping leader commands when the two
        # arms are different models, e.g. X5 leader + L5 follower)
        self.follower_pos_min = follower_robot_config.joint_pos_min.copy()
        self.follower_pos_max = follower_robot_config.joint_pos_max.copy()
        self.follower_gripper_width = follower_robot_config.gripper_width

    def _setup_leader_drag_mode(self):
        """
        Make the leader arm freely draggable: zero kp, low kd, gravity
        feedforward left on. Equivalent to the ROS G_COMPENSATION state.
        """
        gain = self.leader.get_gain()
        gain.kp()[:] = self.drag_kp
        gain.kd()[:] = self.leader_controller_config.default_kd * self.drag_kd_scale
        gain.gripper_kp = 0.0
        gain.gripper_kd = self.drag_gripper_kd
        self.leader.set_gain(gain)

    def _follower_gripper_target(self, leader_width):
        if leader_width <= self.gripper_close_threshold:
            return 0.0
        return float(np.clip(leader_width, 0.0, self.follower_gripper_width))

    def start(self):
        print("Resetting both arms to home position...")
        self.leader.reset_to_home()
        self.follower.reset_to_home()
        # reset_to_home() ramps the gain back to default, so re-apply the
        # drag-mode gain to the leader afterwards
        self._setup_leader_drag_mode()
        print(
            "\nLeader arm is now in gravity-compensation (drag) mode.\n"
            "  [space] start / pause teleoperation\n"
            "  [h]     both arms go home\n"
            "  [q]     quit\n"
            "Press [space] to start teleoperation."
        )

    def go_home(self):
        print("\nGoing home...")
        self.teleop_started = False
        self.command_enabled = False
        time.sleep(self.controller_dt * 5)  # let the control loop finish its cycle
        self.follower.reset_to_home()
        self.leader.reset_to_home()
        self._setup_leader_drag_mode()
        self.command_enabled = True
        print("Home position reached. Press [space] to start teleoperation again.")

    def go_home_and_stop(self):
        self.teleop_started = False
        self.command_enabled = False
        time.sleep(self.controller_dt * 5)
        self.follower.reset_to_home()
        self.leader.reset_to_home()

    def close(self):
        """Release both controllers after the caller has joined the control loop."""
        self.command_enabled = False
        self.teleop_started = False
        errors = []
        for name in ("follower", "leader"):
            controller = getattr(self, name, None)
            if controller is None:
                continue
            try:
                controller.set_to_damping()
            except Exception as error:
                errors.append(f"{name}: {error}")
            finally:
                setattr(self, name, None)
                del controller
        if errors:
            raise RuntimeError("Teleoperation shutdown failed: " + "; ".join(errors))

    def _apply_gripper_force_feedback(self, leader_state, follower_gripper_torque: float):
        """
        Stream a command to the leader arm so that its gripper motor feels
        the follower's grasping force. The joints are commanded with their
        own measured pos/vel so the drag-mode feel is not disturbed (kp = 0
        anyway), while the gripper gets the feedback torque on top of its
        low damping.
        """
        cmd = arx5.JointState(self.dof)
        cmd.pos()[:] = leader_state.pos().copy()
        cmd.vel()[:] = leader_state.vel().copy()
        cmd.gripper_pos = leader_state.gripper_pos
        cmd.gripper_vel = leader_state.gripper_vel
        if self.force_scale != 0.0:
            feedback_torque = -self.force_scale * follower_gripper_torque
            cmd.gripper_torque = float(
                np.clip(feedback_torque, -self.force_torque_limit, self.force_torque_limit)
            )
        self.leader.set_joint_cmd(cmd)

    def run(self, stop_event=None, on_step=None):
        """Optional recorder hook receives pre-command states and the sent target.

        The hook must not block: camera capture and disk I/O belong elsewhere.
        Default arguments preserve standalone teleoperation behavior.
        """
        hold_cmd = arx5.JointState(self.dof)
        next_report = 0.0
        while stop_event is None or not stop_event.is_set():
            if not self.command_enabled:
                time.sleep(self.controller_dt)
                continue
            observation_time = time.monotonic()
            leader_state = self.leader.get_joint_state()
            if self.teleop_started:
                follower_state = self.follower.get_joint_state()
                follower_cmd = arx5.JointState(self.dof)
                # Mirror the leader's joint positions, clipped to the
                # follower's own limits
                follower_cmd.pos()[:] = np.clip(
                    leader_state.pos().copy(), self.follower_pos_min, self.follower_pos_max
                )
                # Partial velocity feedforward reduces the following delay
                follower_cmd.vel()[:] = leader_state.vel().copy() * self.vel_ratio
                follower_cmd.gripper_pos = self._follower_gripper_target(leader_state.gripper_pos)
                self.follower.set_joint_cmd(follower_cmd)
                command_time = time.monotonic()
                # Haptics: let the operator feel what the follower gripper
                # is gripping
                self._apply_gripper_force_feedback(leader_state, follower_state.gripper_torque)
                if on_step is not None:
                    on_step(leader_state, follower_state, follower_cmd,
                            observation_time, command_time)
            else:
                # While paused, hold the follower stiffly at its current pose
                follower_state = self.follower.get_joint_state()
                hold_cmd.pos()[:] = follower_state.pos().copy()
                hold_cmd.vel()[:] = 0.0
                hold_cmd.gripper_pos = follower_state.gripper_pos
                self.follower.set_joint_cmd(hold_cmd)
                # Do not leave the last grasp feedback torque active while paused.
                self._apply_gripper_force_feedback(leader_state, 0.0)
            if self.diagnostics and time.monotonic() >= next_report:
                next_report = time.monotonic() + 1.0
                for name in ("leader", "follower"):
                    controller = getattr(self, name)
                    config = controller.get_controller_config()
                    gain = controller.get_gain()
                    state = controller.get_joint_state()
                    command = controller.get_joint_cmd()
                    print(
                        f"[diagnostics] {name} following={self.teleop_started} "
                        f"gravity={config.gravity_compensation} emergency={controller.is_emergency()} "
                        f"kp={np.round(gain.kp(), 3).tolist()} "
                        f"kd={np.round(gain.kd(), 3).tolist()} "
                        f"torque_cmd={np.round(command.torque(), 3).tolist()} "
                        f"velocity={np.round(state.vel(), 3).tolist()} "
                        f"gripper_mm={state.gripper_pos * 1000:.2f} "
                        f"gripper_cmd_mm={command.gripper_pos * 1000:.2f} "
                        f"gripper_torque={state.gripper_torque:.3f} "
                        f"gripper_kp={gain.gripper_kp:.3f}",
                        flush=True,
                    )
            time.sleep(self.controller_dt)


@click.command()
@click.argument("leader_model")  # Leader (master) arm model: X5 or L5
@click.argument("leader_interface")  # Leader CAN bus name (can0, can1, ...)
@click.argument("follower_model")  # Follower (slave) arm model: X5 or L5
@click.argument("follower_interface")  # Follower CAN bus name (can0, can1, ...)
@click.option(
    "--gripper-open-readout",
    "-g",
    default=-3.2969,
    help="Fully-open gripper motor readout. Use 0 to keep the default config. "
    "Calibrate with examples/calibrate.py (a common value is -3.4 for newer X5 arms).",
)
@click.option("--leader-kp", default=0.0, help="Leader joint kp in drag mode")
@click.option(
    "--leader-kd-scale",
    default=0.05,
    help="Leader joint kd = default_kd * this value in drag mode. "
    "Increase it if the leader arm feels too floppy.",
)
@click.option(
    "--leader-gripper-kd",
    default=0.05,
    help="Leader gripper kd in drag mode. Lower it if the gripper is hard to squeeze.",
)
@click.option(
    "--vel-ratio",
    default=0.3,
    help="Ratio of the leader's velocity fed forward to the follower (0~1)",
)
@click.option(
    "--force-scale",
    default=0.5,
    help="Gripper force feedback gain: leader gripper torque = -this * follower "
    "gripper torque. 0 disables the haptic feedback, negative inverts the "
    "direction.",
)
@click.option("--diagnostics", is_flag=True, help="Print actual gains, output torque and fault state once per second")
@click.option("--leader-gravity-scale", type=click.FloatRange(min=0.0, min_open=True), default=1.2, show_default=True, help="Leader gravity compensation multiplier")
@click.option("--follower-gravity-scale", type=click.FloatRange(min=0.0, min_open=True), default=1.0, show_default=True, help="Follower gravity compensation multiplier")
@click.option("--follower-gripper-open-readout", type=float, default=-3.45369, show_default=True, help="Override the follower's fully-open motor readout; 0 uses its model default")
@click.option("--gripper-close-threshold", type=click.FloatRange(min=0.0, max=0.005), default=0.001, show_default=True, help="Command fully closed below this leader width (m)")
@click.option("--follower-gripper-kp", type=click.FloatRange(min=0.0, min_open=True, max=10.0), default=None, help="Follower gripper position gain override (up to 10)")
def main(
    leader_model: str,
    leader_interface: str,
    follower_model: str,
    follower_interface: str,
    gripper_open_readout: float,
    leader_kp: float,
    leader_kd_scale: float,
    leader_gripper_kd: float,
    vel_ratio: float,
    force_scale: float,
    diagnostics: bool,
    leader_gravity_scale: float,
    follower_gravity_scale: float,
    follower_gripper_open_readout: float,
    gripper_close_threshold: float,
    follower_gripper_kp: float,
):
    # Keep recorder imports usable in terminals without an X server.
    from pynput.keyboard import Key, KeyCode
    from peripherals.keystroke_counter import KeystrokeCounter
    space_keys = (Key.space, KeyCode(char=" "))

    np.set_printoptions(precision=3, suppress=True)
    print(f"SDK: {arx5.__file__}", flush=True)
    print(f"Gravity multipliers: leader={leader_gravity_scale:g}, follower={follower_gravity_scale:g}", flush=True)
    teleop = LeaderFollowerTeleop(
        leader_model,
        leader_interface,
        follower_model,
        follower_interface,
        gripper_open_readout,
        leader_kp,
        leader_kd_scale,
        leader_gripper_kd,
        vel_ratio,
        force_scale,
        follower_gravity_scale=follower_gravity_scale,
        leader_gravity_scale=leader_gravity_scale,
        follower_gripper_open_readout=follower_gripper_open_readout,
        gripper_close_threshold=gripper_close_threshold,
        follower_gripper_kp=follower_gripper_kp,
    )
    teleop.diagnostics = diagnostics
    stop_event = threading.Event()
    control_thread = None
    control_error = []

    def control_loop():
        try:
            teleop.run(stop_event=stop_event)
        except Exception as e:
            control_error.append(e)
            print(f"Teleoperation control failed: {e}", flush=True)

    try:
        teleop.start()
        control_thread = threading.Thread(target=control_loop, daemon=True)
        control_thread.start()
        try:
            with KeystrokeCounter() as key_counter:
                while True:
                    if control_error:
                        raise control_error[0]
                    for key_stroke in key_counter.get_press_events():
                        if key_stroke in space_keys:
                            teleop.teleop_started = not teleop.teleop_started
                            state_str = "STARTED" if teleop.teleop_started else "PAUSED"
                            print(f"\nTeleoperation {state_str}.")
                        elif key_stroke == KeyCode(char="h"):
                            teleop.go_home()
                        elif key_stroke == KeyCode(char="q"):
                            raise KeyboardInterrupt
                    time.sleep(0.02)
        except KeyboardInterrupt:
            pass

        teleop.command_enabled = False
        stop_event.set()
        control_thread.join()
        if control_error:
            raise control_error[0]
        print("\nExiting: resetting both arms to home position...")
        teleop.go_home_and_stop()
    finally:
        teleop.command_enabled = False
        stop_event.set()
        if control_thread is not None:
            control_thread.join()
        teleop.close()
    print("Done.")


if __name__ == "__main__":
    main()
