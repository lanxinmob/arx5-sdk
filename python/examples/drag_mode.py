"""
Put one or more ARX arms into gravity-compensation "zero-g" drag mode.

Each arm cancels its own weight with a URDF-based gravity feedforward and
runs with kp = 0 and low kd — exactly the same drag behaviour as the leader
arm in leader_follower_teleop.py (the equivalent of the ROS version's
G_COMPENSATION state). The grippers are released as well (kp = 0), so
gripper closures can also be dragged.

In drag mode no commands are streamed at all: the gain is set once and the
SDK's background thread keeps sending the gravity feedforward. This gives
the same feel as the leader arm (damping acts against absolute motion).

Useful for demonstrations, teaching poses / trajectories, or simply
repositioning the arms without active joint control.

Usage:
    # Any number of MODEL INTERFACE pairs
    python examples/drag_mode.py X5 can0
    python examples/drag_mode.py X5 can0 X5 can1
    python examples/drag_mode.py X5 can1 X5_umi can0 -g -3.4

Keys:
    [space] toggle drag / hold (freeze all arms at their current pose)
    [h]     all arms go home, then return to drag mode
    [q]     quit: all arms go home and stay position-held there
            (pass --exit-damping to go limp instead)
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
from pynput.keyboard import Key, KeyCode
from peripherals.keystroke_counter import KeystrokeCounter, KeyCode

# Depending on the pynput version/backend, the space key arrives either as
# Key.space or as KeyCode(char=' '), and these do NOT compare equal
SPACE_KEYS = (Key.space, KeyCode(char=" "))


class DragModeArm:
    """One arm configured for gravity-compensation drag mode."""

    def __init__(
        self,
        model: str,
        interface: str,
        gripper_open_readout: float,
        kd_scale: float,
        drag_gripper_kp: float,
        drag_gripper_kd: float,
        gravity_scale: float,
        gravity_scale_model: str,
    ):
        # Build and modify the configs BEFORE constructing the controller
        # (get_robot_config() returns a copy, post-construction edits are
        # ineffective)
        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        self.controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            "joint_controller", robot_config.joint_dof
        )
        # Gravity feedforward cancels the arm's own weight: the core of the
        # zero-g drag mode
        self.controller_config.gravity_compensation = True
        if gripper_open_readout != 0.0:
            robot_config.gripper_open_readout = gripper_open_readout

        # Gravity scale: if the URDF mass model is heavier than the real arm
        # (e.g. X5_umi with a lighter end-effector payload), the arm
        # overcompensates gravity and the wrist tilts up/back on its own.
        # Scaling gravity_vector scales the feedforward torque proportionally
        # (verified: inverse_dynamics output is exactly linear in it).
        self.gravity_scaled = False
        if gravity_scale != 1.0 and gravity_scale_model in ("", model):
            robot_config.gravity_vector = robot_config.gravity_vector * gravity_scale
            self.gravity_scaled = True

        self.controller = arx5.Arx5JointController(
            robot_config, self.controller_config, interface
        )
        self.dof = robot_config.joint_dof

        # Drag gain: identical to the leader arm in leader_follower_teleop.py.
        # kp = 0 (no position pull), kd = default_kd * kd_scale (light damping)
        self.drag_gain = self.controller.get_gain()
        self.drag_gain.kp()[:] = 0.0
        self.drag_gain.kd()[:] = self.controller_config.default_kd * kd_scale
        self.drag_gain.gripper_kp = drag_gripper_kp
        self.drag_gain.gripper_kd = drag_gripper_kd

        # Hold gain: normal position control, used to freeze the arm in place
        self.hold_gain = arx5.Gain(
            self.controller_config.default_kp,
            self.controller_config.default_kd,
            self.controller_config.default_gripper_kp,
            self.controller_config.default_gripper_kd,
        )

    def enter_drag(self):
        self.controller.set_gain(self.drag_gain)

    def enter_hold(self):
        # Pin the interpolator to the current pose BEFORE switching on the
        # position gain, otherwise the stale command (e.g. home) would make
        # the arm jerk
        state = self.controller.get_joint_state()
        cmd = arx5.JointState(self.dof)
        cmd.pos()[:] = state.pos().copy()
        cmd.vel()[:] = 0.0
        cmd.gripper_pos = state.gripper_pos
        self.controller.set_joint_cmd(cmd)
        time.sleep(0.01)  # let the background thread pick up the new command
        self.controller.set_gain(self.hold_gain)

    def hold(self):
        """Stream hold commands. Call this every cycle while in hold mode."""
        state = self.controller.get_joint_state()
        cmd = arx5.JointState(self.dof)
        cmd.pos()[:] = state.pos().copy()
        cmd.vel()[:] = 0.0
        cmd.gripper_pos = state.gripper_pos
        self.controller.set_joint_cmd(cmd)


class DragModeManager:
    def __init__(self, arms: list[DragModeArm], controller_dt: float):
        self.arms = arms
        self.controller_dt = controller_dt
        self.mode = "drag"
        self.command_enabled = True

    def enter_drag(self):
        for arm in self.arms:
            arm.enter_drag()
        self.mode = "drag"

    def enter_hold(self):
        for arm in self.arms:
            arm.enter_hold()
        self.mode = "hold"

    def toggle(self):
        if self.mode == "drag":
            self.enter_hold()
            return "HOLD (arms frozen at current pose)"
        else:
            self.enter_drag()
            return "DRAG (arms freely movable)"

    def go_home(self):
        self.command_enabled = False
        time.sleep(self.controller_dt * 5)
        for arm in self.arms:
            arm.controller.reset_to_home()
        self.enter_drag()
        self.command_enabled = True

    def run(self):
        while True:
            if self.command_enabled and self.mode == "hold":
                # In drag mode nothing is streamed at all — the gain set by
                # enter_drag() plus the SDK's background send/recv thread
                # (which keeps applying the gravity feedforward) do
                # everything, exactly like the leader arm in
                # leader_follower_teleop.py
                for arm in self.arms:
                    arm.hold()
            time.sleep(self.controller_dt)

    def shutdown(self, exit_damping: bool):
        self.command_enabled = False
        time.sleep(self.controller_dt * 5)
        for arm in self.arms:
            arm.controller.reset_to_home()
            if exit_damping:
                # Limp exit (ROS PROTECT style). Off by default: without
                # position feedback the wrist slowly sags onto its
                # mechanical limit due to gravity model errors.
                arm.controller.set_to_damping()


@click.command()
@click.argument(
    "model_interface_pairs",
    nargs=-1,
    required=True,
    metavar="[MODEL INTERFACE ...]",
)
@click.option(
    "--gripper-open-readout",
    "-g",
    default=-3.4,
    help="Fully-open gripper motor readout. Use 0 to keep the default config. "
    "Calibrate with examples/calibrate.py (a common value is -3.4 for newer X5 arms).",
)
@click.option(
    "--kd-scale",
    default=0.05,
    help="Joint kd = default_kd * this value in drag mode. Same default as the "
    "leader arm in leader_follower_teleop.py. Increase for a heavier feel.",
)
@click.option(
    "--gripper-kp",
    default=0.0,
    help="Gripper kp in drag mode. 0 releases the gripper so it can also be dragged.",
)
@click.option(
    "--gripper-kd",
    default=0.05,
    help="Gripper kd in drag mode.",
)
@click.option(
    "--exit-damping/--no-exit-damping",
    default=False,
    help="On quit, go limp (damping) after homing instead of staying position-held at home.",
)
@click.option(
    "--gravity-scale",
    default=1.0,
    help="Scale the gravity compensation feedforward. Use < 1 for arms whose URDF model "
    "is heavier than the real one (e.g. wrist keeps tilting up/back on its own), "
    "or > 1 if the arm keeps sagging down. Try 0.9, 0.85, 0.8 ...",
)
@click.option(
    "--gravity-scale-model",
    default="",
    help="Apply --gravity-scale only to arms of this model (e.g. X5_umi). "
    "Empty means all arms.",
)
def main(
    model_interface_pairs,
    gripper_open_readout: float,
    kd_scale: float,
    gripper_kp: float,
    gripper_kd: float,
    exit_damping: bool,
    gravity_scale: float,
    gravity_scale_model: str,
):
    if len(model_interface_pairs) % 2 != 0:
        raise click.BadParameter("Arguments must be MODEL INTERFACE pairs, e.g.: X5 can0 X5 can1")
    pairs = list(zip(model_interface_pairs[::2], model_interface_pairs[1::2]))
    interfaces = [interface for _, interface in pairs]
    assert len(interfaces) == len(set(interfaces)), "Duplicated CAN interface given"

    arms = []
    for model, interface in pairs:
        print(f"Initializing {model} on {interface} ...")
        arm = DragModeArm(
            model,
            interface,
            gripper_open_readout,
            kd_scale,
            gripper_kp,
            gripper_kd,
            gravity_scale,
            gravity_scale_model,
        )
        if arm.gravity_scaled:
            print(f"  {model}: gravity compensation scaled by {gravity_scale}")
        arms.append(arm)

    manager = DragModeManager(arms, arms[0].controller_config.controller_dt)

    print("\nResetting all arms to home position ...")
    for arm in arms:
        arm.controller.reset_to_home()
    for arm in arms:
        arm.enter_drag()

    print(
        "\nAll arms are now in zero-g drag mode (gravity compensated, freely movable).\n"
        "  [space] toggle drag / hold (freeze at current pose)\n"
        "  [h]     all arms go home\n"
        "  [q]     quit\n"
    )

    control_error = []

    def control_loop():
        try:
            manager.run()
        except Exception as e:
            control_error.append(e)

    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    try:
        with KeystrokeCounter() as key_counter:
            while True:
                for key_stroke in key_counter.get_press_events():
                    if key_stroke in SPACE_KEYS:
                        print(f"\nMode: {manager.toggle()}")
                    elif key_stroke == KeyCode(char="h"):
                        print("\nGoing home ...")
                        manager.go_home()
                        print("Home reached, back to drag mode.")
                    elif key_stroke == KeyCode(char="q"):
                        raise KeyboardInterrupt
                time.sleep(0.02)
    except KeyboardInterrupt:
        pass

    if control_error:
        raise control_error[0]

    print("\nExiting: resetting all arms to home ...")
    manager.shutdown(exit_damping)
    if not exit_damping:
        print("Arms are position-held at home. Kill this process to release them.")
    print("Done.")


if __name__ == "__main__":
    main()
