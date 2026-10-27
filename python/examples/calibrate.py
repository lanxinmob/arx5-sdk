import os
import sys
import time
import math
import click

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.argument("joint_id")
def calibrate_joint(model: str, interface: str, joint_id: int):
    if type(joint_id) == str:
        joint_id = int(joint_id)
    joint_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", 6
    )
    joint_controller_config.gravity_compensation = False
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    joint_controller = arx5.Arx5JointController(
        robot_config, joint_controller_config, interface
    )
    gain = arx5.Gain(joint_controller.get_robot_config().joint_dof)
    joint_controller.set_gain(gain)
    joint_controller.calibrate_joint(joint_id)
    while True:
        state = joint_controller.get_joint_state()
        pos = state.pos()
        print(", ".join([f"{x:.3f}" for x in pos]))
        time.sleep(0.1)


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
@click.option("--verify-zero", is_flag=True, help="Measure closed/open/closed readouts without resetting the motor zero")
@click.option("--gripper-open-readout", type=float, default=-3.34, help="Current calibrated fully-open motor angle")
def calibrate_gripper(model: str, interface: str, verify_zero: bool, gripper_open_readout: float):
    joint_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", 6
    )
    joint_controller_config.gravity_compensation = False
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    
    if not gripper_open_readout or not math.isfinite(gripper_open_readout):
        raise click.BadParameter("Must be finite and nonzero", param_hint="--gripper-open-readout")
    robot_config.gripper_open_readout = gripper_open_readout

    click.echo(
        f"Gripper startup: model={model}, interface={interface}, "
        f"gripper_open_readout={robot_config.gripper_open_readout:g} rad, "
        f"gripper_width={robot_config.gripper_width:g} m."
    )
    click.echo(
        "Stop other controllers on this CAN interface and support the arm; "
        "gravity compensation is OFF. The controller checks the current gripper "
        "position before calibration can reset its zero."
    )
    input(
        "Before connecting CAN, gently close the empty gripper until the fingertips "
        "touch without squeezing/deforming them, then press Enter (Ctrl+C to cancel): "
    )
    try:
        joint_controller = arx5.Arx5JointController(
            robot_config, joint_controller_config, interface
        )
    except RuntimeError as exc:
        raise click.ClickException(
            f"Controller initialization failed: {exc}. The motor zero was not reset. "
            "Check the first SDK error above. For a gripper position error, check "
            "the physical closed position, motor zero and signed fully-open readout "
            f"(currently {gripper_open_readout:g} rad). If already closed, changing "
            "the readout magnitude alone cannot fix a wrong sign or zero. "
            "See docs/hardware/setup.md#calibration in the repository root."
        ) from exc
    if not verify_zero:
        joint_controller.calibrate_gripper()
        return
    print("Verification only: motor zero will not be reset. Support the arm; gravity compensation is OFF.", flush=True)
    for label in ("closed_1", "fully_open", "closed_2"):
        prompt = "Gently close until the fingertips touch, without squeezing/deforming them" if label.startswith("closed") else "Fully open the gripper"
        input(f"{label}: {prompt}, hold still, then press Enter: ")
        samples = []
        for _ in range(50):
            state = joint_controller.get_joint_state()
            angle = state.gripper_pos / robot_config.gripper_width * robot_config.gripper_open_readout
            samples.append((angle, state.gripper_pos * 1000, state.gripper_torque))
            time.sleep(0.01)
        angles, widths, torques = zip(*samples)
        print(f"[zero-check] {label} angle_rad={sum(angles)/len(angles):.6f} "
              f"angle_range=[{min(angles):.6f}, {max(angles):.6f}] "
              f"width_mm={sum(widths)/len(widths):.3f} "
              f"torque_Nm={sum(torques)/len(torques):.3f}", flush=True)


@click.command()
@click.argument("model")  # ARX arm model: X5 or L5
@click.argument("interface")  # can bus name (can0 etc.)
def check_motor_movements(model: str, interface: str):
    joint_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", 6
    )
    joint_controller_config.gravity_compensation = False
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    joint_controller = arx5.Arx5JointController(
        robot_config, joint_controller_config, interface
    )
    while True:
        state = joint_controller.get_joint_state()
        pos = state.pos()
        print(", ".join([f"{x:.3f}" for x in pos]))
        time.sleep(0.1)


if __name__ == "__main__":
    #check_motor_movements()
    ## To actually calibrate, uncomment one of the following lines
    #calibrate_joint()
    calibrate_gripper()
