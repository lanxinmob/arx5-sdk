import os
import sys
import time
import math
import socket
import struct
import click

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, ROOT_DIR)
os.chdir(ROOT_DIR)
import arx5_interface as arx5


class _GripperCAN:
    """SocketCAN recovery for the SDK's DM gripper; never sends joint commands."""

    FRAME = struct.Struct("=IB3x8s")
    # Same MIT encoding/ranges as include/hardware/arx_can.h and libhardware.
    ZERO_TORQUE = bytes.fromhex("7f ff 7f f0 00 00 07 ff")
    DISABLE = b"\xff" * 7 + b"\xfd"
    ENABLE = b"\xff" * 7 + b"\xfc"
    RESET_ZERO = b"\xff" * 7 + b"\xfe"

    def __init__(self, can_socket, motor_id, torque_limit):
        self.socket = can_socket
        self.motor_id = motor_id
        self.torque_limit = torque_limit

    def send(self, data):
        frame = self.FRAME.pack(self.motor_id, 8, data)
        if self.socket.send(frame) != len(frame):
            raise RuntimeError("Incomplete CAN frame transmission")

    def receive(self):
        frame, _, flags, _ = self.socket.recvmsg(self.FRAME.size)
        if flags & socket.MSG_DONTROUTE:
            raise RuntimeError("Another local CAN writer is active; stop it before recovery")
        if len(frame) != self.FRAME.size:
            return None
        can_id, length, data = self.FRAME.unpack(frame)
        # libhardware routes standard CAN ID 0 to DM_can_data_repack.
        if can_id != 0 or length != 8 or data[0] & 0x0F != self.motor_id:
            return None
        status = data[0] >> 4
        if status not in (0, 1):
            raise RuntimeError(f"Gripper motor fault status=0x{status:x}; raw={data.hex()}")
        angle = int.from_bytes(data[1:3], "big") * 25.0 / 65535 - 12.5
        speed = ((data[3] << 4) | (data[4] >> 4)) * 90.0 / 4095 - 45.0
        torque = (((data[4] & 0x0F) << 8) | data[5]) * 36.0 / 4095 - 18.0
        return angle, speed, torque * 0.424

    def measure(self, label=None):
        angles = []
        speeds = []
        torques = []
        for _ in range(10):
            # Discard queued feedback before requesting a new zero-torque reply.
            self.socket.settimeout(0.0)
            for _ in range(1000):
                try:
                    self.receive()
                except BlockingIOError:
                    break
            else:
                raise RuntimeError("CAN receive queue stays busy; stop other controllers")
            self.socket.settimeout(1.0)
            self.send(self.ZERO_TORQUE)
            deadline = time.monotonic() + 1.0
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("Timed out waiting for fresh gripper feedback")
                self.socket.settimeout(remaining)
                try:
                    state = self.receive()
                except socket.timeout as exc:
                    raise RuntimeError("Timed out waiting for fresh gripper feedback") from exc
                if state is not None:
                    break
            angle, speed, torque = state
            if abs(speed) > 0.2 or abs(torque) > self.torque_limit:
                raise RuntimeError(
                    f"Gripper must be still and unloaded: speed={speed:.3f} rad/s, "
                    f"torque={torque:.3f} Nm"
                )
            angles.append(angle)
            speeds.append(speed)
            torques.append(torque)
            time.sleep(0.01)
        average = sum(angles) / len(angles)
        if label:
            click.echo(
                f"[recovery] {label} angle_rad={average:.6f} "
                f"angle_range=[{min(angles):.6f}, {max(angles):.6f}] "
                f"speed_max_rad_s={max(abs(v) for v in speeds):.6f} "
                f"torque_Nm={sum(torques)/len(torques):.6f}"
            )
        if max(angles) - min(angles) > 0.02:
            raise RuntimeError("Gripper moved during measurement; hold it still")
        return average


def _recover_gripper_zero(robot_config, interface):
    if not hasattr(socket, "AF_CAN"):
        raise click.ClickException("Zero recovery requires Linux SocketCAN")
    motor_id = robot_config.gripper_motor_id
    if (robot_config.gripper_motor_type != arx5.MotorType.DM_J4310
            or not 1 <= motor_id <= 15 or motor_id in robot_config.motor_id):
        raise click.ClickException("Zero recovery requires a separate DM_J4310 gripper motor ID (1..15)")
    click.echo(
        f"Zero recovery: {interface}, gripper motor ID {motor_id} only. "
        "No arm controller or arm damping will be started. Support the arm, stop "
        "other CAN controllers and remove anything held by the gripper."
    )
    input("Gently close until the fingertips touch without squeezing, then press Enter (Ctrl+C to cancel): ")
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can_socket:
        can_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        can_socket.settimeout(1.0)
        can_socket.bind((interface,))
        gripper = _GripperCAN(can_socket, motor_id, robot_config.gripper_torque_max / 2)
        completed = False
        try:
            gripper.send(gripper.DISABLE)
            time.sleep(0.02)
            gripper.send(gripper.ZERO_TORQUE)
            gripper.send(gripper.ENABLE)
            closed = gripper.measure("closed_before_reset")
            input("Keep the fingertips gently touching; press Enter to WRITE the gripper motor zero: ")
            if abs(gripper.measure("closed_before_write") - closed) > 0.02:
                raise RuntimeError("Closed position changed before reset; motor zero was NOT written")
            gripper.send(gripper.RESET_ZERO)
            click.echo("[recovery] zero_reset_sent; awaiting fresh feedback")
            time.sleep(0.1)
            zero_check = gripper.measure("closed_after_reset")
            if abs(zero_check) > 0.02:
                raise RuntimeError("Zero reset was sent but fresh feedback is not near zero; recovery NOT verified")
            input("Fully open the gripper by hand, hold still, then press Enter: ")
            opened = gripper.measure("fully_open")
            if abs(opened) < 0.1:
                raise RuntimeError("Fully-open readout is too close to zero; check the physical open position")
            input("Gently close the gripper again, hold still, then press Enter: ")
            closed = gripper.measure("closed_again")
            if abs(closed) > 0.02:
                width_mm = closed / opened * robot_config.gripper_width * 1000
                raise RuntimeError(
                    f"Repeated closed readout is not near zero: angle={closed:.6f} rad, "
                    f"change_from_zero_check={closed-zero_check:.6f} rad, "
                    f"model_width={width_mm:.3f} mm. The motor zero WAS written earlier; "
                    "the fully_open measurement above is diagnostic only, calibration NOT accepted. "
                    "Use --verify-zero to check closure repeatability without writing zero again."
                )
            click.echo(f"Fully-open joint position readout: {opened:.6f}")
            click.echo("Record this signed readout and the measured opening width in every control entry point.")
            completed = True
        finally:
            # Do not leave an enabled gripper after success, timeout or Ctrl+C.
            try:
                can_socket.settimeout(1.0)
                gripper.send(gripper.DISABLE)
            except (OSError, RuntimeError) as exc:
                if completed:
                    raise
                click.echo(f"Gripper disable also failed: {exc}", err=True)


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
@click.option("--recover-zero", is_flag=True, help="Recover an invalid gripper zero via SocketCAN without constructing an arm controller")
@click.option("--gripper-open-readout", type=float, default=-3.34, help="Current calibrated fully-open motor angle")
def calibrate_gripper(model: str, interface: str, verify_zero: bool, recover_zero: bool, gripper_open_readout: float):
    if recover_zero and verify_zero:
        raise click.UsageError("--recover-zero resets the motor zero and cannot be combined with --verify-zero")
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
    if recover_zero:
        try:
            _recover_gripper_zero(robot_config, interface)
        except (OSError, RuntimeError) as exc:
            raise click.ClickException(f"Gripper zero recovery failed: {exc}") from exc
        return
    joint_controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller", 6
    )
    joint_controller_config.gravity_compensation = False
    
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
            "For a confirmed closed gripper with an invalid zero on SocketCAN, "
            "use --recover-zero after reading the calibration instructions. "
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
