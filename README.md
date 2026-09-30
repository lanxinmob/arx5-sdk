# C++ && Python Controller for ARX5 Robot Arm

## Patched Local Controller

The local controller now latches state/overcurrent faults without blocking
state queries or shutdown. `is_emergency()` reports the latch; joint commands,
Cartesian commands, gain changes and homing are rejected until a new controller
is created after the physical fault is resolved. Fault output is damping only,
without position or gravity feedforward torque. This is not power removal or
a certified emergency stop; an unsupported arm may settle under gravity.

Failed Cartesian IK is rejected before changing the trajectory. Python state
and gain array views retain their owning objects, including views obtained
from temporary `get_joint_state()` and `get_eef_state()` results.

Build on this robot host using the existing `arx5` environment and Eigen 3.4:

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build --target arx5_interface -j 4
PYTHONPATH="$PWD/python" python -c \
  'import arx5_interface as a; print(a.__file__); assert hasattr(a.Arx5JointController, "is_emergency")'
```

The printed module must be the `.so` under this repository's `python/`, not
the older installed wheel. Eigen 5 is ABI-incompatible with the prebuilt
solver. The guarded server selects the local module and refuses an unpatched
one; its `GET_STATE` response includes `emergency`.
Cartesian ZMQ servers also return `ee_cmd_pose` and `gripper_cmd_pos` alongside
the measured pose and gripper state. UMI clients use the command values after
`RESET_TO_HOME` to initialize their hold target against the guarded server's
command limit.

The task arm here is `X5_umi / can0`; `can1` is the demonstration arm. Start
`python/communication/zmq_server_agent_safe.py X5_umi can0` with measured
`--workspace-min X Y Z` and `--workspace-max X Y Z` bounds. Initialization
enables hardware; `RESET_TO_HOME` moves the arm and opens the gripper. Do not
start another controller on the same bus. Close the gripper while powered off
before a power cycle; do not bypass an out-of-range reading by changing its
zero or calibration sign.

Recovery verification: offline fault injection covered invalid initialization,
gripper range, overcurrent, non-finite feedback, negative joint outliers, IK
rejection and clean shutdown. On `can0`, a 2 mm upward target produced about
0.59 mm measured travel, followed by a return within 0.024 mm of the baseline.
This verifies basic movement, not accurate 2 mm or full six-axis tracking.
The temporary live probe independently checked incoming CAN frame freshness;
SDK state timestamps alone do not prove fresh motor feedback. Keep a physical
stop available and validate telemetry freshness, tracking and workspace
geometry before autonomous manipulation.

## ZMQ Communication

### Quest ROS 2 Bring-up (Steps 3-6)

The installed Quest app has been observed sending Unity ROS-TCP
`arm_control/PosCmd` packets over Wi-Fi to `192.168.31.137:10000`. On this host,
`quest_ros_tcp.py` publishes them as ROS 2 `/ARX_VR_R` and `/ARX_VR_L`; it never
opens CAN or starts an X5 controller. The official `serial_port_node` is an
alternative only when an ARX USB receiver whose by-id name contains
`USB_Single_Serial` is connected. The two ARX USB2CAN adapters are not that
receiver. Do not start both publishers at once. Do not run the vendor
`ARX_VR.sh`, which also opens several topic terminals, or `port.sh`, which
changes global udev permissions.

On this Jazzy host, build the vendor message package once:

```bash
cd /home/hjadmin/projects/ARX_X5/ARX_VR_SDK/ROS2
source /opt/ros/jazzy/setup.bash
env PATH=/usr/bin:/bin /usr/bin/colcon build --packages-select arm_control \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

For the observed Wi-Fi path, start the local publisher from this repository
after sourcing the vendor message package. The Quest app must target this
host's address and TCP port 10000. Run this on a trusted LAN only:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hjadmin/projects/ARX_X5/ARX_VR_SDK/ROS2/install/setup.bash
cd /home/hjadmin/projects/arx5-sdk
/usr/bin/python3 python/communication/quest_ros_tcp.py --bind 192.168.31.137
```

The publisher closes a TCP session after five seconds without incoming bytes
and keeps listening for a new connection. If Quest sleeps, wake the headset
and return to the app; a connected socket or increasing packet counts alone
do not prove that controller poses are updating.

If instead using the official USB receiver, first install its serial
dependencies and build `serial_port` in the same vendor workspace:

```bash
sudo apt-get install ros-jazzy-serial-driver ros-jazzy-asio-cmake-module
env PATH=/usr/bin:/bin /usr/bin/colcon build --packages-up-to serial_port \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Check that the USB receiver is connected before starting the vendor node:
this version of its constructor indexes the device list even when it is
empty. The user must have access to the serial device, usually through the
`dialout` group. On a new host, run
`sudo usermod -aG dialout "$USER"` once and log out/in before launching:

```bash
ls /dev/serial/by-id/*USB_Single_Serial*
ros2 run serial_port serial_port_node
```

In a second terminal, source the same `install/setup.bash` and inspect the
right controller first. Leave CAN controllers stopped. The vendor Quest guide
says to unlock the app with right A+B and left X+Y together, then press both
joystick buttons. Hold the right index trigger while slowly moving that
controller; test the side trigger separately. Confirm that XYZ, RPY and
gripper change and determine their units and ranges before interpreting them
as robot commands. A stream of unchanged values does not verify controller
tracking or the trigger mapping.

```bash
ros2 topic echo /ARX_VR_R --once
cd /home/hjadmin/projects/arx5-sdk
/usr/bin/python3 python/communication/quest_arx5_bridge.py --mode raw
```

For target preview, build this checkout's patched Stanford SDK for Jazzy's
Python 3.12. The existing `arx5` environment contains Python 3.9 and its
module cannot be imported by ROS 2 Jazzy. From this repository root:

```bash
ARX5_ENV="$HOME/miniconda3/envs/arx5"
CONDA_PREFIX="$ARX5_ENV" cmake -S . -B build/quest-jazzy \
  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="$ARX5_ENV" -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
CONDA_PREFIX="$ARX5_ENV" LIBRARY_PATH="$ARX5_ENV/lib" \
  cmake --build build/quest-jazzy --target arx5_interface -j 4
LD_LIBRARY_PATH="$ARX5_ENV/lib:$LD_LIBRARY_PATH" PYTHONPATH="$PWD/python:$PYTHONPATH" \
  /usr/bin/python3 python/communication/quest_arx5_bridge.py \
  --mode target --side right --model X5_umi --scale 0.01 \
  --xyz-map 0,y,0 --speed 0.002 --radius 0.002
```

The preview uses the model's zero-joint FK pose as its starting point, anchors
the first Quest sample, scales relative XYZ motion, and prints a speed- and
radius-limited Cartesian target. RPY and gripper targets stay at their home
values. On the Wi-Fi path, repeated packets with an unchanged nonzero
`time_count` do not refresh pose age; after `--timeout`, the preview holds its
target and reanchors on the next pose update. It also reanchors after large
position jumps.
It does not open CAN, reset an arm, or send robot commands. Run with
`--side left` to inspect the left topic. If the official topic reports XYZ in
millimeters, add `--input-meters-per-unit 0.001` after confirming it from live
samples. `--xyz-map` lists the Quest source for robot X,Y,Z; for example,
`--xyz-map 0,y,0` previews Quest -Y as robot -Y while leaving robot X and Z
fixed. The default `x,y,z` preserves the original preview behavior; it is not
a calibrated robot frame. Check the frame directions, units, gripper range and
trigger behavior from live Quest data before enabling any X5 motion. In the observed
Wi-Fi stream, `time_count` advanced with new poses and stayed fixed while the
last pose was repeated; gripper values could still change independently. In a
rightward-and-return trial on this headset, Quest Y fell by about 0.17 source
units on the outward leg and rose on return, identifying Quest -Y as body-right
for this setup. X/Z and RPY also changed, so robot base-frame direction,
orientation mapping, and exact source units remain unverified. Replaying the
logged samples with one source unit treated as one meter and a 2 mm target
radius reached the limit at `--scale 0.02`; `--scale 0.01` stayed within it.
The advancing counter is a pose-freshness check, not yet a verified trigger
state. The vendor serial node does not populate `time_count` or a dedicated
trigger field.

### Quest Single-Arm XYZ (Steps 7-8)

Use one `X5_umi / can0` arm first. Confirm the robot's home position and a
physically clear workspace before enabling it. The Quest topic alone cannot
establish the arm's clearance or coordinate directions. Stop keyboard teleop,
recorders and other controllers on `can0`; keep the arm unloaded and a physical
stop available. `RESET_TO_HOME` moves the arm and opens its gripper.

The guarded Cartesian server runs in the `arx5` environment; the ROS bridge
runs under Jazzy's system Python and needs PyZMQ:

```bash
sudo apt-get install python3-zmq
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
python communication/zmq_server_agent_safe.py X5_umi can0 \
  --bind 127.0.0.1 --port 8765 --max-position-delta 0.001 \
  --max-home-translation-radius 0.002 \
  --workspace-min X_MIN Y_MIN Z_MIN --workspace-max X_MAX Y_MAX Z_MAX
```

Replace the six workspace placeholders with measured base-frame coordinates
in meters. Both bounds must contain the SDK home pose printed by the target
preview and cover only a physically clear region. The guarded server checks
each command against these bounds, a 1 mm per-command position step and a
2 mm home radius. The bridge also uses the server's radius when it is smaller
than the requested radius and clamps its target to the server's XYZ box. Its
default two-second command watchdog switches to SDK damping if accepted pose
commands stop; state queries and rejected commands do not extend that
deadline. Damping is not power removal or a position hold.
An SDK failure during reset also disarms the server and requests damping.

In another terminal, with the Quest publisher still running, source ROS and
start with every translation axis disabled. The bridge waits for three
advancing nonzero `time_count` values and at least 5 mm of Quest XYZ travel
within one second before contacting the guarded server. Hold the right index
trigger and move the tracked controller gently to supply fresh poses. The
explicit reset flag allows the bridge to command home after that gate, then
it sends only the held home pose and gripper value:

```bash
source /opt/ros/jazzy/setup.bash
source /home/hjadmin/projects/ARX_X5/ARX_VR_SDK/ROS2/install/setup.bash
cd /home/hjadmin/projects/arx5-sdk
/usr/bin/python3 python/communication/quest_arx5_bridge.py \
  --mode control --side right --model X5_umi --interface can0 \
  --reset-to-home --scale 0.01 --xyz-map 0,0,0 \
  --speed 0.002 --radius 0.002
```

Only after verifying the Quest and robot directions with live observations,
stop both processes, restart the guarded server, and replace the axis map with
one enabled axis, for example `--xyz-map 0,y,0` if robot -Y is verified as
the intended body-right direction. Repeat for the
other axes before combining them. Control mode requires explicit mapping,
scale, speed, radius and interface; it caps this initial stage at 0.01 m/s and
a 0.02 m home radius. The recommended first trial is only 2 mm because offline
IK on this `X5_umi` model rejected some -X targets 4 mm from zero-joint home;
offline IK does not prove physical clearance or tracking. Source jumps over
0.1 m and counter resets clear the startup motion gate; jumps also reanchor
the target. Stale poses hold the last target, and a measured XYZ error over
6 mm stops the bridge. RPY and gripper remain at their reset values. A stopped
bridge leaves the guarded server's watchdog in charge; do not leave a powered
arm unattended.

### Quest Dual-Arm Recording (Steps 12-14, Pending)

The existing `examples/record_episodes.py` records one leader/follower pair
and at most two named camera views; it is not yet a Quest dual-arm recorder.
Its camera capture and HDF5 writer can be reused after both arms are validated.
On 2026-09-30, Vin 1 (`/dev/video0`) delivered a live overhead tabletop view,
while Vin 2/3/4 returned only constant-gray frames despite reporting 60 FPS.
Both wrist feeds need valid images before three-view collection can be verified.

### Quest VR Connection Diagnostics

The inspected ARX VR APK contains Unity ROS-TCP code, not the ZMQ joint
protocol. Its serialized `ROSConnection` destination is
`192.168.31.137:10000`. This Quest was observed connecting from
`192.168.31.107` and sending both arm topics. Sharing Wi-Fi alone is not a
substitute for seeing an actual TCP connection and increasing message counts.

To inspect raw packets instead of publishing ROS topics, stop
`quest_ros_tcp.py`, then run the standard-library-only diagnostic receiver
from `python/`:

```bash
python communication/vr_receiver.py --protocol ROS2
```

Open the APK on Quest. Look for `Connected`, `__publish` registrations, and
increasing message counts for `ARX_VR_L` and `ARX_VR_R` (possibly
with leading slashes), as observed from the APK. Its ROS2 `arm_control/PosCmd`
messages are 240 bytes: a four-byte CDR header, 25 float64s and nine int32s.
The receiver displays decoded `pos_cmd` fields, including pose, raw gripper,
mode1/mode2, temporary arrays and time_count. Gripper values are not SDK meters;
the observed raw range was 0-5 and must not be passed directly to `set_joint_pos`.
The output also includes the advertised message type and up to 128 payload
bytes in hex, at most once per second per topic. Use Ctrl+C
to stop. It listens on all IPv4 interfaces at port 10000; use only on a
trusted LAN. A connection idle for 15 seconds is closed and may reconnect.
The default handshake advertises ROS1; use `--protocol ROS2` only for an APK
built for ROS2. This is a diagnostic subset of the
[Unity ROS-TCP protocol](https://github.com/Unity-Technologies/ROS-TCP-Endpoint),
not a ROS node; it does not provide subscriptions or services.

This receiver never opens CAN, connects to ZMQ, resets, or moves the arms.
It is **not a motion bridge**.
Do not interpret raw VR pose values as joint positions.

### Joint Position Server

From `python/` in the `arx5` environment, replace the Cartesian server with:

```bash
python communication/zmq_joint_server.py X5_umi can0 X5 can1 --port 8765 --port 8764 --timeout-action hold
```

For this dual-arm setup, can0/8765 is the right arm and can1/8764 is the left
arm. Stop existing controllers on those CAN buses first. The server uses the
local SDK and the same `gripper_open_readout=-3.37472` calibration as
`zmq_server.py`. Verify that calibration for each physical arm.

The service binds to localhost by default; use `--bind` for a trusted remote
client. The SDK protocol uses Python pickle and is not suitable for untrusted
networks. Each arm is an independent process; this does not synchronize the
two arms' motion clocks.

| Command | Request data | Reply data |
| --- | --- | --- |
| `GET_STATE` | `None` | Existing state fields plus `armed`, `holding`, `timeout_action` and `emergency` |
| `RESET_TO_HOME` | `None` | `"OK"`; moves home, opens gripper, enables joint commands |
| `SET_JOINT_POS` | `{"joint_pos": six_joint_angles, "gripper_pos": width}` | Robot state |
| `HOLD_POSITION` | `None` | `"OK"`; holds measured joints and gripper, disables new joint commands |
| `SET_TO_DAMPING` | `None` | `"OK"`; disables joint commands |

Angles are absolute ARX5 joint positions in radians, in SDK motor order;
gripper width is in meters (0 closed, SDK `gripper_width` fully open).
This endpoint takes one target, not a trajectory chunk. Errors return
`{"cmd": original_command, "data": "ERROR: ..."}` and do not accept the target.

The existing client now supports `set_joint_pos`:

```python
from communication.zmq_client import Arx5Client

right = Arx5Client("127.0.0.1", 8765)
right.reset_to_home()  # Moves the arm and opens the gripper.
state = right.get_state()
right.set_joint_pos(state["joint_pos"], state["gripper_pos"])
```

Send targets continuously after reset. By default, 2 seconds without an
accepted joint target switches to damping and requires another explicit reset.
Use `--timeout-action hold` for VR: an expired command deadline instead holds
the measured position and requires another explicit reset. `GET_STATE` and
rejected commands do not refresh this deadline. An SDK emergency still forces
damping, and server shutdown uses SDK damping/passive behavior rather than
leaving a powered position hold behind.

Targets must be finite and inside SDK joint/gripper limits. Defaults reject
changes exceeding 0.1 rad per joint or 0.005 m gripper width, relative to both
measured state and the previous accepted target. Targets use a 0.1 s SDK linear
interpolation horizon, with additional delta limits based on SDK velocity
limits. These checks are not collision avoidance or a full trajectory safety
guarantee. Use `--help` for `--timeout`, `--duration`, and step limits.

ALOHA joint outputs still need an explicitly validated mapping to ARX5 motor
order, signs, offsets and gripper units. A 14-dimensional dual-arm array alone
does not establish that compatibility.

From the `python/` directory, start one arm with the default port 8765:

```bash
python communication/zmq_server.py X5 can0
```

Start multiple arms with one command. Repeat `--port` once per arm; ports
correspond to the `MODEL INTERFACE` pairs in order:

```bash
python communication/zmq_server.py X5 can0 X5 can1 --port 8765 --port 8766
```

Use the actual model for each arm (for example, `X5_umi`). Duplicate ports
and CAN interfaces are rejected. The default bind address is `0.0.0.0`;
add `--bind 127.0.0.1` for local connections only. Each arm runs in a separate
child process. Ctrl+C or SIGTERM stops all children; if any child exits,
the launcher stops the others and reports an error.

One policy process can connect to both arms:

```python
from communication.zmq_client import Arx5Client

left = Arx5Client("127.0.0.1", 8765)   # can0
right = Arx5Client("127.0.0.1", 8766)  # can1
```

For a remote policy, replace `127.0.0.1` with the robot host's IP address.
Read state and send each arm's action through its corresponding client.
Calls are blocking request/reply operations; separate ports do not guarantee
synchronized execution. The existing protocol requires `reset_to_home()`
before pose commands; this moves the arm and opens its gripper.
These launcher options apply to `zmq_server.py`; the guarded
`zmq_server_agent_safe.py` remains a separate single-arm entry point.

## Safety-Related Configs

 - The safety checks in this controller are implemented through joint position, velocity, and torque limits. However, there is a trade-off between safety and reactiveness when setting these limits. To achieve a reasonable precision for our experiements, **the default values are insufficient to guarantee safety when the robot is close to singularity or the input control signal includes a lot of noise**.
- We highly recommend that users should **apply safety checks before sending control signals** (joint/eef) to the controller
- Users could also modify the limit values of the joint velocity at their early stage of deployment. Please change the config values before instantiating the controller in python, similar to [this example](https://github.com/real-stanford/arx5-sdk/blob/709f7ab7429f97c83e18687e650f3ee77d14719a/python/examples/test_joint_control.py#L31). You need to set the values with a new numpy array, e.g. `robot_config.joint_vel_max=np.array([2,2,2,2,2,2])`, rather than indexing some of the existing values `robot_config.joint_vel_max[0]=2.0`, which will raise an error.

## Major Update (2026.03.20)
- Enable direct pip install for both `x86_64` and `aarch64` platforms, supporting python 3.8~3.14. You may use `pip install arx5-interface` to install the package without any conda / system dependencies. If you need to updated any C++ files, you may also run `wheels/build_wheel_single_ver.sh` to build the wheel for your specific python version and directly install this wheel in pip or uv.
- Set the gripper home position to be the fully opened position. To keep using the closed position as home, you may set the `target_state.gripper_pos` to 0 in `src/app/controller_base.cpp:187`.
- Use relative path for the `urdf_path` in `include/app/config.h` based on the dynamic library loading path. Thanks [Jimmy Wu](https://jimmyyhwu.github.io/) for the PR.

## Update (2026.01.11)
- In some recent robot models, the gripper motor is mounted in a different direction, which will lead to error: "Gripper position error: got xxx but should be in xxx". Please set the `robot_config.gripper_open_readout` to a negative number and it should work normally. According to a github issue, a typical readout might be `gripper_open_readout=-3.4` and `robot_config.gripper_width=0.082`. If the readout gripper width is inaccurate, you may also calibrate yourself in `python/examples/calibrate.py`

Other Update Logs: Please refer to [update_logs.md](update_logs.md) for more details.

## Features
- Run without ROS
- No `sudo` requirement for building the library (all dependencies are managed under conda environment, thanks to [Cheng Chi](https://cheng-chi.github.io/))
- Simple python interface with complete type hints (see `python/arx5_interface.pyi`, please include it into your vscode config `"python.analysis.extraPaths"`)
- Joint controller runs at 500Hz in the background (motor communication delay ~0.4ms)
- Cartesian space controller with keyboard and SpaceMouse tele-operation and teach-replay (thanks to [Cheng Chi](https://cheng-chi.github.io/))
- Control multiple arms in the same process through C++ multi-threading (much better than Python multi-processing)

## Build & Install

### pip install (Recommended)

You may use `pip install arx5-interface` to install the package without any conda / system dependencies. If you have already build the package inside the conda environment, you need to delete the existing `.so` file under `python` folder to ensure the pip wheel is being used.

If you encounter error "ERROR: No matching distribution found for arx5-interface", please upgrade pip by `pip install --upgrade pip` and try again.

If you need to updated any C++ files, you may also run `wheels/build_wheel_single_ver.sh` to build the wheel for your specific python version and directly install this wheel. 

If you only need to update the config values, you don't need to change any C++ code. Please refer to `python/examples/test_joint_control.py` and `python/examples/spacemouse_teleop.py` to use `robot_config` and `controller_config` before instantiating the controller.

### Build from source (Legacy)

We set up a conda environment for all the cmake dependencies, so no system package is required. If you want to run `cmake` and `make` after modifying the C++ source files, please make sure you are under the created conda environment (`arx-py310` etc.).  

We recommend [mamba](https://github.com/conda-forge/miniforge?tab=readme-ov-file#install) for creating conda environments, which takes only about 1min. You can also use `conda`, but it takes significantly longer (~10min).

``` sh
mamba env create -f conda_environments/py310_environment.yaml
# if you do not have mamba, you can also use conda, which takes significantly longer
# Currently available python versions: 3.8, 3.9, 3.10, 3.11, 3.12
conda activate arx-py310
mkdir build && cd build
cmake ..
make -j
# At this point, you should be able to run test scripts below.
```
``` sh
# To install the C++ package your system, run:
make install
```

## EtherCAT-CAN setup

Use a USB cable to power the EtherCAT-CAN adapter and an ethernet cable to connect it to your computer. After running `ip a` in your terminal, you should find the interface name, usually `eth.` (existing ethernet port) or `en..........` (additional USB-Ethernet adapters).

Then you should enable the ethernet access of your Python interpreter (usually in your `bin` folder). Note that `which python` usually gives you a symbolic link (say `~/miniforge3/envs/arx-py310/bin/python`) and doesn't work in this case. You need to find out the actual file (usually `python3.x`). 
```sh
mamba activate arx-py310
ls -l $(which python)
sudo setcap "cap_net_admin,cap_net_raw=eip" your/path/to/conda/envs/arx-py310/bin/python3.10
```

To run C++, you need to enable the executable every time after compiling. You also need to update the C++ scripts with the correct interface.
```sh
sudo setcap "cap_net_admin,cap_net_raw=eip" build/test_cartesian_controller
sudo setcap "cap_net_admin,cap_net_raw=eip" build/test_joint_controller
```

## USB-CAN setup

You can skip this step if you have already set up the EtherCAT-CAN adapter.

``` sh
sudo apt install can-utils
sudo apt install net-tools
```

There are 2 popular firmware types of usb-can adapter, `SLCAN` and `candleLight`. After plugging the adapter, you can find out the correct firmware type by the following rules:
- Run `ls /dev/ttyACM*`. If there is a **new** `/dev/ttyACM*` device (where * is a number), this adapter is using `SLCAN` firmware
- Run `ip a`. If there is a **new** `can*` interface (where * is a number), this adapter is using `candleLight` firmware.

### For adapters using SLCAN framework
Get serial number by:
``` sh
udevadm info -a -n /dev/ttyACM* | grep serial
# Replace the * by the actual number if there are multiple ttyACM devices connected to your computer.
```
You will get something like:
```
ATTRS{serial}=="209738924D4D"
ATTRS{serial}=="0000:00:14.0"
```
Then edit CAN rules file:
``` sh
sudo vim /etc/udev/rules.d/arx_can.rules
```
Copy and paste the following, and replace the serial number with yours. If you are registering multiple adapters, you can use other `SYMLINK` names (e.g. `arxcan1`) and make sure the following commands are updated accordingly.
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="209738924D4D", SYMLINK+="arxcan0"
```

Finally, activate CAN connection by: (**the second line should be run every time after connection**)
``` sh
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo slcand -o -f -s8 /dev/arxcan0 can0 && sudo ifconfig can0 up
```

Alternatively, if you want not to run the second line everytime, you can also setup a system service:

```sh
sudo vi /etc/systemd/system/arxcan-setup.service
```

Copy the following content to the service file

```service
[Unit]
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'slcand -o -f -s8 /dev/arxcan0 can0 && ip link set can0 up'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

And activate the service

```
sudo systemctl daemon-reload
sudo systemctl enable arxcan-setup.service
sudo systemctl start arxcan-setup.service
sudo systemctl status arxcan-setup.service
```

### For adapters using candleLight framework
After plugging the adapter and running `ip a`, you should immediately find a can interface (usually `can0`). If you only have one arm, simply run 
```sh
sudo ip link set up can0 type can bitrate 1000000
```
and you are good to go. You should run it **every time** after connecting a usb-can adapter.
If you have multiple arms and you want to fix the CAN interface name mapping for each arm, you need to register the adapter to the CAN rules:
```sh
sudo dmesg # Find the idVendor, idProduct and serial number of your adapter
sudo vim /etc/udev/rules.d/arx_can.rules
```
and then append the following line to your `arx_can.rules`. Make sure you've replaced the serial number of your adapter and your desired CAN name.
```
SUBSYSTEM=="net", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="0040001E3730511620323746", NAME="can0"
```
Run the following line to update the changes
```sh
sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger
```
Finally, reconnect your adapter and run (**This line should be run every time after plugging the usb-can adapter**)
```sh
sudo ip link set up can0 type can bitrate 1000000
```


## Spacemouse setup (for Cartesian control)
All the configurations are tested using 3Dconnexion spacemouse. You can skip this step and use keyboard to test Cartesian control.
```sh
sudo apt install libspnav-dev spacenavd
sudo systemctl enable spacenavd.service
sudo systemctl start spacenavd.service
```

## Test scripts

Arguments for `test_joint_control.py`, `keyboard_teleop.py`, `spacemouse_teleop.py` and `teach_replay.py`: 
- (required) model: `X5` (silver and black) or `L5` (all black metal with blue or red LED light). **Choosing the wrong model may lead to dangerous movements!**
- (required) interface: `can0`, `enx6c1ff70ac436` etc. (run `ip a` to check your interface name)
- (optional) urdf_path `-u`: by default `../models/arx5.urdf`

```bash
cd python
python examples/test_joint_control.py X5 can0 # replace X5 with L5 for the other model 
python examples/test_bimanual.py # For two X5 arms using can0 and can1, each arm will act the same as test_joint_control.py
python examples/keyboard_teleop.py X5 can0
python examples/spacemouse_teleop.py X5 can0
python examples/teach_replay.py X5 can0
```
To use python sdk from other directories, please make sure `./python` is in `$PYTHONPATH` and `./lib/x86_64` or `./lib/aarch64` (depend on your computer system) is in `$LD_LIBRARY_PATH`. 

``` sh
export PYTHONPATH=$PYTHONPATH:/path/to/your/arx5-sdk/python
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/your/arx5-sdk/lib/your_arch
```

After compiling the `arx5_interface` pybind dynamic library (usually `python/arx5_interface.cpython-version-arch-linux-gnu.so`), you can run it under other python environments (need to be the same python version as the one you built).
 
## Projects Using this Repository
- **[UMI-on-Legs](https://umi-on-legs.github.io/)**
- [UMI](https://umi-gripper.github.io/): [deployment code](https://github.com/real-stanford/umi-arx)
- [UVA](https://unified-video-action-model.github.io/): [deployment code](https://github.com/real-stanford/umi-arx/tree/uva)
- [Vision-in-Action](https://vision-in-action.github.io/)
- [Latent Policy Barrier](https://latentpolicybarrier.github.io/)
- [DynaGuide](https://dynaguide.github.io/)
- [Minimalist Compliance Control](https://minimalist-compliance-control.github.io/)
- [Gated Memory Policy](http://gated-memory-policy.github.io/)

More cool projects on the way!

## Citation
If you find this repo helpful, please cite our corresponding paper [UMI-on-Legs](https://umi-on-legs.github.io/)
```
@inproceedings{ha2024umilegs,
  title={{UMI} on Legs: Making Manipulation Policies Mobile with Manipulation-Centric Whole-body Controllers},
  author={Huy Ha and Yihuai Gao and Zipeng Fu and Jie Tan and Shuran Song},
  year={2024},
  booktitle={Proceedings of the 2024 Conference on Robot Learning},
}
```
