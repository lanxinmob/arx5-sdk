# C++ && Python Controller for ARX5 Robot Arm

## 真机文档与 Agent 索引

人类日常操作请读 [操作者手册](docs/hardware/operator-guide.md)：开机检查、启动命令、按键、采集与退出。

从 [真机操作索引](docs/hardware/README.md) 开始，按顺序完成环境配置、设备核对、标定和控制入口选择。
其中包含 [本机配置快照](docs/hardware/current-machine.md)、[新机器配置](docs/hardware/setup.md)、
[日常操作](docs/hardware/operations.md)、[其他策略模型接入](docs/hardware/model-integration.md) 和
[故障排查](docs/hardware/troubleshooting.md)。命令区分无硬件检查与会驱动电机的操作；本机参数不能作为所有机械臂的默认标定。

## Quest Dual-Arm Teleoperation

On this machine, the right arm is `X5_umi/can0` and the left arm is `X5/can1`.
The Quest app sends both controller streams to `192.168.31.137:10000` over
Wi-Fi. The two commands below are the complete host-side workflow; do not run
another controller on either CAN interface or another receiver on port 10000.
Use the local `arx5` environment and the locally built `arx5_interface` module.

Before using these commands, check the [current calibration overrides](docs/hardware/current-machine.md#calibration-overrides).
The joint server currently applies one hardcoded gripper readout to both arms;
it does not automatically use the per-arm leader/follower calibration.
The preview layout below is a configured layout, not evidence of live camera
signals. The [2026-10-12 check](docs/hardware/current-machine.md#devices) found
no valid video input on either Vin 1 or Vin 4.

In terminal 1, start the joint servers:

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
python communication/zmq_joint_server.py X5_umi can0 X5 can1 --port 8765 --port 8764
```

The joint server enters gravity-compensated drag mode when the client requests
`DRAG`. If an arm tilts up or backward while it is released, lower the
feedforward only for that run, for example:

```bash
python communication/zmq_joint_server.py X5_umi can0 \
  --port 8765 --gravity-scale 0.85
```

Tune this value with the arm supported and in a clear workspace; `1.0` remains
the default. The server clears the active SDK trajectory before `DRAG`, so a
stopped policy cannot move the arm while the episode recorder is closing.

Absolute-joint policy clients should submit a complete future chunk with the
`SET_JOINT_TRAJ` command. Its `joint_pos`, `gripper_pos`, and `time_from_now`
arrays are sent once; the SDK then interpolates the stored trajectory in its
500 Hz controller loop. The UMI policy adapter linearly resamples its 5 Hz
predictions to 100 Hz before sending them. `SET_JOINT_POS` remains available
for single-point teleoperation and homing helpers.

For human-to-policy handoff, the joint server handles `HOLD` by first pinning
the SDK interpolator to the measured joint state, then enabling position gains.
The gains ramp over 300 ms, which prevents the old reset/home command or a
residual hand motion from producing a shock when `c` is pressed.

Pressing Ctrl+C while the joint server is in `DRAG` or `HOLD` first locks the
measured pose when needed, then returns the arm slowly to the SDK home pose and
finally enters damping. If the server has not completed its initial reset, it
does not force a home motion during shutdown.

In terminal 2, start the Quest bridge:

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
python communication/vr_joint_bridge.py --right-model X5_umi --left-model X5
```

Starting the bridge also opens a 4:3 desktop window. The first row shows the
iPhUMI iPhone main camera over USB; the second row shows the left wrist (Vin 4)
and right wrist (Vin 1). The iPhone app must be in Deployment USB streaming mode.
To use the iPhUMI ultrawide camera in the primary view, add
`--primary-camera ultrawide` to the bridge command; `main` is the default.
The ultrawide USB stream uses port 5556 and is included when recording.
The preview reconnects if a camera is unavailable without interrupting arm control.
Closing the window leaves teleoperation running; stopping the bridge closes it.
Use `--right-camera` and `--left-camera` to override the stable V4L2 device paths,
or `--no-preview` when running without a desktop display.
The `arx5` environment needs `pypeertalk==0.1.2` for iPhUMI USB streaming;
it is installed on this machine. On another machine, install it once with
`python -m pip install pypeertalk==0.1.2`.
If the main view says `IPHUMI USB UNAVAILABLE`, unlock and trust the iPhone,
select Deployment USB streaming in iPhUMI, and check that it appears in
`python -c 'import pypeertalk; print(pypeertalk.get_connected_devices())'`.
If the list stays empty, reconnect the iPhone directly to a PC USB port.

Add `--record-video` to the bridge command to save all three views in the 4:3 window
as an MP4 in `~/Videos/arx5/` with a timestamped filename. To choose a path,
use `--record-video /path/to/session.mp4`. Recording ends and the MP4 is finalized
when the preview window closes or the bridge stops. Existing files are not overwritten.

Open or reconnect the Quest app after the bridge starts. Once both hand
streams arrive, the bridge moves both arms to SDK home and opens the grippers.
Keep their full homing paths clear. The Quest app connects over Wi-Fi as a TCP
client to the PC at `192.168.31.137:10000` and publishes ROS2 `PosCmd` messages
for `/ARX_VR_R` and `/ARX_VR_L`. `vr_receiver.py` decodes their ROS-TCP/CDR
payloads. The bridge anchors each current hand pose to the measured arm
end-effector pose, maps subsequent hand movement to an end-effector target, and
uses the SDK `Arx5Solver` to calculate six joint targets by inverse kinematics.
It sends those targets over localhost ZMQ (`8765` right, `8764` left) to the
joint servers. Each server uses the SDK and SocketCAN (`can0`/`can1`) to drive
its motors and returns measured joint feedback for the next bridge update.
Each side trigger controls its gripper. Long-press right A or left X to return
that arm slowly to joint zero while holding its current gripper opening. Once
fresh hand tracking resumes, the bridge realigns that hand to the measured arm
pose and continues teleoperation. If tracking pauses during the button reset,
the bridge stays running and holds that arm at home until tracking resumes.
This is end-effector pose mapping, not human shoulder/elbow retargeting. The
observed Quest pose counter gates arm movement; it is not a trigger signal.

The bridge steps valid joint targets within the SDK joint velocity limits. It
averages five IK joint targets before the velocity step by default. Use
`--joint-average-window 1` to disable averaging or a larger value for more
smoothing at the cost of more lag. Averaging cannot correct missing Quest poses
or unreachable IK targets. The bridge has no additional workspace-radius or
Cartesian speed cap. Unreachable poses hold at the IK boundary. When the bridge
receives measured feedback up to 0.1 rad outside an SDK joint bound, it clips
the planning seed back to that bound and continues commanding within the SDK
limits. Larger excursions stop with the joint number and amount in the error.
When the bridge stops, including Ctrl+C, Quest packet loss outside a button reset,
or a larger measured joint excursion, it asks
each healthy armed server to move slowly to joint zero in two waypoints, keeping
the current gripper opening, then requests damping. A server also attempts this return
after two seconds without accepted targets or when it stops. Return planning
uses about 0.4 rad/s or less per joint. An SDK emergency, lost feedback, or
power loss can prevent a controlled return; damping does not remove power or
prevent collisions.

During the A/X button trial, the installed Quest app produced an undocumented
`PosCmd` pattern: `time_count=1107`, `gripper=0`, and near-zero rotation. The
bridge interprets one second of this pattern as a reset request, so those
packets do not close the gripper. Side-trigger packets with changing gripper
values keep normal gripper control. If both sides show the pattern together,
the bridge holds position instead of treating either as a single-arm reset.
The button reset has been confirmed on powered arms.
If the Quest app changes the packet pattern, the button reset will no longer
be recognized.

The locally patched SDK holds a blocked gripper at its measured contact width
at 0.75 Nm and relieves the command by 1 mm. Open by at least 0.5 mm to release
the hold. The SDK's 1.5 Nm overcurrent emergency remains active.

If the local Python module is missing or the SDK C++ sources change, rebuild
it from the repository root:

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk
cmake -S . -B build -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build --target arx5_interface -j 4
```

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

For direct SpaceMouse control on this machine, stop other controllers on the
chosen CAN interface, activate `arx5`, and run from the `python` directory:

```bash
python examples/spacemouse_teleop.py X5_umi can0
# Left arm:
python examples/spacemouse_teleop.py X5 can1 --gripper-open-readout -3.2969
```

The script uses the local SDK first and applies the gripper calibration before
creating the controller. `--gripper-open-readout` (`-g`) defaults to `-3.45369`
for the current right arm; pass another value to override it, or `-g 0` to use
the model default. These readouts are specific to the current arms;
use `examples/calibrate.py` after changing hardware. Startup immediately moves
the arm to home with the gripper fully open; tracking starts and resumes after
a two-button reset from the fully open gripper target. Keep the home path clear.
A negative measured gripper opening
can indicate an incorrect motor-angle sign; resolve the configuration before
restarting after an SDK emergency.

If a target exceeds joint limits or is unreachable, direct SpaceMouse control
keeps the last valid target without exiting or accumulating rejected input.
Move the mouse back toward reachable poses to resume tracking.

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
