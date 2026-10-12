# 新机器真机环境配置

在 Linux 主机配置 SDK 与外设。本文的离线块不创建机器人控制器；标注“电机操作”的步骤会初始化/驱动机械臂。已有本机环境直接参考 [快照](current-machine.md)，无需重装系统服务。

<a id="software"></a>
## 软件安装与 SDK 选择

模型推理与机器人控制推荐使用独立环境：推理需要的 PyTorch/CUDA 不必装入 `arx5`。通过本机 ZMQ 连接；仅用 Python 客户端的模型环境不需要加载 SDK。需要 FK/IK 的模型桥接进程则必须安装相同模型配置的 SDK。

### 本地补丁版（本仓库真机入口）

新主机以仓库 YAML 创建环境，名称自定。先确认 `conda` 或 `mamba` 已存在；两者选一种：

```bash
conda env create -n arx5 -f conda_environments/py39_environment.yaml
conda activate arx5
conda install -c conda-forge 'eigen>=3.3,<4'
```

默认 YAML 的 eigen 未锁 major，而预编译 `libsolver.so` 需要 Eigen 3.x；CMake 已阻止 Eigen 4。SpaceMouse bridge/Quest 使用 SciPy，采集/预览使用 OpenCV：

```bash
python -m pip install 'scipy<1.14'
python -m pip install -r python/recording/requirements.txt
# 仅需要 iPhUMI USB 预览时安装
python -m pip install pypeertalk==0.1.2
```

上述 SciPy 上限面向此处 Python 3.9/NumPy 1.x 环境，不代表所有 Python 版本的通用限制。其他 Python 版本选择仓库相应 YAML，仍要约束 Eigen 3.x。aarch64 的 conda SOEM 包缺失时按 YAML 注释从 SOEM 源码安装至该环境，不能直接复制 x86_64 库。

从仓库根目录构建（需要 Eigen3 系统包时先安装 `libeigen3-dev`）：

```bash
sudo apt install libeigen3-dev
conda activate arx5
cmake -S . -B build -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build build --target arx5_interface -j 4
```

可以将 `Eigen3_DIR` 改成环境内经核验的 Eigen 3.x CMake 目录。构建成功会把扩展直接写入仓库 `python/`。更换 Python、架构、编译器或前缀时使用新构建目录，避免复用旧 cache。CMake 最低 3.16.3，C++14；依赖 KDL、kdl_parser、fmt/spdlog、Boost、pybind11、SOEM、Threads，硬件/solver 库已随仓库提供但没有完整实现源码。

配置完成的检查（无 CAN）：

```bash
cd python
python -B - <<'PY'
import sys
from pathlib import Path
import arx5_interface as sdk
print("python:", sys.executable)
print("sdk:", sdk.__file__)
assert hasattr(sdk.Arx5JointController, "is_emergency")
assert hasattr(sdk.Arx5CartesianController, "is_emergency")
for name in ("X5", "X5_umi"):
    r = sdk.RobotConfigFactory.get_instance().get_config(name)
    assert Path(r.urdf_path).is_file(), r.urdf_path
    solver = sdk.Arx5Solver(r.urdf_path, r.joint_dof, r.joint_pos_min,
                           r.joint_pos_max, r.base_link_name,
                           r.eef_link_name, r.gravity_vector)
    print(name, "DOF:", r.joint_dof, "URDF:", r.urdf_path)
print("SDK config/solver check passed; no controller created")
PY
```

通过条件：路径是预期本地扩展，两种控制器 emergency API 存在，所需机型 URDF 可找到且 solver 可构造。此检查不证明 CAN/电机有效；更换机型要改检查列表。

### pip 发行版（通用 SDK 用途）

上游支持 `python -m pip install arx5-interface`，不需要构建依赖；但不能假设 PyPI wheel 包含此分支的故障锁存、接触保持或其他补丁。新真机 agent 接入默认选择本地构建，并核对 API 和源码版本。若确实使用 wheel，应在新环境和仓库外检查 `sdk.__file__`，避免仓库 `.so` 遮蔽它；不要无检查删除当前本地扩展。需要制作修改后 wheel 时参考 [wheel 脚本](../../wheels/build_wheel_single_ver.sh)，其 Docker 流程与本地 CMake 构建是两条路径。

联网失败时按项目经验先用 `http://127.0.0.1:7897`，再比较直连，保留状态码及原始错误，不改全局配置。pip 可显式 `--proxy`；conda/git 使用仅该命令生效的环境变量。模型服务的 localhost 通信应排除代理（例如 `NO_PROXY=127.0.0.1,localhost`）。

<a id="can"></a>
## CAN 与设备权限

先盘点，不改接口：

```bash
lsusb
ip -brief link
ls -l /dev/serial/by-id
pgrep -af 'slcand|zmq_.*server|leader_follower|record_episodes|drag_mode|teleop'
ss -lntp
```

同时出现 ttyACM 与 can 接口未必是 candleLight：SLCAN 已有 slcand 时也会呈现 can 接口，结合 USB ID、序列号与进程判断。CAN 名字和左右关系不得靠插入顺序猜测。

### SLCAN

```bash
sudo apt install can-utils
udevadm info --query=property --name=/dev/ttyACM0
udevadm info --attribute-walk --name=/dev/ttyACM0
```

对每个实物适配器记录序列号及连接的臂。在 `/etc/udev/rules.d/arx_can.rules` 写入实际 VID/PID/序列号，例如本机可用 [快照中的规则](current-machine.md#devices)；新机器替换参数。稳定 symlink 比 ttyACM 序号可靠。

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/arxcan0 /dev/arxcan1
```

仅在没有管理该设备的 slcand/service、目标臂控制程序已退出时启动所需接口：

```bash
sudo slcand -o -f -s8 /dev/arxcan0 can0
sudo ip link set can0 up
# 第二臂按需启动
sudo slcand -o -f -s8 /dev/arxcan1 can1
sudo ip link set can1 up
ip -details -statistics link show can0
ip -details -statistics link show can1
```

`-s8` = 1 Mbps。SLCAN 的内核接口可显示 `bitrate 0`。要自动启动，可沿用 [本机 oneshot service](current-machine.md#devices)，然后 `systemctl daemon-reload`、`enable`、`start`。这个服务不是热插拔恢复方案：`active (exited)` 不能证明 USB 重插后两接口已恢复，必须看 symlink、slcand 和实际 link。不要每次启动控制器都再次运行 slcand。

### candleLight

新插入的适配器由内核直接产生 can 接口，不需要 slcand。单臂、无正在运行的控制器时：

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details -statistics link show can0
```

多臂可按根 README 的 net udev 规则用序列号固定名称；确认真实 VID/PID，不能套用本机 SLCAN 的 tty 规则。接口已 up 但需要改 bitrate 时先停止控制器、再 down/configure/up。USB 重插后重新核对状态。

### EtherCAT-CAN

使用 USB 供电及以太网连接适配器，参数 `INTERFACE` 是实际网络口，如 `en...`，不是 can0。按根 README 授予 Python 真实可执行文件 `cap_net_admin,cap_net_raw=eip`；`which python` 可能是 symlink，先 `readlink -f`，只对选定文件 `setcap`。编译后 C++ 示例可执行文件需重新授予 capability。仅此方式需要原始以太网访问；SLCAN/candleLight 普通 Python 操作 SocketCAN 通常不需要给解释器 setcap。当前主机使用 SLCAN，本次未验证 EtherCAT。

CAN 通过条件：序列号到实物臂的映射已确认，目标接口 up，适配器配置为 1 Mbps，没有重复 writer；在后续已授权控制过程中反馈推进、错误/丢包计数没有异常增长。历史 RX/TX 数量本身不是在线反馈证明。

<a id="robot-config"></a>
## 机型与控制参数

权威定义：[config.h](../../include/app/config.h)，[Python 类型/API](../../python/arx5_interface.pyi)。当前工厂模型：`X5`、`X5_umi`、`X5_iphumi`、`L5`、`L5_umi`、`L5_iphumi`（6 DOF），`X7_left`、`X7_right`（7 DOF）。`models/` 中有文件不等于工厂已注册，例如 `_new` / `X5_no_gripper` 不是现成可传入的名称。

| 参数 | 配置原则 |
| --- | --- |
| `joint_dof`、`motor_id`、`motor_type` | 6 DOF 电机 ID 顺序默认 `[1,2,4,5,6,7]`，7 DOF `[1,2,3,4,5,6,7]`；数组索引是关节序号，不是电机 ID |
| X5/L5 | X5 前三关节 EC_A4310，L5 前三关节 DM_J4340，后三关节 DM_J4310；夹爪 DM_J4310 ID 8 |
| `joint_pos_min/max`、`joint_vel_max` | 必须匹配实物和安装；SDK 6 DOF 默认速度约 5~5.5 rad/s，不是初次部署建议速度 |
| `urdf_path`、link names | FK/IK/重力都依赖此处；默认 `base_link` 到 `eef_link`，`*_umi`/`*_iphumi` 不是可任意互换的标签 |
| `gravity_vector` | 正常安装 `[0,0,-9.807]`；翻转/侧装应转换到机器人基座坐标系，倒装示例 `[0,0,9.81]` |
| `gripper_width`、`gripper_open_readout` | 实测全开宽度 m 与全开电机角度 rad（保留正负号），参见标定 |
| `controller_dt`、`background_send_recv` | 默认 0.002 s / 开启；SDK 在后台 500 Hz 收发，Python/模型不用以相同频率推理 |
| `gravity_compensation` | 默认开启，使用 URDF 逆动力学；拖动 kp=0 仍会输出重力力矩 |
| `default_preview_time` | joint 默认 0，Cartesian 默认 0.1 s；直接 SDK 调用的时间戳应使用该控制器时钟 |

配置要在构造控制器之前设置。getter 和工厂返回副本，事后修改 getter 不影响在运行的控制器。配置中的 NumPy/Eigen 属性需用整个新数组赋值；`Gain.kp()`、`JointState.pos()` 等方法返回的数组可切片写入，但保留所属对象并按需 `.copy()` 保存快照。

```python
# 配置示例；这一段本身不创建电机控制器
import numpy as np
import arx5_interface as sdk
robot = sdk.RobotConfigFactory.get_instance().get_config("X5_umi")
robot.gripper_open_readout = -3.45369  # 仅本机右臂；新硬件需标定
robot.joint_vel_max = np.full(robot.joint_dof, 0.3)
config = sdk.ControllerConfigFactory.get_instance().get_config(
    "joint_controller", robot.joint_dof)
config.gravity_compensation = True
# 电机操作从 sdk.Arx5JointController(robot, config, "can0") 开始
```

新增硬件机型时，从最接近配置派生，修改 URDF/DOF/限位/电机/夹爪/重力；只改 Python 配置不需重编译，新增工厂条目/C++ 绑定则需要。未知电机不能假设受预编译硬件库支持。solver 的日志显示运动学到 `eef_link`，逆动力学到最后活动关节 `link6/link7`；root-link inertia 也有 KDL 警告。增加固定末端工具质量时要核验重力输出，不能只改固定 link 的质量就宣称补偿正确。现有 `calibrate.py` 固定取 6 DOF ControllerConfig，不能直接用于 X7；先改为从 robot_config 获取 DOF 并检查实物零位。

<a id="calibration"></a>
## 夹爪标定（电机操作）

先结束占用该 CAN 的控制程序、支撑机械臂、清除夹爪物体。标定关闭重力补偿，机械臂可能下坠。换机械臂/夹爪后重新做；仅调增益不能修复错误零点。

```bash
cd python
python examples/calibrate.py X5_umi can0
```

默认入口是 `calibrate_gripper`：手动轻轻闭合指尖，回车重设电机零点；手动完全张开，回车读取 `Fully-open joint position readout`。它不会自动持久化全开角度或测量实物宽度。记录带符号角度和实测宽度，将其应用到所有会构造该臂控制器的入口（见 [覆盖表](current-machine.md#calibration-overrides)）。脚本当前提示更新 config.h，但 CLI/Python 构造前覆盖同样可用。

不重设零点的验证（下面数值仅本机右臂）：

```bash
python examples/calibrate.py X5_umi can0 --verify-zero \
  --gripper-open-readout -3.45369
```

按 closed_1 / fully_open / closed_2 的提示轻轻操作，保存三行 `[zero-check]`。`angle_rad = SDK开度 / gripper_width * gripper_open_readout`；`width_mm` 是模型换算，不是尺子测量。两次闭合不接近零提示零偏，重复性差提示间隙或操作问题，不能用挤压变形设零。正常换算为 `width = motor_angle / open_readout * gripper_width`。

若一开始就因 `Gripper position error` 无法构造控制器，先对照实物检查符号、当前开度和已知正确读数；不要反复用默认参数重启，更不能放宽保护来硬跑标定。关节零位只在明确更改零点的任务中操作；`calibrate_joint` 当前未接到默认 CLI，需显式选择函数，参数是 0 起始关节索引且必须有实物零位依据。

<a id="peripherals"></a>
## 外设与无 CAN 验收

### SpaceMouse

```bash
sudo apt install libspnav-dev spacenavd
sudo systemctl enable spacenavd
sudo systemctl start spacenavd
cd python
python -B communication/spacemouse_joint_bridge.py --dry-run --seconds 10
```

YAML 已包含 Python spnav。dry-run 只读输入并解 IK，不连接机器人；通过条件是鼠标轴/按钮响应且无设备丢失错误。Universal Receiver 在本机使用 spacenavd，其他型号需重新核验驱动。`--check` 则会访问已启动 server 的 GET_STATE，区别于 dry-run。

### V4L2 与 iPhUMI

```bash
sudo apt install v4l-utils usbmuxd
ls -l /dev/v4l/by-id
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
# 只打开相机；没有 CAN 控制器；SSH 用 --no-preview
cd python
python -B examples/record_episodes.py --check-cameras --no-preview
```

检查其他设备使用 `--wrist-camera /dev/v4l/by-id/实际路径`。如权限失败看 `id`、节点 owner/group；需要时给用户正确的 video 权限并重新登录，不用 root 启动整套模型。通过条件是连续有效新帧，实测尺寸/FPS、图像内容、视角顺序正确。当前采集默认 MJPG 1920x1080/60 FPS，经等比缩放居中裁剪成为 256x256 RGB；不要把 UI 叠字画面作为模型输入。

iPhone 解锁并信任主机，iPhUMI 选 Deployment USB streaming：

```bash
python -B -c 'import pypeertalk; print(pypeertalk.get_connected_devices())'
```

设备列表非空只证明能枚举，需预览检查 main/ultrawide 实际画面。USB 主视角通过 PeerTalk 传输，不是 `/dev/video*`；当前 HDF5 recorder 的 `--main-camera` 只接受 OpenCV/V4L2 类设备，不能把手机 USB 端口直接填进去。

## 配置交付

留下 [索引中的交接记录](README.md)，并依 [操作](operations.md) 和 [模型接入验收](model-integration.md#migration) 完成所选路径。记录 SDK 路径、git 版本、有效标定和真实设备映射；离线检查、相机检查、实机动作验收分别注明。没有实际控制反馈时只报告软件/外设配置完成。
