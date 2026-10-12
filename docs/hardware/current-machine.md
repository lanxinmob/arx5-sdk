# 当前真机配置快照

此页是机器特定信息，不是可移植默认值。2026-10-12 以只读系统查询、Python 导入和源码核对更新，基线 `d93325a`。实物左右与标定来自已有仓库记录，未在本次重新运动确认。

<a id="environment"></a>
## 软件环境

| 项目 | 已核验结果 |
| --- | --- |
| 仓库 | `/home/hjadmin/projects/arx5-sdk` |
| OS / 架构 | Ubuntu 24.04.4 LTS / x86_64（本地扩展文件架构） |
| 环境 | `conda activate arx5`，`/home/hjadmin/miniconda3/envs/arx5/bin/python` |
| Python | 3.9.25 |
| 本地 SDK | `python/arx5_interface.cpython-39-x86_64-linux-gnu.so` |
| 补丁 API | `Arx5JointController.is_emergency` 存在 |
| Python 包 | NumPy 1.26.4、pyzmq 27.0.2、click 8.1.8、SciPy 1.13.1、h5py 3.10.0、opencv-python 4.8.1.78、pypeertalk 0.1.2、spnav 0.9 |
| URDF 解析 | `X5_umi` 配置返回仓库 `models/X5_umi.urdf` |
| 构建目录 | 核验时仓库内无 `build/CMakeCache.txt`；已存在 `.so`，不能据此证明它与所有当前 C++ 源码一致 |

在 `python/` 内运行加载检查：

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
python -B -c 'import sys, arx5_interface as s; print(sys.executable); print(s.__file__); print(hasattr(s.Arx5JointController, "is_emergency"))'
```

<a id="devices"></a>
## 设备映射

| 用途（已有操作记录） | 机型 | CAN / 端口 | USB 序列号 / 稳定设备 |
| --- | --- | --- | --- |
| 右臂 / 主从模式的从臂 | `X5_umi` | `can0` / joint ZMQ `8765` | `003500344C48570920373330`，`/dev/arxcan0`，当次 `ttyACM1` |
| 左臂 / 主从模式的主臂 | `X5` | `can1` / joint ZMQ `8764` | `004100295330530320323656`，`/dev/arxcan1`，当次 `ttyACM0` |
| 腕部采集相机 / Quest 默认右腕 | Vin 1 | V4L2 | `/dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0`，当次 `/dev/video0` |
| Quest 默认左腕 | Vin 4 | V4L2 | `/dev/v4l/by-id/usb-MACROSILICON_Vin_4_20250814-video-index0`，当次 `/dev/video6` |
| SpaceMouse | Universal Receiver | USB `256f:c652` | spacenavd；本机已有记录认为官方 Linux 3DxWare 1.8.0 不支持此接收器 |
| iPhUMI | iPhone USB | 手机服务 `5555` main / `5556` ultrawide | pypeertalk + usbmuxd，手机须解锁、信任主机并启用 Deployment USB streaming |
| Quest 入流 | `/ARX_VR_R`、`/ARX_VR_L` | PC TCP `10000` | README 记录的 PC Wi-Fi IP `192.168.31.137`；启动前用 `ip -brief address` 重新核对 |

两个 CAN 适配器均为 SLCAN，USB VID/PID `16d0:117e`。系统现有规则：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="003500344C48570920373330", SYMLINK+="arxcan0"
SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", ATTRS{serial}=="004100295330530320323656", SYMLINK+="arxcan1"
```

`/etc/systemd/system/arxcan-setup.service` 的启动命令为：

```bash
slcand -o -f -s8 /dev/arxcan0 can0 && ip link set can0 up && slcand -o -f -s8 /dev/arxcan1 can1 && ip link set can1 up
```

本次查询两接口均 `UP / LOWER_UP / ERROR-ACTIVE`，RX/TX 错误计数为 0。SLCAN 的 `ip -details` 显示 `bitrate 0`，速率实际由 `slcand -s8` 指定为 1 Mbps，不要因此按 candleLight 方式重设接口。`arxcan-setup`、`spacenavd`、`usbmuxd` 均 active；存在两路 slcand 进程。查询时未发现已知遥操/采集/ZMQ 服务进程，8764/8765/10000 未监听；这只是当次状态，下一次操作仍要查占用。

相机的设备存在不等于有图像。采集手册记录 2026-10-06 Vin 1 约 59.8 FPS，Vin 2/3/4 为全像素 7 的无信号画面；根 README 的 Quest 三视图描述反映较早布局。2026-10-12 本次分别检查 Vin 1 与 Vin 4，两路均能打开并报告 MJPG 1920x1080/60 FPS，但都返回全像素 7 的无信号画面，检查退出码 1。Vin 2/3 本次未检查。因此当前不能宣称任一腕部相机已可用；使用前确认输入信号后重新 `--check-cameras`。`video-index1` 不作为默认采集节点。

可复现的失败检查（无 CAN，`--wrist-camera` 换成 Vin 4 路径可检查另一节点）：

```bash
python -B examples/record_episodes.py --check-cameras --no-preview \
  --wrist-camera /dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0
```

当次原始错误：`RuntimeError: Cameras not delivering new frames: cam_wrist. ... near-black/no-signal frame (pixel range 7..7); check camera input/channel`。该证据能证明无有效输入，不能区分源相机未供电、线缆或输入选择等原因；本次没有改动物理接线或尝试启动机器人。

<a id="calibration-overrides"></a>
## 标定与入口覆盖差异

每个入口可能覆盖工厂配置。创建控制器前的最后赋值才是有效值；命令只换 `MODEL` 不会自动换掉脚本硬编码的标定。
旧键盘、teach/replay、关节/力矩测试等的额外覆盖值与使用限制见 [其他入口表](operations.md#other-examples)，不应拿其默认值验收本机。

| 入口 | 当前实际覆盖 | 迁移方式 |
| --- | --- | --- |
| `RobotConfigFactory` | X5/X5_umi 全开读数 `-3.25952`；宽度分别 `0.088/0.086 m` | Python 构造前赋值，或修改工厂并重编译 |
| `leader_follower_teleop.py`、`record_episodes.py` | 主臂 `-3.2969`，从臂 `-3.45369`；主重力倍率 1.2、从 1.0 | `-g`、`--follower-gripper-open-readout`、两臂 gravity-scale；0 读数表示用机型默认值 |
| `spacemouse_teleop.py` 直连 | 全开读数默认 0，表示保留机型默认值 | 本机右臂显式 `--gripper-open-readout -3.45369`，左臂 `-3.2969` |
| `zmq_joint_server.py` | 所有机型硬编码 `-3.37472` | 修改 `serve()` 中构造前赋值，双臂时按 model/interface 分配；当前没有标定 CLI |
| `zmq_server_agent_safe.py` | 所有机型硬编码 `-3.37472` | 修改构造中的 `robot_config` 赋值；另设 `--gripper-max` 匹配宽度 |
| `zmq_server.py` 旧服务 | 首次构造硬编码 `-3.37472`；超时后重建走机型简写构造，可能恢复工厂值 | 接新模型优先采用 documented joint/guarded 入口；旧服务要核对重建路径 |
| `drag_mode.py` | 所有臂共用 `-g`，默认 `-3.4`；重力默认 1.0 | 不适合一条命令表达本机两臂不同读数；单臂分别启动或修改构造参数 |
| `calibrate.py` | 默认 `--gripper-open-readout -3.34`，关闭重力补偿 | 验证模式显式传每臂当前值；标定会重设闭合电机零点 |

主从/采集闭合阈值为 `0.001 m`，夹爪 kp 默认 5。已有记录指出实物闭合小缝及重复性仍未完全解决；这些读数不是精度保证。SDK 接触保持在默认阈值 `0.75 Nm` 触发，1 mm 卸载，反向命令超过 0.5 mm 释放；默认 `1.5 Nm` 过流检测仍保留，见 [排障](troubleshooting.md#gripper)。

## 证据与更新

- 系统事实：`/etc/os-release`、`lsusb`、`/dev/serial/by-id`、`/dev/v4l/by-id`、`/etc/udev/rules.d/arx_can.rules`、`/etc/systemd/system/arxcan-setup.service`、`ip -details -statistics link show can0/can1`、`systemctl is-active`、`pgrep -af`、`ss -lntp`。
- 软件事实：本页 Python 导入检查、包 metadata、[配置工厂](../../include/app/config.h) 及各入口的构造代码。
- 历史真机记录：[根 README](../../README.md)、[采集手册](../../python/recording/README.md)。当前夹爪各值还可追溯提交 `2e01969`，主臂重力倍率提交 `e300b7f`，SDK 接触保持提交 `516a9d4`。
- 这页不记录密钥、账户或模型服务凭据。USB 重插、网段变化、脚本覆盖修改后更新此页，并在实验记录中保留当次实际有效值。

本次离线核验全部 8 个工厂机型：URDF 存在，限位中点 FK -> IK -> FK 位置/旋转误差均小于 `1e-5`，IK status=0，逆动力学输出 finite。SpaceMouse 2 s dry-run 成功打开输入并跑 IK，观察到按钮事件；未验证实物轴方向/跟随。KDL 输出 root-link inertia 不支持的警告，日志显示 `eef_link` 用于运动学、`link6/link7` 用于逆动力学，不能据此假定新增固定工具的质量完整计入重力模型。
