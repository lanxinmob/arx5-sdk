# 真机操作与配置索引

这组文档供配置新机器、接入其他策略模型及接手真机操作的 agent 使用。入口是仓库根 README；不需要依赖对话历史。最后核对日期：2026-10-12，源码基线 `d93325a`。后续代码有变更时以对应源码为准。

人类操作者从 [操作者手册](operator-guide.md) 开始，按开机、模式选择、操作、退出与交接的顺序使用；需要安装、标定或接新模型时再查下面的技术文档。

## 阅读顺序与完成标准

1. [本机快照](current-machine.md)：识别现有两臂、CAN、相机、Python 环境和入口间参数差异。
2. [环境配置](setup.md)：从新 Linux 主机完成依赖、本地 SDK、CAN、外设与夹爪标定。
3. [操作流程](operations.md)：选择唯一控制入口，了解启动、回零、暂停、退出和超时行为。
4. [模型接入](model-integration.md)：把 checkpoint 的观测和动作定义转换成 SDK/ZMQ 合同，再分阶段验收。
5. [故障排查](troubleshooting.md)：按报错、原始证据和实际反馈恢复。

“配置完成”指环境/SDK 路径、设备映射、标定、模型动作合同和所选控制入口均有验收记录。无硬件检查通过不能证明机械臂到位、无碰撞或模型任务成功。本文编写时没有新执行电机控制、回零或标定；实物标定与操作经验来自仓库已有记录，新核验范围见本机快照。

## 操作边界

- `import arx5_interface`、读取配置、构造 `Arx5Solver` 不连接 CAN。构造 `Arx5JointController` / `Arx5CartesianController` 会初始化电机、收发 CAN 并启动后台线程，即使只想读取状态也不是纯只读硬件操作。
- 每个 CAN 接口同时只允许一个控制器。ZMQ server 已持有 CAN 时，遥操/模型客户端只能连接该 server；不能再启动直连示例、采集或标定。同一 server 也只允许一个发动作的客户端。
- 回零会运动，通常还会全开夹爪。保持整段路径空闲并确认供电、固定安装、负载和硬急停。软件阻尼不是断电；不同入口在断流时可能回零或阻尼，见操作流程。
- 本任务交付的是操作文档。接手 agent 执行硬件操作时应依据当次任务授权、现场状态与本文通过条件，不把安装验证等同于允许运动。

## 按任务查找

| 要做什么 / 搜索词 | 文档入口 | 权威源码或原始记录 |
| --- | --- | --- |
| conda、pip、CMake、Eigen ABI、`.so` 加载 | [安装与构建](setup.md#software) | [CMake](../../CMakeLists.txt)、[Python CMake](../../python/CMakeLists.txt)、[环境 YAML](../../conda_environments/py39_environment.yaml) |
| SLCAN、candleLight、EtherCAT、udev、1 Mbps | [CAN 配置](setup.md#can) | 根 README 的适配器章节、[CAN API](../../include/hardware/arx_can.h) |
| 本机序列号、can0/can1、左右臂、Vin | [当前机器](current-machine.md#devices) | 系统 udev/service + 已有 README 记录 |
| 模型名、DOF、电机 ID、URDF、重力、增益 | [机型配置](setup.md#robot-config) | [配置工厂](../../include/app/config.h)、[绑定与类型](../../python/arx5_interface.pyi) |
| 夹爪符号、全开角度、闭合零点、verify-zero | [标定](setup.md#calibration) | [标定入口](../../python/examples/calibrate.py)、[标定实现](../../src/app/joint_controller.cpp) |
| SpaceMouse、spacenavd、双按钮、限位保持 | [鼠标遥操](operations.md#spacemouse) | [直连示例](../../python/examples/spacemouse_teleop.py) |
| Quest、10000、PosCmd、A/X、reset marker | [Quest 遥操](operations.md#quest) | [VR bridge](../../python/communication/vr_joint_bridge.py)、[解码器](../../python/communication/vr_receiver.py) |
| 主从、拖动、重力倍率、力反馈 | [主从与拖动](operations.md#leader-follower) | [主从入口](../../python/examples/leader_follower_teleop.py)、[拖动入口](../../python/examples/drag_mode.py) |
| HDF5、record、RGB、频率、训练格式 | [采集流程](operations.md#recording) | [完整采集手册](../../python/recording/README.md)、[录制入口](../../python/examples/record_episodes.py) |
| keyboard、teach/replay、waypoint、torque、X7、C++ 示例 | [其他入口与前置条件](operations.md#other-examples) | 表中逐项链接到入口源码；含额外硬编码标定 |
| iPhUMI、USB、5555/5556、预览/MP4 | [相机配置](setup.md#peripherals)、[预览](operations.md#preview) | [相机预览](../../python/communication/wrist_camera_preview.py) |
| agent/VLA、ZMQ 合同、absolute/delta、TCP/RPY | [模型接入](model-integration.md#contract) | [joint server](../../python/communication/zmq_joint_server.py)、[guarded server](../../python/communication/zmq_server_agent_safe.py) |
| checkpoint、动作 chunk、跨环境、dry run | [迁移流程](model-integration.md#migration) | [动作 guard](../../python/communication/agent_action_guard.py) |
| timeout、damping、home、emergency、SIGKILL | [退出矩阵](operations.md#lifecycle)、[故障](troubleshooting.md) | [控制器生命周期](../../src/app/controller_base.cpp) |

快捷搜索示例（仓库根目录）：

```bash
rg -n 'gripper_open_readout|gravity_vector|joint_vel_max' include/app/config.h python
rg -n 'RESET_TO_HOME|SET_JOINT_POS|SET_EE_POSE|timeout' python/communication
rg -n 'emergency|contact hold|Over current|Gripper position error' src/app
```

## 交接记录模板

配置另一个模型或主机时，把以下记录放入对应实验目录/模型仓库；不要只写“已跑通”。未测项目标为未测。

```text
日期 / 操作者 / SDK git commit / 工作树改动:
主机 OS / 架构 / 环境路径 / Python / SDK __file__:
机械臂 model / DOF / CAN / 适配器序列号 / 服务进程:
每臂 gripper_width / gripper_open_readout / 零点验证结果:
URDF / base_link / eef_link / 工具变换 / 重力 / 负载:
控制入口 / bind / port / timeout / duration / 关节及工作空间限值:
相机稳定路径 / 视角顺序 / 有效帧 / RGB变换 / 实测FPS / 时钟:
checkpoint 路径与版本 / normalization key / 观测与动作形状:
动作 absolute或delta / joint或EEF / frame / rotation / gripper方向与单位:
模型推理频率 / 执行频率 / chunk长度 / 过期动作处理:
无硬件检查命令、退出码和结果:
现场回零及小步控制的目标、实测反馈、停止结果:
尚未验证的项目 / 已知问题 / 下一次启动命令:
```

维护规则：新增控制入口时同步索引、标定覆盖表和退出矩阵；更换相机/夹爪/工具后重新核对快照及动作合同。测试产物用临时目录，验证完删除，不在仓库新增永久 test 文件。

## 本次文档验证

2026-10-12 SpaceMouse 直连限位保持更新：无硬件替身验证轨迹/单点两模式下的连续 IK 拒绝、位姿与夹爪增量不累积、反向恢复、双按钮回零重置及其他 RuntimeError 继续抛出；脚本语法与 `--help` 通过，临时测试文件已清理。未构造 SDK controller，未验证真机保持或实测到位。

2026-10-12 无电机操作核验：所有本组文档的相对文件链接/显式锚点和 Python 示例语法通过，setup 的离线 SDK 检查实际执行通过；7 个关键入口 `--help` 的文档参数存在；8 个工厂机型离线 FK/IK/逆动力学通过。模拟采集两轮，验证 `sim=True`、`complete=True`、`status=saved`、成功标签、7 维 action、256x256 RGB uint8、相机序号推进及样本数，临时目录已删除。

外设历史核验：已删除的 SpaceMouse ZMQ bridge 曾通过 dry-run；当前使用直连示例，它没有 dry-run 参数。Vin 1/Vin 4 相机检查失败，当前没有有效输入，原始错误见快照。未新做电机初始化、标定、回零或策略真机执行；安装到全新主机与现场运动验收需按手册完成，不属于本次已验证结果。
