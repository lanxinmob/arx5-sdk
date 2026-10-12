# 真机操作流程

先完成 [配置](setup.md) 并核对 [本机参数覆盖](current-machine.md#calibration-overrides)。本页非 dry-run / help / 相机检查命令均属于电机操作。示例以本机 `X5_umi/can0`、`X5/can1` 为准。

<a id="preflight"></a>
## 每次开机与入口选择

1. 确认机械臂固定、供电、夹爪负载和硬急停；支撑在阻尼或无补偿模式下可能下坠的机械臂，清空回零全路径。
2. 检查 CAN 映射与占用。已有服务持有接口时，只启动其客户端；切换入口先正常退出旧进程并等待析构，核对进程和端口，不用 `pkill python`。
3. 激活 `arx5`，在仓库 `python/` 内核对 SDK 路径。确认此入口的最终标定与重力配置；修改 Python/C++ 后必须重启进程，C++ 变更还需重编译。
4. 需要相机时先 `record_episodes.py --check-cameras --no-preview`；模型动作与可视化使用的视角分别核对。
5. 选下面唯一入口，确认它的启动及退出行为后执行。日志中的 “home 完成” 和 “目标已发送”不代替实测到位检查。

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
pgrep -af 'zmq_.*server|vr_joint_bridge|spacemouse|leader_follower|record_episodes|drag_mode'
ss -lntp
ip -details -statistics link show can0
ip -details -statistics link show can1
python -B -c 'import arx5_interface as s; print(s.__file__)'
```

| 需求 | 入口 | 持有 CAN 的进程 |
| --- | --- | --- |
| 一臂鼠标，直接 SDK | `examples/spacemouse_teleop.py` | 直连示例 |
| Quest 双臂 | 两路 joint server + `vr_joint_bridge.py` | 两个 server 子进程 |
| 主从拖动示教 | `examples/leader_follower_teleop.py` | 该进程持有两路 CAN |
| 单执行臂 HDF5 采集 | `examples/record_episodes.py` | recorder；内部复用主从逻辑 |
| 单臂/多臂自由拖动 | `examples/drag_mode.py` | 该进程 |
| 策略输出绝对关节目标 | joint server + 模型适配客户端 | joint server |
| 策略输出绝对末端目标 | `zmq_server_agent_safe.py` + 模型适配客户端 | guarded server |

<a id="spacemouse"></a>
## SpaceMouse

停止选定 CAN 的 server/其他 controller 后：

```bash
python examples/spacemouse_teleop.py X5_umi can0 --gripper-open-readout -3.45369
# 或只运行左臂
python examples/spacemouse_teleop.py X5 can1 --gripper-open-readout -3.2969
```

启动立即回零，不需要双按钮 arm。默认 100 Hz 目标更新、平移 0.1 m/s、旋转 0.3 rad/s、夹爪 0.04 m/s；这些在脚本内设置。双按钮请求回零并重置遥操目标；Ctrl+C 回零再阻尼。没有外设断流检查/显式 arm 状态；发生未捕获异常时不能假定会回零。此入口使用 shared memory/atomics、spnav。

SDK 报 `Inverse kinematics rejected: ...`（目标不可达或关节超限）时，直连版保持最后接受的目标，不退出、不累计被拒绝的位姿或夹爪增量；反向操作至目标可达后自动恢复。首次拒绝显示原始错误，连续拒绝不重复提示；其他异常继续向外抛出，SDK emergency 保护仍生效。保持的是命令目标，不代表实测已到位。

直连版的 `target_gripper_pos` 初始为 0，双按钮回零后也重置为 0；因此回零虽张开夹爪，开始/恢复遥操时会从闭合目标开始。平移 0.1 m/s 是各轴输入的速度系数，未按三维平移范数限速，不能作为合成速度上限。

<a id="quest"></a>
## Quest 双臂

先确认 server 的每臂标定覆盖，以及模型名与两路 CAN。终端 1：

```bash
python communication/zmq_joint_server.py X5_umi can0 X5 can1 \
  --port 8765 --port 8764
```

终端 2：

```bash
python communication/vr_joint_bridge.py --right-model X5_umi --left-model X5
# 没有桌面时加 --no-preview
```

Quest app 作为 TCP client 连接 PC 当前 Wi-Fi IP 的 10000 端口。bridge 默认监听 `0.0.0.0:10000`，而 joint server 默认 loopback；两端口用途不同。不要同时启动第二个 Quest receiver。收到两手新鲜流后会自动回零双臂并全开夹爪，锚定当前手势到实测末端姿态，再经 IK 发关节绝对目标。

- 侧扳机控制各夹爪，pose counter 用于识别新动作，不是扳机值。
- 长按右 A / 左 X 一秒慢回对应臂关节零位，保持该臂夹爪开度；恢复跟踪后重锚定。复位期间追踪暂停会保持 home，继续等待。
- 已安装 app 的未公开 reset pattern 是 `time_count=1107`、`gripper<0.01`、各旋转量绝对值 `<0.05`；双手同时出现 pattern 时保持位置。已有 README 记录按钮复位曾在通电臂上确认，但 app 升级后须重查，不能当成通用 Quest 协议。
- `--scale` / `--rotation-scale` 默认 1，`--right-frame-rpy` / `--left-frame-rpy` 可改手势映射；这是相对于锚点的末端映射，不是肩/肘人体重定向。
- 默认 50 Hz 更新、5 个 IK 关节目标均值后按 SDK 关节速度步进；`--joint-average-window 1` 关闭平均。没有额外的工作空间半径或 Cartesian 速度上限，初次场景验收需另限动作。
- 默认 `--hold-timeout 0.3`、`--input-timeout 0.6`。stale pose 先保持，持续失流/断连会停止；未超 0.1 rad 的关节限位反馈偏差会裁剪规划 seed，超出则报关节号并停止。

Ctrl+C / packet loss / 大偏差停止时向健康 armed server 请求 `RETURN_TO_HOME`，保持夹爪，再请求阻尼。每臂回零两 waypoint，规划约 0.4 rad/s 或更低；请求或反馈失败会报错，不保证到位。server 自身超时还可能独立触发回零。

<a id="preview"></a>
## Quest 相机预览与 MP4

bridge 默认启动桌面 4:3 预览：iPhUMI main + 左腕 Vin 4 + 右腕 Vin 1。`--primary-camera ultrawide` 切为 ultrawide；`--right-camera`、`--left-camera` 覆盖 V4L2 路径。2026-10-12 的检查中 Vin 1/Vin 4 都无有效输入，使用前重新核对相机，见 [本机快照](current-machine.md#devices)。相机失败会重连，不中断 Quest 遥操；关窗口也不停止遥操，Ctrl+C bridge 才走机械臂退出流程。无桌面用 `--no-preview`。

```bash
python communication/vr_joint_bridge.py --record-video
# 或 --record-video /path/to/session.mp4
```

默认保存 `~/Videos/arx5/` 带时间戳文件；存在文件不覆盖。关闭预览或停止 bridge 会完成 MP4。此 MP4 是合成 UI 三视图，不是 HDF5 训练采集，也不是原始多相机严格同步数据。`--record-video` 要求有桌面且预览开启。

<a id="leader-follower"></a>
## 主从与拖动

```bash
python examples/leader_follower_teleop.py X5 can1 X5_umi can0 --diagnostics
```

启动依次回零主/从臂；主臂重力补偿拖动、kp=0、kd=SDK 默认的 0.05 倍，从臂位置保持。桌面键盘监听 `space` 开始/暂停跟随，`h` 两臂回零，`q` 或 Ctrl+C 正常退出回零。跟随用主臂关节绝对角度裁剪至从臂限位，速度前馈 `--vel-ratio 0.3`；夹爪主臂 <=1 mm 时从臂目标 0，否则裁剪至从臂宽度。

暂停主从遥操时从臂位置保持，不能像 recorder 待机那样自由拖动。主臂重力倍率 1.2、从臂 1.0；夹爪力反馈 `--force-scale 0.5`，反馈力矩夹至 0.6 Nm，0 禁用。诊断同时显示 gravity、emergency、kp/kd、输出力矩、速度、夹爪测量/命令 mm 和力矩；输出力矩不是电机实际产生的力矩。主从控制线程异常路径没有 recorder 那样统一恢复保证，需看退出实测状态。

单臂自由拖动（也是电机操作，启动先 home）：

```bash
python examples/drag_mode.py X5_umi can0 -g -3.45369
```

`space` 拖动/保持、`h` 回零后拖动、`q` 回零退出。`--gravity-scale` 默认 1；可按实测漂移调整，不能用它掩盖错误工具负载/URDF。脚本 `--no-exit-damping` 表示不主动调用阻尼，但 SDK 默认析构仍约 2 s 阻尼，因此不能保证进程退出后位置保持。多臂版本共用一个 `-g`，本机不同读数用单臂分别配置。

<a id="recording"></a>
## HDF5 主从采集

完整字段、参数、交互见 [采集手册](../../python/recording/README.md)。它记录一条从臂执行轨迹（6 joints + gripper = 7），不是双执行臂 14 维数据。

```bash
# 不连接 CAN，只验相机
python examples/record_episodes.py --check-cameras --no-preview
# 电机操作，recorder 自身持有两臂；不能再运行主从脚本
python examples/record_episodes.py X5 can1 X5_umi can0 \
  --dataset-dir data/demonstrations
```

先检查相机及预览，再两臂同步回零；准备期间两臂均重力拖动。选择语言指令，`r` 回车开始，限速对齐不入 demo；采集中 `ok/f` 保存成功/失败、`d` 丢弃、`q` 中断退出。默认 60 s 后停采并等待标签，不能当模型自动判断成败。正常退出、信号、预览关闭、控制/写盘/相机异常均尝试慢回零；断电/CAN 丢失/SIGKILL 不保证。

默认 MJPG 1080p/60 FPS 入流，保存 256x256 RGB；腕相机新帧触发采样。图像 timestamp 是主机 `read()` 返回时间，不是曝光时间；多视角最近帧配对不是硬同步。每轮 `.partial.hdf5` 写完关闭后发布正式文件，丢弃/中断数据仍保留，训练只用已完成文件/`data/demo_N`。

`action` 为提交给从臂的绝对关节目标和夹爪宽度 m，`observations/qpos` 是实测值，`observations/ee_pose` 是同一关节状态经 FK 的基座系末端 m/RPY rad，不能互相替代。夹爪 effort 末列是电机力矩 Nm，不是接触力 N。

<a id="other-examples"></a>
## 键盘、示教回放与其他示例

这些是现有实验入口，均须先审阅有效标定与轨迹；文件名含 `test` 不表示无硬件。它们没有 joint/guarded server 的统一动作 watchdog，本次未执行其真机路径。

| 入口 | 参数与实际行为 | 使用前检查 |
| --- | --- | --- |
| [keyboard_teleop.py](../../python/examples/keyboard_teleop.py) | `MODEL INTERFACE`；启动 home；方向键上下 +/-x、左右 +/-y，PageUp/Down +/-z，q/a +/-roll、w/s +/-pitch、e/d +/-yaw，r/f 开闭夹爪，space home，Ctrl+C home/阻尼 | 硬编码全开读数 `-3.46933`，无标定 CLI；100 Hz、0.04 m/s、0.5 rad/s、夹爪 0.02 m/s；q 是旋转，不是退出 |
| [teach_replay.py](../../python/examples/teach_replay.py) | `MODEL INTERFACE`；先 home，t 开始手动拖动记录、q 保存，再 home，r 回放、q 结束 | 硬编码 `-3.42`；`data/teach_traj.npy` 会写入/覆盖，pickle 只加载可信文件；记录 EEF 与夹爪，无图像/语言/完整时间戳，回放按 controller_dt，不等同 HDF5 recorder |
| [test_joint_control.py](../../python/examples/test_joint_control.py) | `MODEL INTERFACE`；home 后自动跑预设关节轨迹，再 home | 全开读数 `-3.2`；不能作为只读通信检查，也不能直接用于 7 DOF |
| [joint_waypoint_scheduling.py](../../python/examples/joint_waypoint_scheduling.py) | `MODEL INTERFACE`；home 后单点/多点关节调度 | 默认工厂标定；预设 6 维 waypoint；使用 controller-relative timestamp，不能用 Unix 时间 |
| [cartesian_waypoint_scheduling.py](../../python/examples/cartesian_waypoint_scheduling.py) | `MODEL INTERFACE`；home 后末端单点/多点轨迹 | 工厂标定；模型/IK/工作空间及时间戳，预设动作未经当前场景碰撞验证 |
| [test_bimanual.py](../../python/examples/test_bimanual.py) | `MODEL0 INTERFACE0 MODEL1 INTERFACE1`；两臂 home 后自动运动 | 两臂硬编码 `-3.4`，接口必须不同；不适用本机独立标定 |
| [aloha.py](../../python/examples/aloha.py) | 主模型/接口、从模型/接口；home 后简单跟随 | 较早主从实验入口，无新版本的独立夹爪 CLI/诊断/采集生命周期，日常使用 leader_follower/recorder |
| [test_torque_control.py](../../python/examples/test_torque_control.py) | `MODEL INTERFACE`，另有 `--urdf_path`；关闭重力、home 后改增益并发第 1 关节力矩 | 硬编码 `-3.24502`；不是自由拖动入口；检查实际 URDF 参数是否被使用，不仅看 CLI 名 |
| [test_gripper_force_compensation.py](../../python/examples/test_gripper_force_compensation.py) | `MODEL INTERFACE`，另有 `--urdf_path`；home 后根据夹爪加速度给力矩 | 工厂标定；专用实验，不能据此假定力矩是接触力或可调零点 |
| [test_upside_down.py](../../python/examples/test_upside_down.py) | `MODEL INTERFACE`；倒装重力 `[0,0,9.81]`、低增益运行 | 仅实际倒装臂，不对正常安装运行 |
| [test_x7.py](../../python/examples/test_x7.py) | 无 CLI，硬编码 `X7Left/can0` | 当前工厂名是 `X7_left`，原样会失败；先改机型、DOF/硬件与增益，不能当 X7 开箱验收 |
| [arx5_zmq.py](../../python/examples/arx5_zmq.py) | 无 CLI；顶层连接 8765、reset 并发硬编码 EEF 动作 | 导入即执行；不是 joint 协议，也不核对 guarded identity，不能当库 helper 导入 |
| [C++ joint](../../examples/test_joint_controller.cpp)、[C++ Cartesian](../../examples/test_cartesian_controller.cpp) | 构建出的 executable，无 model/interface CLI | 全局构造硬编码 `L5/can0`；启动即连接，先改配置和现场轨迹；EtherCAT 重编译后重新 setcap |
| [test_solver.py](../../python/examples/test_solver.py) | 无 CLI，只构造 solver 做 FK/IK/ID | 不连接 CAN，但默认是 X5 示例；新模型用 setup 中配置检查并显式选机型 |

旧实验入口的 hardcoded 值应在构造前改为目标臂的已验证读数；同步确认退出路径，不在已有 server 持有 CAN 时运行。新 agent 的安装冒烟优先采用 setup 的离线配置/solver、相机检查与 SpaceMouse 输入连接检查。

<a id="lifecycle"></a>
## 回零、超时与退出矩阵

| 入口 | 启动 / arm | 没有动作时 | 正常停止 |
| --- | --- | --- | --- |
| SDK controller | 构造即收发电机，初始 kp=0/阻尼增益；可能有重力输出 | 没有应用层 watchdog，后台继续最后插值/输出 | 默认析构关闭重力并约 2 s 阻尼；非断电 |
| joint ZMQ | 启动 disarmed；`RESET_TO_HOME` 全开夹爪并 armed | armed 后默认 2 s 无接受关节目标，尝试两段慢回零保持夹爪，再 disarm/阻尼 | server 停止同样尝试慢回零；`SET_TO_DAMPING` 直接 disarm |
| guarded Cartesian ZMQ | disarmed；reset 后用 SDK reset 命令位姿 arm | 默认 2 s 无接受末端目标，阻尼/disarm；不回零 | Ctrl+C close 请求阻尼；SIGTERM 无自定义 handler，不当成同等恢复路径 |
| SpaceMouse 直连 | 启动立即 reset；双钮再次 reset | IK 拒绝时保持最后有效目标，反向至可达后恢复；无外设断流检查 | Ctrl+C 回零再阻尼；其他异常不保证回零 |
| Quest bridge | 两手流到达后双臂 reset | stale hold 后 packet loss 停止，reset 阶段例外 | 健康 armed 臂慢回零保持夹爪，再阻尼 |
| 主从 teleop | 主从依次 reset，space 跟随 | 暂停从臂位置保持，主臂拖动 | q/Ctrl+C 尝试回零，异常恢复不等同 recorder |
| recorder | 相机/预览验证后同步 reset | 待机两臂拖动；每轮结束不自动 home | 信号/故障等尝试同步慢回零全开夹爪，然后 SDK 析构 |
| 旧 `zmq_server.py` | Cartesian controller；默认 bind 0.0.0.0:8765 | 默认 60 s 无消息，会 home/阻尼/释放控制器；读请求可使行为不同于 accepted-action watchdog | 旧路径，恢复配置与 guard 不同；新 agent 不应默认选它 |

SDK `reset_to_home()` 先第 3 关节 0.03 rad 接近点，再安排 0.5 s 归零。函数打印结束时末段可能仍在执行，没有统一实测到位判定。joint server 的 `RESET_TO_HOME` 额外等待 0.55 s；它的 `RETURN_TO_HOME` 则检查每个 waypoint 最大关节误差 <=0.05 rad。recorder 的慢回零也检查实测位置。不要在 blocking reset/return 调用期间依赖 ZMQ stop 请求立即响应，server 同线程处理且可能尚未接受下一条消息；紧急情况依靠硬件停止。

SDK emergency 锁存后只发阻尼且拒绝运动/增益/回零，须排除故障再重启 controller。`SET_TO_DAMPING` 不清锁、不保证各臂同一物理行为；健康状态下该方法没有关闭重力开关，emergency/默认析构路径会去掉重力输出。正常退出应先确认实物状态和支撑再停进程；kill -9、断电和通信失效都跳过可控恢复。
