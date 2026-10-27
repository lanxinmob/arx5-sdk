# 真机故障排查与恢复

保留完整错误、命令、有效参数、SDK 路径、git 版本、目标和反馈。先验证原因，区分事实与推断；相同假设反复无新信息时更换检查入口。硬件故障先处理实物与支撑，不能用重试动作、增大限位或关闭保护代替诊断。

<a id="software"></a>
## 安装、构建与加载

| 信号 | 核查与处理 | 通过条件 |
| --- | --- | --- |
| `No matching distribution found` | 核查 Python/平台、pip 版本、registry 实际响应；上游建议升级 pip。补丁版可按 setup 源码构建 | 成功导入预期扩展，不只 pip 退出码 |
| CMake Eigen 4 / 运行异常或 ABI 疑点 | 预编译 solver 需要 Eigen 3.x；看 `Eigen3_DIR` 与环境包版本，按 setup 选系统/环境 Eigen 3 并重构建 | CMake config 正确，solver 离线 FK/IK finite 且往返匹配 |
| import 找不到或 emergency API 缺失 | 打印 `sys.executable`、`sdk.__file__`；检查本地 `.so` 是否匹配 Python 3.x、pip 遮蔽、脚本 `sys.path` | 加载本地补丁 SDK；required API 存在 |
| URDF 找不到 | 工厂从加载的动态库目录推算项目根，不仅由 cwd 决定；看 `robot.urdf_path`，保持 `python/` 与 `models/` 布局 | Path 存在，选定机型 solver 可构造 |
| NumPy/OpenCV/h5py 二进制接口错误 | 不混装新 wheel 到旧 NumPy；本机已验证 NumPy 1.26.4、h5py 3.10、OpenCV 4.8.1，用 recording requirements | 正常导入并验证相机/模拟 HDF5 |
| aarch64 链接 SOEM 失败 | 环境 YAML 已说明该架构 conda SOEM 缺失；从源码装入正确 prefix | 链接无缺库，不使用 x86_64 预编译库 |
| 下载失败 | 先单命令代理 `127.0.0.1:7897`，再比较直连；保留 TLS/DNS/HTTP 状态，不改全局配置 | 检查实际下载内容与安装结果，不把 HTTP 请求发出当成功 |

```bash
conda activate arx5
cd /home/hjadmin/projects/arx5-sdk/python
python -B -c 'import sys, arx5_interface as s; print(sys.executable); print(s.__file__); print(s.RobotConfigFactory.get_instance().get_config("X5_umi").urdf_path)'
```

## CAN、进程与 server

| 信号 | 核查与处理 |
| --- | --- |
| can 接口不存在/down | `ls -l /dev/serial/by-id /dev/arxcan*`、`pgrep -af slcand`、`ip -details`；按真实固件走 SLCAN/candleLight，不重复启动 daemon |
| service active 但无法通信 | oneshot `RemainAfterExit` 不是在线检查；看 USB 是否重插、进程是否还在、symlink 是否指向原设备；控制器退出后再修复接口 |
| SLCAN `bitrate 0` | 本机速率由 `slcand -s8` 设置，核对命令；此显示本身不是配置失败 |
| `None of the motors are initialized` | 检查电机供电、CAN 线/适配器、1 Mbps、对应臂与电机 ID/type、重复 writer；全零状态不能当有效反馈 |
| `Address already in use` | `ss -lntp` 找指定端口的真实 owner；确认服务种类和对应 CAN，正常退出或选未占用端口并同步客户端 |
| SDK logger 同名/异常反馈/相互争抢 | 每个 CAN 仅一个 controller，同进程重复 model/interface 也会冲突；先识别并结束旧入口 |
| REQ socket 卡住/EFSM/ZMQError | 服务是否正在阻塞回零、端口/协议是否匹配？超时后重建 REQ socket；不能继续在等待 recv 的 socket 发动作 |
| GET_STATE 正常但数秒后开始回零 | joint armed watchdog 只由接受动作续期；读状态不续期。换用正确生命周期设计，不能误称“只读查询导致动作” |
| 模型请求被拒绝、RESET required | 确认 server 身份与 armed/emergency；修复 payload，若已 disarm 需经明确流程重新 reset，不循环自动 reset |

被动检查 CAN 帧可 `timeout 2s candump can0`，不发控制指令；无新帧也可能因为电机只响应控制请求，不能据此直接判断硬件损坏。`ip -statistics` 累计数据也不能证明反馈新鲜。实时反馈需在已有控制服务中核对，不为查状态另开 controller。

<a id="gripper"></a>
## 夹爪与接触保持

`Gripper position error` 检测开度小于 -0.005 m 或大于 `gripper_width+0.005 m`，会锁存 emergency。新夹爪电机方向不同可能需要负 `gripper_open_readout`，但不能只把任意 `-3.4` 当正确结果。逐入口核对 [覆盖表](current-machine.md#calibration-overrides)，重设零点则按 [标定](setup.md#calibration)。

普通标定和 `--verify-zero` 必须先通过 controller 初始化检查，构造时报错就尚未重设零点。若配置已经为负，不能再把通用日志的“改成负数”当成诊断。本次 `-3.34 rad / 0.086 m` 配置下，张开 `-0.072 m`、闭合 `-0.160 m` 分别对应约 `+2.80/+6.21 rad`，支持闭合零点偏移的判断。轻闭后仍被初始化阻断时，按 [独立夹爪恢复](setup.md#calibration-recovery) 使用 `--recover-zero`，不修改普通 controller 的保护，也不靠改另一个负数消除符号错误。恢复会写夹爪电机零点；其他关节需可靠支撑。

空载闭合时同时看实物、`gripper_mm`、`gripper_cmd_mm`、torque、kp 与日志：

| 事实 | 下一步 |
| --- | --- |
| 命令还没到 0 | 核对主臂测量、1 mm 闭合阈值及转换；阈值只映射目标，不改电机零点 |
| 目标 0、测量明显大于 0 | 查 `Gripper contact hold`、机械摩擦/传动和增益，不先写负开度 |
| 测量近 0、指尖有缝 | `--verify-zero` 比较两次轻闭与全开；测量开度是角度换算，检查零点、间隙与实物宽度 |
| 恢复时设零后接近 0，再次闭合超差 | 零点已写但标定未通过；保留 `fully_open` 及各阶段角度/范围/力矩，先用 `--verify-zero` 检查重复性，不自动再次设零或放宽阈值。本次 `-0.061532 rad` 按临时参数约为 1.58 mm，尚不能确定是力度、传动还是零点保持问题 |
| 重力拖动下坠/上飘 | 查 gravity 开关、倍率、URDF 负载/质心、安装方向；阻尼只降速度，不补静态模型误差 |

默认夹爪最大力矩 1.5 Nm，接触保持触发条件包含 `abs(torque)>=gripper_torque_max/2` 及受阻方向判断：在默认值下为 0.75 Nm。锁存接触宽度，并沿卸载方向退目标 1 mm；向反方向发超过 0.5 mm 的命令释放。不能把接触保持后的测量误差解释成 SDK 忽略目标，也不能无限增大 kp/过流阈值。主从可用 `--follower-gripper-kp` 比较（上限 10），但这不修复零点或传动问题。

## emergency、跟随与回零

SDK 会检测非 finite 数据、非常异常的关节读数、夹爪范围和过流。过流计数默认超过 20 次连续检测才 emergency，按实测 torque 检查，不是外部力 N。普通 joint 位置 sanity 容差远大于应用工作空间，底层保护不能替代碰撞、奇异点和速度检查。

故障锁存日志：`Emergency latched: damping only; motion rejected until controller restart.` 此状态每周期阻尼且无位置/前馈/重力力矩；set command/gain/home 会被拒绝。恢复顺序：

1. 支撑机械臂并停止模型发送，记录最早的故障日志及有效配置。
2. 检查错误来源：供电/CAN、夹爪符号/零点、负载、finite/shape、机械阻挡和目标/反馈。
3. 结束占用 CAN 的故障 controller 并等待退出，修正对应配置/实物。需要重新编译时先做离线验证。
4. 确认现场路径后重启控制器；先检查 emergency=false、实际反馈，再执行单独验收流程。不提供在原 controller 内清锁的接口。

`Cannot set kp ... far from current pos` 表示 kp 从 0 恢复时旧命令与测量相距 >0.2 rad 且目标 kp>1；先将目标锚定实测、按流程平滑恢复增益，参考 `drag_mode.py::enter_hold` / recorder lifecycle，不能取消这项检查。源码此分支还停止后台收发，不能认为异常捕获后控制器仍正常。

IK 返回 status=0 才成功，同时检查 finite、joint limits 和 FK 误差。unreachable 时保持或减小目标，不能发失败的 IK 返回关节值；六关节回零附近可能有奇异姿态。SpaceMouse 直连示例会输出跟随误差，但没有原 ZMQ bridge 的 0.25 rad 停止阈值；Quest 允许实测越界 <=0.1 rad 仅作为 seed 容差，不是扩大目标限位。

`Arm did not reach home`、`return to home failed` 或 recorder 的 `Home target was sent but measured position did not reach home` 是恢复失败。记录目标与实测、检查 emergency/反馈/阻挡；不能在碰撞路径上重复 home。SDK reset 的打印结束不等于实测到位，参见 [退出矩阵](operations.md#lifecycle)。停止 server 后先确认机械臂有稳定支撑，阻尼和断电都可能使其下坠。

## 相机、预览与采集

| 信号 | 排查 |
| --- | --- |
| 能打开设备但无帧 | 节点实际信号源、采集卡 HDMI/相机输出、稳定路径/video-index0、格式支持及占用；用 `fuser /dev/video0` 查看 owner，不同时 ffplay 与 recorder |
| 灰帧/`NO CAMERA SIGNAL` | recorder 检查最大值 <=32 且像素范围 <=2 的暗灰帧；Vin 2/3/4 曾全像素 7，设备存在不证明有视频 |
| 请求 60 FPS 但实际不同 | 看驱动实际尺寸/FPS、sequence/time 推进与 recorder actual_fps，不能只读请求参数 |
| 旧帧/时间戳陈旧 | 检查 host monotonic age、序号，控制/读取/写盘各线程状态；多相机不是硬同步 |
| `IPHUMI USB UNAVAILABLE` | 手机解锁、信任、Deployment USB streaming；查 usbmuxd、pypeertalk device list、5555/5556 选择；必要时直接连接 PC USB |
| SSH / pynput DISPLAY 错误 | 主从/drag 键盘监听需要桌面后端；recorder 用终端交互并 `--no-preview`；Quest 可 no-preview。不要为此切 root |
| 关 Quest 窗口但机械臂仍响应 | Quest 预览关闭不停止 bridge；Ctrl+C bridge 走回零退出。recorder 关窗则是中断/回零，两者不同 |
| 队列满/写盘失败/partial | 停录保留原始文件，核对空间/磁盘及 metadata/status；只用完成 demo 训练，不把 partial 改名冒充完成 |

录制中 stale 样本可被丢弃并告警；断开/参考序号倒退/队列满等会终止当前 episode。详情以 [采集手册](../../python/recording/README.md) 与源码为准。训练前核对 `complete/status/sim`、视角列表、实际帧率和 action 语义。

## 交接故障

写明“已验证事实、已排除原因、尚未确定、下一项能区分假设的检查”。附首次错误及命令退出码、进程/端口/CAN状态、SDK路径与参数、相机真实画面/帧统计、目标与反馈；不附密钥。测试目录清理前保留必要结果摘要，不能用删除原始采集数据作为清理测试的替代。
