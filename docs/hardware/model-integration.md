# 其他策略模型的真机接入

本库提供机器人控制、FK/IK、遥操与采集，不提供所有 VLA/checkpoint 的现成部署器。接入目标是把模型明确的观测/动作合同转换为本库可接受的目标；维度相同不足以证明语义相同。硬件机型配置另见 [setup](setup.md#robot-config)。

<a id="architecture"></a>
## 最小进程结构

```text
相机 + 实测状态 -> 模型环境中的观测适配器 -> checkpoint 推理
                                              |
                                     动作还原/校验/限速
                                              |
                         localhost ZMQ REQ -> REP 机器人服务
                                              |
                                   本地 SDK -> SocketCAN -> 电机
```

机器人服务在 `arx5` 环境加载补丁 SDK，模型服务使用其专用 CUDA/PyTorch 环境。只有机器人服务持有目标 CAN。相机与动作适配器可以在独立进程运行，但要有统一时钟/序号、唯一动作 writer。不能让推理调用阻塞 watchdog 的持有者，或把 500 Hz SDK 周期误当成推理频率。

选择接口：

| 模型经还原后的动作 | 本库接口 | 接入前必须补足 |
| --- | --- | --- |
| ARX 绝对关节角 + 夹爪宽度 | joint server `SET_JOINT_TRAJ` / `SET_JOINT_POS` | 对应实物 DOF/关节顺序，逐步速度/加速度约束及场景边界；server 仅校验位置，不提供完整 Cartesian guard |
| ARX 基座系绝对 EEF 位姿 + 夹爪宽度 | guarded server `SET_EE_POSE` | 工作空间、夹爪上限、per-command 步幅及有效标定 |
| tool/base-frame EEF 增量 | 适配器还原成绝对 EEF，再 guarded server | 增量参考点、左乘/右乘、时间/角度单位和工具变换 |
| 关节速度、力矩或其他机器人的关节值 | 当前两个 server 无直接支持 | 在明确动力学/动作定义后实现适配，不能将值塞入 position 字段 |

<a id="contract"></a>
## SDK 与 ZMQ 合同

### 状态、单位与时间

| 字段 | 实际语义 |
| --- | --- |
| `joint_pos/vel/torque` | 按 RobotConfig 顺序，rad / rad/s / SDK 换算 Nm |
| `ee_pose` | `base_link` 到配置的 `eef_link`，`[x,y,z,roll,pitch,yaw]`，m/rad |
| RPY | `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`；SciPy `Rotation.from_euler("xyz", rpy)` 与此约定一致 |
| `gripper_pos/vel/torque` | 全闭 0、全开 gripper_width，m / m/s / Nm；不是归一化 0~1 或夹持力 N |
| `timestamp` | 此 controller 启动以来的秒数；不等于 Unix/模型时间，不直接跨控制器相减 |
| `emergency` | 补丁 SDK 故障锁存；为 true 时不重试发运动 |

ZMQ 使用 Python `send_pyobj` / `recv_pyobj`（pickle），不是 JSON/HTTP/ROS；只连接可信本机进程，默认 loopback bind。若要跨机器运行，优先把模型 RPC 与机器人 pickle 通道分离，用 SSH 隧道或本地适配器。HTTP 模型 API 不能直接连接机器人端口。

公共请求/回复形状：

```python
request = {"cmd": "GET_STATE", "data": None}
# reply = {"cmd": "GET_STATE", "data": { ... 实测状态 ... }}
# 失败 reply["data"] 为 "ERROR: ..."，动作成功或 OK 不代表物理到位
```

REQ/REP 必须一发一收；超时后重建 REQ socket，不继续用等待回复中的 socket 发停止。给发送、接收设置超时和 LINGER=0；阻塞 homing 的 timeout 要独立设置长于预计轨迹，普通 action 不应用相同长等待。仓库 `communication/zmq_client.py::Arx5Client` 旧客户端默认无 socket 超时，也没有 guarded 身份验证；新适配器参考 `vr_joint_bridge.py::JointClient` 的 socket 生命周期，但 guarded server 的 info 回复需自行处理。

### joint server

启动（电机操作，先核对硬编码全开读数）：

```bash
python communication/zmq_joint_server.py X5_umi can0 \
  --bind 127.0.0.1 --port 8765 --timeout 2 --duration 0.05
```

`DRAG` clears any active policy trajectory before enabling the low-gain,
gravity-compensated mode. If the released arm rises or leans backward, pass a
lower `--gravity-scale` (for example `0.9`, `0.85`, or `0.8`) after checking
the real arm is supported; the default is `1.0`. The scale is applied before
the SDK controller is constructed.

| cmd | data / 结果 |
| --- | --- |
| `GET_STATE` | None；返回 joint/EEF/gripper/timestamp/emergency/armed/mode |
| `RESET_TO_HOME` | None；回零全开夹爪并 arm，回复 OK |
| `DRAG` | None；清空旧插值目标，进入重力补偿拖动并 disarm |
| `HOLD` | None；锁定当前实测关节位置，恢复位置增益并 arm |
| `SET_JOINT_POS` | 精确包含 `joint_pos` 和 `gripper_pos`；返回实测状态 |
| `SET_JOINT_TRAJ` | 精确包含 `(N, joint_dof)` 的 `joint_pos`、`(N,)` 的 `gripper_pos` 和 `(N,)` 的 `time_from_now`；一次替换 SDK 的未来轨迹并返回实测状态 |
| `RETURN_TO_HOME` | None；仅 armed/健康时慢回零，保持当前夹爪；回复 OK 后仍 armed |
| `SET_TO_DAMPING` | None；disarm/阻尼，回复 OK |

在 `DRAG` 或 `HOLD` 状态按 Ctrl+C 停止 server 时，server 会先锁定拖动状态
下的当前实测关节位置（如有需要），再按同一慢速回零流程回到 SDK home，最后
进入阻尼。启动后尚未完成 `RESET_TO_HOME` 时只进入阻尼，不强制移动。

```python
{"cmd": "SET_JOINT_POS", "data": {
    "joint_pos": [q0, q1, q2, q3, q4, q5],  # 绝对 rad
    "gripper_pos": width_m,                 # 数值标量，不能 None
}}
```

shape 必须为 `(joint_dof,)` 和标量，全部 finite 且在 SDK 范围内；不接收 velocity、timestamp 等额外 data 字段。server 使用 `controller.get_timestamp()+duration` 安排插值，默认 0.05 s；要求 `0 < duration < timeout`。命令回复的 state 是提交后当前反馈，尚未等目标到达。

连续动作应优先使用 `SET_JOINT_TRAJ`，避免每个控制周期用 `SET_JOINT_POS` 覆盖 SDK 的未来轨迹。上游策略可以先将稀疏动作按时间线性重采样到控制网格，再提交整段轨迹：

```python
{"cmd": "SET_JOINT_TRAJ", "data": {
    "joint_pos": [[q0, q1, q2, q3, q4, q5], ...],
    "gripper_pos": [width_m, ...],
    "time_from_now": [0.20, 0.40, ...],
}}
```

三个数组长度必须相同，时间严格递增且单位为秒；server 会丢弃已经到期的点，将剩余点转换为 SDK 时间戳后一次调用 `set_joint_traj()`。SDK 在内部按控制器频率连续插值，回复仍是提交时的实测状态。

joint server 没有 `GET_SERVER_INFO`，不回传有效 calibration/config；从启动日志与配置记录证明实际机型，用同一 URDF 的 FK 比较反馈姿态可补充检查，但单姿态匹配不足以证明所有模型/标定完全一致。接受的 `SET_JOINT_POS`、`SET_JOINT_TRAJ` 和成功 reset 更新动作计时，return 在每段回零命令提交时更新；已在 0.05 rad home 容差内直接返回时不刷新。单纯 GET_STATE 不续命。armed 后无接受目标默认 2 s 触发慢 home/阻尼，详见退出矩阵。

### guarded Cartesian server

启动（工作空间参数填写现场测量值，以下大写 token 是占位符，不可原样执行）：

```bash
python communication/zmq_server_agent_safe.py X5_umi can0 \
  --bind 127.0.0.1 --port 8765 --gripper-max 0.086 \
  --workspace-min X_MIN Y_MIN Z_MIN \
  --workspace-max X_MAX Y_MAX Z_MAX
```

仍需先修正脚本构造前的 `-3.37472` 标定覆盖。`--gripper-max` 与实物模型宽度一致；X5 的工厂宽度 0.088，不能机械套用 X5_umi 的 0.086。

| cmd | data / 结果 |
| --- | --- |
| `GET_SERVER_INFO` | None；name=`arx5_agent_safe`、protocol_version=1、model/interface/armed/home_pose/limits |
| `GET_STATE` | None；实测状态，另含 `ee_cmd_pose`、`gripper_cmd_pos`，state 中无 armed 字段 |
| `RESET_TO_HOME` | None；SDK home 后额外 settle 默认 2 s，再以 reset 命令姿态 arm |
| `SET_EE_POSE` | 精确包含 `ee_pose` 和 `gripper_pos`；成功返回当前实测状态 |
| `SET_TO_DAMPING` | None；disarm/阻尼 |
| `GET_GAIN` / `SET_GAIN` | kp/kd 数组和 gripper_kp/kd 标量；不用于绕过动作 guard |

guard 默认限制：相对上次接受目标平移范数 <=0.002 m，wrapped RPY 分量差的范数 <=0.02 rad，夹爪差 <=0.002 m，距 home 平移半径 <=0.45 m，夹爪范围 0~0.086 m，额外 workspace 可选。它不是随时间归一化的速度限值，也不是 SO(3) 旋转距离；过快多次提交仍要在适配器按真实 dt 限速。只有 SDK 提交成功后更新参考值。

```python
{"cmd": "SET_EE_POSE", "data": {
    "ee_pose": [x, y, z, roll, pitch, yaw],  # 绝对基座系 m/rad
    "gripper_pos": None,                   # None = 上次接受的夹爪目标
}}
```

reset 后 guard 使用 `ee_cmd_pose/gripper_cmd_pos` 作为首个目标基准，不是仅用 `ee_pose`；机械跟随误差可能大于一步额度。先读取 info/state，检查实际跟随及目标，再从该命令基准生成受限步骤，不能直接以测量位姿一次跳到远处。2 s 无接受 pose 命令后阻尼/disarm；GET_STATE/info、增益操作、被拒绝命令均不延长计时。重启/timeout/阻尼后需重新 reset，不能自动重放旧 action chunk。guard 范围不包含 reset 的回零路径，因此现场碰撞检查仍必要。

<a id="frames"></a>
## 动作表示与坐标转换

接入前从 checkpoint 的训练配置、normalization stats 和部署代码确认：关节还是末端，absolute 还是 delta，哪个 frame，旋转是 RPY/quaternion/rotvec/6D，夹爪是开度/二值/归一化且正方向是什么。保留对应路径/版本，不能只从模型名称推断。

### EEF 与 UMI TCP

`communication/zmq_client.py::tcp2ee/ee2tcp` 不是同名字段转换：UMI TCP 用 rotation vector，SDK EEF 用 RPY，并有固定轴变换。

```text
C = [[0,0,1], [-1,0,0], [0,-1,0]]
R_tcp = R_eef @ C
R_eef = R_tcp @ C.T
```

这组函数保留同一 xyz，没有工具长度平移。若新工具尖端相对 `eef_link` 有偏移，必须使用完整刚体变换：`T_base_eef = T_base_tool @ inverse(T_eef_tool)`；不能只复用轴矩阵。不同 `*_umi` URDF 的固定末端 link 也须纳入。rotation-vector 有等价分支，按旋转矩阵/相对旋转角校验；旧 `ee2tcp` 的零/近 pi 旋转处理存在数值边界，不用 rotvec 三维差的大小证明姿态错误，新的转换优先 SciPy Rotation。

delta 组合必须写明训练语义，例如完整工具系增量 `T_next=T_now@DeltaT_tool`、完整基座系刚体增量 `T_next=DeltaT_base@T_now`；很多 checkpoint 实际只规定 `p_next=p_now+delta_p_base`、`R_next=delta_R_base@R_now`，这与完整左乘齐次矩阵不等价。以训练定义为准，不能给所有模型套一个组合公式，也不能用 RPY 三分量相加代替任意旋转组合。

rotation6D 必须核对向量排列（行/列）与正交化规则；quaternion 必须核对 xyzw/wxyz。夹爪归一化若 1=开且线性，才可 `width=clip(g,0,1)*gripper_width`；若 1=闭则反向，若训练数据是力/二值/非线性宽度就按原定义实现。不要直接把米填成毫米、把物理范围换成训练归一化值或发负宽度。

### 数据与频率

本库 HDF5 单执行臂 qpos/action 为 `(T,7)`，末列宽度 m；双臂模型需要明确左/右两套观测及动作，不可复制同一条从臂数据凑 14 维。joint 差分、EEF 差分、LIBERO action 和此处绝对关节 action 不能仅靠重命名互转。

采集默认 256x256 RGB，模型可能要求 224、其他分辨率、多视角或时序历史：从原始有效图像按 checkpoint 规则转换，并记录色彩、resize/crop、视角顺序。OpenCV 输入 BGR，HDF5 保存 RGB。Quest 合成预览 MP4 与 recorder 保存图像是不同产品，不把前者的叠字和拼接当原始输入。

动作 chunk 需要截止时间、当前执行索引及重规划替换规则；只执行仍新鲜的短片段，收到 stop/重锚定/新 episode 就丢弃旧队列。推理超过 server watchdog 时，要显式选择流程：由控制适配器按计划持续受限保持，或允许超时并人工/流程重新 arm；GET_STATE 不能保持 armed。joint timeout 会运动回零，guarded timeout 是阻尼；选择必须与现场相符。保持动作会延长 watchdog，因此适配器必须另有推理/观测过期和操作者停止规则，不能无限续命。

<a id="migration"></a>
## 接入与验收顺序

| 阶段 | 要做的事 | 通过证据 |
| --- | --- | --- |
| 1. 基础环境 | 按 setup 安装，确认 SDK/设备/每臂有效配置 | SDK 路径、版本、CAN 映射、标定与相机检查输出 |
| 2. checkpoint 合同 | 列清 state/action/image shape、normalization key、frame、频率、夹爪 | 实际训练/部署配置和样本，不接受“看起来像” |
| 3. 离线适配 | 不创建 controller：还原一个输出、finite/范围/步幅、旋转往返、FK/IK 与单位检查 | 输入/输出数值、误差/IK status；正常、NaN、越界、过期、stop 样本的决策 |
| 4. mock/shadow | 验证完整观测到推理到受限目标及日志；明确是否连接硬件 | 日志显示提交关闭；若环境启动会 reset，明确标记不是无硬件模拟 |
| 5. 通信只读 | 已授权启动对应 server，查询 state/info 并核对身份、反馈/模型 | emergency=false、shape、配置身份、反馈时间推进；不 arm 的离线客户端 |
| 6. 现场回零 | 清空轨迹、授权 reset，核对实测位置与夹爪 | 实测到位误差和实际耗时，不只 OK |
| 7. 短步执行 | 空载逐轴极小动作，再夹爪；核对 base/tool 正方向与限速 | 目标与反馈，IK 成功/跟随误差，操作者 stop、超时、掉线后的预期行为 |
| 8. 模型试运行 | 短片段、受限工作空间、实时退出；保存数据和响应 | 无过期重放、观测新鲜、有效反馈及任务标签；完成交接模板 |

阶段 3/4 可使用无硬件 SDK solver 与 mock response；它们不是模型任务有效性的验证。阶段 5 后可能初始化/驱动电机。对 L5/X7/新负载等未在本文真机复验的组合，各阶段不能跳过。

仓库外的可选参考（同一主机存在，不是本库依赖）：`/home/hjadmin/projects/umi-arx/AGENT_CONTROL.md` 描述 guarded EEF + 外部 planner；该客户端 shadow 仍会通过环境执行启动 reset，不是纯离线。`umi-arx/OPENVLA_ARX5.md` 描述 OpenVLA 与旧 ZMQ 的已有流程，其中 TCP 增量与夹爪映射属于该客户端合同，旧 server 的标定/网络暴露要单独核对。其他模型从其自身 checkpoint 配置重新证明合同，不把这些路径当通用部署适配器。
