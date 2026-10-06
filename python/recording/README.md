# ARX5 主从 HDF5 采集

一台主臂用于示教、一台从臂执行任务，记录的是**单执行臂**数据。
复用 `examples/leader_follower_teleop.py` 的限位、速度前馈及夹爪力反馈逻辑。

## 启动

在 `arx5-sdk/python` 目录、已有 ARX5 SDK 的 Python 环境中执行：

```bash
python -m pip install -r recording/requirements.txt
python examples/record_episodes.py X5 can1 X5_umi can0 \
  --dataset-dir data/demonstrations
```

不要同时运行原来的 teleop 脚本；recorder 自身驱动主从控制。
当前只有一台腕部相机，默认使用 Vin 1 的设备路径 `/dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0`（当前为 `/dev/video0`），记录为 `cam_wrist`；默认不开启主视角相机。
2026-10-06 单腕部相机检查通过，实测接收约 59.8 FPS。Vin 2/3/4 返回固定像素值 7 的无信号灰画面，不作为采集视角。
启动时先检查相机连续出帧，再让两臂同时回到 home，然后**两臂均进入重力补偿拖动模式**，可分别用手调整位置。
参数 `-g` 默认 `-3.4`，沿用当前示例；应使用已标定的夹爪值。
若拖动模式明显下坠，可添加 `--diagnostics`，每秒输出两臂实际重力补偿开关、故障状态、kp/kd、SDK 输出力矩及实测关节速度。启动时会打印加载的 SDK 路径；输出力矩是 SDK 命令，不代表电机实际产生了同等力矩。

1. 在同一入口选择语言指令或输入 `q` 退出，不必先填写指令。首次可直接输入“把红色方块放进碗里”。
2. 后续列出历史：输入 `1` 复用第一条，`e 1` 将第一条放进可编辑输入行，直接输入文本创建新指令。
   `n 文本` 强制作为新指令，适用于纯数字等特殊文本。历史保存在当前数据目录 `instructions.json`，中文以 UTF-8 保存。
3. 输入 `r` **回车**后，从臂先平滑恢复位置增益、限速对齐主臂，然后开始记录。
   对齐期间请保持主臂平稳，对齐运动不会写入 demo。准备阶段 `n` 更换指令，`h` 回零后恢复双臂拖动，`q` 退出。
4. 采集中 `ok` / `f` 回车分别标记成功/失败并保存；`d` 丢弃本轮；`q` 中断本轮并退出，当前数据不作为已标注 demo。
   已移除 `s` 和“结果未知”保存选项。默认到 60 秒停止采集、恢复双臂拖动，再等待 `ok/f/d/q` 选择。
   无人值守运行可显式给 `--result-on-timeout success` 或 `failure`；这代表操作者预设标签，不是程序识别了任务成败。
5. 保存或丢弃后暂停跟随，**主从臂都保持重力补偿、关节 kp=0、夹爪反馈清零**。
   主臂阻尼默认 SDK kd 的 0.05 倍，从臂为 0.5 倍，抑制从臂自由拖动时的下坠速度。
   写盘、选择下一条语言指令和等待开始时均保持这一模式；不会在每轮结束时自动回零。
   正常退出、Ctrl+C、SIGTERM、SIGHUP、输入流关闭、预览关闭以及相机/写盘/控制线程异常，均先停止采集和主从线程，再尝试两臂慢速回 home。
   中断数据保留在 partial 文件或多 demo 文件的 `interrupted` 分组。

默认打开一个桌面预览窗口，腕部和可选主视角并排显示，包含相机名称、实际图像尺寸、图像年龄和采集状态，默认刷新 30 FPS。
准备期间显示最新相机画面；进入采集后，写入线程完成一行后从 HDF5 图像 dataset 回读，再送到窗口，标注 `HDF5 READBACK FRAME` 及其帧号。
窗口使用的是 HDF5 回读副本，不引用 sampler 提交的内存图像，也不会自行选择另一张相机帧。模式切换时会丢弃队列中遗留的实时预览帧；到时等待结果标签时保留最后一张已写入图像。UI 忙时可能跳过部分显示帧，不影响数据记录。
图像仍会缩小以适应窗口并加文字，写入的数据不包含这些窗口缩放和文字。写入完成不等于每帧都 fsync，仍按原周期 flush。
相机默认请求 **1920×1080**，每帧采用 UMI `get_image_transform` 的同一规则：保持比例缩放，再居中裁剪，最后从 BGR 转为 RGB；HDF5 固定保存 **256×256 RGB**。
因此 1920×1080 输入会先缩为 456×256，再居中取 256×256，而不是先裁原始图再缩放。准备预览和采集期间的 HDF5 回读预览看到的也都是这张 256×256 图。
`--width/--height` 只控制请求的输入分辨率，`--image-size` 控制方形保存尺寸，默认 256。程序启动会打印每路实际保存尺寸。预览以独立进程运行，编辑指令时也能持续刷新。
窗口关闭、按 Q 或 Esc 触发中断并慢速回 home；这时当前轮次按中断数据保存。
预览启动失败会在创建机器人控制器前终止，运行中预览故障也会触发回零。
终端控制支持 SSH；没有桌面时显式指定 `--no-preview`。仅在输入语言指令时使用 readline 编辑。
单 demo 模式 `d` 保留 `.discarded.hdf5`；多 demo 模式保留在 `discarded` 组。训练时只读取已完成文件及其已完成 demo。
原始数据没有自动删除；长时间断电或强制 kill 导致的 HDF5 损坏不保证可恢复。

可先只检查相机（不会连接 CAN 或移动机械臂）：

```bash
python examples/record_episodes.py --check-cameras
```

与已验证的 ffplay 输入参数一致地检查 1080p / 60 FPS：

```bash
python examples/record_episodes.py --check-cameras \
  --wrist-camera /dev/v4l/by-id/usb-MACROSILICON_Vin_1_20250811-video-index0 \
  --fourcc MJPG --width 1920 --height 1080 --camera-fps 60
```

正常采集默认请求 1920×1080 / 相机 60 FPS，再变换保存为 256×256；`--fps` 控制数据记录频率，相机时钟模式下须与 `--camera-fps` 相同。
ffplay 与 recorder 不应同时占用同一个采集节点。

若设备能打开但 5 秒内没有图像，检查相机/采集卡输入信号和设备节点；设备支持时也可测试 `--fourcc YUYV`。
启动连接机器人之前、每轮开始之前及对齐完成后，均要求每个相机连续出现至少 3 次新帧。
检查图像尺寸/类型、空帧、全黑及采集卡暗灰空帧（例如所有像素为 7 或 16）、时间戳新鲜度，以及帧序号和时间戳是否推进。
暗灰判定为最大像素值不超过 32 且全帧像素范围不超过 2；预览会标明 `NO CAMERA SIGNAL`，不再只显示黑色。
允许切换格式后短暂的起流空帧，但必须在 2 秒内收到连续有效新帧；窗口就绪还会等待第一组图像实际送达并绘制。
旧帧反复返回、陈旧帧或无画面时不会开始采集；已连接机械臂的故障退出仍执行同步回零。
检查不能判断构图、焦点、蓝屏/彩条等内容是否正确，开始前仍应查看预览窗口。
依赖锁定为本机 Python 3.9 / NumPy 1.26 验证过的 h5py 3.10、OpenCV 4.8.1，避免新 wheel 与当前 NumPy 的二进制接口不兼容。

## 回零行为及边界

`--home-seconds 3` 指定**两臂同步回零的目标总时长**，含 0.3 秒增益过渡和至少 0.35 秒末段。
两臂共用同一时间轴，每个循环都给两臂发送指令，不再依次等待。
从当前实测位置开始，平滑恢复增益，以五次插值回零；保留 SDK 的第 3 关节 0.03 rad 接近点，再归零，夹爪全开。
默认规划关节速度上限 1.5 rad/s、夹爪速度上限 0.1 m/s，并取 SDK 各轴限速与这些值的较小者。
一般姿态约 3 秒；可设 `--home-seconds 2.5`，但远距离回零为满足限速会自动延长，完成后检查实测位置。
开始采集前对齐仍保持原先的 0.35 rad/s、夹爪 0.025 m/s，不随回零加速。
一臂报错时另一臂继续执行，未到位会明确报错。轨迹不包含场景碰撞规划。
回零时重复 Ctrl+C/SIGTERM/SIGHUP 不会打断已经开始的退出恢复动作。

recorder 不再主动调用 `set_to_damping()`。**SDK 原有析构行为仍会进入阻尼模式**，所以先完成回零才退出程序。
无法保证断电、拔掉 CAN、硬急停、SDK/进程崩溃或 SIGKILL 时回零；软件没有机会执行或无法驱动电机。
如果回零尝试失败，报错并保留数据，不能将“已发送 home 目标”当成“已到达 home”。

## X5_umi 下坠调整

从臂重力补偿已显式开启；待机保持 `kp=0`，不通过锁定位置来实现拖动模式。
从臂待机阻尼改为独立参数 `--follower-idle-kd-scale 0.5`，不再跟随主臂的 0.05 倍阻尼。
阻尼可以降低漂移速度，但无法在静止时补足重力模型误差。
当从臂型号为 `X5_umi` 时，默认 `--follower-gravity-scale 1.05`（增加 5%），其他型号默认 1.0。
该值是针对下坠反馈的初始调整，未经当前末端负载标定；若出现上飘或某些姿态反向漂移，应减小倍率并检查 URDF 的负载质量/质心。
这个倍率在创建从臂控制器前应用，影响从臂整个运行周期的重力补偿，不改动主臂和原始 URDF；倍率和实际重力向量写入 episode 元数据。
可以显式恢复原补偿量：`--follower-gravity-scale 1.0`。负载模型不准时，单一倍率不保证所有姿态都无漂移。

## 相机与频率

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--control-hz` | 500 | Python 主从循环目标频率；底层 SDK 保持配置中的 2 ms 周期 |
| `--fps` | 60 | 状态、动作、图像配对记录频率；相机时钟模式下须与 camera-fps 相同 |
| `--camera-fps` | 60 | 向 V4L2 请求的相机帧率 |
| `--sample-clock` | camera | `camera`：腕部相机每个新帧触发一次记录；`periodic`：定时采样最新帧 |
| `--width --height` | 1920 1080 | 向相机请求的输入图像尺寸 |
| `--image-size` | 256 | UMI 等比例缩放、居中裁剪后的方形 RGB 保存边长 |
| `--fourcc` | MJPG | 相机输入格式，可改 YUYV |
| `--episode-seconds` | 60 | 自动结束时长；0 表示手动结束 |
| `--max-camera-age` | 0.03 | 图像接收时间最大允许滞后，单位秒 |
| `--queue-size` | 64 | 磁盘写入队列上限，避免内存无限增长 |
| `--preview-fps` | 30 | 预览刷新频率；采集时显示已写入图像 |
| `--demos-per-instruction` | 1 | 1：一条 demo 一个文件；大于 1：同指令每组 N 条；0：同指令不限条数 |
| `--home-seconds` | 3 | 双臂同步回零目标总秒数，含增益过渡和末段 |
| `--home-speed` | 1.5 | 回零规划关节速度上限 rad/s，远距离自动延长时间 |
| `--follower-idle-kd-scale` | 0.5 | 从臂待机阻尼 / SDK 默认 kd |
| `--follower-gravity-scale` | X5_umi: 1.05；其他: 1.0 | 从臂重力补偿倍率 |

未来接上主视角相机后，只需添加参数：

```bash
python examples/record_episodes.py X5 can1 X5_umi can0 \
  --dataset-dir data/demonstrations \
  --main-camera /dev/video2
```

`/dev/video2` 仅为示例，以实际视频采集节点为准；推荐使用 `/dev/v4l/by-id/...` 稳定设备路径。
未提供 `--main-camera` 时不创建主视角字段，也不会用黑图填充。添加相机后可继续同一目录，但训练应检查每个 episode 的视角列表。
相机驱动不一定接受请求的分辨率/FPS：启动时打印实际图像尺寸及驱动报告 FPS，并写入元数据。
GoPro/HDMI 采集默认使用 `--sample-clock camera`：500 Hz 机器人控制循环不等待相机，只有检测到
`cam_wrist` 序号增加时才提交一条样本，因此参考相机帧不会重复写入。多相机情况下腕部相机是参考时钟，
其他相机仍按最近时间帧配对；这属于基于主机接收时间的软件同步，不是曝光硬件同步。
录制期间任一视角超过 `--max-camera-age` 时只丢弃当前样本并限频打印告警，不退出机器人控制线程；
相机断开、读取失败或参考相机序号倒退仍作为设备故障停止录制。
主从控制、相机读取、HDF5 写入独立运行；队列满、断流、陈旧帧、写盘失败均终止当前 episode，不静默丢掉记录。

同一采样周期先读取主从状态，再发送由主臂状态计算并按从臂限位裁剪的目标，最后将这组状态与目标交给 recorder。
每个样本取当时最新可用图像，相机时间是 OpenCV `read()` 返回后的主机单调时钟，**不是硬件曝光时间**。
相机帧可能在读取机器人状态后、打包样本前到达。不同相机没有硬件同步，不能视为严格同步多视图。
重复图像通过相机 sequence 检测；漏掉的采样周期写入 quality，实际记录频率写入 `actual_fps`。
SDK 时间戳分别相对于各自控制器启动时间，不能直接跨控制器相减；统一对齐使用 host monotonic 时间。

## HDF5 内容

默认每轮独立文件 `episode_000000.hdf5`，自动选择未占用编号，不覆盖已有文件。
先写 `.partial.hdf5`，队列写完并关闭后才发布正式文件名。每 30 帧 flush；中断文件包含已提交样本数。

同一条指令采集 450 条 demo，写入一个 episode 文件：

```bash
python examples/record_episodes.py X5 can1 X5_umi can0 \
  --demos-per-instruction 450 --episodes 450
```

只选择一次指令，每轮结束后按 `r` 开始下一条，不再重复询问语言。成功或失败且已保存的 demo 都计数，丢弃和中断不计数。
`--episodes` 表示总共保存多少条 demo 后退出，不是文件数。
`--demos-per-instruction 0` 表示同一指令持续采集直到手动更换或退出；准备阶段 `n` 可提前结束当前文件并更换指令。
指定 N 时每 N 条结束当前文件，再选择下一组指令；`--instruction '文本'` 可预设指令。

多 demo 模式结构：

```text
episode_000000.hdf5
  language_instruction
  data/demo_0/{observations, action, timestamps, ...}
  data/demo_1/{observations, action, timestamps, ...}
  discarded/demo_2/...
  interrupted/demo_3/...
```

每条 demo 有独立的语言、成功/失败标签、样本数和采集元数据。下表的字段在多 demo 模式下位于 `/data/demo_N/` 内。
采集中使用 `.partial.hdf5`，每条结束即提交分组；更换指令或退出时发布正式文件。
后续 demo 中断不会丢掉此前已完成的 demo：已完成数据留在 `data`，中断数据在 `interrupted`；崩溃残留的 `_pending` 不用于训练。
根属性 `num_demos`、`num_samples` 只统计 `data` 中已完成的 demo。没有已完成 demo 的文件保持 partial。

| 路径 | 形状 / 内容 |
| --- | --- |
| `/language_instruction` | 单个 UTF-8 字符串；同样保存为根属性 |
| `/observations/qpos` | `(T,7)`，从臂 6 关节角 rad + 夹爪宽度 m |
| `/observations/qvel` | `(T,7)`，关节 rad/s + 夹爪 m/s |
| `/observations/effort` | `(T,7)`，关节与夹爪电机力矩 Nm；末列不是接触力 N |
| `/observations/ee_pose` | `(T,6)`，相同 qpos 经从臂 FK 得到的 x,y,z,roll,pitch,yaw，m/rad，从臂基座坐标系；模拟模式不生成 |
| `/observations/images/cam_wrist` | `(T,256,256,3)`，RGB uint8，腕部相机 |
| `/observations/images/cam_high` | 同上，可选主视角 |
| `/action` | `(T,7)`，该控制周期提交给从臂的绝对关节目标 + 夹爪宽度，已按从臂位置限位裁剪 |
| `/action_velocity` | `(T,7)`，提交的速度前馈；夹爪速度沿用 teleop 默认 0 |
| `/leader/qpos`, `/leader/qvel` | `(T,7)`，主臂原始实测状态，未裁剪 |
| `/timestamps/observation`, `/timestamps/command` | `(T,)`，主机读取状态前、提交从臂命令后的 monotonic 秒 |
| `/timestamps/relative` | `(T,)`，距离本轮首个样本的秒数 |
| `/timestamps/leader_sdk`, `/timestamps/follower_sdk` | `(T,)`，SDK 原始状态时间戳 |
| `/timestamps/cameras/<name>` | `(T,)`，所用相机帧的主机接收时间 |
| `/quality/camera_sequence/<name>` | `(T,)`，相机帧序号，重复值意味着复用相同图像 |
| `/quality/missed_sample_periods` | `(T,)`，相机时钟模式为参考相机跳过的序号数；周期模式为循环延迟跳过的目标周期数 |

根属性包括 `sim`、`schema_version`、`complete`、`status`、`success`、`stop_reason`、`num_samples`、`actual_fps`、
`sample_clock`、`reference_camera`、语言指令及 JSON 配置。
`metadata_json` 保存模型、CAN、相机、频率、控制参数、墙上时间与单调时钟锚点、单位及对齐语义。
图像使用 HDF5 LZF 无损压缩；沿用 ALOHA 中 `compress=False` 表示“不是 JPEG 字节数组”，h5py 读取自动解压。
64 个 256×256 双视图样本的队列最多约占 24 MiB 图像内存；真实图像的存储量取决于内容，需留足磁盘空间。

该格式采用 ALOHA 的 observations/action 结构，便于转换为训练格式。
它是带语言指令的真实机器人数据，**不是开箱即用的 LIBERO 数据集**：LIBERO 的组织及末端增量动作语义与此处的关节绝对目标不同。
`action` 是应用层提交目标，底层 SDK 仍可能插值/限速；不是电机最终输出，也不是从臂实测位置。
这里保留绝对关节目标和 FK 状态，后续转换应明确坐标系、动作空间与夹爪归一化，不能只重命名字段。

参考原始实现：

- [Mobile ALOHA record_episodes.py](https://github.com/MarkFzp/mobile-aloha/blob/main/aloha_scripts/record_episodes.py)：分轮保存、观测/目标配对、HDF5 字段。
- [KAI0 DAgger collector](https://github.com/OpenDriveLab/kai0/blob/main/train_deploy_alignment/dagger/agilex/agilex_openpi_dagger_collect.py)：人工开始/停止、保存/丢弃、30 Hz 配置及多相机观测。

## 无硬件验证

```bash
python examples/record_episodes.py --simulate \
  --dataset-dir /tmp/arx5-recorder-smoke \
  --instruction '把红色方块放进碗里' --auto-start \
  --episodes 2 --episode-seconds 2 --result-on-timeout success
python -m unittest discover -s tests -p 'test_hdf5_recorder.py' -v
```

模拟模式不导入机器人接口、不打开 CAN/相机，生成带 `sim=True` 的 HDF5。
模拟模式默认不打开桌面窗口；加 `--preview` 可测试窗口，例如：

```bash
python examples/record_episodes.py --simulate --preview --check-cameras --main-camera /dev/video2
```

`--instruction` 可用于批量采集时固定语言指令；正常人工采集时省略，逐轮选择。
无交互批量运行需同时给 `--instruction --auto-start --episodes N --episode-seconds S --result-on-timeout success`（或 failure）。
各频率是目标值，不是实时性保证；模拟验证通过后，还需在实机检查帧率、图像视野和跟随质量。
