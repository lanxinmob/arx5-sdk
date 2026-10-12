# Agent Instructions

- 设计保持精简，用户应能从 README 及其链接完成操作。
- 真机任务先读 [真机索引](docs/hardware/README.md)，按 [环境配置](docs/hardware/setup.md)、[操作流程](docs/hardware/operations.md) 查阅；其他策略模型接入读 [动作合同与验收](docs/hardware/model-integration.md)。
- 本机设备与标定见 [配置快照](docs/hardware/current-machine.md)；它是有日期的事实记录，启动前重新确认，不作为新硬件默认值。
- 创建 SDK controller 就会收发电机；无硬件验证用配置、solver、dry-run 或 simulate。查已有服务状态时不要再创建占用相同 CAN 的 controller。
- 参数修改在构造 controller 前生效，确认各入口的标定覆盖和退出/超时行为；保留实测反馈，不能将目标发送成功当作到位。
- 实现时小步提交，commit message 为英文一行；保持线性历史，远端有新提交时 rebase，遵循 Git best practice。
- 验证使用临时目录，完成后删除本次 test 文件与产物，不删除原有示例或真实采集数据。
- 联网优先尝试 `http://127.0.0.1:7897`，失败时比较直连，不擅自修改全局配置。
- 保留原始错误、状态码和实际结果；先验证原因再下结论。相同方法反复无效时换工具、入口或假设，不只调参数。
- 交付区分源码事实、本次验证、历史真机记录和未验证项；更新入口时同步真机索引、参数覆盖表及退出矩阵。
