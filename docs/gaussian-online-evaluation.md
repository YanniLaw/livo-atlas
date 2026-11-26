# 在线封口留出评估

在已有[训练隔离](gaussian-holdout-isolation.md)上，子地图封口后对预选留出相机进行
只读评分。任务使用现有串行 GPU worker，保存派发时的模型和位姿来源；这是在线建图
期间的历史回看，不能解释为留出图像到达时的预测质量。[最终快照评分](gaussian-heldout-scoring.md)
仍为独立工具，结果标记为 `final_snapshot`，本功能标记为 `online_snapshot`。

## 启用与运行

先按训练隔离文档，从相同前端环境和配置的参考数据冻结 manifest，再从空模型运行。
使用短 bag 前缀时，要提前冻结对应前缀，不能在训练结束后调整划分。注意 bag 的记录
时间可能与消息时间相差多年，应按记录时间截取回放，再核对消息时间确定前端帧范围。

统一在 `config/gaussian.yaml` 配置：

```yaml
gaussian:
  enabled: true
  persist_submaps: true
  socket_path: /absolute/private-ipc/worker.sock
  output_directory: /absolute/new-gaussian-output
  joint_render_enabled: true
  joint_render_policy: spatial  # temporal 或 spatial
  evaluation:
    enabled: true
    online_enabled: true
    split_file: /absolute/frozen-split.json
    max_output_mib: 256
    min_free_disk_mib: 1024
```

这些字段用于补充现有配置；其余 worker、超时和 SLAM 参数仍使用原文件。
`online_enabled` 默认关闭，要求持久化、冻结划分和 joint render 引擎，但不要求发布
ROS 预览。ROS1、ROS2读取同一参数结构。启动 worker 的流程与原有在线建图一致。
worker 在握手时必须确认 `heldout_evaluation_v1` 及磁盘预算，不支持时明确失败。

两个基线回放脚本也支持：

```text
--gaussian-persist-submaps --gaussian-joint-render
--gaussian-evaluation-split /absolute/frozen-split.json
--gaussian-online-evaluation --joint-render-policy spatial
--gaussian-evaluation-max-output-mib 256
--gaussian-evaluation-min-free-disk-mib 1024
```

仍需给出脚本原有的 bag、输出和 socket 等必需参数。输出目录必须全新。
256 MiB 不是任意数据量的保证；worker 先按完整预选查询计划估算最大输出，再检查
剩余空间。更大的计划需要提高预算。不会为凑预算偷偷删除查询。

## 调度和状态

封口通知只登记查询元数据及封口时间，owner 不读取图像。最多保留 manifest 中64条
预选查询。读取已提交关键帧、重建观测、传输、推理和产物写盘均在现有 worker 路径上。
没有第二个 GPU 模型，也没有积压的图像队列。

内容确认及地图同步完成后，最多两个内容尝试让出一次评分/预览机会，两者交替。
派发只捕获一个查询，包含 `map_sequence`、完整位姿前沿、content/asset/binding身份。
地图确认额外携带已确认组的完整 primary 位姿补充，最多32组、512个位姿、64 KiB。
worker 将其与该次有界地图目录一起冻结，评分必须逐项匹配，报告保存两份 SHA256证明。
后续地图更新不会抹掉已经完成的历史成绩。空间选块使用与预览相同的 C++ 投影排序，
只考虑历史 owner 之前的块；时间策略选择直接前一块。原有预览仍要求最后保留的训练相机。

`evaluations/online-status.json` 覆盖整个预选计划，包含查询状态和报告 SHA256：

- `waiting_seal` / `pending` / `in_flight`：进行中；
- `succeeded`：worker 已发布对应报告并返回身份匹配的回执；
- `failed`：评分失败，原因写入状态；
- `cancelled`：正常停止前尚未派发，区分封口前和派发前；
- `skipped`：例如 `missing_model` 或未进入回放前缀的 `not_in_scope`。

正常停止取消未派发查询，在途工作在原有退出预算内完成。超时未收到回执记为失败。
退出时先接收邮箱中已经验证的评分回执，再处理无回执的查询。最终仅有界状态元数据
在 worker join 后补写；写入失败标记退出不完整。状态保留结构化错误码，每条诊断
截到128字节，使64条失败查询也能写入128 KiB状态预算。
评分 OOM、磁盘预算不足或参考观测不匹配会使该查询失败，保留训练状态；协议或
连接损坏仍按原 worker 错误处理。请求了在线评分且出现失败时，节点不会报告成功退出。

## 结果与解释

每个成功查询保存 `evaluations/views/<ID>/report.json`、浮点 NPZ及预览 PNG。
报告包含封口/派发时间、实际 `as_of`、单块/联合块指标、相机、来源及完整训练 lineage
摘要。`software` 保存worker握手中的Python/Torch/CUDA/gsplat/GPU信息及NumPy版本。
`inference.single` / `joint` 分别保存Gaussian数量、`timings`（变换上传、渲染、总耗时）
以及 `gpu_peak_allocated_bytes` / `gpu_peak_reserved_bytes`；没有联合推理时joint为null。
`inference.memory_peak_scope="worker_lifetime"` 表示当前worker进程在对应设备上累计的
分配器峰值，包含此前训练/评分，不能解释成单查询显存增量或显卡总显存占用；
评分不重置CUDA峰值。
浮点产物可用[固定指标](gaussian-heldout-scoring.md)复算。联合渲染不存在时 `joint`
为空，不能据此声称优于单块。评分只读取模型，不训练、不修改优化器或训练 RNG。

留出隔离针对 Gaussian 训练；SLAM 前端仍使用完整视觉和 LiDAR 数据，估计位姿不是
真值。封口后派发也可能使用后来的位姿优化结果，必须同时报告 `as_of` 和等待时间。
测试及本机真实回放证据见本工作空间 `validation/online-heldout-20261126`。

## 已知边界

原生与 Python 的查询相机齐次行统一使用1e-9绝对容差，空间查询逐项核对宽高与
owner保留相机一致，不能只比较像素总数。仍须通过冻结相机身份及SO(3)校验。
不把当前短段成绩解释为大场景画质或长期实时性能结论。

## 本机验证记录（2026-11-26）

Noetic 原生节点读取 `Retail_Street.bag` 的35秒记录时间前缀，预先冻结42帧，
对应5个预选查询。最终代码使用独立冻结的源文件、节点和动态库，以及空 Gaussian
目录运行，`joint_render_policy=spatial`。3个查询成功，2个在正常退出前尚未派发，
记为 `cancelled/shutdown_pending`，0失败；包含一次双块评分。

- 34次实际训练、680次损失计算均不包含 heldout；126个前端帧记录/PNG/PCD与参考
  数据逐字节一致。全部报告/产物摘要和 native回执一致，浮点样本精确重算指标。
- 查询25：单块 PSNR 9.705209 dB、SSIM 0.230214；联合块9.764113 dB、0.231824。
  这是同一快照的单个样本，不能据此推广质量改善。
- 评分耗时约0.53–0.75秒，封口到派发等待约1.24–4.70秒；这些是该次 GPU运行的
  测量值，不是调度的时间上界。GPU累计分配峰值约96.5 MiB，保留峰值114 MiB，
  worker RSS约1.12 GiB。正常退出约0.96秒，347次LIO、346次VIO。
- 完整 Python/CUDA 305项；SDK 82项测试；Noetic/Humble参数19/21项；退出进程7/9项，包含未协商评分能力
  时的非finalize退出。该故障在修复前返回0，修复后返回1并记录失败会话。
- 源码审查发现的5项重要问题均有先失败再通过的回归测试；争用测试改为独立线程
  持锁。完整 Python/CUDA 回归覆盖旧预览、训练隔离、来源、固定指标和新在线路径。

证据主文件：`validation/online-heldout-20261126/reviewed-audit.json`、
`reviewed/summary.json`、`reviewed-worker.json`、`reviewed-gaussian/evaluations/`。
ROS2在Humble容器中完成编译、参数和退出验证；本轮真实GPU/bag评分在Noetic上运行。

## 实施取舍

- 独立 `online_enabled`，训练隔离仍由 `enabled` 控制，默认行为不变；代价是需要显式
  开启两个开关。只要求joint render引擎，不要求ROS预览发布。
- 来源头共用编解码，查询独立；有效历史回执不按最新地图丢弃，代价是成绩必须连同
  `as_of` 阅读，不能当成最终状态质量。
- 计划至多64条元数据、同一worker至多一个在途查询；代价是繁忙或退出时可能取消评分。
- 前端停止、worker join之后允许补写有界终态元数据；代价是退出路径有少量同步I/O，
  失败会使退出不完整。
- 新能力的地图确认附带有界完整位姿证明；代价是最多64 KiB额外传输，以及显式能力
  协商。不支持新能力的worker在开启评估时拒绝运行。
- 按每查询最多2 MiB报告预留磁盘；代价是小预算可能更早被拒绝，而不是评分后删减计划。

### 协议边界与每查询运行证据补充

齐次行四个分量在容差内/外使用同一原始socket载荷核对两端验收；相机尺寸测试包含
同面积不同宽高与不同面积。实际CUDA单块评分检查软件、推理计时、累计峰值，并确认
训练状态和RNG不变。双块报告字段由35秒真实回放审计检查，报告与浮点指标仍做摘要
核对及精确重算。证据目录为 `validation/evaluation-evidence-20261126`。

单独改变高度的协议用例与有联合块的报告断言现已纳入常规回归，双块用例实际执行CUDA，
同时核对模型、优化器及训练RNG不变。完整配对验证见[空间留出对照](spatial-heldout-comparison.md)。

本轮完整Python/CUDA回归308项、SDK82项通过；Noetic/Humble节点编译通过。
使用新报告字段的Noetic35秒回放中，3个查询成功、2个正常取消；单块与联合块软件、
计时、累计显存字段通过审计。126个前端资产文件仍与参考一致，35次训练/700次损失
计算未包含留出帧；报告/产物摘要与浮点指标复算通过。无需新增开关或调整已有配置。
