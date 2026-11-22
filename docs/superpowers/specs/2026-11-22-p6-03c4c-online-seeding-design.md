# P6-03c4c 在线覆盖优先采样试验

将 C4b 的 `coverage` 接入已有持久子地图 worker，使用 Noetic 回放检查训练调度、渲染接收、
封口与资源开销。在线 worker 默认使用 `uniform`，本阶段把覆盖优先作为显式选项。
[C4b 配对结果](../../p6-03c4b-validation.md)

## 接入位置

策略属于 GPU worker。`online_worker.py --seed-policy {uniform,coverage}` 在启动时读取，
整个连接使用同一个值。ROS1 Noetic 和 ROS2 Humble 通过现有 Unix socket 调用同一个 worker。
启动方式与现有独立进程一致，试验沿用当前握手和资产格式。

1. 普通 `view` 使用该策略初始化本帧。仍按原有时序校验、更新本地相机、训练，再提交候选。
2. `rebuild` 从已确认观测重训，每一帧使用同一策略。已有候选需要 owner 确认后才取代确认态。
3. `reuse` 按原方式重绑相机并预览，训练步数为 0。`discard` 从确认态恢复模型，后续候选沿用
   worker 的启动策略。checkpoint、地图提交及 stop 继续由既有流程处理。
4. schema 1 使用 `uniform`。选择 `coverage` 后收到 schema 1 握手，worker 在创建模型前
   返回 `protocol_error`。非法 CLI 值在绑定 socket 前被拒绝。预渲染失败沿用现有 worker
   错误通路，失败的候选没有 owner 确认，也不会生成新的持久引用。

复用 C4b 的 `< 0.05` alpha 阈值和至多半数配额。每训练帧最多一次预渲染、1,000 个种子和
20 步，窗口最多 10 帧，单块最多 10,000 个高斯。rebuild 对保留帧各训练 20 步。预渲染和
训练在同一个串行 worker 上执行。联合渲染与发布沿用当前资源边界和超时设置。

## 本机预检

`42c9ac9` 的 C++、ROS 入口和配置与 C3c2b 的 `e0a26e9` 相同。本机复用已校验的 Noetic
Release 二进制，在独立目录用当前 Python worker 回放 `Retail_Street.bag` 前 10 秒。
持久化、空间联合渲染、ROS 预览和 Finalize 均开启。RTX 4060 Laptop、节点固定 CPU 0，
回放倍率 1，发布图像队列 32，OMP/BLAS 线程为 1。

| 均匀采样预检 | 本机结果 |
| --- | --- |
| 观察到的 IMU / 点云 / 图像消息 | 2,028 / 101 / 100，符合 bag 区间统计 |
| 训练确认 / 输入截止前收到的 READY | 5 / 4 |
| 最终可用并持久化的子地图 | 1 / 1，Finalize 成功且 worker 正常退出 |
| PyTorch 峰值 allocated / reserved | 67.13 / 88.00 MiB |
| worker 峰值 RSS / Finalize | 1,085.11 MiB / 0.172 s |

历史完整回放有 172 帧、13 个子地图、2 次回环接纳，最终位姿版本为 2。一次均匀采样完整回放
报告了 18 次合并、21 次 stale 和 17 次 rebuild。真实调度取决于异步处理时序，成对验证须
分别记录两次实际执行的训练帧和操作。证据在 `validation/P6-03c4c-preflight-20261122` 的
`preflight.json`、`uniform-short/` 和 `g-uniform-short/`。
[完整回放基线](../../p6-03c3c2b-validation.md)

## 对照和验收

受控 GPU 用例覆盖两种策略、重建、重用、拒绝后恢复及握手。普通训练和 rebuild 都检查实际调用
的策略。失败用例检查此前已确认资产和地图引用的身份。保留原始候选消息字段，继续用现有解析器
验证 wire 消息。覆盖优先的实测记录需要同时看到预渲染执行和非零预留配额。

先在两种策略上各回放前 10 秒，再各回放完整 bag。固定 bag、二进制、GPU 依赖、节点 CPU
亲和性、回放速率、图像队列以及所有 ROS/Gaussian 参数。每次使用独立 worker、私有 socket
和新输出路径，持久化、空间联合渲染、预览发布和 Finalize 均开启。比较以下实际结果。

| 核对项 | 判定 |
| --- | --- |
| 前端与回环 | 输入计数和处理周期一致。轨迹、前端数值文件、原始关键帧、原话题语义严格一致。完整回放两侧均为 172 帧、2 次回环接纳及位姿版本 2 |
| 调度及资产 | 两侧 worker 正常退出、无失败或资源拒绝、pending 和 in-flight 各至多 1。Finalize 成功，eligible 集合一致且全部 current/durable，完整回放 13 块 |
| 联合渲染和 ROS 接收 | 两侧都由独立接收与图像证据确认截止前 READY。完整回放沿用至少 6 次双块贡献、至少 3 个查询组的检查 |
| 性能 | 列出 view 与 rebuild 的 worker 耗时、预渲染次数及 P50/P95、峰值显存/RSS、首帧 READY 时间、合并与丢弃计数以及封口耗时 |

验证脚本在 worker 外层记录实际训练调用和 owner 的 accept/discard 对应关系。按会话、job 和
训练序号关联证据。每次训练记录有效策略、关键帧、保留窗口、步数和 C4b 种子计数。统计 view 与
rebuild 时区分整项 worker 操作耗时和单次训练耗时，分开列出首次训练和后续训练。对训练输入、
位姿版本、保留窗口相同的视角再列配对耗时和匹配数。报告基于现有超时的余量及各侧的实际样本数。

在线预览来自已确认训练视角。画质收益仍以 C4b 固定留出为依据，本阶段报告在线代价与运行可靠性。
实测范围为本机 Noetic。Humble 使用同一 worker 参数和协议，GPU 回放可用同样的验收方法。

[实施计划](../plans/2026-11-22-p6-03c4c-online-seeding.md)
