# SLAM 异步位姿图工作队列实施计划

3.2 将关键帧登记、约束调度和结果验收交给 `pose_graph::PoseGraph` 的串行执行线。系统线程继续写入关键帧、子地图和正式位姿，发送命令与提交回执。归档写入迁移在 3.3 处理。 [阶段设计](../specs/2026-11-24-slam-engineering-optimization-design.md#32-工作队列与约束计算)

## 1. 求解与回环

- [x] 将现有 GTSAM 求解迁入 `optimization/Optimizer`，旧 `backend/pose_graph.h` 提供源码兼容入口。把候选搜索、几何配准和 `LoopClosure` 的任务执行放在 `loop_closure`。保留现有求解、候选排序、阈值与离线入口。
- [x] 保留 `CheckLoopCommit` 的 SDK 内部测试入口。图的执行线验收派发前缀，系统所有者根据发布时的真实尾帧生成 `prepared`，随后发布正式位姿。

## 2. 队列与提交

- [x] 接入现有 Cartographer `WorkQueue`、`WorkItem`、`Task` 和单线程 `ThreadPool`。关键帧提交后交付自有位姿、时间和成员修订。待登记关键帧按 FIFO 处理。每个执行任务处理有界批次。回环几何通过固定 `KeyframeReadView` 按需读取点云。
- [x] 图状态仅由执行线修改。保留一个在途回环查询和一个最新待查查询。成功或异常作为控制项回到执行线。在途结果交付系统所有者处理正式发布，图在有序回执后变更已接纳约束和版本。短锁保护队列及交接槽，配准、求解和写盘在锁外执行。
- [x] 增加 `pose_graph.max_pending_keyframes`。系统内置路径在写入原始关键帧前拒收满载输入并进入终止状态。完成回执和封存可以通过已满队列。`Finish` 先封存输入，再排空登记、回环及发布回执，最后写完成摘要。
- [x] 保留 `LoopBackend` 和 `livo_atlas::online_backend` 的 SDK 入口，共用同一个图协调实现。完整快照回调及派生地图消费者在系统所有者执行。

## 3. 验收

- [x] 受控慢任务期间继续提交原始关键帧并进入队列，约束任务让出线程后按顺序登记。测试单线程池队列满载仍交付完成回执、超限拒收且保留已提交资产、最新待查合并、异常传播、回环拒绝和正式位姿写入前后失败。
- [x] 测试旧 SDK 调用、仅检测模式和安装消费者。构建 SDK、Noetic 和 Humble。用同一机器同样设置对照 10 秒及完整 `Retail_Street.bag`，核对前端、已提交关键帧及版本化输出，记录实际调度差异并本地提交。

## 复核与实际执行边界

- 一个执行任务最多处理 16 个工作项。队列默认容纳 64 个待登记关键帧，满载时封存、回环完成和发布回执均可入队。查询结果释放后才入队回执，随后可派发下一查询。
- 独立 `LoopBackend` 活跃时由其 `Finish()` 排空系统。旧 `PoseGraph` 求解类保留独立类型身份、原构造和方法，内部使用 `Optimizer`。正式位姿发布后若最终查询记录写入失败，保留发布版本和接纳计数，归档状态为 incomplete。
- Noetic 将队列容量设置为 4。挂起回环后仍可提交后续 4 帧并封存，释放任务后完成提交。超限会拒绝第 9 帧并保留已提交的 8 帧。满载、超限、独立后端收尾及发布后写入失败四个用例分别重复 20 次通过。
- 独立 SDK 的 76 个 CTest 目标与 4 个安装后消费者通过，覆盖旧求解头文件、`livo_atlas::online_backend` 和检测入口。Noetic 与 Humble 各通过 9 个相关测试目标，均验证 `pose_graph.max_pending_keyframes` 的参数读取与零值拒绝。Humble 共享入口在模拟时钟暂停时也完成归档与关闭。

## 回放记录

对照基线为 `0a3b6c1`。两侧使用 Noetic、逻辑 CPU 2、OpenMP 单线程、1× 速度，启用回环、局部几何、全局几何和关键帧轨迹。10 秒和完整回放分别有 98/97 与 1,352/1,351 次 LIO/VIO。

- 完整回放 13 次查询全部完成，接纳同样的 2 条回环。前端轨迹和 ROS 载荷、516 个关键帧文件、185 份成员记录以及两个正式位姿文件逐字节一致。归档检查器确认版本 2。
- 最终轨迹与 Path 一致，最终显示地图在 13 个子地图中采样 100,000 点，两次逐点一致。完整回放各收到 28 个地图样本，22 个有相同版本、帧数和成员修订。21 个逐点一致，版本 1 的第 157 帧样本有 62 个点的坐标在 float32 精度范围内不同，最大绝对差为 `3.82e-6` 米。异步轨迹发布分别交付 125 和 127 个样本。
- 完整回放中待登记峰值为 1，单个在途查询。173 项观测与封存的排队时长中位数约 0.183 ms，P95 约 0.595 ms，最长约 655 ms。该样本包含与约束任务共用单线程期间的等待。

复现时将 `--workspace` 指向包含独立源码与二进制快照的 Noetic 工作区，给两侧分别设置新的输出目录。短回放使用 `--duration 10`。

```bash
OMP_THREAD_LIMIT=1 OMP_NUM_THREADS=1 python3 tools/run_baseline.py \
  --workspace /path/to/snapshot --bag /path/to/Retail_Street.bag \
  --output /path/to/new-result --duration 136 --rate 1 --cpu-affinity 2 \
  --image-queue-size 32 --semantic-output --geometry-submaps --geometry-map \
  --keyframe-trajectory --loop-closure --stage-timing
```

对照摘要 `queue-verification-verified.json`、归档检查和测试日志位于本机 `validation/slam-pose-graph-queue-20261124/`。
