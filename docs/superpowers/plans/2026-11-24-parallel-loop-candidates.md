# 回环候选并行验证实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 在 PoseGraph 的 Cartographer 线程池中并行验证同一查询的独立候选，保持确定性归并和串行提交。

**Architecture:** LoopClosure 复用同步入口的准备、验证和接纳逻辑。内部调度入口预先注册有限批次的 Cartographer Task 依赖图，单线程池也能完成。候选共享不可变源几何，各自持有不可变目标几何；可变 GeometryMapper 只在准备/归并任务使用。独立候选完成后按检索顺序归并，再执行私有试算与几何复核，提案回到 PoseGraph WorkQueue 接纳。每个系统仍只有一个活动查询。

**Tech Stack:** C++17、现有 Task/ThreadPool、PCL、GTSAM、Noetic、Humble。

**Spec:** `docs/superpowers/specs/2026-11-24-slam-engineering-optimization-design.md` 的 Task 依赖及候选顺序要求。

## Global Constraints

- 复用同一线程池，不增加后台线程或 worker 内等待。
- 保持公共同步 `ProposeLoopUpdate`、`LoopClosure::Run` 和注入式 LoopTaskRunner 行为。
- 线程池为 N 时，每批最多 max(1, N-1) 个候选，同时不超过几何驻留数减一；查询源从关键帧加载一次。每次读取目标前刷新源的缓存访问序号，保持源驻留。
- 纯元数据准备在 PoseGraph 锁外完成；在短临界区内注册完整依赖图，后续工作项不再 Schedule。Stop 只在注册完成后销毁线程池。
- 候选任务异常按原候选顺序报告；不从工作项抛出未处理异常。
- 存档、图状态、版本接纳及提交顺序由已有串行 WorkQueue 持有。
- 保持队列满、封存、退出、失败和独立 LoopBackend 生命周期约定。

## Review Focus

1. N=1 的依赖图不会阻塞；退出期间不会向已销毁线程池调度。
2. 小几何驻留预算下，批次快照不会无限滞留。
3. 并行结果按候选顺序归并，异常和相同得分不依赖完成先后。
4. 封存、满队列及发布等待仍允许已派发查询完成。
5. 同步 SDK 消费者及显式注入 runner 继续使用原协议。

### Task 1: 分离查询阶段并接入依赖调度

**Files:** `src/loop_closure/loop_admission.cc`、内部查询头及实现、`src/loop_closure/loop_task.cc`、`src/pose_graph/{pose_graph,loop_backend}.{h,cc}`、`src/core/livo_system.cc`、现有回环与生命周期测试。

**Interfaces:** 内部准备阶段生成有序候选列表；内部完成阶段消费有序验证报告。内部 `PrepareLoopTask` 在锁外验证任务并生成初始报告，`ScheduleLoopQuery` 接受该报告、自有 LoopTaskResult、固定选项、现有 ThreadPoolInterface、线程预算和完成回调；空 runner 表示使用此原生调度，显式 runner 继续作为单个自有任务执行。

- [x] 添加单线程完成、多候选重叠、顺序和预算边界测试，观察原实现的失败。
- [x] 提取同步及异步共用的准备/接纳阶段，实现有界批次和依赖归并。
- [x] 接入 LivoSystem 与默认 LoopBackend，保留显式 runner 测试入口；验证失败回收和停止。
- [x] 通过完整 SDK、Noetic/Humble、安装消费者及生命周期重复测试。
- [x] 固定 CPU 和前端预算，完整 bag 对比后台 1/4/6 线程、候选次序、最终位姿、归档检查、耗时和内存。
- [x] 独立复核两项工作，更新执行记录，本地提交 `feat: verify loop candidates with dependent tasks`。

证据放 `validation/parallel-loop-candidates-20261124/`。不重复生成大型高斯归档；本次边界为候选验证至已提交位姿。

## 验证记录

实现与验证已完成。SDK 76 个 CTest 目标、Noetic 9 个核心目标和 40 个 ROS 用例、Humble 10 个目标、4 个安装后消费者通过。完整 bag 的五组预算对照及最终程序的补充回放通过，前端资产字节一致，最终位姿差低于 1e-12。实际并发、故障注入、独立复核与待补测试见 [验证记录](../../threading-validation.md)。
