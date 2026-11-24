# 后续优化与工程改造建议

C4c 对照基准为 `2318335`。该阶段已用 Noetic 跑通几何回环、在线高斯训练和封口的配对回放。
以下记录待推进项和工程改造建议。[C4c 验证](p6-03c4c-validation.md)

## 后续 3DGS 优化

| 方向 | 交付和验收 |
| --- | --- |
| 在线留出画质与指定相机渲染 | 固定仅用于高斯评分的帧，记录回放时已确认的地图版本。给定相机内参和位姿，渲染 RGB、深度和覆盖掩码。对比 `uniform` 与 `coverage` 的画质、深度和开销，分别记录运行中结果与 Finalize 结果。首轮可沿用 Retail_Street |
| 自由视角浏览与高斯地图导出 | 用已确认快照和指定相机驱动查看。导出训练后高斯参数，并用实际查看器验证子地图位姿、尺度、颜色和不透明度。现有 COLMAP 导出继续提供重建观测 |
| 画质和几何一致性 | 从留出视角定位空洞、重影和跨块接缝。测得有效密度不足时，试验定额内增密与裁剪，同时核对共同深度误差、显存和训练延迟 |
| 跨场景与真值复验 | 有合适数据时补测不同光照、动态物体及重访。使用有真值的路线评估轨迹误差、回环误检和留出视角画质 |
| Humble GPU 全链路 | 使用共用 worker 验证实际训练、回环修正、ROS2 接收及封口，记录跨电脑复用的构建和启动步骤 |
| 长时地图与续建 | 处理历史高斯资产的总容量和跨会话续建，验证版本、故障恢复及重访时的地图一致性 |
| P6-04 高斯辅助定位 | 先记录基于确认地图的位姿诊断，再用有真值的退化场景对照收益，结合已有视觉量测的相关性考虑估计器接入 |

在线留出和自由视角查看可以共享一个指定相机的渲染入口。地图质量优化按留出结果选定首个问题。
[在线 Gaussian 边界](online-gaussian.md) · [C4b 留出结果](p6-03c4b-validation.md) ·
[阶段路线](superpowers/specs/2026-11-08-online-submaps-design.md#p6-01-之后的阶段路线)

## ROS 执行器与后端线程

当前两端共用 `src/app/livo_main.cc`。Noetic 用 `ros::AsyncSpinner` 接收消息，主循环调用
`ProcessNext()`。Humble 使用 `MultiThreadedExecutor`，处理定时器串行持有前端状态。
启用相关功能后，`PoseGraph`、`OnlineGaussianMapper`、Gaussian 预览输出、轨迹和几何地图输出
各有后台线程。GPU 训练另由 Python worker 进程执行。编译支持时，LIO 和 VIO 内部还会并行计算。
[共用主入口](../src/app/livo_main.cc) · [Humble 节点](../src/ros2/livo_node.cc) ·
[回环所有权与停止](online-loop-backend.md#sdk-所有权与停止)

位姿图使用 Cartographer 2.0.0 的 `ThreadPool`、`Task` 和 `WorkQueue`。单线程执行线
依序写入观测及成员修订、调度回环、验收提案并提交正式位姿。几何验证、GTSAM 试算和写入在交接锁外执行。
`LivoSystem` 收取已提交前沿并向 SDK、轨迹、几何地图和高斯输出交付。
[Cartographer 来源与许可](cartographer-scheduler.md) ·
[位姿图与归档](../src/pose_graph/pose_graph.cc) · [系统发布边界](../src/core/livo_system.cc)

线程池和日志迁移后，Noetic 完整回放的轨迹、前端数值输出和原始关键帧与 C4c 均匀采样侧一致。
13 次回环查询均完成，接纳 2 次，位姿版本为 2，13 个高斯子地图完成封口。工作区记录位于
`validation/cartographer-thread-pool-20261122/full-verified-comparison.json`。

C4c 的 Retail_Street 完整均匀侧有 13 次查询，其中 4 次的候选数大于 1。第 12 项任务的
5 个候选验证耗时合计约 809 ms，GTSAM 求解约 6.8 ms。候选并行的收益需要按端到端延迟、
前端负载和内存复测。原始记录位于工作区的
`validation/P6-03c4c-20261122/full-uniform/keyframes/loop_closure/queries/12.yml`。
[候选验证与优化](../src/loop_closure/loop_admission.cc)

### 后续候选并行

1. `LoopWorker` 从同一版本快照读取候选，独占现有可变几何缓存。向线程池交付带所有权的只读
   几何对象，每批的活跃对象和任务数受预算约束。
2. 同一回环查询的独立 `VerifyLoop()` 可并行。完成结果按原候选顺序汇总，随后由后台执行线
   串行筛选、试算和复核。池内任务直接返回验证结果，等待批次结果发生在池外。
3. `LoopBackend` 独占接受历史与位姿提交权。保留同一时刻一个在途查询和一个最新待查 ID 的
   背压合同，提交前重验来源版本与成员修订。结束时按现有封输入、排空、join 和提交顺序收尾。
4. 单所有者持有的串行状态无需跨线程锁。任务队列和完成通知使用短临界区和条件变量，具体计算
   在锁外。若实测表明队列同步成为瓶颈，再评估具备明确单生产者、单消费者契约的方案。

拟议运行参数 `loop_closure.worker_threads` 缺省为 1，此时走串行验证。功能回归沿用现有
bag 和 CPU 亲和性。性能对照为两侧分配同一组至少两个可用逻辑 CPU，
固定前端 OpenMP 预算，对比 1 与 2 个候选验证线程的回环耗时、前端 P95、实际并发、过期结果
和 RSS。核对候选排序、接纳 ID、位姿版本、旧任务丢弃和停止排空。

## 构建能力与 YAML 运行选择

CMake 根据目标平台选择接入代码和桥接库，YAML 选择本次会话的功能。
同一个节点可启动为纯 LIVO、回环检测或检测加位姿图优化。节点在创建输出目录前校验参数组合。

| 功能 | 编译期 | 运行期 |
| --- | --- | --- |
| GTSAM 位姿图与几何回环 | 常规构建需要 GTSAM 4.2 和 PCL 的 registration、kdtree | `loop_closure.enabled` 控制检测，`pose_graph.enabled` 控制用检测约束优化位姿 |
| 在线 Gaussian | Linux 目标编入 C++ socket 桥接 | `gaussian.enabled` 控制连接和建图，Python worker 在运行时检查 CUDA 等依赖 |
| OpenMP | 工具链探测 C++ OpenMP，按目标链接 `OpenMP::OpenMP_CXX` | 用 `frontend.openmp_threads` 指定并行区预算，1 表示串行 |

[当前 YAML 解析](../src/core/livo_config.cc) · [在线回环配置](online-loop-backend.md) ·
[Gaussian 节点装配](../src/app/online_gaussian.cc)

建议 `LIVO_ATLAS_USE_OPENMP` 支持 `AUTO`、`ON`、`OFF`。`AUTO` 以目标工具链实际探测结果为
准，`ON` 要求工具链提供 OpenMP，`OFF` 显式关闭。目标、测试和安装依赖应使用同一探测结果，
涵盖普通 SDK、Catkin 和 Ament。ROS 版本由构建目标选择。

平台相关的选项根据 `CMAKE_SYSTEM_PROCESSOR` 判断目标架构，x86_64 和 ARM 均通过编译器
探测决定 OpenMP 能力。现有 `ProcessorCount()` 在配置机器上求值并将上限为 4 的线程数写入
并行宏。运行参数可取代该编译期定值，首版在前端和后台同时运行时核对各自的实际线程预算。
架构判断可使用工具链。若添加指令集优化，需核对它与外部 PCL 和 Eigen 的对齐约定。
选项实现沿用项目 CMake 3.16 基线。本机已核对 x86_64 的现有配置，ARM 的配置和运行结果
需要用对应目标环境验证。[当前 OpenMP 设置](../cmake/core.cmake) ·
[PCL 内存兼容回归](../tests/pcl_memory_compatibility_test.cc) ·
[CMake 目标处理器](https://cmake.org/cmake/help/v3.16/variable/CMAKE_SYSTEM_PROCESSOR.html) ·
[FindOpenMP](https://cmake.org/cmake/help/v3.16/module/FindOpenMP.html) ·
[ProcessorCount](https://cmake.org/cmake/help/v3.16/module/ProcessorCount.html)

建议的 ROS1 配置形态如下。Humble 的参数适配器提供同名字段给共用读取逻辑。线程数示例用于
对照，OpenMP 的默认预算按同机基线和目标机器的资源确定。

```yaml
frontend:
  openmp_threads: 2
loop_closure:
  enabled: true
  worker_threads: 2
pose_graph:
  enabled: true
gaussian:
  enabled: true
```

示例展示功能和线程选择。开启相应功能时按照现有配置提供 keyframes、online submaps、
Gaussian socket 和输出目录。[在线回环参数](online-loop-backend.md#构建与开启) ·
[在线 Gaussian 参数](online-gaussian.md)

## 可以一起收敛的工程问题

- 启动时汇报目标架构、编译能力、有效运行开关和线程数。缺少能力时给出具体依赖和开关名称。
- 从共用 `main` 提取会话装配、轮询和收尾代码，明确线程 join、系统销毁及 ROS publisher 的
  生命周期。改动覆盖两端入口、停止次序及暂停仿真时间下的结果交付。
- 给回环分阶段计时，单列几何读取、候选验证、GTSAM 试算和 `Poll()` 中的持久提交。利用实际
  数据判断下一处开销，对照前端回调尾延迟及后台高水位。

建议先收敛编译能力和运行参数，再验证有界的候选并行。两步各用现有 SDK 消费者、双 ROS
启动检查和 Retail_Street 回放验证。
