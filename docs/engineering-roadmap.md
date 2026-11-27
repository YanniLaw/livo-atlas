# 后续优化与工程改造建议

跨电脑复用的 SDK、双 ROS 测试、安装消费者、几何基准及可选回放入口已整理为
[工程回归工具](regression-tools.md)。两次 Humble 完整 bag 回放已通过，
[验证记录](humble-full-validation.md)包含同环境重复性和跨 ROS 数值差异。
跨环境首次分歧及 Eigen 版本效应已完成 [单变量诊断](cross-ros-numerics.md)。
构建依赖、线程预算记录及回归工具两项边界已收尾，见 [工程回归工具](regression-tools.md)。
联合渲染邮箱竞争路径与日志布尔词尾校验已收尾，后续已完成
[真实 GPU 与双 ROS 全链路验证](gpu-slam-validation.md)，并修复 Humble 预览接收工具的
数值类型转换问题。[双 ROS 正常退出](graceful-shutdown-validation.md)已完成 STOPPED
接收、暂停仿真时钟、启动取消及末段重复信号验证。
重复信号与超时报告的直接断言也已补齐。[在线留出评估](gaussian-online-evaluation.md)
已实现训练前冻结、完整训练历史隔离、独立查询、封口调度、固定指标及资源验收。
[最终快照评分](gaussian-heldout-scoring.md)支持与在线共用C++空间选块内核；
完整Noetic配对验证见[空间留出对照](spatial-heldout-comparison.md)。
随后完成[固定预算采样与空洞诊断](controlled-gaussian-sampling.md)：每次150次训练/3000步，
覆盖收益保留，但单块SSIM与共同深度L1存在取舍。随后完成
[预留比例参数化与六次敏感性验证](gaussian-coverage-ratio.md)：25%提供联合质量折中，
单块SSIM仍有损失。随后完成[定额替换与十次同版本对照](gaussian-seed-replacement.md)，
uniform原有覆盖保留明显改善，但未恢复单块SSIM且有深度代价，默认仍为uniform/resample；下一项先诊断覆盖与纹理冗余的关系。
随后完成[CBD Building第二场景复验](cbd-building-sampling.md)：85次训练/1700步、五条件各两次，
覆盖保留收益复现，单块SSIM/共同深度代价扩大；补齐归档相机容差与原生准备器的一致性。
已补齐[有界种子诊断](gaussian-seed-diagnostics.md)：真实像素/XYZ、历史稀疏支持及120步参数变化，
双场景各一批点的12次追踪核验通过。随后完成[均值/尺度冻结试验](gaussian-optimization-ablation.md)：
双场景三条件各两次完整前缀训练，冻结均值或尺度均降低画质与覆盖，并增大共同深度误差；
不采用完全冻结，不改默认。随后完成[正均值学习率减半试验](gaussian-means-rate.md)：
复用八份经源指纹验收的原率/零率归档，仅新增四次训练；半率仍降低画质、覆盖及共同深度表现，
保留原率，不继续本轮网格搜索。随后完成[有界覆盖诊断](gaussian-coverage-diagnostics.md)：
两个开发查询的四次诊断，记录实际选点/预渲染掩码与保留集合，重建投影和GPU渲染。
多数最终空洞附近缺少4像素邻域中心，但约97%空洞也缺查询深度，不能确定唯一成因。
随后完成[全量候选与实际种子对照](depth-candidate-support.md)：复用四份模型，新增训练/渲染为零。
4像素邻域中约76%–80%空洞附近有输入候选，但窗口内实际种子仅约4%–7%；采样支持损失代理
显著，仍不能证明因果或直接加点。已开始[固定预算二维分层采样对照](stratified-seed-sampling.md)：
审查修正后438项回归、静态304批重放、16次完整前缀训练及独立审计通过。分层提高覆盖/PSNR，
但SSIM和原有覆盖保留变差，共同深度收益依赖场景/策略，保持默认不变；
随后完成[固定共同覆盖纹理诊断](common-coverage-texture.md)：复用16份归档，无新增训练/渲染，
审查修正后454项Python/CUDA回归及33查询独立审计通过。共同weak的SSIM均下降，CBD部分strong改善；
全图MSE收益主要来自新增覆盖，并被丢失覆盖部分抵消。补齐旧split/静态来源复查。
随后完成[仅对补入点分层的对照](stratified-additions.md)：保留 flat 基底和移除顺序，
两场景联合覆盖约增加 0.12/0.09 个百分点，共同弱梯度 SSIM 和深度 L1 均改善。
Retail 的[两对 Noetic 整包回放](noetic-stratified-additions.md)均完成前端、回环、在线训练和封口。
两对联合覆盖分别增加 0.40 和 0.38 个百分点，共同弱梯度 SSIM 均上升，深度 RMSE 方向相反。
候选的全部训练 P95 首对高 8.20 ms、反序对低 1.29 ms，两次封口均约长 0.9 秒。
随后完成 [Retail 深度定位与 CBD 在线整包复验](cbd-noetic-stratified-additions.md)。
Retail 65 号视角的互换试验定位到所属子地图 4 的模型。CBD 联合覆盖增加 0.15 个百分点，
共同弱梯度 SSIM 增加 0.00866，共同深度 RMSE 降低 10.04 mm，封口相差约 0.04 秒。
随后完成 [Retail 遮挡区域深度诊断](retail-depth-visibility.md)。25、65 号多帧支持的高误差
像素上，分层补点的近层权重增加，且加权中位数使误差增大。后续用独立视角和多帧支持度
评估近层几何与权重。
随后完成 [双场景多帧深度支持复验](multiview-depth-support.md)。两种邻域下，参考层重复支持且
近层未重复的像素中，三对单块和联合的 L1/RMSE 均下降。正平方误差增量在该组以外更集中，
后续优先评估深度标签可见性，并定位仍退化的视角。
已将多帧支持接入[离线评分命令](gaussian-depth-support.md)，六份归档与既有 CPU 结果一致。
Retail 1 号的净增量集中在扩大邻域后显现近层竞争的像素。CBD 73 号在两种半径均支持的
像素上仍有增量，主要由渲染深度向远处移动贡献。
同环境精确回归与跨环境数值对照继续分开解释；3DGS、长距离数据、ARM 实机等方向保留如下。

C4c 对照基准为 `2318335`。该阶段已用 Noetic 跑通几何回环、在线高斯训练和封口的配对回放。
以下记录待推进项和工程改造建议。[C4c 验证](p6-03c4c-validation.md)

## 后续 3DGS 优化

| 方向 | 交付和验收 |
| --- | --- |
| 在线留出画质与指定相机渲染 | 基础链路已实现，见[完整对照](spatial-heldout-comparison.md)。[设计](superpowers/specs/2026-11-26-online-heldout-evaluation-design.md)：训练前冻结 manifest，普通训练、重建及恢复均禁止使用留出数据；对在线封口快照与最终模型评分，按同源查询对比 `uniform`/`coverage`。首轮沿用 Retail_Street |
| 自由视角浏览与高斯地图导出 | 用已确认快照和指定相机驱动查看。导出训练后高斯参数，并用实际查看器验证子地图位姿、尺度、颜色和不透明度。现有 COLMAP 导出继续提供重建观测 |
| 画质和几何一致性 | 从留出视角定位空洞、重影和跨块接缝。测得有效密度不足时，试验定额内增密与裁剪，同时核对共同深度误差、显存和训练延迟 |
| 跨场景与真值复验 | 有合适数据时补测不同光照、动态物体及重访。使用有真值的路线评估轨迹误差、回环误检和留出视角画质 |
| Humble GPU 全链路 | 已完成本机共用 worker 的完整 bag 训练、回环修正、ROS2 接收及封口，[记录](gpu-slam-validation.md)包含复现步骤；[正常退出通知](graceful-shutdown-validation.md)也已验证，后续补其他电脑复验 |
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

### 候选并行

`LoopClosure` 的原生查询通过 `Task` 依赖连接准备、验证、汇总和试算。准备任务独占
可变几何缓存，验证任务只读快照。下一批读取几何前，上一批必须全部完成并释放目标快照。
批次上限同时受后台线程数减一和几何驻留数减一约束，单线程池沿相同依赖关系推进。

结果按检索顺序归并，私有试算与几何复核由单个后续任务执行。`PoseGraph` 的串行
`WorkQueue` 独占接受历史与正式位姿提交。仍保留一个在途查询和一个最新待查 ID。
完整任务图在短临界区内注册，后续计算在锁外，工作线程无需等待或再次 Schedule。
[回环调度](../src/loop_closure/loop_query.cc)

后端线程池使用 `pose_graph.num_background_threads`，缺省为 4，可设为 6，1 用于单线程回归。
线程池负责归档与回环任务，候选验证复用同一线程池及 `Task` 依赖。
当前预算矩阵的回放与故障注入已完成，见 [线程验证记录](threading-validation.md)。
分段计时已完成，见 [回环计时结果](loop-stage-timing.md)。早期回放加载了线程审计探针，
后续采样发现它显著影响 FLANN 查询，原阶段占比不能直接代表日常运行；数值回归仍有效。

候选并发已确认，端到端延迟尚未显示稳定下降。进一步优化依照各阶段测量推进。本轮已完成
[验证内部剖析与局部查询复用](loop-verification-reuse.md)，完整配对的反向查询次数下降约57%。
已完成 [GICP 协方差剖析与复用](gicp-covariance-reuse.md)，完整配对的协方差数组计算由76次降至40次。
异常估计器子计时已补齐，另做不加载线程探针的 CPU 采样与重复完整对照，见
[采样记录](gicp-cpu-profile.md)。进一步优化依据无探针测量，避免继续追逐审计开销。

已有四次无探针回放的几何准备归因与 owner 交付边界已补齐，见
[几何准备与交付记录](geometry-delivery-timing.md)。缓存逐次记录请求、构建及版本切换；
接纳回环单独记录位姿交付延迟，避免把普通归档发布等待混入其中。
本轮保持3块驻留预算与原有失效策略；跨查询共享仍需单独评估。

随后完成 [几何完整构建累计优化](geometry-build-reuse.md)：未发布临时 builder 直接累计，
公开增量入口保留异常回滚。三组预载真实点云的离线配对显示完整构建中位耗时下降约
36%～39%，一组完整 bag 的数值与归档不变；局部基准收益不代表全系统固定加速。

性能对照为各侧分配同一组至少六个可用逻辑 CPU，
固定前端 OpenMP 预算，对比 1、4、6 个工作线程的回环耗时、前端 P95、实际并发、过期结果
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
[Gaussian 节点装配](../src/app/gaussian_mapping.cc)

`LIVO_ATLAS_USE_OPENMP` 支持 `AUTO`、`ON`、`OFF`。`AUTO` 以目标工具链实际探测结果为
准，`ON` 要求工具链提供 OpenMP，`OFF` 显式关闭。目标、测试和安装依赖应使用同一探测结果，
涵盖普通 SDK、Catkin 和 Ament。ROS 版本由构建目标选择。

平台相关的选项根据 `CMAKE_SYSTEM_PROCESSOR` 判断目标架构，x86_64 和 ARM 均通过编译器
探测决定 OpenMP 能力。`frontend.openmp_threads` 默认 2，取代配置机器上的处理器计数；
每个前端实例的并行区使用自己的预算。无 OpenMP 时记录串行回退。
架构判断可使用工具链。若添加指令集优化，需核对它与外部 PCL 和 Eigen 的对齐约定。
选项实现沿用项目 CMake 3.16 基线。本机已核对 x86_64 的现有配置，ARM 的配置和运行结果
需要用对应目标环境验证。[当前 OpenMP 设置](../cmake/core.cmake) ·
[PCL 内存兼容回归](../tests/pcl_memory_compatibility_test.cc) ·
[CMake 目标处理器](https://cmake.org/cmake/help/v3.16/variable/CMAKE_SYSTEM_PROCESSOR.html) ·
[FindOpenMP](https://cmake.org/cmake/help/v3.16/module/FindOpenMP.html)

ROS1 配置形态如下。Humble 的参数适配器提供同名字段给共用读取逻辑。
OpenMP 的缺省预算为 2，可按同机基线和目标机器的资源调整。

```yaml
frontend:
  openmp_threads: 2
loop_closure:
  enabled: true
pose_graph:
  enabled: true
  num_background_threads: 4
gaussian:
  enabled: true
```

示例展示功能和线程选择。开启相应功能时按照现有配置提供 keyframes、online submaps、
Gaussian socket 和输出目录。[在线回环参数](online-loop-backend.md#构建与开启) ·
[在线 Gaussian 参数](online-gaussian.md)

## 可以一起收敛的工程问题

- 正常退出流程及报告断言已收尾，见 [退出验证](graceful-shutdown-validation.md)。
- 启动时汇报目标架构、编译能力、有效运行开关和线程数。缺少能力时给出具体依赖和开关名称。
- 从共用 `main` 提取会话装配、轮询和收尾代码，明确线程 join、系统销毁及 ROS publisher 的
  生命周期。改动覆盖两端入口、停止次序及暂停仿真时间下的结果交付。
- 给回环分阶段计时，单列几何读取、候选验证、GTSAM 试算和 `Poll()` 中的持久提交。利用实际
  数据判断下一处开销，对照前端回调尾延迟及后台高水位。

建议先收敛编译能力和运行参数，再验证有界的候选并行。两步各用现有 SDK 消费者、双 ROS
启动检查和 Retail_Street 回放验证。
