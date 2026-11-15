# P6-02c3b：优化历史轨迹与几何地图输出

2026-11-15。基于 `aea7538`；产品 C++ 沿用 `39f7494`。
状态：C3b-01 已按用户继续指令实现并完成正式验收及审阅问题闭环；见[使用说明](../../keyframe-trajectory.md)及[验证记录](../../p6-02c3b1-validation.md)。C3b-02 已另写[详细设计](2026-11-15-p6-02c3b2-geometry-map-design.md)，尚未编码。
执行者按此前“按照你推荐的来”的委托选择下述路线与默认值；这些选择仍可调整，不能写成用户逐项确认过的要求。
正式编码前已通知用户切换模型，随后收到继续实施指令。本阶段只实现 C3b-01。

## 目的与分步范围

C3a 已有当前 `map -> camera_init` 修正及 `/global_pose`，但历史关键帧应使用各自的
`P_v(i)`。把整条旧轨迹乘上当前修正，会丢失非刚性历史修正。
本阶段使应用能读取有明确来源的历史结果，并为随后显示经过优化的子地图准备边界。
原始观测、持久位姿版本和局部几何继续作为 SLAM/3DGS 的输入。

| 步骤 | 本次确定的交付 | 当前状态 |
| --- | --- | --- |
| C3b-01 | 普通 C++ 轨迹快照、双 ROS 版本化关键帧轨迹、标准 Path、有限后台输出 | 已完成实施与阶段验收 |
| C3b-02 | 同一位姿版本下的后台几何地图输出、来源与显示预算 | 已有独立详细设计，尚未编码 |

C3b-01 不加入 Gaussian 训练、iSAM2、回环算法变更、稠密重建服务或通用任务系统。
不修改 vikit，不合并分支或发布标签。所有新增 C++ 遵循仓库 Google/C++17 约定，
直接声明成员，函数实现之间空一行，对跨线程所有权和坐标约定补充简短注释。

## 现状与测量

- `OnlineSubmapUpdate::poses` 是拥有数据的不可变完整 `PoseSnapshot`；注册回调必须在首帧之前。
  目前单个回调由共同 main 连接 `LoopBackend::Observe`，不能另注册一个回调覆盖它。
- `GetMapCorrection()` 只取常数规模数据；`GetPoseSnapshot()` 会物化整个历史。
  `OnlineSubmapManager::Notify()` 已调用后者，因此本方案不承诺前端通知整体为 O(1)。
  仅开启历史输出也会激活这条完整快照路径，必须实测这一增量成本。
- `KeyframeReadView` 固定可读前沿、不持有 writer/index，适合后台读取已提交文件；
  `ReadTimestamp()` 仍要打开 `frame.yml`，不能放在发布回调或 `Observe` 内。
- `GetGeometrySubmap()` 可能重建、读云、改变缓存并设置核心失败状态，不能直接当成轻量发布查询。
  后台线程也不能调用活的 `LivoSystem`、前端 `GeometryMapper` 或其缓存。
- 单一入口仍为 `src/app/livo_main.cc`。Noetic/Humble 的普通配置和调度逻辑保持共用，
  新增 ROS 文件保持职责、类型名和入口方法对称。

测量使用已有停止数据集 `replay-noetic-global-on-loops-on-136/keyframes`：
172 个关键帧、13 个子地图，0.2 m 几何体素；CPU 0、nice 10、OMP 单线程，串行导出。

| 物化位姿版本 | 全部局部体素数 | 导出耗时 | 峰值 RSS | 导出文件字节数 |
| --- | ---: | ---: | ---: | ---: |
| 0 | 168485 | 1.42 s | 97600 KiB | 46640755 |
| 2 | 168502 | 1.43 s | 97984 KiB | 46640338 |

版本 2 的持久优化前沿为 169，导出将其尾部物化至当前 172 帧。
数据与可复现命令见工作区 `validation/P6-02c3b-design-20261115/geometry-measurements.json`
及 `measure_geometry.py`；迁移电脑时需单独复制 validation 目录。
每版本仅测量一次，文件缓存未控制；耗时包含目录校验、重建和 CSV 写入，
既不是纯算法重建耗时，也不是在线输出延迟或长时间运行证明。

## 路线选择

| 路线 | 好处 | 代价 |
| --- | --- | --- |
| 直接在 ROS 发布路径遍历全部历史与子地图 | 接线少 | 读盘、重建、消息构造占用前端；旧结果来源不明确 |
| 一次引入完整轨迹/全图分块传输及持久派生产品系统 | 能覆盖更多场景 | 同时扩大消息、缓存与恢复协议，当前数据不足以确定预算 |
| 先版本化轨迹，再单独设计后台几何显示 | 先验证历史位姿、跨 ROS 协议和资源边界；复用既有持久源 | 几何全图话题后续交付 |

采用第三条作为执行者建议。先完成一个可单独验收的输出，再以其来源和调度经验细化地图。
不提前增加事件总线、插件接口、泛化 worker 池或第二套 raw 位姿权威。

## C3b-01：普通 C++ 结果

新增普通值类型 `TimedKeyframePose` 与 `TrajectorySnapshot`，放在
`include/livo_atlas/mapping/trajectory_snapshot.h`；不包含 ROS 头。
`TimedKeyframePose` 包含 `id`、原始 double 秒 `timestamp` 和 `parent_from_body`。
`TrajectorySnapshot` 包含下列元数据、状态与完整的关键帧数组：

| 字段 | 语义 |
| --- | --- |
| `producer_id` | 当前输出实例的随机 128 bit 标识，32 个小写十六进制字符；不是持久 dataset UUID |
| `revision` | 同一 producer 内严格递增的输出状态编号，初始空状态从 1 开始；允许跳号 |
| `pose_version` | 正式提交的位姿版本 v |
| `frame_count` | 本次固定历史前沿 M |
| `optimized_frame_count` | 正式优化前沿 N；N 之后是既有规则物化的尾部 |
| `parent_frame` | 来自 PoseSnapshot 的 Odom/Map 坐标标识 |
| `status` / `error` | Ready、ResourceLimited 或 Error；错误文本有长度上限 |
| `keyframes` | Ready 时恰为 ID 0…M−1，按 ID 递增；其他状态为空 |

Ready 且 M=0 表示合法空状态，不能和资源不足混淆。不静默抽帧或只保留末尾一段，
资源不足也不发送“完整”的前缀轨迹。

位姿规则：

1. v=0 使用每帧原始 odom 位姿，旋转采用现有 `ProjectPose` 的 SO(3) 投影，平移不变。
   ROS frame_id 使用 `camera_init`，不因字段旧名 `map_from_body` 而误标为 map。
2. v>0 使用每帧 `P_v(i)`，要求已有 projected-tail 策略并校验刚体变换；
   ROS frame_id 使用 C3a 的 `map_frame`。legacy 正版本在线输出显式报 Error，
   不改写已有离线读取能力。这与 C3a 的在线前提一致。
3. 从有效旋转构造并归一化四元数；比较使用旋转/平移误差，不依赖四元数符号。
   不复用旧 ROS 发布器的 Euler 路径，不用当前一个 C 修正全部历史。
4. 时间取同 ID 的不可变 `frame.yml`。不按时间重新排序；重复或回退的采集时间不改变 ID。
5. 一次结果只使用一个完整 PoseSnapshot，绝不混用另一个版本或前沿中的位姿。

## C3b-01：ROS 消息与配置

在同一个 `livo_atlas` 包内增加两份共享的 `.msg` 源：

- `TrajectoryKeyframe.msg`：`uint64 id`、`float64 timestamp`、`geometry_msgs/Pose pose`。
- `KeyframeTrajectory.msg`：`std_msgs/Header`、producer/revision/v/M/N、三种状态常量及状态、
  错误文本、`TrajectoryKeyframe[] keyframes`。

Header.frame_id 对应上文 parent_frame；该消息通过 frame_id 保留坐标语义。
Ready 的 Header.stamp 为最后一个关键帧的采集时间，空状态与非 Ready 状态为零。
ROS1/ROS2 的语义相同；其线上编码不承诺二进制兼容。不得使用 Header.seq、
字符串化整段 JSON 或另一个无法原子配对的 metadata 话题代替版本字段。
新消息名不与现有普通 `KeyframePose` 等类型冲突。

| 话题 | 内容 | 投递策略 |
| --- | --- | --- |
| `/keyframe_trajectory` | 上述完整版本化结果，程序消费入口 | Noetic latch、queue 1；Humble reliable、transient_local、depth 1 |
| `/keyframe_path` | 同次 Ready 结果转成 nav_msgs/Path，供 RViz 显示 | 同上；非 Ready 时发布空 Path，清除旧显示 |

Path 的各 PoseStamped 保留对应采集时间；总 Header 与版本化消息一致。
两个话题不构成跨话题原子事务，Path 不承担精确版本协议。程序以版本化消息为准。
Humble 的晚加入消费者需选择匹配的 transient_local QoS 才验证历史状态交付。
Error 文本限制为 1024 字节，不传原始图像、点云或任意日志。

ROS 时间转换支持两端共同的非负范围：0 ≤ 秒 < 2^31，纳秒规范化；
SDK 保留原始有限 double 秒，线上时间为转换后的纳秒值。范围外的 Ready 结果转 Error，
清空数组和 Header.stamp，不用溢出或饱和时间冒充有效数据。

共用 `GlobalOutputOptions` 增加一组轨迹选项，参数清单仍只有一份：

| 参数 | 默认 | 启用时约束 |
| --- | --- | --- |
| `global_output.trajectory.enabled` | false | ROS 节点要求 global_output.enabled、关键帧存储及 online submaps 已开启 |
| `global_output.trajectory.max_frequency_hz` | 1.0 | 有限且 0 < f ≤ 10，表示正常运行发布上限，使用 steady clock |
| `global_output.trajectory.max_keyframes` | 20000 | 整数，1…100000；超过时 ResourceLimited |

配置在创建文件/线程之前校验。未启用时不创建新 publisher、worker 或轨迹回调；
轨迹关闭时其预算选项不生效。SDK 的普通构建函数不要求 ROS 或 C3a publisher。
开启轨迹不自动开启回环、GTSAM 或几何；ROS 对 global_output.enabled 的要求是本方案为
统一 map_frame 与 RViz TF 使用作出的选择，不能扩展成核心库依赖。
默认 20000 条序列化关键帧数据约 1.44 MB（每项 72 字节，不含消息固定开销）；
最终须用两端实际序列化验证，不能以此推导整个进程的内存上限。

Noetic 使用 message_generation/runtime，Humble 使用 rosidl 生成并导出类型支持。
在 package.xml 与对应 cmake 分支声明条件依赖；ROS_VERSION=0 完全不运行消息生成，
普通安装消费方不需要任何 ROS 环境。保留同一个 main；不得重新拆分 ros1/ros2 main。

## C3b-01：后台输出与所有权

增加直接成员的 `TrajectoryOutput`，放在 core，接收普通快照消费函数；
消费函数在它的唯一 worker 上调用。SDK 可消费普通结果，ROS 使用对称的
`ros1::KeyframeTrajectoryPublisher` / `ros2::KeyframeTrajectoryPublisher` 转换并发布。
ROS 类型、时间转换、QoS 和 publisher 仍只存在于外围。

- 唯一在线回调由 main 直接组合已有 backend.Observe 与 trajectory.Observe；
  两者均不得重入系统或向外抛异常。轨迹 Observe 只检查小规模头、保留最新 poses 指针，
  将异常记录为本组件的失败；不复制图像/点云、不读盘、不整理整条轨迹。
- 同一 v/M 的 sealing 通知不改变轨迹内容，不新增 revision。初始空状态需要显式送入输出。
  每个新 v 或 M 分配 revision；这个编号不要求和 pose_version 相同。
- 所有者的 after_process 先执行 backend.Poll，再执行 trajectory.Poll；
  后者接收最新回调状态并在 worker 空闲、达到节流时间时取固定 KeyframeReadView。
  view.frame_count 必须 ≥ M，任务只读取 ID < M；不得从后续文件补齐本次状态。
- 最多一个执行任务、一个合并后的最新待处理状态，没有无限请求队列。
  待处理状态可以跳过多个 revision；繁忙期间不逐帧排队。worker 独占时间缓存，
  仅缓存预算内已读取的 timestamp，不复制第二份 raw 轨迹或持有观测资产。
- worker 读取缺少的时间、构造完整普通结果，再调用消费函数；ROS 消息转换、序列化发起和
  新话题发布均在这条后台路径。不能 Take 后回到前端遍历 M 帧构造消息。
  publisher 句柄在启动 worker 前创建，输出对象销毁/join 先于这些句柄销毁。
- 完成结果只要 revision 大于最后交付的 revision 就可交付，即使已有更新请求。
  不能因为前端持续追加就不断丢弃当前完整结果，造成地图/轨迹永远不发布。
  “过期”指不大于已经交付的编号，不等于“落后于最新观测”。结果明确标注自身 v/M/N，
  允许暂时落后于当前 `/global_pose`，不能换上最新标签掩盖延迟。
- 在位姿数据不变时不重复构建/周期广播。频率是上限，不是实时完成期限。
  初始空状态和正常停止后的最终 SDK 交付允许绕过该上限。

Noetic 的空闲主循环在 backend 或轨迹开启时使用 WallRate，保证 /clock 暂停仍可调度；
Humble 继续复用已有 wall timer/单线程 executor。历史状态不套用 C3a 当前测量的
“stamp 必须严格递增”门槛：同一最后采集时间下的新正式版本仍应输出。
新版本、重复时间和空闲提交不产生新的当前 body TF 或 `/global_pose`。

超过 max_keyframes 时交付带真实头部的 ResourceLimited，不读更多文件、不构建完整数组，
也不额外长期保留该超预算完整快照。该限制只约束派生输出；
已有核心、完整 PoseSnapshot 通知和存储仍会随历史增长，不能宣称进程总内存有界。

读取/投影失败时交付 Error 并停止该派生组件的后续工作，原始前端、后端提交和已有发布继续；
失败不能回滚正式位姿版本，也不能伪造 Ready。若消费函数自身抛异常，记录组件失败和日志，
不在同一个失败 sink 上递归重发，不承诺还能送达 Error 消息。
资源不足属于正常状态，与核心已有的粘性写入失败、后端失败语义分开处理。
保留提交、合并、交付、资源限制、失败数及最后交付 v/M/revision，供验收与诊断。

正常停止先停止传感器处理，沿用 backend.Finish（或 system.Finish）完成既有持久提交；
再令轨迹输出取得最终只读前沿、合并为最终任务，完成 SDK 消费并 join。
若相同 revision 已交付则无需重复交付；Finish 可重复调用，完成后不再接收新状态。
回调捕获对象须在前述 Finish 期间存活。异常析构只取消待处理、结束/join 已执行任务，
不读活系统、不做新的最终快照或抛异常。文件 I/O 无强制中断能力，不承诺硬停止时限。
ROS shutdown 后不承诺最终话题送达；已有关键帧/正式位姿文件是可恢复来源，
本步不另建持久派生轨迹数据库，也不将一次在线话题作为最终持久化证明。

## C3b-02：先固定这些约束

几何显示的消息、配置、显示范围和预算已在[C3b-02详细设计](2026-11-15-p6-02c3b2-geometry-map-design.md)中细化；
下面保留原阶段边界，不把轨迹 worker 扩展成通用调度器。

1. 输入为同一 PoseSnapshot、完整的目标成员索引与固定 KeyframeReadView；
   callback 中的 revisions 是增量，任务必须持有能独立构建目标前沿的完整索引。
2. worker 私有 GeometryMapper 逐子地图读取/重建，绝不调用前端的 GetGeometrySubmap。
   保持现有相对位姿改变重建、整体刚体变换复用的区分；先测工作集淘汰引起的重复读盘。
3. 每个关键帧只作为 primary 融合一次，support 不再次累计；从局部几何按本次各锚点位姿变换，
   不能把旧整图只乘一个 C。局部 voxel/spatial_covariance 不冒充测量不确定性。
4. 输出必须携带 v/M/N、子地图 ID 与 membership_revision、表示来源及显示采样信息。
   representation_revision 当前由具体 GeometryMapper 的重建历史分配；后台私有实例与前端
   的计数不能直接互认，必须明确生产者/缓存域或另定内容标识后才对外约定。
5. 有限缓存不等于最终合并云有界。显示点数、合并/降采样内存和消息大小均须显式限定；
   截断/采样/范围限制写入同一版本化结果。允许有来源的旧完整结果，禁止旧结果覆盖已交付新结果。
6. ROS 合并云仅供地图显示。在线 3DGS 使用原始图像、标定、局部几何、逐帧位姿及修订关系；
   不把降采样全局云升格为唯一训练输入或新的地图权威。
7. 已有停止后 ExportGeometry 保留；在线最终完成/取消、保存位置和精确版本协议需在 C3b-02
   设计中确定，不能把本轮 1.4 s 离线 CSV 测量当作该实现的性能承诺。

## C3b-01 验收与进入编码的条件

本节是设计验收目标，尚未执行，也不是实施计划。

- 普通测试：构造每帧修正不同的历史，逐项核对 P_v(i)，验证与“整条轨迹乘单个 C”有可辨差异；
  覆盖 v0 投影、正版本优化前沿/尾部、空状态、legacy 正版本错误和非有限输入。
- 时间/协议：固定元数据文件读取；ID 保持递增而允许重复/回退采集时间；
  同 v 追加、同时间换 v、重启新 producer、合并跳号与版本单调性；禁止跨版本拼接。
- 并发/预算：慢消费函数和慢读盘时仍可合并，持续输入仍交付完整结果；
  运行槽/待处理槽有限，预算耗尽交付明确状态；错误不向核心传播；销毁/Finish 无悬空回调。
- 两端真实 ROS：线上字段精确一致，Path 与对应数组值一致，晚加入状态读取、故障时空 Path、
  paused/repeated/backward 的非负 /clock、正常与异常停止；旧话题值及 TF 拓扑不变。
- 构建/迁移：ROS_VERSION=0 的 SDK shared/static 与依赖开关组合按现有阶段约定验证；
  Noetic/Humble 生成、安装并由外部消费方实际使用新消息；普通头不泄露 ROS/rosidl 依赖。
- 官方数据：在相同产品基线上做默认关闭与开启轨迹的完整 Retail_Street 回放，
  对照原始四数组、odometry、旧导出；按固定 v/M 和 frame.yml 对照新结果。
  同时覆盖回环开/关及 geometry 关；不将不同平台浮点差异判成同平台回归。
- 资源：记录 Observe/Poll 自身、原有 Notify 完整快照的耗时、worker 时间、发布延迟、
  峰值保留快照数和 RSS。用预算边界合成测试补足 172 帧规模；
  当前 135 s bag 不能代替长距离或长期实测。不得声称新阶段全部实现 O(1)。

2026-11-15，独立 builder check 已完成，结论为 `nothing important left`；
审读范围仅为 C3b-01 的可实施性，不代表代码或运行验收。
记录见工作区 `validation/P6-02c3b-design-20261115/builder-check.md`。
用户确认设计并切换模型后，再编写 C3b-01 实施计划。
C3b-02 的消息与资源选择已在独立详细设计中细化；其产品代码尚未实施，正式编码前继续遵循用户的模型切换通知要求。
