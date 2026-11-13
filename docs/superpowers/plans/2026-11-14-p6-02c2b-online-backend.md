# P6-02c2b 在线回环调度与持久接纳实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Preserve current-branch and inline-execution preferences.

**Goal:** 将已有 C2a 接入在线封存事件，以单 worker 计算、所有者持久接纳，并能在停止后解释已提交版本。

**Architecture:** 普通 core 保持串行；拥有的固定前沿只读视图交给单 worker，结果仅由所有者检查和提交。
prepared 与正式 pose 配对作为接纳权威；SealInput、排空和 Finish 分开，两个 ROS 使用同一个 main 和处理后 hook。

**Tech Stack:** C++17、std::thread/condition_variable、现有 Eigen/PCL/OpenCV/GTSAM 4.2.2、Noetic/Humble。

**Spec:** [C2b 设计](../specs/2026-11-14-p6-02c2b-online-backend-design.md)。状态：**实施草案，未编码；正式编码前通知用户切换模型**。

## Global Constraints

- 基线 `99d5f27`；只修改 livo-atlas，不修改 vikit，保留当前开发分支，不自动 merge/push/tag。
- Google C++17；函数定义之间空一行，适当说明所有权/失败边界；不使用 Pimpl、线程池、插件系统或第二个 main。
- 复用 C2a 的算法、默认门限、固定 sigma、正 Huber 和 projected tail，禁止为得到接纳数调低阈值。
- 两个既有构建开关同时 ON 才创建 online_backend；runtime 默认关闭。普通 core 不链接可选后端。
- 单任务/结果共享容量 1、最新待处理 ID 容量 1；没有存放历史图像/云的队列。默认图预算 5000 帧/200 回环。
- worker 不扫描活体索引、不访问 LivoSystem 或 ROS；所有正式写入/提交属于唯一系统所有者。
- 只有 prepared＋匹配正式 pose 才算接纳；提交后通知失败终止会话，不回滚、不重试，不宣称掉电事务。
- 正常停止封存→排空接纳→join→最终 Finish；析构只停线程，不提交。错误保留原始异常。
- 重型构建串行 nice10/-j1；回放 CPU0/OMP1、1 倍速、队列32、无 RViz；旧 bag 和旧阶段证据只读。

## Review Focus

1. 写 pose 前/后失败及通知抛异常：已发布版本保留，prepared-only 不算成功；Task 4/5 故障用例覆盖。
2. 略非正交 raw 旋转与尾帧增长：完整 N 前沿不偷换，M 尾部按 projected policy 物化；Task 1/5 覆盖。
3. 无输入时完成任务、ROS2 while 排空、Finish 尾块：不饿死结果、不在回调重入或持锁 join；Task 3/5/6 覆盖。
4. 任务过载或跨过帧预算：只合并查询、不丢元信息前缀，不绕过预算提交在途提案；Task 3/5 覆盖。
5. 不完整/重复 YAML 和静态传递依赖：拒绝错误记录，安装消费不依赖源码私有目标；Task 4/6/7 覆盖。

## 文件与共享接口约定

- `include/livo_atlas/io/keyframe_read_view.h`，方法在现有 `src/io/keyframe_store.cc` 中共用读取助手。
- `include/livo_atlas/backend/loop_task.h`、`src/backend/loop_task.cc`：拥有的任务值和实际 C2a runner。
- `include/livo_atlas/backend/loop_worker.h`、`src/backend/loop_worker.cc`：有界线程传递。
- `include/livo_atlas/io/loop_closure_store.h`、`src/io/loop_closure_store.cc`：会话文件与只读检查。
- `src/io/loop_admission_io.h/.cc`：从 proposer 提取共享序列化/选项读取；原 CLI schema 保持兼容。
- `include/livo_atlas/core/loop_backend.h`、`src/core/loop_backend.cc`：所有者调度和接纳。
- `src/core/loop_commit.h/.cc`：无副作用的时效检查与实际候选物化。
- `cmake/online_backend.cmake`：只在双开时生成并安装 `livo_atlas::online_backend`。
- `tools/check_loops.cc`：停止后的只读检查 CLI。
- `tests/core/loop_backend_fixtures.h` 定义唯一 `LoopBackendTestPeer`，不安装，不进入生产调用路径。

为避免实施者在相邻任务间猜接口，冻结以下值类型，成员全部使用普通 C++ 类型：

```cpp
struct LoopTask {
  std::uint64_t task_id = 0;
  LoopAdmissionInput input;
  std::vector<double> timestamps;
  KeyframeReadView frames;
};
struct LoopTaskResult {
  LoopTask task;
  std::optional<LoopAdmissionResult> admission;
  std::exception_ptr error;  // 与 admission 恰有一个有效。
};
enum class LoopDisposition {
  kRejected, kAccepted, kStale, kResourceLimit, kFailed
};
struct LoopSessionStats {
  std::uint64_t frame_count = 0, pose_version = 0;
  std::uint64_t submitted = 0, completed = 0, rejected = 0, accepted = 0;
  std::uint64_t stale = 0, skipped = 0, resource_limited = 0;
  std::uint64_t peak_work_payloads = 0, peak_pending = 0;
};
enum class LoopRecoveryStatus { kConsistent, kIncomplete, kInconsistent };
struct LoopRecoveryReport {
  LoopRecoveryStatus status = LoopRecoveryStatus::kIncomplete;
  std::uint64_t committed_version = 0;
  std::vector<std::uint64_t> uncommitted_prepared;
  std::vector<std::string> issues;
};
```

无须虚构默认 ReadView：LoopTask 用聚合初始化提供真实视图；视图的合法构造只由 KeyframeStore 完成。
Move 所需方法/默认成员声明遵守实际 C++ 类型约束，不添加可伪造目录/前沿的公开构造函数。

实施时建立全新 `$C2B_BUILD`，设置兼容的 `$C2B_GTSAM_DIR` / `$C2B_SOPHUS_DIR`。
配置 ROS_VERSION=0、两开关 ON、BUILD_TESTING=ON、Release。共同 suite 命令：

```bash
nice -n 10 cmake --build "$C2B_BUILD" -- -j1
(cd "$C2B_BUILD" && OMP_THREAD_LIMIT=1 nice -n 10 ctest --output-on-failure)
```

必须读到非零测试数和 100% 通过；CMake 3.16 不使用 `ctest --test-dir`。

### Task 1：已提交关键帧读取视图

**Files:** 新增 read_view 头及 `tests/io/keyframe_read_view_test.cc`；修改 keyframe_store.h/.cc、livo_system.h/.cc、cmake/tests.cmake。

**Interfaces:**
- Produces: KeyframeReadView 的 directory()/frame_count()/ReadRawBodyPose()/ReadTimestamp()/ReadBodyCloud()，
  KeyframeStore::CaptureReadView() const、LivoSystem::GetKeyframeReadView() const。
- 构造私有，友元 KeyframeStore；视图自有绝对目录和 count，不持有 writer 指针或 ids_ 引用。

- [ ] 先写 `FrozenPrefixSurvivesAppendAndWriterDestruction`：Append 0→capture→Append 1；旧视图只读0，
  Read(1) 越界；销毁 writer 后0仍可读；新视图覆盖2；空视图所有 ID 越界。
- [ ] 写 `ReadsWithoutIndexScanOrImage`：捕获后临时破坏 dataset/在线索引、删除图像、存在 `.pending-1`，
  已提交0的元信息和 body cloud仍按既有规则读取，不吸收 pending/未来1。
- [ ] 写 `SharesPcdBoundsAndRawNumbers`：NaN、点预算、超大布局拒绝；原始近似旋转逐项保留。
- [ ] 构建测试观察缺API的编译失败；再提取共用内部读取函数，原 KeyframeStore 读取也委托同一实现。
- [ ] 运行 read_view/既有 store 测试及共同 suite，检查不读取图像的既有约定，提交 `feat: capture committed keyframe read views`。

### Task 2：输入封存与最终关闭分离

**Files:** 修改 livo_system.h/.cc、online_submap_manager.h/.cc；新增 `tests/core/input_seal_test.cc`，更新原在线生命周期测试。

**Interfaces:**
- Produces: `void LivoSystem::SealInput()`、`void OnlineSubmapManager::SealInput()`；封存后允许 pose 更新，最终 Finish 后拒绝。
- 现有 Finish 未显式封存的调用方仍幂等，保留 writer 错误优先路径；失败后原始异常重抛。

- [ ] 写 `SealAllowsCommitBeforeFinish`：真实 committed frames→Seal 两次只发一次尾块修订→合法 projected update
  提交成功→Finish 两次→后续 update/input/processing 拒绝。
- [ ] 写 `EmptyAndLegacyFinishRemainCompatible`：空在线、无子地图、旧分组三类正常结束，不重复资产或通知。
- [ ] 写 `SealFailureIsTerminal` 与 `PublishedPoseSurvivesNotificationFailure`：文件失败不继续输入，已发布 pose不回滚。
- [ ] 观察测试失败，再以窄状态区分接受输入、已封存、最终关闭；不创建通用生命周期框架。
- [ ] 跑新测试、原在线/几何/pose 系统 suite及共同 suite，提交 `feat: seal online input before final shutdown`。

### Task 3：实际任务 runner 与有界 worker

**Files:** 新增 loop_task.h/.cc、loop_worker.h/.cc、`tests/backend/loop_worker_test.cc`、`tests/backend/loop_task_test.cc`；
新增 online_backend.cmake，接入 CMakeLists 和测试注册。

**Interfaces:**
- Consumes: Task 1 的 ReadView，现有 C2a/GeometryMapper。
- Produces: 上述 LoopTask/Result；`LoopAdmissionResult RunLoopTask(const LoopTask&, const LoopAdmissionOptions&)`；
  `LoopWorker(LoopTaskRunner)`、idle/Submit/Take/Wait/Stop，按设计约束共享容量1。
- 新 online_backend 初始链接 loop_backend、io，不给基础 core 增加可选依赖；后续 Task 5增加core组件。

- [ ] 先写屏障测试 `OnePayloadUntilTaken`：第一任务执行中/结果就绪时重复Submit均拒绝；
  Take只一次，取走后才接受下一任务；记录同时任务/结果载荷峰值为1。
- [ ] 写 `StopWaitsAndWakesWithoutDetach`：慢 runner 用条件变量停住，Stop等待；释放后join完成，Wait不挂死。
- [ ] 写 `PropagatesBadAllocAndUnknown`：worker不终止进程，结果保存同一异常类别，后台不再私自吞掉。
- [ ] 写真实 `DispatchFrontierAndGeometryViews`：任务包含活动尾部的N帧raw但只有封存块；
  时间戳vector覆盖N且有限；私有缓存对 current/preview 重新建图，不扫描新增磁盘帧；真实 C2a结果与同任务直接调用一致。
- [ ] 观察 RED 后实现一个线程和条件变量；构图/读云/求解/join 均不持锁，析构只Stop。
- [ ] 跑 worker/task、C2a及共同 suite；检查单开/全关无 online_backend 目标；提交 `feat: execute bounded online loop tasks`。

### Task 4：接纳准备记录与只读解释

**Files:** 新增 loop_closure_store.h/.cc、loop_admission_io.h/.cc、tools/check_loops.cc、
`tests/io/loop_closure_store_test.cc`；修改 loop_proposer.h/.cc、cmake/loop_backend.cmake、online_backend.cmake与测试。

**Interfaces:**
- Produces: `ReadLoopAdmissionOptions(path)` 共用实现；上文 LoopDisposition/SessionStats/RecoveryReport。
- `LoopClosureStore(const KeyframeReadView&, const LoopAdmissionOptions&)`；
  `WritePrepared(const LoopTaskResult&, const PoseSnapshot&)`；
  `WriteQuery(const LoopTaskResult&, LoopDisposition, uint64_t current_version, uint64_t frame_count)`；
  `WriteCoalesced(first,last,count)`、`WriteResourceLimit(query_id,frame_count)`、`Finish(const LoopSessionStats&)`。
- `LoopRecoveryReport InspectLoopDataset(const std::filesystem::path& dataset)`，只读、不修复；CLI按状态映射0/3/1，语法2。
- 失败诊断采用尽力写入，原始异常优先；不为统计成功补一个新的提交权威。

- [ ] 先测试 `PreparedIsNotCommit`、`PublishedPoseMatchesPrepared`：真实Store/MakePoseSnapshot，
  prepared-only返回incomplete/未接纳；匹配pose后识别版本；缺prepared或篡改update/expected返回inconsistent。
- [ ] 写 `ContinuousVersionAndProvenance`：版本缺口、多个孤立prepared、错误成员/原始pose/固定sigma、
  优化前缀退缩、假complete、重复YAML字段/组均拒绝；未封存/前沿未齐保留可解释前缀并标incomplete。
- [ ] 写 `AtomicSummaryAndImmutableRecords`：summary从complete0原子替换为1，其他正式文件拒绝覆盖；
  预放pending/制造写失败时旧摘要与已有prepared/pose保持可检查，所有输入文件字节不改。
- [ ] 为提取公共I/O先跑C2a旧测试与schema断言；补 `PreviewRejectionThenLaterSuccess` 真数据集CLI用例，
  首次几何预览拒绝后后续真实查询成功，验证缓存不留旧预览；不修改默认/实数据门限。
- [ ] 实现schema和停止后检查：prepared存完整N帧原始矩阵/时间戳，session存≤64KiB原dataset元信息字节；
  Inspect逐项核对这些来源，不从优化结果猜raw。loop_admission_io.cc两消费者共用；online_backend直接编入
  loop_options.cc和loop_diagnostics.cc，不依赖未安装的loop_io目标。不读图像/不重新GICP；检查CLI新输出目录及退出码，跑共同suite。
- [ ] 提交 `feat: persist and inspect online loop acceptance`。

### Task 5：唯一所有者调度、时效检查与接纳

**Files:** 新增 loop_backend.h/.cc、loop_commit.h/.cc、`tests/core/loop_commit_test.cc`、
`tests/core/loop_backend_test.cc`、loop_backend_fixtures.h；修改 LivoSystem 私有接入及 online_backend.cmake。

**Interfaces:**
- Consumes: Tasks1–4，主系统只在所有者线程访问。
- Produces: `LoopBackend(LivoSystem&, LoopAdmissionOptions)`、Observe noexcept/Poll/Finish；
  GetStats() const 返回 LoopSessionStats；不可复制，普通私有成员直接声明。
- 内部 `LoopCommitStatus {kEligible,kStale}`、`LoopCommitCheck {status, optional<PoseSnapshot> poses}`；
  `CheckLoopCommit(const LoopTask&, const LoopAdmissionResult&, const LoopAdmissionInput& current,
  const std::vector<double>& current_timestamps)`，比较时间戳前缀；LoopCommitCheck缺省stale/空候选。
- LivoSystem friend LoopBackend 的 ClaimLoopPoseUpdates/ApplyLoopPoseUpdate/FailLoopBackend(exception_ptr)，
  claim只允许零帧零版本新系统；领取后外部Apply拒绝；基础core实现不需要链接online_backend。
- 测试Peer仅能调用私有 runner 构造重载及经现有store/manager追加fixture帧，不安装、不进入 main。

- [ ] 先用真实C2a结果测试纯 `TailGrowthPreservesTaskPrefix`：N任务、M>N当前前沿，候选优化覆盖N，
  新尾部按projected策略；近似raw旋转保持原输入，原始前端状态不变。
- [ ] 写纯 `RejectsStaleVersionRawAndMembership`：分别改合成current版本、raw矩阵元素、成员修订；
  返回stale；提案自身缺失/前沿损坏则抛异常。不能放开公开提交限制来制造旧版本。
- [ ] 测试Peer用真实RunLoopTask的屏障包装器写 `LatestOnlyButCompleteMetadata`、
  `WaitingPumpAcceptsResult`、`CrossedBudgetDoesNotCommit`：pending峰值1、元信息前缀完整、超M预算不提交。
- [ ] 写 `TrueProposalCommitsBeforeFinalFinish`：经真实Store/Manager交付三维fixture，真实C2a产生提案，
  prepared先于pose、accepted计数/版本连续，pose-only通知同一路径；seal尾块参与排空；析构不补提交。
- [ ] 写 `PostPublicationFailureIsCommittedAndTerminal`：通知/诊断分别在pose发布后失败，
  prepared+pose仍匹配，Inspect标未正常完成但识别已接纳版本；不重试同版，所有后续变更重抛原错误。
- [ ] 写 `ExclusivePoseWriterAndObserverFailure`：公开Apply受限、Observe分配/校验失败在Poll终止，
  runner未知异常传播，Stop/join在系统析构之前完成，重复Finish在成功/失败两态行为明确。
- [ ] 实现先检查结果再派发、先分配/记录再提交、成功后无异常交换；所有预算检查在大复制之前。
- [ ] 跑新测试、C2a/几何/生命周期及共同suite，提交 `feat: admit online loop updates on the system owner`。

### Task 6：共用 main 与双 ROS 接线

**Files:** 修改 livo_config.h/.cc、ros1/ros2 的livo_node.h/.cc、src/app/livo_main.cc、cmake/node.cmake、
catkin/ament导出；更新共享参数与实际ROS节点测试、安装消费者。

**Interfaces:**
- LivoConfig新增 `bool loop_closure_enabled=false`、`std::string loop_closure_options_file`，
  读取 `loop_closure.enabled` / `loop_closure.options_file`，不用复制C2a四组选项为ROS参数。
- 两端构造函数第四参数 `std::function<void()> after_process = {}`，系统ProcessNext返回后调用，包括Waiting。
- main只在启用时创建LoopBackend、注册Observe、注入Poll；显式Finish由backend排空并关闭system，
  关闭时直接走原system.Finish。支持未知异常清理并退出1；ROS对象先析构，backend再析构，system最后析构。

- [ ] 写配置测试：默认false；启用要求在线关键帧/≤64主成员；错误options、unsupported build在输出I/O之前失败；
  runtime关闭不读取不存在的options文件。两种ROS参数获得相同普通配置。
- [ ] 写两端真实节点测试：Waiting也调用after_process，hook异常传回main，原有发布/输入语义保持。
  通过单入口检查确保没有新增分版本main；ROS2定时器的while仍经同一ProcessNext。
- [ ] 扩展安装consumer：双开实际构造空在线system/backend并Finish、Inspect为consistent；
  单开/全关检查新目标缺席；shared/static和catkin/ament均不依赖源码私有目标。
- [ ] 实现新可选目标链接/导出：双开节点可加载后端库，runtime关闭不创建线程/接纳目录；
  默认全关无GTSAM依赖，不引入dlopen或第二个可执行入口。
- [ ] 跑实际节点、配置、安装消费和共同suite，提交 `feat: wire one online loop backend into both ROS nodes`。

### Task 7：官方数据、异常边界与文档验收

**Files:** 新增 docs/online-loop-backend.md、docs/p6-02c2b-validation.md；更新文档索引及本设计/计划状态。
本阶段构建、原始日志和产物放包外独立validation目录，不覆盖C2a或旧阶段。

- [ ] 冻结最终产品源码，SDK六组合、双ROS shared/static全关/双开及所有安装消费者通过；
  检查缺席目标、非零测试数、XML根计数、实际加载库，不重复catkin嵌套计数。
- [ ] Noetic/Humble各runtime OFF/ON完整136秒和短3秒回放，共8次；OFF严格对照C2a同平台全关/双开基线，
  ON严格核对同平台原始前端四数组、输入数量/载荷、原始帧、成员records和57旧导出。
  `online_submaps/poses` 与 `loop_closure` 的新增版本另验，不混入原资产相等比较。
- [ ] ON检查全部prepared/pose配对、task前沿N/接纳M、接纳/拒绝/过期/跳过/预算及槽位峰值、最终排空，
  记录RSS/耗时与输入摘要；bag零接纳不失败，真实fixture整链必须正向提交。
- [ ] 明确双开runtime OFF现在允许加载后端共享库，检查无后端工作/目录；默认全关仍无GTSAM。
  不沿用C2a wrapper对所有ON进程“禁止加载后端”的旧断言。
- [ ] 保存中途失败数据集：prepared-only、已发pose后通知失败、未知worker异常、未封存退出；
  只读检查器不改字节，返回0/3/1与schema相符，不声称恢复运行或掉电事务。
- [ ] 做一次整分支独立审阅；重要问题先失败复现再修复，相关suite全绿；填写边界、取舍与延期项。
  文档链接、Google格式、函数间空行和git diff --check通过，提交 `docs: validate online loop acceptance`。

计划自查：读取视图→runner、worker/store→owner、owner→ROS的接口在各任务Consumes/Produces中固定。
所有影响实际接纳的错误路径都有确定性测试；真实前端和线程竞争由官方bag补充，长距离与3DGS仍属后续范围。
