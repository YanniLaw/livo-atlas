# C3b-01 Keyframe Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 输出具有明确版本、前沿和传感器时间的完整关键帧轨迹，供 SDK、Noetic 和 Humble 消费。

**Architecture:** 普通 mapping 函数逐帧构建结果，core 的一个 worker 读取已提交时间并交付结果。
共同 main 组合已有后端与轨迹观察者；对称 ROS publisher 在后台转换和发送同一协议。

**Tech Stack:** C++17、Eigen、现有 KeyframeReadView/PoseSnapshot、catkin message_generation、Humble rosidl。

**Spec:** [C3b 设计](../specs/2026-11-15-p6-02c3b-history-design.md)，本计划只实施 C3b-01。

## Global Constraints

- 仅修改 livo-atlas 包；保留 `framework_completion` 当前工作分支，不合并、推送或打 tag。
- Google C++17、直接成员、无 Pimpl；函数实现间空一行，关键所有权补充注释。
- 保留 `src/app/livo_main.cc` 唯一入口、普通 SDK 无 ROS 依赖；不修改旧估计/TF/输出数值。
- 默认关闭；频率 1 Hz、有效 (0,10]；默认最多 20000 帧、有效 [1,100000]；错误文本最多 1024 字节。
- 一个执行任务、一个合并待处理状态；Observe 不重入、不 I/O、不物化整条轨迹。
- 新消息含 producer/revision/v/M/N；v0 使用 camera_init，正版本使用 map_frame 与 projected tail。
- 重型构建/回放串行、nice 10、-j1、OMP_THREAD_LIMIT=1；现有 135 s bag 不作为长距离证明。

## Review Focus

- 持续追加快于后台处理：仍交付当前完整结果，不能因“不是最新请求”而永久饿死（Task 2）。
- 两次正式位姿版本相同采集时间：版本递增仍更新历史，不能沿用 C3a 时间门槛（Tasks 2/4）。
- 超出预算、元数据损坏与 sink 抛异常：清空派生显示或明确记录失败，不终止原始估计（Tasks 2/3）。
- 正常/异常停止：回调、publisher 与 worker 生命周期正确，Finish 幂等且无悬空系统访问（Tasks 2/4）。
- 新消息安装消费与普通静态 SDK：必须真实生成/链接，无 ROS 分支不能泄露生成器依赖（Tasks 3/5）。

---

### Task 1: 普通轨迹快照

**Files:** Create `include/livo_atlas/mapping/trajectory_snapshot.h`, `src/mapping/trajectory_snapshot.cc`, `tests/mapping/trajectory_snapshot_test.cc`; modify `cmake/core.cmake`, `cmake/tests.cmake`。

**Interfaces:**
- Consumes: `PoseSnapshot` 与 ID 顺序的 `std::vector<double>` 时间。
- Produces: `enum class TrajectoryStatus { kReady, kResourceLimited, kError };`；`TimedKeyframePose{id,timestamp,parent_from_body}`；设计中规定的 `TrajectorySnapshot` 字段。
- Produces: `TrajectorySnapshot BuildTrajectorySnapshot(const PoseSnapshot&, const std::vector<double>&)`；无效源抛 invalid_argument，错误状态由 Task 2 封装。

- [x] 写 `UsesIndividualOptimizedPoses`：三个优化位姿的平移分别为 (1,2,3)、(8,5,6)、(9,0,1)，逐项核对，并确认不能用末尾一次平移修正解释全部。
- [x] 写空状态、v0 非正交旋转投影/保留平移、尾部/坐标/ID、一一对应时间（含重复/回退）、非法快照/时间/legacy 正版本测试。
- [x] 将测试加入 cmake，配置新的 validation 构建目录，运行目标，确认缺失接口导致 RED 并留日志。
- [x] 实现值类型及函数，复用私有 pose_math::ProjectPose；正版本按 ValidatePoseSnapshot 与严格刚体验证；不新增投影算法。
- [x] 构建并运行 `trajectory_snapshot_test` 与原有全部原生测试，Expected: 全通过；提交 `feat: add versioned keyframe trajectory snapshots`。

### Task 2: 有限后台输出和共用配置

**Files:** Create `include/livo_atlas/core/trajectory_output_options.h`, `include/livo_atlas/core/trajectory_output.h`, `src/core/trajectory_output.cc`, `tests/core/trajectory_output_test.cc`; modify global_output_options、livo_config、对应测试及 cmake。

**Interfaces:**
- Consumes: Task 1 的普通结果及 `LivoSystem::GetKeyframeReadView()`、`OnlineSubmapUpdate::poses`。
- Produces: `TrajectoryOutputOptions{enabled=false,max_frequency_hz=1,max_keyframes=20000}` 和 `ValidateTrajectoryOutputOptions(const TrajectoryOutputOptions&)`。
- Produces: `TrajectoryOutput(LivoSystem&, TrajectoryOutputOptions, Consumer)`，`Consumer=std::function<void(const TrajectorySnapshot&)>`，worker 上调用、期间借用参数。
- Produces: `Observe(const OnlineSubmapUpdate&) noexcept`, `Poll()`, `Finish()`, `GetStats() const`；析构停止/join，不新取系统状态。
- Produces: `GlobalOutputOptions::trajectory`，共用参数键严格采用设计中的三个名称。

- [x] 写真实存储/系统测试：启动空状态；固定 view 的时间读取与缓存；非刚性优化结果；同时间换版本；同 v 增 M；seal 不重复；重启 producer 不同。
- [x] 用 condition_variable 控制真实消费函数：阻塞 worker 时连续 Observe/Poll，释放后旧完整结果交付、后续合并到最新 revision；不靠定时 sleep 猜测顺序。
- [x] 写 max_keyframes=1 时两帧产生 ResourceLimited 空数组；删除 frame.yml 产生一次 Error 且系统继续；sink 抛异常不递归重发；Finish 幂等/超预算停止/异常析构覆盖。
- [x] 写配置测试：trajectory 开但 global 关报错、预算边界/非有限频率、关闭时忽略预算、加载不创建目录；RED: 目标构建缺失接口或测试断言失败，保存日志。
- [x] 实现 owner 的合并状态、独占时间缓存 worker 和消费函数；共享数据/统计由 mutex 保护；按最后交付时间节流。正版本/容量/读取失败在派生输出边界封装，不写核心失败。
- [x] 普通测试跑全套，Expected: 全通过；提交 `feat: schedule bounded background trajectory outputs`。

### Task 3: 双 ROS 消息与对称发布器

**Files:** Create `msg/TrajectoryKeyframe.msg`, `msg/KeyframeTrajectory.msg`, 两端 `include/livo_atlas/ros{1,2}/keyframe_trajectory_publisher.h`, `src/ros{1,2}/keyframe_trajectory_publisher.cc`, `tests/ros{1,2}/keyframe_trajectory_publisher_test.cc`; modify CMakeLists、package.xml、cmake/ros{1,2}.cmake、tests.cmake。

**Interfaces:**
- Consumes: `TrajectorySnapshot`，map_frame。
- Produces: 两端同名 `KeyframeTrajectoryPublisher(node, map_frame)`、`Publish(const TrajectorySnapshot&) const`，只操作其自身的两个 publisher。
- Produces: 同一份 msg 源与 `/keyframe_trajectory`、`/keyframe_path`，exact fields 为 producer_id、revision、pose_version、frame_count、optimized_frame_count、status、error、keyframes。

- [x] 写真实 ROS 收发用例：逐项位姿/时间/元数据、v0/正版本 frame_id、非 Ready 清空 Path、错误文本上限、越界时间转 Error、晚加入状态交付；RED: 生成类型缺失或目标断言失败。
- [x] 增加消息生成/类型支持/安装依赖；普通 ROS_VERSION=0 分支不调用生成器；确保生成器目标名不和已有核心目标冲突。
- [x] 实现两端发布器；双消息在同一后台调用内构造完成后发布。保留 Header.seq 的默认值；ROS1 latch/1、ROS2 reliable/transient_local/1。
- [x] Noetic/Humble 实际 ROS 测试通过；实际序列化 20000 条结果，记录字节数；两端安装外部消费者使用消息通过，普通 SDK 构建通过。
- [x] 提交 `feat: publish versioned trajectories on Noetic and Humble`。

### Task 4: 统一入口、停止流程及回放采集

**Files:** Modify `src/app/livo_main.cc`, `tools/run_baseline.py`, `tools/run_ros2_baseline.py`, `config/avia.yaml`（必要时补配置注释）；新增/修改 CLI Python 行为测试与 ROS 生命周期测试。

**Interfaces:**
- Consumes: Tasks 2/3 的 controller、publisher；已有 backend.Observe/Poll/Finish。
- Produces: 同一 main 的组合回调、backend 后再 trajectory 的 Poll、延长 Noetic WallRate 使用条件，以及安全停止顺序。
- Produces: 两端回放工具 `--keyframe-trajectory`，同时启用所需 global/online storage；单独记录 `keyframe-trajectory.json`，不改变旧数组/计数含义。

- [x] CLI 测试请求新选项并检查实际转换后的参数/录制结果；运行 RED。ROS 测试覆盖 paused/repeated/backward 非负时间下历史 revision 继续更新、当前 TF/pose 不被补造。
- [x] main 先创建发布句柄，再创建轨迹 controller；唯一回调直接组合；已有 raw 发布和 backend.Poll 的相对顺序保留。
- [x] 正常停止先 backend/system Finish，再 trajectory Finish；异常路径 controller 先于 publisher/system 析构。后端异常仍保留原 fatal 行为。
- [x] 采集脚本对新消息保留 exact metadata 与所有关键帧值；复用同平台基线比较，新增状态不污染旧 semantic 输出。
- [x] 跑当前普通全套、两端 ROS 与 Python 工具测试，Expected: 全通过；提交 `feat: wire historical trajectory outputs into the shared node`。

### Task 5: 完整验收、迁移与阶段审阅

**Files:** Update `docs/README.md`, design status；create `docs/keyframe-trajectory.md`, `docs/p6-02c3b1-validation.md`；验证脚本和原始证据保存在包外新 validation 阶段目录。

**Interfaces:**
- Consumes: 冻结产品提交、上述消息协议、C3a 已冻结基线、现有 bag。
- Produces: 可复现构建/迁移/回放报告；独立全阶段审阅结论及问题闭环。

- [x] 保存新源码归档；串行执行现有 6 SDK + 4 Noetic + 4 Humble 模式，缓存可复用但每模式日志、安装消费证据和结果独立保存。Expected: 全套通过，无不该存在的 GTSAM/ROS 依赖。
- [x] 两平台分别完整回放默认关、轨迹开/回环关、轨迹开/回环开；另用短段覆盖 geometry 关和资源不足。先冻结比较输入与 checker，再运行，不修改历史原始记录。
- [x] 核对旧 raw 数组/odometry/导出无同平台回归；新版按各结果 v/M 和时间逐项复算，检查 producer/revision 单调、状态/Path 对应与最终离线可恢复性。
- [x] 汇总 Observe/Poll、完整快照 Notify、worker 和 RSS/延迟；测试预算边界的合成规模，并明确现有 bag 的时长限制。
- [x] 文档、公开头、消息安装外部消费和 diff/style 检查通过后，提交验证文档；Expected: 无缺失证据或未披露失败。
- [x] 使用 executing-plans 要求的 fresh 最强模型整阶段审阅；按一次 RED→GREEN 修复 Important/Critical，完整复验相关矩阵；Minor 和裁定写明影响。完成后保留本地分支并向用户报告。

## 执行授权与自查

用户已于本轮要求继续实施，并此前指定当前会话直接实施、自动运行所需验证；采用 inline execution。
不另建或切换工作分支。设计的来源、类型/生命周期、协议、预算、默认关闭及验收各有对应 Task。
接口沿 Task 1→2→3→4 传递，Task 5 使用冻结产品提交；C3b-02 不在本计划中。

## 实施闭环

C3b-01 已完成，证据与审阅修复见[验证记录](../../p6-02c3b1-validation.md)。
停止竞态和 Humble 静态类型支持均已 RED→GREEN；最终静态模式从空前缀安装消费。
ROS2 构建修复后复验四组 Humble，未重复不受影响的六SDK/四Noetic；原因与代价已记录。
分支保留在本地，C3b-02 仍需单独细化。
