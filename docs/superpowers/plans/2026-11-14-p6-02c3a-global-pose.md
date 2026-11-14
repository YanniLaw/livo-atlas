# P6-02c3a 全局 TF 与当前位姿实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Preserve inline execution and the current branch.

**Goal:** 将已提交后端位姿派生为常数规模 map 修正，并在单入口双 ROS 接入中发布成对 TF 与当前全局位姿。

**Architecture:** 原所有者从 OnlineSubmapManager 的冻结快照和 raw 锚点读取修正；普通结果回调只读该值，publisher 按实际已发布 odom 四元数组合全局位姿。C2b 调度、文件提交及前端状态仍是原有路径。

**Tech Stack:** Google C++17、Eigen、既有 PCL/OpenCV/Sophus、Noetic tf、Humble tf2；不新增运行依赖或安装库。

**Spec:** [C3a 设计](../specs/2026-11-14-p6-02c3a-global-pose-design.md)。用户已在模型切换通知后以“继续”授权本会话实施。

## Global Constraints

- 基线 d5e46c5；只修改 livo-atlas，保留 framework_completion，不自动 merge/push/tag，不修改 vikit。
- 普通 C++ 核心、同一 ROS main、Noetic/Humble、Google C++17、直接成员及函数间空行；不使用 Pimpl。
- C3a 仅增加 map 修正、当前 TF/pose；C3b 的历史轨迹、地图与精确 ROS 版本协议不在本轮实现。
- `C_v=P_v(k)*inverse(ProjectPose(O_raw(k)))`，k=optimized_frame_count-1；零版本单位修正、无锚点访问。
- 正版本 legacy tail 明确拒绝；不改变现有 legacy 资产、物化数学、C2a 门限或 C2b 接纳/停止顺序。
- getter O(1) 时间/额外空间，无完整物化、轨迹复制、磁盘、云、GTSAM；只在原所有者线程读取。
- 当前全局位姿组合实际发布的原始四元数，保留旧原始表示；不拿 projected raw 替代旧发布值。
- 默认 global_output.enabled=false；publish_tf=true，map_frame=map。开启要求 keyframes+online submaps，不要求 GTSAM/回环。
- 新 TF/global_pose 使用同一次原始 odometry 的整数 ROS 时间；严格正且递增才新增输出，暂停/回退不影响原始发布和后端泵送。
- 新 pose 队列1，Noetic 非 latch，Humble reliable/volatile；一个 broadcaster 向量发送两条边，禁止 map->aft_mapped 第二父边。
- 无新线程、总线、回调分发器、第二份 raw 历史、未知全局 twist/协方差、关闭时伪造补发。
- 独立 validation/P6-02c3a-20261114 证据；重型构建串行 nice10/-j1，回放 CPU0/OMP1/1倍速/队列32/无RViz。

## Review Focus

1. 部分发布后异常与停止排空：正式版本不能回滚，最后采样后的提交不承诺出现在 ROS；Tasks4/5 的故障/空闲测试固定行为。
2. 近似 raw 旋转与非交换变换：getter 必须等于 projected tail，ROS live 必须等于实际 raw TF 链；Tasks1/2/4 分别测两种表示。
3. 时钟0、重复、回退及恢复：只跳过新增输出，原始发布与 Poll 仍运行；Task4 独立时钟进程和 Task5 真实主循环覆盖。
4. 回调阶段与前沿增长：结果回调只读已提交状态，新版本在下一次合法 LiDAR 发布可见；Task2/5 测 N<M、同版本增长和空闲提交。
5. 直接 SDK/publisher 与可选构建：不能靠 ROS 参数验证代替直接入口验证，不能由全局输出引入 GTSAM；Tasks1/3/4/5 覆盖。

---

所有任务的完整 native 回归命令：
`cmake -E chdir /home/yanni/livo-atlas_ws/validation/P6-02c3a-20261114/core env OMP_THREAD_LIMIT=1 nice -n 10 ctest --output-on-failure`。
新构建目录先配置 ROS_VERSION=0、BUILD_TESTING=ON、两个可选开关 ON、Release/shared，使用已验证的本机 Sophus/GTSAM 前缀。
初始构建和安装消费由包外 run_sdk.py both-on baseline 保存；不得以“没有找到测试”为成功。
每任务先观察新测试 RED，再实现并 GREEN，最后完整 native suite、格式/diff 检查和提交。

### Task 1：普通 MapCorrection 与刚体组合

**Files:** 新增 include/livo_atlas/mapping/map_correction.h、src/mapping/map_correction.cc、tests/mapping/map_correction_test.cc；修改 cmake/core.cmake、cmake/tests.cmake。

**Interfaces:** Consumes RigidTransform；Produces `MapCorrection {uint64_t pose_version,frame_count,optimized_frame_count; RigidTransform map_from_odom;}` 和 `RigidTransform ApplyMapCorrection(const MapCorrection&, const RigidTransform& published_odom_from_body)`。

- [x] 写 `IdentityBeforeAnyKeyframe`、`ComposesNoncommutingRigidTransforms`：默认修正可作用于任意有效发布刚体；C/O 两个非零旋转和平移，结果必须为 C*O，输入不变。
- [x] 写 `RejectsInvalidHeadersAndTransforms`：v0 非零优化前缀/非单位 C；v>0 优化前缀0或>N；NaN/Inf、反射和1.000045非刚体旋转均拒绝。v0 frame_count>0 合法。
- [x] 配置新增 test target，build `map_correction_test`。Expected：缺少新头或声明的编译 RED，原因与新增接口一致。
- [x] 在既有 keyframes 库实现值类型和常数规模校验/组合，严格刚体旋转以1e-9矩阵/行列式判定；不暴露 ROS 或 GTSAM。
- [x] 新测试及完整 native suite 通过；Expected：非零测试数，0 failures。
- [x] 提交 `feat: add ordinary map correction values`。

### Task 2：从原位姿权威读取常数规模修正

**Files:** 修改 include/livo_atlas/core/livo_system.h、src/core/livo_system.cc、src/core/online_submap_manager.h/.cc、src/mapping/pose_math.h、src/mapping/pose_snapshot.cc、cmake/core.cmake（core私有包含路径）；新增 tests/core/map_correction_system_test.cc，扩展 online_submap_manager_test.cc、cmake/tests.cmake。

**Interfaces:** Consumes Task1 MapCorrection；Produces `MapCorrection LivoSystem::GetMapCorrection() const`、OnlineSubmapManager 同名 getter；私有 `pose_math::ProjectPose(const RigidTransform&)` 和 `pose_math::ProjectedCorrection(const RigidTransform& map_anchor,const RigidTransform& raw_anchor)` 复用现有投影公式。

- [x] 写 `EmptyAndGrowingVersionZero`、`ProjectedAnchorMatchesMaterializedTail`：真实 manager/store 追加近似 raw 矩阵，在2帧优化后追加尾帧；v/N/M正确，C*Project(raw_tail) 等于现有物化尾部，不改原始资产/旧快照。
- [x] 写 `ReadsAfterPublishedNotificationFailure`：真实 pose 文件已发布后 callback 抛异常，getter仍能诊断已提交版本，后续变更/Finish继续原错误；不通过 getter 恢复运行。
- [x] 写 `RejectsLegacyPositiveVersion`、`RejectsNononlineSystem`：正 legacy getter 抛 invalid_argument；disabled/groups 系统 getter 抛 logic_error；版本0仍可读取。
- [x] 用真实 LivoSystem 的普通 ResultCallback 验证 getter 对当前已提交关键帧前沿可见；不得在 OnlineSubmapCallback 内重入。build 新测试。Expected：缺少 getter 的编译 RED。
- [x] 实现 getter 仅读冻结头、k和对应 raw；提取原 ProjectPose 但不改变 Make/Materialize 运算顺序、legacy 分支或校验容差。
- [x] 相关新旧位姿/生命周期测试及完整 native suite 通过，检查 getter 没有完整物化、历史遍历或 I/O。
- [x] 提交 `feat: expose committed map correction on the system owner`。

### Task 3：共享输出选项与前置验证

**Files:** 新增 include/livo_atlas/core/global_output_options.h、src/core/global_output_options.cc、tests/core/global_output_options_test.cc；修改 livo_config.h/.cc、cmake/core.cmake、cmake/tests.cmake。

**Interfaces:** `GlobalOutputOptions {bool enabled=false; bool publish_tf=true; string map_frame="map";}`，`void ValidateGlobalOutputOptions(const GlobalOutputOptions&)`；LivoConfig 新增 `GlobalOutputOptions global_output`，共享 schema 读取三个 global_output.* 字段。

- [x] 写 `DisabledPreservesLegacyConfiguration`、`ValidFrameNamesAndIndependentControls`：关闭不检查 map 名与在线前提；开启可用 map、atlas/map_2、123；publish_tf=false 不关闭 pose。
- [x] 写 `RejectsUnsafeOrConflictingFrameNames`：空、首尾/、连续/、空白、非ASCII、.、:、camera_init/aft_mapped/world均拒绝；直接选项 validator 同样校验。
- [x] 用仅提供普通参数值的 test ParameterReader 输入真实 LoadLivoConfig，写 `ReadsGlobalSchemaBeforeOutputIo`、`RequiresOnlineStorageOnlyWhenEnabled`：读取三个字段、依赖错误及已有 sentinel 不改；loop_closure 仍false。
- [x] build `global_output_options_test`。Expected：新选项类型/成员/验证接口尚不存在的 RED。
- [x] 实现共享选项和读取；在线依赖检查属于 LoadLivoConfig，纯选项 validator 不检查系统或打开目录。
- [x] 新测试及完整 native suite 通过，提交 `feat: validate shared global output options`。

### Task 4：两端发布器的成对 TF 与全局 PoseStamped

**Files:** 修改 ros1/ros2 livo_publisher.h/.cc、tests/ros1/ros2/livo_publisher_test.cc、cmake/tests.cmake、package.xml；新增 tests/ros1/global_output_clock_test.cc、tests/global_output_clock.test、tests/ros2/global_output_clock_test.cc。

**Interfaces:** Consumes Tasks1/3；两个 LivoPublisher 构造末尾 `GlobalOutputOptions global_output={}`，Publish 末尾 `const std::optional<MapCorrection>& correction=std::nullopt`。直接成员保存选项、global_pose publisher、`int64_t last_global_stamp_ns=0`，不增加隐藏实现。

- [x] 先写两端 `GlobalPoseUsesActualLegacyQuaternionAndPairedTf`：订阅真实 tf2_msgs/TFMessage 和 global_pose，输入近似 raw 旋转及非单位 C；raw数据保留，TFMessage含map->camera_init和camera_init->aft_mapped，整数时间相同，global pose等于这条链；与错误的 C*Project(raw) 明确有区别。
- [x] 写 `DisabledDoesNotAdvertiseGlobalPose`、`PoseOnlyModeKeepsSingleTf`、`MissingOrInvalidCorrectionFailsBeforePublication`：默认无新增pub；publish_tf=false只有旧TF但有global pose；启用而缺修正或无效时不产生半个新结果。
- [x] 新实测时钟用例 `ZeroRepeatRollbackAndResume`、`VisualWaitingAndPropagationDoNotAdvanceGlobalOutput`：0/10/10/9/11序列，仅10/11新增，恢复使用最新C，raw仍按旧分支发。Noetic 时钟为进程级，单独rostest executable；Humble 以独立 use_sim_time node测试，防污染旧用例。
- [x] 在本阶段独立 Noetic/Humble source copy 编译新测试，Expected：构造/Publish参数缺失的 RED；不修改C2b的source/cache/log。
- [x] 实现读取一次现有 raw odometry 时间、由实际raw四元数组合、先验证计算后发送、同broadcaster向量发两条TF、pose深度1和时间门控；clock异常仍保留旧TF，不改原发布Euler规则或旧消息时钟调用位置。
- [x] 声明 tf2_msgs 为直接测试依赖；不为产品新增消息包/自定义消息，Humble沿用已有tf2_msgs测试接线。
- [x] 两端实际 publisher/clock 测试、新旧普通suite全部通过；native完整suite通过，提交 `feat: publish paired global transforms and current poses`。

### Task 5：节点只读接线、安装消费与可复现回放入口

**Files:** 修改 ros1/ros2 livo_node.cc、两端 node_options_test.cc/node_test.cc、tests/installed_admission_consumer/main.cc、tools/run_baseline.py、tools/run_ros2_baseline.py及其现有Python测试；保持 src/app/livo_main.cc 单入口。

**Interfaces:** Consumes getter/options/Publish签名；节点把配置交给publisher，仅普通结果回调 updated且非Visual时读取 correction；原 after_process/Poll顺序不变。既有runner新增 `--global-output`，只启用 global_output.enabled 与所需在线存储，不隐式开启回环。

- [x] 两端参数真实读取用例覆盖默认、enabled/storage、publish_tf/map_frame、错误类型/名称与pre-I/O sentinel；正常SDK scalar值与node参数一致。
- [x] 写 `GlobalOutputTracksOnlyCommittedOwnerVersion`：真实 system处理观测、发布后接纳 projected 更新，下一有效原始样本才看到新C；Waiting/idle Poll继续，无虚假新TF/pose。构造选择global开但算法全关时也能发布单位修正。
- [x] 写 `FinalDrainDoesNotInventPublication`：真实所有者最终排空可更新文件/getter，但没有新原始样本则无新ROS消息；原失败优先级不变。
- [x] Python parser RED：--global-output+keyframe_groups在任何I/O前拒绝，默认路径不变；单global开仅要求online存储、loop仍关，非法queue原规则继续。
- [x] 观察node未传新参数/修正导致新用例失败后接线；runner只做参数启动和观测记录，TF/pose记录与旧前端摘要分开，保留旧摘要键及原始输入校验。
- [x] 扩展 installed_admission_consumer：显式include新普通头，所有构建调用ApplyMapCorrection；可用在线系统 getter验证v0；不依赖src私有头、ROS或隐式GTSAM。
- [x] Python suite、实际双ROS节点/时钟测试、安装消费者和完整native suite通过，提交 `feat: wire global pose outputs through both ROS nodes`。

### Task 6：冻结、官方回归与使用文档

**Files:** 新增 docs/global-pose-output.md、docs/p6-02c3a-validation.md；更新文档索引/本spec与plan。所有执行脚本、快照与结果位于包外本阶段validation目录。

- [x] 冻结产品源码并逐文件核对双ROS副本；SDK六组合、双ROS八组合、全部安装消费通过，统计XML根、不重复catkin wrapper；检查目标缺席、63个以上普通头（以实际数为准）、GTSAM缺席与static消费无livo共享库。
- [x] 按spec每端3完整+3短段共12次回放，旧四数组/资产/发布字段严格对照C2b；新参数、global_pose及新TF另验。记录实际运行库SHA/CPU/OMP/nice/RSS/时间和输入摘要。
- [x] 从正式pose/raw锚点算允许修正，验证paired TF/pose同整数时间、无第二父、正递增时间；同值修正对应多版本时报告可解释版本范围，不声称从无版本字段的ROS消息唯一反推版本。native测试固定真正版本。
- [x] 支持/不支持后端构建的启动约束、zero/paused/backward时钟、publish_tf=false、非法配置pre-I/O和最终排空行为有独立证据；仅global开启不要求GTSAM或回环文件。
- [x] 文档明确live TF与历史优化轨迹的区别、投影/发布表示、clock跳过与恢复、最后版本不保证补发，以及C3b/3DGS边界；状态此时为实现/主验收完成、独立审阅待完成。
- [x] Google格式/函数空行/本地链接/diff检查通过，完整native suite通过，提交 `docs: validate global pose outputs`。

## 整阶段审阅与结束

Tasks1–6 的 task-done 全部完成后，按 executing-plans 做一次新的整阶段独立审阅；范围以本计划提交为BASE。
Important/Critical先RED再修复，一次修复轮次及完整suite，不发起第二次审阅；Minor记录延期。
产品修复有独立源码与受影响验证记录，不把旧回放冒称新二进制回放。最后补齐审阅状态、收尾提交，
把本计划scratch移至本阶段证据归档，保留本地分支。既有用户授权的inline方式与当前分支偏好继续有效，不逐任务询问是否继续。

计划自查：Task1→2/4/5、Task2→5、Task3→4/5、Task4→5、Task5→6接口已固定；
完整C3历史地图不混入本轮；所有五类Review Focus均在拥有代码的任务中指定测试。


收尾记录：六项任务及整阶段独立审阅完成。产品代码冻结39f7494，主验收/审阅候选文档77760be；
14组构建与消费者、12组真实入口、12次官方回放通过，无Critical/Important。
1项回放验收器版本解释顺序检查Minor延期，当前六份global ON记录已另行只读核对无回退；
具体证据、裁定和限制见[验证记录](../../p6-02c3a-validation.md)。保留本地framework_completion，不合并或推送。
