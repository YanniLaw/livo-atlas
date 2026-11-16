# P6-03c2b 在线 Gaussian 子地图实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户已选择当前会话直接实施，保留此执行方式；最后一次独立整阶段代码审阅，不增加逐任务代理链。

**Goal:** 在线保留并恢复已确认 Gaussian 工作集，按完整绝对位姿区分刚性重用与组内重建，输出有版本和真实覆盖的持久目录。

**Architecture:** C++ 所有者控制有界目录、目标版本和接纳，现有后台线程处理 IPC/目录写入；Python 单 GPU worker 保留一个 CPU 确认快照、管理不可变资产。IPC schema 2 仅在新持久化开关开启时使用；默认 C1 路径保持不变。

**Tech Stack:** C++17 / Eigen / OpenCV / 现有 Linux socket 与普通 SDK；Python 3.10、NumPy 1.26.4、PyTorch 2.3.0+cu121、预编译 gsplat 1.5.3；ROS1 Noetic / ROS2 Humble 同源接入。

**Spec:** [C2b 设计](../specs/2026-11-17-p6-03c2b-online-gaussian-submaps-design.md)。资产读取/写入沿用 [C2a 设计](../specs/2026-11-17-p6-03c2-gaussian-submaps-design.md)，冲突按 C2b 明确的在线接纳责任处理，不暗改资产 schema。

## Global Constraints

- 起点 `75c7e17`。只修改 livo-atlas，不动 vikit、不引入 GPU/ROS 到默认核心。
- 当前分支 `framework_completion` 原地实施；包外新验证目录，不覆盖 C1/C2a 证据。
- 继续共用 src/app/livo_main.cc，ROS1 Noetic / ROS2 Humble 只负责参数与接入。
- Google C++17，函数实现间空一行，说明必要的坐标/生命周期；不用 Pimpl、通用插件或新服务。
- gaussian.persist_submaps 默认 false；false 保留 IPC schema 1。true 要求 Gaussian enabled。
- gaussian.shutdown_timeout_ms 正整数、默认 3000；持久化示例显式 30000。
- GPU 模型最多一个；CPU 稳定确认快照一个，单 in-flight 跨块加载最多另一个临时状态。
- 工作集最多 10 视图、每视图 1,000,000 像素、每来源 1,000 高斯、总 10,000。
- 前沿/逻辑目录最多 512；拥有确认内容的块最多 32；资产最多 32 引用 + 1 替换中。
- 头 64 KiB、正文 16 MiB；primary 二进制 128×count 字节；计数/精确长度先于分配校验。
- 资产保留 C2a schema 1 和上限；map.json ≤512 KiB；新写入前预留最大资产及 1 GiB 空间。
- 仅全部源 primary 在 pose_math::Equal 的 1e-9 位姿门限内不变才允许刚性复用。
- view 20 步，reuse 0 步，rebuild 每保留帧 20 步且最多 200；不以假 worker 替代 CUDA 验收。
- pending / in-flight 各一个；前台 view 最多两个后必须尝试一个历史 repair；目标绝对合并。
- 先资产提交、再目录文件+目录 fsync、再 map_committed 确认、最后 prune；不确定时停止写入/清理。
- 本轮只写设计/计划。按用户要求先通知切换模型，之后才开始以下产品代码步骤。

## 文件职责与共享数据

| 新增/修改文件 | 职责 |
| --- | --- |
| include/livo_atlas/gaussian/gaussian_map.h | 普通内容身份、目标绑定、资产描述、目录快照 |
| include/livo_atlas/gaussian/gaussian_map_index.h、src/gaussian/gaussian_map_index.cc | 纯元数据目录、完整 canonical primary 与绝对目标比较 |
| src/gaussian/submap_protocol.h/.cc | schema 2 目标正文编解码；从 mapper 拆出跨语言消息细节 |
| src/gaussian/wire_protocol.cc、tools/3dgs_probe/online_protocol.py、submap_protocol.py | 共享帧边界、独立 schema 2 验证，保留 schema 1 |
| tools/3dgs_probe/submap_worker.py、online_worker.py | CPU 确认回退、加载/重建、checkpoint 和受限清理 |
| include/livo_atlas/io/gaussian_map_writer.h、src/io/gaussian_map_writer.cc | 专用 map.json 持久提交，无 GPU/NPZ 解析 |
| include/livo_atlas/gaussian/online_gaussian_mapper.h、src/gaussian/online_gaussian_mapper.cc | 所有者接纳和公平调度；I/O 仍在现有后台线程 |
| include/livo_atlas/core/online_gaussian_options.h、src/core/online_gaussian_options.cc、src/core/livo_config.cc | 新开关、停止时限与配置约束 |
| src/app/livo_main.cc | 同一个入口输出新增统计，不放算法或资产逻辑 |
| cmake/online_gaussian.cmake、cmake/tests.cmake | 新文件仅加入可选库/对应测试，公开类型随既有 include 目录安装 |
| tests/gaussian/、tests/io/、tests/3dgs_probe/、tests/ros1/、tests/ros2/ | 对应层的 CPU、受控 IPC、实际 CUDA 和接入验证 |
| tools/run_baseline.py、tools/compare_gaussian_replays.py | 持久化参数接线、区分训练/重用、严格成对比较与目录检查 |
| docs/online-gaussian.md、新 docs/p6-03c2b-validation.md、docs/README.md | 使用/迁移、真实结果与范围 |

新目录类型不得持有图像、点云、GPU 张量或逐块完整 PoseSnapshot。全局最新 snapshot 只有
一个共享只读引用；各内容保存完整源 primary 的局部 body 位姿，合计不超过 512 个成员。
不改变 GaussianState 根字段，不将 content_job_id 塞入旧资产 schema。

## Review Focus

1. 候选已原地修改而目标恰好切换；拒绝后仍可取得原驻留 A 的确认内容，不能错回到 B。
2. 连续微小修正或一次跳过多个版本；必须对原 canonical 全成员比较，不能容差累计或只看窗口。
3. 第一次保存没有旧资产可 prune；仍须给 worker 一次持久引用集合确认才允许离开。
4. 目录已 rename 但 fsync 失败；不能删除旧资产或继续生成第二个 staging，也不能假报未提交。
5. 高频 Observe 持续刷新目标/目录序号；不能饿死训练或历史修复，不能靠不断增加队列缓解。

这些焦点分别落实在 Task 3/5、Task 1/3、Task 2/3/5、Task 4/5、Task 5 的回归用例。

## 环境与执行约定

路径由当前电脑提供，示例变量在 shell 中设置，不硬编码到产品：

```bash
export LIVO_REPO=/path/to/workspace/src/livo-atlas
export LIVO_PHASE=/path/to/workspace/validation/P6-03c2b-YYYYMMDD
export LIVO_PYTHON=/path/to/3dgs-env/bin/python
export LIVO_PREPARE_SDK=/path/to/installed-ordinary-sdk
export LIVO_SOPHUS=/path/to/Sophus/build
export LIVO_GTSAM=/path/to/gtsam-install/lib/cmake/GTSAM
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$LIVO_REPO/tools/3dgs_probe:$LIVO_REPO/tests/3dgs_probe"
export LIVO_FIXTURE_BIN="$LIVO_PHASE/prepare-build/p603b_make_fixture"
export LIVO_PREPARE_BIN="$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views"
export LIVO_WIRE_PEER="$LIVO_PHASE/core/gaussian_wire_peer"
export LIVO_REQUIRE_GPU=1 CUDA_VISIBLE_DEVICES=0
```

创建新 LIVO_PHASE；构建前至少 10 GiB 空闲，nice -n10、-j1。复用已安装依赖和独立 Python
环境，不改系统驱动，不覆盖已有 SDK/验证构建。先配置普通 SDK 可选库 ON：

```bash
cmake -S "$LIVO_REPO" -B "$LIVO_PHASE/core" \
  -DLIVO_ATLAS_ROS_VERSION=0 -DBUILD_TESTING=ON \
  -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON -DLIVO_ATLAS_WITH_GTSAM=ON \
  -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON -DSophus_DIR="$LIVO_SOPHUS" -DGTSAM_DIR="$LIVO_GTSAM"
```

实际执行前核对 CMake 选项和平台路径；当前已有 CPU/GPU 测试基线为 80 项 Python 和
62 个普通核心 CTest 目标。新测试会增加数量，不能把原数字当作本阶段已通过证据。
开始产品实施后、首次运行完整 Python suite 前，在新目录构建当前源码的两个准备辅助程序：

```bash
cmake -S "$LIVO_REPO/tools/3dgs_probe" -B "$LIVO_PHASE/prepare-build" \
  -DCMAKE_PREFIX_PATH="$LIVO_PREPARE_SDK" -DBUILD_TESTING=ON
nice -n 10 cmake --build "$LIVO_PHASE/prepare-build" -j1
```

CPU 协议测试不导入 torch；执行实际 GPU 测试时使用独立 Python 环境，不继承 ROS 的
PYTHONPATH。只有本工具和测试两个目录进入该进程的 PYTHONPATH。

### Task 1: 有界目录和绝对位姿绑定

**Files:** 新增 include/livo_atlas/gaussian/gaussian_map.h、
include/livo_atlas/gaussian/gaussian_map_index.h、src/gaussian/gaussian_map_index.cc、
tests/gaussian/gaussian_map_index_test.cc；修改可选库和测试 CMake。

**Interfaces:**
- `GaussianContentId`：session_id、submap_id、source_generation、content_job_id。
- `GaussianTarget`：session_id/generation、OnlineSubmap group、pose_version/frame_count/
  optimized_frame_count/parent_frame、parent_from_submap、按完整 primary 排序的 submap_from_body。
- `GaussianAssetReference`：asset_id、内容身份、manifest_sha256、manifest_bytes/
  gaussian_bytes/observation_bytes。
- `GaussianSubmapEntry` / `GaussianMapSnapshot` 字段遵循 Spec 的“地图目录与覆盖”，
  目录同时带 content_job_id 与 binding_job_id，source primary 和实际 trained IDs 分离。
- `enum class GaussianOperation { kView, kReuse, kRebuild };`
- `GaussianMapIndex(std::string session_id)`；
  `Observe(const OnlineSubmapUpdate&) -> void`；
  `MakeTarget(std::uint64_t submap_id, std::uint64_t generation) const -> GaussianTarget`；
  `CanTrain(std::uint64_t submap_id) const -> bool`；
  `RecordConfirmed(const GaussianTarget&, GaussianOperation, std::uint64_t job_id,
  const std::vector<std::uint64_t>& frame_ids, int gaussian_count) -> void`；
  `RecordCheckpoint(const GaussianAssetReference&) -> bool`（仅匹配现存确认内容才附加）；
  `NextRepair() const -> std::optional<std::uint64_t>`、
  `MarkRepairAttempt(std::uint64_t) -> void`；`Snapshot() const -> GaussianMapSnapshot`。
  RecordConfirmed 保留源身份，并立即用最新绝对目标重判；不能拿旧确认覆盖新目标状态。

- [x] **Step 1: 写 RED。** 添加 WholeBlockMotionKeepsCanonicalContentAndUpdatesBinding、
  ChangedEvictedPrimaryRequiresRebuild、SameVersionFrontierGrowthPreservesOlderCoverage、
  VersionJumpUsesAbsoluteTarget、RepeatedSmallChangesDoNotMoveCanonicalBaseline、
  LogicalTailAndAssetCoverageRemainDistinct、LateConfirmationAndCheckpointCannotRegressTarget、
  ThirtyThirdContentSlotIsMissingWithoutEvictingOldAssets、RepeatedUpdatesStayBounded。
  用手算 Rz90+t(4,-2,1) 检查局部坐标；只改变已被窗口淘汰的源成员仍必须 needs_rebuild；
  分别连续施加小于 1e-9、累计大于 1e-9 的偏移，证明不是逐次近似复用。
- [x] **Step 2: 运行 RED。** 构建并执行新增测试目标。
  Run: `nice -n 10 cmake --build "$LIVO_PHASE/core" --target gaussian_map_index_test -j1`，
  再执行 `"$LIVO_PHASE/core/gaussian_map_index_test"`。
  Expected: 新类型/行为缺失导致失败，不是 fixture 本身非法。
- [x] **Step 3: 实现。** 只存有限元数据；复用 pose_math 验证/比较，检查完整源 primary。
  CanTrain 在分配新组 GPU 工作前检查 32 槽位；missing 行不持有内容或数组。
  reuse 不替换 canonical 内容，view/rebuild 确认才建立新内容基准。修复选择按尝试序号轮转。
- [x] **Step 4: GREEN。** Step 2 通过；运行既有 submap_pose_change_test、pose_snapshot_test
  以及完整 CTest：`cmake -E chdir "$LIVO_PHASE/core" ctest --output-on-failure -j1`。
  Expected: 新目录测试和既有几何契约全部通过，快照无图像/点云借用。
- [x] **Step 5: 提交。** `git diff --check` 后提交目录类型、算法、测试和构建接线。

### Task 2: 受限 schema 2 跨语言协议

**Files:** 新增 src/gaussian/submap_protocol.h/.cc；修改 src/gaussian/wire_protocol.cc、
tools/3dgs_probe/online_protocol.py、
tests/gaussian/wire_protocol_test.cc、wire_peer.cc、tests/3dgs_probe/test_online_protocol.py；
新增 tests/3dgs_probe/test_submap_protocol.py 与 C++ 对应测试目标。
Python schema 2 细节放 tools/3dgs_probe/submap_protocol.py，online_protocol.py 延迟分派，
避免 schema 1 默认路径额外加载新实现；测试共用 submap_fixtures.py。

**Interfaces:** 消费 Task 1 的 GaussianTarget / ContentId / AssetReference。
提供 `EncodeGaussianPrimaryPoses(const GaussianTarget&) -> std::vector<std::uint8_t>`；
Python online_protocol.py 中 `decode_target(header: dict, body: bytes, offset: int = 0) -> dict` 返回
capture context 去掉生成字段与 source_cameras；普通 view 头另带独立 camera_from_body。
schema 2 类型为 Spec 消息表中的 hello/hello_ok、activate/activated、view/candidate、
decision/commit_ok/discard_ok、checkpoint/checkpoint_ok、map_committed/map_committed_ok、
prune/prune_ok、stop/stopped、error。控制正文仅 primary 位姿；view 是 9×pixels +
128×primary_count。asset_ids 最多 32，规范 uint64、有序不重复；空集合合法。
错误类别含 storage_error / commit_uncertain / resource_limit；不改变 schema 1 允许列表。

- [x] **Step 1: 写 RED。** 跨 C++→Python→C++ 保留非单位锚点、完整 512 primary 的
  float64 正文和超过 2^53 的 job/asset ID；检查头仍≤64KiB、正文精确长度。
  错 schema、混用 schema、重复 JSON key、NaN、primary_count=513、溢出长度、错锚点/
  revision/前沿、畸形 SE(3)、第33个 map 引用、非法路径、0步假训练均应拒绝。
  检查 huge ID/count 失败在 numpy/GPU 分配前；重用 steps=0 可通过但不算训练。
- [x] **Step 2: 运行 RED。** `nice -n 10 cmake --build "$LIVO_PHASE/core" --target
  gaussian_wire_protocol_test gaussian_submap_protocol_test gaussian_wire_peer -j1`；
  `LIVO_WIRE_PEER="$LIVO_PHASE/core/gaussian_wire_peer" "$LIVO_PYTHON" -m unittest
  test_online_protocol test_submap_protocol -v`。
  Expected: schema 2 / 新编解码接口未实现而失败，schema 1 原用例仍有效。
- [x] **Step 3: 实现。** 两端先验证小头/计数和正文精确长度，再读 float64；不把 primary
  塞进 JSON。不接受每任务任意路径；hello 只绑定一个本运行专用目录。没有能力确认不发训练。
- [x] **Step 4: GREEN。** Step 2 和 C++ 两个协议测试全部通过；包含 socket 分片、总 deadline、
  截断/关闭、双端解码和已有 schema 1 大 ID 回归。Expected: 纯协议测试不加载 torch/CUDA。
- [x] **Step 5: 提交。** 提交 schema 2 编解码、跨语言测试；不改模型或 ROS 入口。

### Task 3: Python 确认回退、持久切换与重建

**Files:** 新增 tools/3dgs_probe/submap_worker.py、tests/3dgs_probe/test_submap_worker.py；
修改 tools/3dgs_probe/online_worker.py、tools/3dgs_probe/online_model.py（只在复用/渲染所需处
增加明确方法）、tests/3dgs_probe/test_model_state.py、tests/3dgs_probe/test_online_worker.py。

**Interfaces:** `GaussianSubmapWorker(session_id: str, state_directory: Path, model,
software: dict)`；`handle(header: dict, body: bytes) -> tuple[dict, bytes]`。
server 仅 schema 2 分支委托它，schema 1 原路径保持。消费 Task 2 的目标 context、
C2a read_state/write_state/capture_state/from_state，生成 Spec 回执。
内容身份单独保存于 worker/目录回执，不扩展 GaussianState 根字段。

- [x] **Step 1: 写 RED。** 新增实际 CUDA 测试：
  test_rejected_view_restores_prior_confirmed_parameters、test_rejected_activation_of_b_restores_a、
  test_rollback_rebinds_accepted_target、test_first_checkpoint_requires_map_acknowledgement、
  test_rigid_reuse_preserves_arrays_without_rewrite、test_evicted_member_change_requires_rebuild、
  test_rebuild_rejection_never_captures_intermediate_models、test_release_precedes_gpu_load、
  test_prune_requires_durable_reference_removal、test_sync_uncertain_preserves_both_assets。
  比较 CPU 快照独立性、原始参数逐位相同；同环境 RGB/alpha atol1e-6、z atol1e-5/rtol1e-6、
  RGB8最大差1。重建按 N×20 步，训练数据/K/曝光/C_B 保持，检查有限非零梯度/参数更新。
  回退跨块 A 后按 A 最新目标判断可用性，不能固定把 A 判 stale 或固定恢复 current。
- [x] **Step 2: 运行 RED。** `LIVO_REQUIRE_GPU=1 CUDA_VISIBLE_DEVICES=0 "$LIVO_PYTHON"
  -m unittest test_submap_worker -v`。
  Expected: 缺少 schema 2 worker 能力而失败；先核验本机 CUDA/预编译扩展，skip 不计通过。
- [x] **Step 3: 实现。** 按 Spec 确认边界持有原 resident A；捕获 view/rebuild 成功才换
  confirmed_state，reuse 只换 binding/目标状态。discard 先释放修改后的 GPU 再恢复 A。
  跨块加载 CPU 状态在决策后释放/替换，至多两个受限状态临时并存。保存时冻结快照；
  map_committed 引用集合是释放/prune 的前提。连续目标变化仍对原 canonical 比较。
- [x] **Step 4: GREEN。** Step 2、既有 model/worker/rasterizer/state 测试及完整 Python
  suite 全部通过：`timeout 300s "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -v`。
  Expected: 缺 GPU 明确非零；无两个活 GPU 模型，记录 allocated/reserved、RSS、数组/渲染
  差值；C1拒绝清空语义与C2aCPU格式回归仍通过。存储故障采用真实新目录和受控 fsync 注入。
- [x] **Step 5: 提交。** 提交实际模型生命周期、故障/真实 CUDA 测试，不增加仅测试使用的产品接口。

### Task 4: 专用地图目录的持久提交

**Files:** 新增 include/livo_atlas/io/gaussian_map_writer.h、src/io/gaussian_map_writer.cc、
tests/io/gaussian_map_writer_test.cc；仅接到可选 Gaussian 库和对应测试目标。

**Interfaces:** `WriteGaussianMapIndex(const std::filesystem::path& output,
const GaussianMapSnapshot& snapshot) -> void`，目录是已验证的新运行输出根。
`GaussianMapCommitUncertainError : public std::runtime_error` 表示 rename 已完成但同步不确定。
写入仅 map.tmp.json/map.json，不读 NPZ、不删除资产、不引入系统全局写权限调整。

- [x] **Step 1: 写 RED。** 新增 FirstIndexCanReferenceCheckpointWithoutPruning、
  WritesExactSourceAndBindingCoverage、RejectsOversizedOrMalformedIndexBeforePublishing、
  FailureBeforeRenameKeepsPreviousIndex、DirectorySyncFailureReportsVisibleUncertainCommit、
  SourceFilesAndUnrelatedPathsRemainUntouched。使用至少两块、不同 source/target pose version
  和 volatile content，检查文件引用只对应持久成功内容。
- [x] **Step 2: 运行 RED。** 构建并执行 `gaussian_map_writer_test`。
  Expected: 新 writer / 提交边界缺失而失败；fault 注入在文件操作的窄边界，不mock整个writer。
- [x] **Step 3: 实现。** 校验有界拥有数据的 snapshot，≤512KiB 序列化；文件 flush/close/
  fsync、rename、目录 fsync；失败是否已越过提交点明确区分。不沿用预览 writer 的弱同步语义。
- [x] **Step 4: GREEN。** 新目标及普通核心 CTest 全通过；检查 CMake target 图和安装消费者
  没有让默认 io/core 导出反向依赖可选 Gaussian。Expected: writer 无 GPU/ROS/NPZ 库依赖。
- [x] **Step 5: 提交。** 提交持久 writer、故障覆盖和可选构建接线。

### Task 5: 所有者接纳、公平调度与共用参数接入

**Files:** 修改 include/livo_atlas/gaussian/online_gaussian_mapper.h、
src/gaussian/online_gaussian_mapper.cc、include/livo_atlas/core/online_gaussian_options.h、
src/core/online_gaussian_options.cc、src/core/livo_config.cc、
src/app/livo_main.cc；扩展 tests/gaussian/online_gaussian_mapper_test.cc、fake_worker.py、
ros_entry_smoke.py、tests/core/global_output_options_test.cc、tests/ros1/node_options_test.cc、
tests/ros2/node_options_test.cc、tests/installed_gaussian_consumer/main.cc。

**Interfaces:** Options 新增 `bool persist_submaps = false`；公共配置读取同名 boolean 与
gaussian.shutdown_timeout_ms；共用 mapper 新增 `GetMapSnapshot() const ->
std::shared_ptr<const GaussianMapSnapshot>`。空激活成功后调用元数据接口
`GaussianMapIndex::RecordResidentReleased()`，清除已释放的原确认驻留标记。消费前四任务；C1分支不依赖v2目录流程。
后台命令分出 activate/checkpoint/write-map/map-committed/prune，仍是一个命令邮箱和线程，
不把新 protocol/序列化全部塞进已有 mapper 大文件。

- [x] **Step 1: 写 RED。** 在真实 socket 受控 peer 中延迟 candidate/commit/checkpoint/
  map-write 回执，新增 SwitchDuringMutatedCandidateSavesPreviousConfirmation、
  FirstCheckpointIsAcknowledgedBeforeLeaving、PoseUpdateDuringSaveCannotRepublishOldCurrent、
  OldCheckpointCannotReplaceNewerContent、RigidRebindDoesNotResetOrRewriteArrays、
  EvictedMemberChangeInvalidatesImmediately、TwoViewsThenHistoricalRepair、
  RepeatedStatusUpdatesDoNotStarveWork、SkippedGroupsAreMissing、CapacityKeepsOldAssets、
  StopFlushesDirtyOrReportsIncomplete、UncertainIndexPreventsPruneAndFurtherWrites。
  计数 pending/in-flight 峰值各1；大量 Observe 不增加命令队列或 retention。
  新参数默认/类型/依赖约束在公共 reader 和两端参数入口验证，错误前无新输出目录。
- [x] **Step 2: 运行 RED。** 构建普通 Gaussian mapper/config/安装消费者目标并运行新增测试；
  分别构建 Noetic/Humble 参数与入口测试。Expected: 未有持久化流程/参数而失败；C1旧用例
  保持自己的期望，不改成“任何成功都算通过”。
- [x] **Step 3: 实现。** 接受通知时只更新完整绝对目标和目录；decision 两次核验当前性。
  每一确认内容独立记录 source 与 binding；checkpoint 后持久写目录、发送 map_committed，
  最后 prune；dirty 未持久引用前禁止离开。View/repair按2:1执行次数调度，目录写一次后
  必须尝试工作，目标合并不保留历史队列。Finish按timeout输出完整/未完成真实状态。
- [x] **Step 4: GREEN。** 普通核心完整 CTest、schema1/2 Python suite、双 ROS 受控真实入口
  通过；安装普通 shared/static 和 ament 消费者，确认只依赖可选 Gaussian target。
  Expected: 开关关时不创建 states/map.json；开关开但旧 worker 明确失败、前端继续；
  两端相同布尔参数语义，仍只有一个 livo_main。记录回调耗时、队列、资源与停机行为。
- [x] **Step 5: 提交。** 提交所有者接纳、共用参数、统计和对应接入/安装测试。

### Task 6: 最终源码真实回放、迁移说明与独立审阅

**Files:** 修改 tools/run_baseline.py、tools/compare_gaussian_replays.py、
tests/test_compare_gaussian_replays.py、docs/online-gaussian.md、docs/README.md；
新增 docs/p6-03c2b-validation.md。

**Interfaces:** runner 新增 `--gaussian-persist-submaps`，要求已有 socket/output 参数，
设置 persist_submaps=true 和 shutdown_timeout_ms=30000。比较器区分 view/reuse/rebuild，
检查目录版本/会话/持久引用/实际覆盖、每个资产依次 CPU验证和总预算，不能一次载入全部资产。
原 OFF/ON 严格前端比较无容差；新目录检查为附加证据，不替代前端一致性。

- [x] **Step 1: 写 RED。** 比较器新增 test_reuse_does_not_count_as_training、
  test_map_rejects_wrong_session_or_binding、test_map_requires_verified_assets、
  test_map_enforces_asset_budget、test_volatile_content_is_not_durable、
  test_nonrigid_content_cannot_be_current、test_incomplete_shutdown_fails_acceptance，
  覆盖“0步重用冒充训练”、错会话/旧绑定、缺资产/
  错digest、超32引用/33目录、未持久化当已保存、nonrigid仍标current、incomplete停机。
  runner 解析测试验证持久化参数依赖。Expected: 原比较器未识别持久目录语义而失败。
- [x] **Step 2: 实现并 GREEN。** 更新 runner/比较器/对应测试，保留 C1 使用方式；
  `python3 -m unittest discover -s tests -p test_compare_gaussian_replays.py -v` 和完整
  3dgs_probe suite 通过；必要的其他工具回归依实际改动运行。不得写只比源文本的测试。
- [ ] **Step 3: 构建矩阵与受控实测。** 新源码普通 shared/static Gaussian OFF/ON，
  Noetic/Humble Gaussian OFF/ON（ON同时有现有GTSAM/回环依赖），双ROS真实参数/入口、
  安装消费者和公开头检查；CPU对照同源。实际CUDA用保存的真实观测覆盖2块切换、纯刚性、
  非窗口成员变形、跳版本、拒绝回退、写盘期间修正、32/33块容量和提交故障。记录真实 GPU
  覆盖平台；无容器GPU只能标明Humble受控peer，不补写成GPU成功。
- [ ] **Step 4: 官方 bag 成对回放。** 先10秒 OFF/持久ON短段，再完整约135秒成对回放；
  同源码/二进制/共享配置/CPU设置，比较四数组、发布语义、原始关键帧文件、后端最终状态。
  C1模式另做同源码短段回归。核验 bag/源码哈希；检查运行中已产生真实确认及正常停机后
  历史资产可由新GPU实例恢复，保存/加载文件不借用运行实例。Expected: 前端逐值/逐字节
  一致、目录/资产自洽、预算不超；性能只记实测，不设掩盖差异的宽容差。
- [ ] **Step 5: 文档、提交、一次独立审阅与最终复验。** 记录准确源码身份、环境、命令、
  结果、失败案例、稀疏/短数据/平台局限；检查 Google 格式、链接/围栏、依赖/安装边界。
  提交后独立整范围审阅一次，Important先RED再修复；修复后的最终源码复跑受影响测试及
  实际入口/回放，不沿用旧二进制证据。更新完成台账，归档本计划执行记录，保留分支，
  不合并、不推送、不打tag。

真实回放命令沿用[现有操作说明](../../online-gaussian.md)，在 ON 组增加上述 runner 参数。
所有输出选择新目录；不复用 C1/C2a 的正式资产做故障注入，只改自己新建的副本。

## Self-Review

- Task1 定义所有跨任务 C++ 数据；Task2 消费并固定编码，不提高已有消息上限。
- Task3 消费 Task2/C2a，负责模型确认和真实存储；Task4 只提交拥有数据的目录，不接触数组。
- Task5 串联接纳、持久引用确认和公平调度；Task6 才使用实际 ROS/bag 证明集成，不能用
  协议假模型的成功代替 CUDA 或用旧源码的 C1回放代替本阶段最终证据。
- Spec 的CPU临时双状态、cross-block A回退、content/binding分离、map_committed与
  staging上限均有所属任务和负例；没有额外承诺全SLAM恢复、稠密画质或长距离真实回环。
- 实施方式已由用户指定，仍在当前会话逐项完成；本轮文档交付后先通知用户切换模型。
