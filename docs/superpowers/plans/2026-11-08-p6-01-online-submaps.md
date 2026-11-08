# P6-01 Online Submaps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在原 FAST-LIVO2 前端之外提供可验证的在线子地图成员、独立观测交付和位姿修正基础。

**Architecture:** mapping 提供普通类型、切分和坐标计算；io 提供独立在线索引；core 串行组合提交。
沿用现有 keyframes/io/core 物理库，原 Submap 路径默认不变。优化版本与覆盖前缀分别管理，
为后续几何 SLAM 和 Gaussian mapper 共用观测、分别更新地图做准备。

**Tech Stack:** Google C++17、CMake 3.16、已有 Eigen/PCL/OpenCV/Sophus/GTest、Noetic/Humble，无新依赖。

**Spec:** [在线子地图设计](../specs/2026-11-08-online-submaps-design.md)。执行者必须同时阅读。

**状态：用户已切换模型并授权实施；Task 1、2、3、4 已完成，其余任务待实施。**
切换后沿用用户已选择的当前会话直接实施方式，使用 executing-plans；不再要求选择执行方式。

## Global Constraints

- 仅 livo-atlas；规划基线 refactor2/`0a3c0ef`，产品代码基线 `4733692`。开始实施时记录新的实际 HEAD 和工作区差异。
- Google C++17；保留 VIOManager、Frame、Feature、Pose6D、SubSparseMap、VOXEL_LOCATION。
- 普通核心、同一 ROS main、对称 Noetic/Humble 接入保持；不新增 ROS 算法分支或物理库。
- 原 keyframes、poses、submaps schema 1 保持；`submaps.enabled=false`，`submaps.mode=keyframe_groups` 默认。
- 不引入 GTSAM、CUDA、LibTorch、后台训练、几何地图算法、前端修正反馈、通用消息总线或数据库。
- 原估计、同步、着色和导出公式保持。视觉负坐标索引缺陷另案处理；本阶段不顺带修复。
- 原始 bag、旧验证目录和其他 Git 引用保持不动；不切分支、不合并、不推送、不改 tag，不触碰其他包。
- 验证单独建包外目录，编译和回放串行，nice 10、`-j1`、`OMP_THREAD_LIMIT=1`、无 RViz。
- 任何测试数字在实际运行后填写；模拟修正不能证明真实闭环精度，135 秒 bag 不能代替长距离测试。

## Review Focus

1. 后端计算期间追加帧：base_version 仍有效，冻结文件不改写，尾帧延续可重开复现。Task 2/3/4。
2. 原始帧已落盘但索引中途失败：只读可见已提交前缀和未索引尾部，消费者不收到半次事件。Task 3/4。
3. 支持帧跨组、消费者跳过事件：已有资产只修正一次，新资产不伪造旧姿态，旧训练结果不覆盖新版本。Task 2/4。
4. 非规范/巨大 ID、伪造数量、坏旋转和时间回退：在分配或状态推进前拒绝；合法输入仍可继续。Task 1/2/3。
5. 相机与 body 时间不同、无图像 LO/LIO、回调保留跨帧：相对关系和资产所有权明确，不借用前端缓冲。Task 2/4/5。

---

## 文件与依赖分工

下面路径均相对于包根；列出的新源码只在用户切换模型并继续后创建。

| 文件 | 职责 |
| --- | --- |
| `include/livo_atlas/mapping/online_submap.h`、`src/mapping/online_submap.cc` | 在线配置、成员修订、纯分组 Builder |
| `include/livo_atlas/mapping/pose_snapshot.h`、`src/mapping/pose_snapshot.cc` | KeyframePose、优化输入、快照验证及尾部物化 |
| `include/livo_atlas/mapping/submap_pose_change.h`、`src/mapping/submap_pose_change.cc` | 子地图整体/局部修正分类、图像相机位姿 |
| `include/livo_atlas/io/online_submap_store.h`、`src/io/online_submap_store.cc` | 在线记录和冻结位姿文件读写 |
| `include/livo_atlas/core/online_submap_update.h` | 拥有观测的通知数据，无线程或算法实现 |
| `src/core/online_submap_manager.h/.cc` | 私有串行组合，拥有 Builder/Store、原始位姿和冻结快照 |
| 既有 `core/livo_system`、`system_options`、`livo_config` | 调用组合、公共入口、唯一参数清单 |
| 既有 `io/keyframe_store`、私有 `io/dataset_metadata` | 原格式兼容、只读 body 位姿；复用现有元数据写法 |
| 既有 `tools/inspect_submaps.cc`、两个 baseline 脚本 | 显式 online 模式和回放开关 |

依赖方向：mapping → 原有普通标定/数据类型；io → mapping；core → mapping/io。
新的公开头随既有安装规则安装并通过单头编译；core 私有头不安装，不恢复 internal 目录。
各任务将对应 `.cc` 加入 `cmake/core.cmake` 既有目标，将测试加入 `cmake/tests.cmake`。
不改独立 exporter 小测试工程 `tests/io/CMakeLists.txt` 来重复维护新目标。

## 配置、格式与接口约定

| 共用参数 | C++ 所在字段 | 默认与校验 |
| --- | --- | --- |
| `submaps.enabled` | `SystemOptions::submaps.enabled` | false；启用要求 keyframes.enabled |
| `submaps.max_keyframes` | `SystemOptions::submaps.max_keyframes` | 20，保留旧规则；在线同样使用这个上限 |
| `submaps.mode` | `SystemOptions::submap_mode` | `SubmapMode::kKeyframeGroups`；仅接受 keyframe_groups/online |
| `submaps.online.translation_threshold` | `online_submaps.translation_threshold` | 5.0 m，有限且 >0 |
| `submaps.online.rotation_threshold_deg` | `online_submaps.rotation_threshold_deg` | 45.0°，有限且 >0 |
| `submaps.online.time_threshold` | `online_submaps.time_threshold` | 20.0 s，有限且 >0 |
| `submaps.online.support_keyframes` | `online_submaps.support_keyframes` | 2，0..max_keyframes |

`OnlineSubmapOptions` 只含四个 online 字段，不复制 enabled/max_keyframes。
`SubmapMode` 与该类型放在 online_submap.h；旧 `SubmapOptions{enabled, limit}` 初始化仍有效。
共享读取器只在 mode=online 时读取/校验 online 字段，避免旧模式被无关在线参数阻断。
max_keyframes 仍按既有规则在禁用时也校验；mode 拼写错误总是拒绝，不静默回退。
ROS1 参数的 `/` 与 ROS2 的 `.` 仍由已有 ParameterReader 转换，节点不重复定义清单。

新记录路径固定为 `keyframes/online_submaps/`：

- `dataset.yml`：schema=1、mode=online、切分参数；body 为 IMU，anchor 为首个主成员；v0=odom、v>0=map。
- `records/<id>/<revision>.yml`：schema、id、membership_revision、state(active/sealed)、anchor_keyframe_id、keyframe_ids、support_keyframe_ids。
- `poses/<version>.yml`：schema、version、base_version、parent_frame、frame_count、optimized_frame_count、tail_policy、tail_anchor_keyframe_id、poses。

uint64 ID、版本和数量字段用规范十进制字符串，浮点沿用 max_digits10 字符串；schema/配置整数沿用已有规则。
正版本 tail_policy 固定为 last_optimized_anchor，tail_anchor_keyframe_id=N-1；v0 不写文件。
pose 行继续叫 `map_from_body`，保持 KeyframePose 字段/API 兼容；运行快照的 parent_frame 明确解释其坐标，
v0 中该历史字段承载 odom_from_body，正版本承载 map_from_body。新代码禁止只看字段名判断父系。
新快照向量规定 `poses[i].id == i`；旧 WritePoses 接受乱序输入的既有行为保持。

## 验证入口

以下命令是实施阶段执行的命令，本轮不运行。执行者先在工作区设置：

```bash
export LIVO_P6_WS=/home/yanni/livo-atlas_ws
export LIVO_P6_PACKAGE="$LIVO_P6_WS/src/livo-atlas"
export LIVO_P6_VALIDATION="$LIVO_P6_WS/validation/P6-01-20261108"
export OMP_THREAD_LIMIT=1
mkdir -p "$LIVO_P6_VALIDATION"
cmake -S "$LIVO_P6_PACKAGE" -B "$LIVO_P6_VALIDATION/core-shared" \
  -DLIVO_ATLAS_ROS_VERSION=0 -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX="$LIVO_P6_VALIDATION/install-shared"
```

跨电脑只调整工作区、已有依赖位置及允许的 CPU；不下载替换已有依赖来解释数值变化。
若该证据目录已存在，选择新的带日期/后缀目录并记录，不清空旧目录。
各任务命令中的 `core-shared/<test>` 以 `$LIVO_P6_VALIDATION/` 为前缀。RED 阶段只构建明确的目标；
预期缺接口编译失败或指定断言失败，环境/链接错误不算 RED。GREEN 必须零退出、无失败测试。
每个任务先把新测试注册到CMake，再重配同一build目录；不能把“目标不存在”当作契约的RED。
新公开头加入后也重新配置，确保既有单头编译枚举包含它们。

### Task 1: 在线成员状态机

**Files:** 新 online_submap.h/.cc、`tests/mapping/online_submap_test.cc`；修改两份共用 CMake。

**Interfaces:** 消费旧 Submap 和 RigidTransform；提供：

```cpp
enum class SubmapMode { kKeyframeGroups, kOnline };
enum class OnlineSubmapState { kActive, kSealed };
struct OnlineSubmapOptions {
  double translation_threshold = 5.0;
  double rotation_threshold_deg = 45.0;
  double time_threshold = 20.0;
  int support_keyframes = 2;
};
struct OnlineSubmap {
  Submap primary;
  std::vector<std::uint64_t> support_keyframe_ids;
  std::uint64_t membership_revision = 1;
  OnlineSubmapState state = OnlineSubmapState::kActive;
};
void ValidateOnlineSubmapOptions(int max_keyframes,
                                const OnlineSubmapOptions& options);
class OnlineSubmapBuilder {
 public:
  OnlineSubmapBuilder(int max_keyframes, OnlineSubmapOptions options);
  std::vector<OnlineSubmap> AddKeyframe(std::uint64_t id, double timestamp,
                                     const RigidTransform& odom_from_body);
  std::vector<OnlineSubmap> Finish();
};
```

Builder 只保留 ID、锚点位姿/时间和必要计数，不保存图像/点云。返回值按提交顺序排列且独立拥有。
无效输入抛 invalid_argument 且不推进状态；Finish 后 Add 抛 logic_error；溢出显式拒绝。

- [x] Step 1：写 RED。`CountLimitProducesInsertionThenSealingRevision`：上限2、support2，Add0 返回 active{0}/r1；Add1 返回 active{0,1}/r2、sealed{0,1}/r3；Add2 新组{2}/r1、support{0,1}。
  `FinishSealsOnce`：尾组 Finish 返回 sealed{2}/r2，再次为空；空 Builder Finish 无组。

  ```cpp
  OnlineSubmapBuilder builder(2, OnlineSubmapOptions{});
  ASSERT_EQ(builder.AddKeyframe(0, 0.0, RigidTransform{}).size(), 1u);
  const auto revisions = builder.AddKeyframe(1, 1.0, RigidTransform{});
  ASSERT_EQ(revisions.size(), 2u);
  EXPECT_EQ(revisions.back().membership_revision, 3u);
  EXPECT_EQ(revisions.back().state, OnlineSubmapState::kSealed);
  ```

- [x] Step 2：补参数化 RED `ThresholdEqualityStartsNewGroup`：固定其他阈值足够大，分别测试平移5、Rz45°、时间20的等号触发，以及紧邻阈值两侧；旋转使用最短 SO(3) 角，覆盖179°到-179°相对2°。
  `RejectedInputDoesNotAdvance`：NaN/Inf、重复/跳号ID、时间回退、坏旋转失败后，相同下一个合法ID仍成功；相等时间允许。
  首组support为空，后组只取直接前组末尾 min(support,count) 个，support0有效、负值/超上限拒绝。
- [x] Step 3：执行 `nice -n 10 cmake --build "$LIVO_P6_VALIDATION/core-shared" --target online_submap_test -- -j1`；确认 RED 来自新契约。
- [x] Step 4：实现先全量校验、再计算有序修订；运动/时间边界在插入前封存，数量边界在插入后封存。
  同次插入需要的所有计数检查先完成，不能先改旧组再发现新组非法。
- [x] Step 5：同命令构建，运行 `"$LIVO_P6_VALIDATION/core-shared/online_submap_test"`；全部 GREEN。
- [x] Step 6：仅提交本任务文件，提交信息 `feat: add deterministic online submap grouping`。

### Task 2: 位姿快照、尾部延续与修正分类

**Files:** 新 pose_snapshot.h/.cc、submap_pose_change.h/.cc、
`tests/mapping/pose_snapshot_test.cc`、`tests/mapping/submap_pose_change_test.cc`；
修改 keyframe_store.h 引用迁移和两份共用 CMake。

**Interfaces:** 将现有 KeyframePose 定义原样迁入 pose_snapshot.h，不复制定义；原 io 头包含新头。

```cpp
enum class PoseFrame { kOdom, kMap };
struct PoseUpdate {
  std::uint64_t base_version = 0;
  std::uint64_t optimized_frame_count = 0;
  std::vector<KeyframePose> poses;
};
struct PoseSnapshot {
  std::uint64_t version = 0;
  std::uint64_t base_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  std::vector<KeyframePose> poses;
};
void ValidatePoseSnapshot(const PoseSnapshot& snapshot);
PoseSnapshot MaterializePoseSnapshot(const PoseSnapshot& frozen,
    const std::vector<RigidTransform>& raw_poses, std::uint64_t frame_count);
PoseSnapshot MakePoseSnapshot(const PoseUpdate& update, const PoseSnapshot& previous,
    const std::vector<RigidTransform>& raw_poses);
struct SubmapPoseChange {
  std::uint64_t submap_id = 0;
  std::uint64_t old_version = 0;
  std::uint64_t new_version = 0;
  RigidTransform old_parent_from_submap;
  RigidTransform new_parent_from_submap;
  std::vector<std::uint64_t> changed_keyframe_ids;
};
SubmapPoseChange CompareSubmapPoses(const OnlineSubmap& existing_members,
    const PoseSnapshot& old_poses, const PoseSnapshot& new_poses);
RigidTransform CameraPose(const Keyframe& keyframe, const PoseSnapshot& poses);
```

raw_poses 的下标就是 ID；默认空 PoseSnapshot 是 v0。Validate 检查完整顺序、父系/版本/数量、合法 SE(3)。
正版本要求 base_version=version-1、0<N<=M；v0 要求 base_version=N=0、parent=odom。
Materialize 用 v0 原始位姿或正版本固定 C_tail 产生完整前缀；正版本要求请求范围在 frozen.frame_count..raw.size()，
并核对 frozen 尾部确实符合声明规则，不能缩短或超出提交范围。v0 允许0..raw.size()。
Make 验证 base_version、优化前缀不缩短、N>0、N<=raw.size()，生成 previous.version+1、覆盖 raw.size() 的候选快照。
Make/Materialize先验证raw中的所有变换、当前冻结快照和数量边界；尾部一致性沿用1e-9 m/rad的计算舍入判定。
所有函数无副作用。Compare 要求同覆盖前缀，比较已有组主成员与support的并集；changed IDs 为空代表可走整体刚性路径。
CameraPose 无有效图像/相机变换时拒绝，不猜外参或忽略图像时间。

- [x] Step 1：写 `TailExtensionKeepsFrozenPrefix`：raw x={0,1,2}、更新base0/N2/map x={10,11}，候选v1/M3/N2 的x={10,11,12}。
  追加raw x=3后同v1物化M4，x={10,11,12,13}，原冻结对象仍M3且内容不变；不写磁盘。
  `NewFramesDoNotStaleBaseVersion`：base1/N3在raw增加到5后仍接受为v2/M5；base0拒绝；N回退、0、超范围均拒绝。

  ```cpp
  // frozen/raw_poses use the literal trajectory above, after appending ID 3.
  const auto view = MaterializePoseSnapshot(frozen, raw_poses, 4);
  EXPECT_EQ(view.version, 1u);
  EXPECT_EQ(view.optimized_frame_count, 2u);
  EXPECT_DOUBLE_EQ(view.poses[3].map_from_body.translation.x(), 13.0);
  EXPECT_EQ(frozen.frame_count, 3u);
  ```

- [x] Step 2：写 `InvalidSnapshotRejected`：重复/缺号/乱序/超当前前缀的64位ID、UINT64_MAX版本回绕、伪造frame_count、错误parent、坏旋转/NaN；不按未经校验的数量 reserve。
  超32位值不得截断；其拒绝原因是超出现有数据，而不是任意限制为32位ID。
  `VersionZeroHasNoTailAnchor`：空快照及原始完整前缀有效；正版本错尾帧在物化时拒绝。
- [x] Step 3：写 `RigidAndLocalChangesSeparate`：两帧原Rz90，平移(1,2,3)/(1,4,3)，点(1,0,0)/(2,0,0)，原局部点为(1,0,0)/(4,0,0)。
  全部位姿左乘同一 Rx90+t(10,-3,2)，changed为空，局部点不变；独立设置新anchor I/t(10,0,0)、第二帧Rz90/t(10,5,0)，第二局部点变(0,7,0)，changed={1}。
  比较阈值固定为1e-9 m/rad，用稳定的 SO(3) 角差计算覆盖极小旋转；原始点和图像摘要保持。
  支持帧单独改变必须出现在changed中，anchor自身局部恒为I，不能误报；外部anchor修正只应用一次。
- [x] Step 4：写 `CameraRelativePoseSurvivesCorrection`：使用 KeyframeResult 的非同一状态/图像时间，验证
  `inverse(T_map_B)*T_map_C == inverse(T_odom_B)*inverse(T_C_odom)`；旧相机快照和时间不变，LO/LIO缺图像拒绝CameraPose。
  `SkippedMembershipInitializesOnlyNewObservations`：从消费过的旧OnlineSubmap成员集比较v0到v2；新增ID不列为旧资产的修正，旧/新快照缺已有ID时拒绝。
- [x] Step 5：分别构建/运行 `pose_snapshot_test`、`submap_pose_change_test` 确认 RED，然后实现纯函数，使用 double 组合变换并复用 ValidateExtrinsic。
  尾部遵循 `C_tail=T_new(N-1)*inverse(T_raw(N-1))`；不改前端状态、速度、bias或原始资产。
- [x] Step 6：运行两个新目标及旧 `keyframe_store_test`、`submap_store_test`，全部 GREEN；提交 `feat: define versioned submap pose corrections`。

### Task 3: 不可变在线索引与冻结位姿存储

**Files:** 新 online_submap_store.h/.cc、`tests/io/online_submap_store_test.cc`；
修改 keyframe_store.h/.cc、`tests/io/keyframe_store_test.cc`、两份共用 CMake。

**Interfaces:** 消费 Task 1/2 类型。新增 `RigidTransform KeyframeStore::ReadRawBodyPose(std::uint64_t id) const`，
Store构造时沿用dataset.yml的schema验证；方法只解析对应frame.yml的id/标定标识和odom_from_body，
不解码图像/点云；原Read仍校验完整资产。旧frame.yml没有schema字段，不修改旧格式补字段。

```cpp
class OnlineSubmapStore {
 public:
  OnlineSubmapStore(const std::filesystem::path& directory,
      const KeyframeStore& keyframes, int max_keyframes, OnlineSubmapOptions options);
  OnlineSubmapStore(const std::filesystem::path& directory, const KeyframeStore& keyframes);
  void AppendRevision(const OnlineSubmap& submap);
  OnlineSubmap Read(std::uint64_t id) const;
  std::size_t size() const;
  std::uint64_t indexed_frame_count() const;
  void WritePoseSnapshot(const PoseSnapshot& snapshot);
  PoseSnapshot ReadPoseSnapshot(std::uint64_t version) const;
  PoseSnapshot MaterializePoseSnapshot(std::uint64_t version, std::uint64_t frame_count) const;
};
```

Store 不拥有 KeyframeStore，前者先析构；单一串行所有者。新建目录存在即拒绝，重开只读、不恢复追加。
Read 返回最新已提交修订；index覆盖前缀可短于原始数据。ReadPoseSnapshot(0)物化当前全部原始帧，正版本返回冻结范围。
输入非法/重复目标抛 invalid_argument 且不污染 writer；真实写入失败锁住写实例并保留原异常。
磁盘损坏抛带路径的 runtime_error；所有只读操作不修复、不新建文件。
Append只允许首revision含一个主成员，后续一次追加一个成员或仅封存；support和anchor不变，新组只能在前组封存后出现。
WritePoseSnapshot要求覆盖当时全部已提交原始帧、版本连续且优化前缀不缩短；先验证全部数据再创建pending文件。

- [x] Step 1：写磁盘 RED `RevisionsRoundTripAndRemainImmutable`：Task1的全部修订落盘/重开，旧修订字节不变；active尾组仍active；空数据集无虚构组。
  `RawPoseReadDoesNotLoadAssets`：用测试副本删除PNG/PCD后ReadRawBodyPose仍返回元数据位姿，完整Read失败；坏frame.yml/未知ID仍拒绝。
- [x] Step 2：写 `RejectsCorruptHistoryBeforeAllocation`：缺修订、改anchor/已有成员、seal后新revision、主归属重复/有洞、support非前组末尾、两个active、未知schema/ID、非规范字符串、伪造巨大count均拒绝。
  读取所有提交修订验证历史，不能只信最大文件名；pending文件忽略；中间缺号或最高现存文件损坏时拒绝，不能退回旧revision伪装成功。
  没有独立提交清单，无法识别整段末尾文件被删除；明确这一格式限制，不写不可能通过的检测测试。
- [x] Step 3：写 `PartialCommitRemainsInspectable`：原始0..2已提交，组0 sealed{0,1}，新组尚未提交；重开indexed_count=2、未索引尾部=1；无自动补齐。
  把记录目录替换为普通文件造成真实写失败，之后Append/Write仍抛原错误，已提交字节不变。
  同样覆盖旧组封存后新组首修订失败、active插入成功但seal失败；无跨文件回滚假象。
- [x] Step 4：写 `FrozenPoseFileReproducesGrowingView`：v1/M3/N2写入后追加raw3，Read仍M3，Materialize(v1,4)尾帧13，文件摘要不变。
  旧API仍只认识原poses；新Store拒绝覆盖v1、跳版本、base过期、优化前缀缩短、非法尾策略/尾锚点和伪造尾姿态。
  只写pending时重开看不到新版本；真实位姿写失败后v1仍可读，v2不可见。

  ```cpp
  // reader is reopened after raw ID 3 is committed.
  EXPECT_EQ(reader.ReadPoseSnapshot(1).frame_count, 3u);
  const auto extended = reader.MaterializePoseSnapshot(1, 4);
  EXPECT_EQ(extended.frame_count, 4u);
  EXPECT_DOUBLE_EQ(extended.poses[3].map_from_body.translation.x(), 13.0);
  EXPECT_THROW(reader.MaterializePoseSnapshot(1, 5), std::invalid_argument);
  ```

- [x] Step 5：构建并运行 `online_submap_store_test`、`keyframe_store_test` 确认 RED。
  实现沿用私有 dataset_metadata 解析和checked-close；临时文件放在目标同目录，最后rename；单writer下拒绝覆盖，所有计数与实际节点/已提交范围核对后再分配。
  只缓存必要位姿/成员元数据，不在打开索引或物化位姿时加载全部图像/PCD。
- [x] Step 6：两个测试及旧 `submap_store_test` 全部 GREEN；提交 `feat: persist online submap revisions and pose snapshots`。

### Task 4: 共用系统的提交顺序与拥有观测通知

**Files:** 新 online_submap_update.h、私有 online_submap_manager.h/.cc、`tests/core/online_submap_system_test.cc`；
修改 livo_system.h/.cc、system_options.h/.cc、livo_config.cc、两份共用 CMake、两个 ROS 的 node_options_test.cc。

**Interfaces:** SystemOptions 增加上述 submap_mode/online_submaps。通知数据：

```cpp
struct OnlineSubmapUpdate {
  std::shared_ptr<const Keyframe> keyframe;
  std::vector<OnlineSubmap> revisions;
  std::shared_ptr<const PoseSnapshot> poses;
  std::vector<SubmapPoseChange> pose_changes;
};
// LivoSystem public additions; all calls remain on its serial owner.
using OnlineSubmapCallback = std::function<void(const OnlineSubmapUpdate&)>;
void SetOnlineSubmapCallback(OnlineSubmapCallback callback);
std::shared_ptr<const PoseSnapshot> GetPoseSnapshot() const;
void ApplyPoseUpdate(const PoseUpdate& update);
```

using 放在 LivoSystem 内。三个方法要求已启用online，否则logic_error；Set在第一帧前调用，空回调也允许随时停用。
Finish/失败后Get仍能读取最近成功状态；Finish后Apply拒绝。非法更新不写文件、不推进版本，之后仍可接收合法更新。
私有 OnlineSubmapManager 的 constructor 为 `(const std::filesystem::path&, KeyframeStore&, int, OnlineSubmapOptions)`，
方法为 `AppendCommittedKeyframe(std::shared_ptr<const Keyframe>)`、`SetCallback(LivoSystem::OnlineSubmapCallback)`、
`GetPoseSnapshot() const`、`ApplyPoseUpdate(const PoseUpdate&)`、`Finish()`；其Get返回类型同公开入口，其余void。
只组合状态，不另造基类/线程。System负责Capture和原Store.Append；Manager负责剩余在线提交、只读快照和通知。

通知在成员/位姿完整提交后同步调用；Keyframe非空表示新观测，Finish只有revision，后端更新只有pose_changes，
每次通知poses覆盖本次成功交付的全部帧。消费者如需跨回调使用，复制通知或保留shared_ptr，不保存回调引用。
回调不得重入System/调用Store/改cv::Mat或PCL别名；需要修改资产先复制。意外抛出按终止错误保存。
无回调时每个新帧只追加原始位姿元数据；Get/有回调才物化完整vector，成本O(M)，不写每帧完整位姿文件。

- [x] Step 1：写 `AllModesPreserveFrontendAndRawAssets`：复用 test::Options/Scan，LO/LIO/LIVO三模式分别比较禁用、旧分组、online的states/clouds/image_hashes及旧原始文件字节。
  禁用时无子地图目录，开启默认模式仅旧目录，online仅新目录；keyframes关闭而submaps开启在任何输出前失败。
  两端参数测试覆盖默认、online覆盖、模式拼写/类型错误、无效阈值和support范围；旧mode忽略在线阈值但保留旧max校验。
- [x] Step 2：写 `OwnedObservationIsPublishedAfterCommit`：回调内只读打开独立Store，确认原帧和所有revision已可见，poses覆盖其ID；保留shared_ptr跨后续Process和System析构，图像/点云仍相同。
  使用借用源内容后来被覆盖的输入证明深复制；LO/LIO通知无图像，不产生虚构camera。
  写 `CallbackExceptionRemainsVisible` 和 `CallbackRegistrationHasDefinedBoundary`：回调异常在后续Process/Finish重抛；开始后注册非空回调拒绝，空回调停用有效；重复Finish不通知。
- [x] Step 3：写 `FailedRevisionPublishesNoPartialEvent`：使用Task3实际文件阻断法，在原帧已提交后令索引失败，无该帧通知，重复Process/Finish重抛原错误，旧资产和已交付快照不变。
  写 `FailedPoseCommitKeepsPublishedVersion`：v2持久化失败时Get仍v1，无v2通知；合法v1提交后再输入过期结果仅拒绝输入，不终止写入。

  ```cpp
  // Storage is deliberately blocked after successfully committing version 1.
  EXPECT_THROW(system.ApplyPoseUpdate(update_v2), std::runtime_error);
  EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
  EXPECT_EQ(last_delivered.poses->version, 1u);
  EXPECT_THROW(system.Finish(), std::runtime_error);
  ```

- [x] Step 4：写 `OptimizationDoesNotFeedBackIntoFrontend`：在两次Process之间用已知非刚性更新Apply，得到Task2分类并使后续新帧按C_tail延续；前端与无更新参考严格一致。
  模拟消费者从v0直接消费v2、跳过成员revision；只修正自己已有成员、新增独立初始化。
  模拟训练结果携带(submap_id,membership_revision,pose_version)，任一不匹配时拒绝提交；representation_revision仅在有效输出提交后增加。
  该消费者留在测试中，P6-01不为三字段比较新增通用生产接口或后台任务。
- [x] Step 5：构建 `online_submap_system_test` 确认 RED，实现上述组合。串行顺序固定：Capture一次→原帧Append→全部成员修订Append→登记raw pose/覆盖前缀→通知。
  Apply先验证/构造候选→计算同当前前缀的变化→写冻结文件→替换当前版本→通知；计算/分配尽可能先于持久化。
  新位姿只通过普通API/通知交付，既有ROS发布和TF保持原语义；map/odom接入留到真正后端阶段。
  持久化或回调异常后锁住本次写运行；Finish只封存一次，空/重复Finish不通知；旧ResultCallback签名保持。
- [x] Step 6：普通新测试与旧system/keyframe/submap全套 GREEN；Noetic运行 `catkin_make run_tests_livo_atlas -j1` 和 `rostest livo_atlas lidar_callbacks.test`，Humble运行实际 `ros2_node_test` 参数用例。
  两环境按既有[构建说明](../../ros-builds.md)配置独立候选目录，记录命令/产物摘要；提交 `feat: integrate committed online submap observations`。

### Task 5: 显式检查、回放开关与独立安装消费

**Files:** 修改 tools/inspect_submaps.cc、tools/run_baseline.py、tools/run_ros2_baseline.py；
新增 `tests/installed_consumer/online_submap_scenario.h` 并修改 main.cc；新增 `tests/tools/online_submap_cli_test.py`。
不引入pytest依赖，CLI测试用标准库unittest/subprocess及C++生成的小数据集。

**Interfaces:** 保留旧 `livo_atlas_inspect_submaps DATASET_DIRECTORY [POSE_VERSION]` 行为；新增：
`livo_atlas_inspect_submaps DATASET_DIRECTORY --online [POSE_VERSION]`。
online默认v0；正版本按用户显式online请求物化到当前全部提交帧，并在结果中披露冻结/延续范围。
online JSON包含 inspection=metadata、mode、submaps、active_submaps、primary_frames、support_references、unindexed_frames、pose_version、
stored_frame_count、frame_count、optimized_frame_count；support只计引用，不冒充新增帧或独立点云。
v0的stored_frame_count=0表示没有冻结文件，frame_count为当前原始帧数、optimized_frame_count=0。
明确打印检查为metadata级，普通Read的图像/点云完整校验由安装消费/验收执行。
参数错误退出2、数据错误退出1、成功0；未知版本不猜测；所有检查只读。

两个baseline增加 `--submap-mode {keyframe_groups,online}` 默认keyframe_groups；online自动开启save_submaps/save_keyframes。
沿用 `--save-submaps` 的旧行为，summary加入submap_mode；ROS1/ROS2分别配置同一共用字段。
不顺带改已延期的ROS2模块启动导入；仍以脚本路径运行。

- [ ] Step 1：写CLI RED：旧格式输出保持；online显式读取新索引，active尾组/未索引尾部和冻结M3→物化M4字段准确；未知版本、重复选项、非规范数字、仅含旧索引却请求online均非零；文件摘要不变。
  两种命名空间都存在的离线副本按显式选项读取，不能自动混合两者或因为旧索引存在就拒绝合法online索引。
  Python测试入口为 `python3 tests/tools/online_submap_cli_test.py --executable PATH --fixtures DIR`；自定义参数解析后仅把unittest参数传入runner。
  fixture目录由安装消费者运行时写入，不依赖旧验证数据或真实bag。

  ```python
  # result is json.loads(stdout) from the v1/M3/N2 fixture with four raw frames.
  self.assertEqual(result["inspection"], "metadata")
  self.assertEqual(result["stored_frame_count"], 3)
  self.assertEqual(result["frame_count"], 4)
  self.assertEqual(result["optimized_frame_count"], 2)
  ```

- [ ] Step 2：实现CLI分派及两个脚本参数，运行 `--help` 和参数检查；online明确、旧默认保持。
  测试两个脚本参数映射；不新增只有脚本表面mock通过而未实际回放的“集成成功”声明。
- [ ] Step 3：扩展installed_consumer：公开API执行online LO/LIO/LIVO、两次模拟位姿更新、保留观测、Finish后重开Store，检查Task2几何断言。
  增加可选 `--online-fixtures DIR` 输出确定的旧/在线小数据集；正常无参模式继续原全部场景并清理自建临时资产。
  在fixture副本上把选定正online版本物化到完整当前前缀后显式调用旧WritePoses，ExportColmap可使用该副本；另用旧Builder/Store生成兼容分组后验证旧Submap工具。
  记录optimized前缀与外推尾帧，旧版本号冲突明确拒绝，不覆盖；不在在线运行中自动双写两套位姿权威。
- [ ] Step 4：安装后以独立CMake工程运行消费者；移走安装前缀后重配/重编译消费，拒绝借用源码include或构建树目标。
  运行CLI实际测试全部 GREEN；提交 `feat: expose online submap inspection and portable consumption`。

### Task 6: 跨环境回归、交付文档与阶段结论

**Files:** 新 `docs/online-submap-dataset.md`、`docs/p6-01-validation.md`；
修改 docs/README.md、docs/core-build.md、docs/submap-dataset.md；新辅助证据仅在本阶段包外验证目录。
本任务不修改前端数值容差，不重新处理旧报告为新成绩。

**Interfaces:** 消费全部公开API和已安装CLI；产出可复现日志、源码/库摘要、实际测试清单、bag配对结果及限制。

- [ ] Step 1：普通shared全套运行；static与无OpenMP分别在独立目录配置、编译、测试和安装消费，不并行构建。
  shared示例：

  ```bash
  nice -n 10 cmake --build "$LIVO_P6_VALIDATION/core-shared" -- -j1
  (cd "$LIVO_P6_VALIDATION/core-shared" && ctest --output-on-failure)
  cmake --install "$LIVO_P6_VALIDATION/core-shared"
  cmake -S "$LIVO_P6_PACKAGE/tests/installed_consumer" -B "$LIVO_P6_VALIDATION/consumer-shared" \
    -DCMAKE_PREFIX_PATH="$LIVO_P6_VALIDATION/install-shared"
  nice -n 10 cmake --build "$LIVO_P6_VALIDATION/consumer-shared" -- -j1
  "$LIVO_P6_VALIDATION/consumer-shared/installed_consumer"
  ```

  static增加 `-DBUILD_SHARED_LIBS=OFF`，无OMP增加 `-DLIVO_ATLAS_USE_OPENMP=OFF`，每组独立安装前缀。
  预期全套零失败，包括新增头的单头编译；以实际GTest用例数报告，不预填计数。
- [ ] Step 2：Noetic/Humble最终源码分别构建、运行全套普通及实际ROS/DDS测试、启动参数保护、SDK/ament移动安装消费。
  本机沿用ros2容器，候选目录使用 `/tmp/livo_atlas_p6_01`；迁移机器先记录真实路径/环境，不能把旧脚本硬编码路径当成功证明。
  两端加载的自有库、节点与源码摘要一致后再回放；每端只在一个固定CPU上运行，默认本机CPU0，异机检查允许集。
- [ ] Step 3：每端对同环境冻结P5-04基线和新实现运行默认、旧分组、online的配对输入；先30s冒烟，再完整136s请求覆盖bag。
  Noetic online示例（workspace必须指向实际候选构建，不指向冻结参考）：

  ```bash
  python3 "$LIVO_P6_PACKAGE/tools/run_baseline.py" \
    --workspace "$LIVO_P6_VALIDATION/noetic-ws" \
    --bag "$LIVO_P6_WS/Retail_Street.bag" --output "$LIVO_P6_VALIDATION/noetic-online-full" \
    --duration 136 --rate 1 --cpu-affinity 0 --image-queue-size 32 \
    --semantic-output --submap-mode online
  ```

  Humble使用同一ROS1 bag经既有convert_ros1_bag.py转换并核验过的ROS2 sqlite3数据目录；run_ros2_baseline.py不直接读取ROS1文件。
  优先只读复用已核验的转换数据及其manifest，异机转换方法见[双版本说明](../../ros-builds.md)。明确 `--executable` 指向候选节点、
  两个YAML指向候选配置，同样duration/rate/affinity/image-queue/submap-mode；保留summary中的实际加载条件。
  检查输入/发布计数、四数组和原始关键帧资产精确一致；短段额外启用 `--save-products` 验证旧产品逐字节一致。
  默认关闭关键帧的运行只比较前端/发布；原始关键帧资产比较使用同样开启keyframes的配对运行。新增索引单独核对，禁止把它计入旧资产比较。
  参考正常完整计数为1351 LIO、1350 VIO、172关键帧；若启动变体出现，记录并用冻结参考同条件复现，不放宽精度阈值。
- [ ] Step 4：用真实回放数据只读打开online索引，检查唯一主归属、support、封存与所有原始资产；Humble产物由Noetic普通SDK交叉读取。
  在仅复制原始关键帧的新副本中用Builder/新建Store重建在线索引，再应用已知刚性及组内位姿修正，检查冻结文件、尾帧延续、原始资产摘要和相机变换。
  已有在线Store不能重开写入，因此不直接向原运行目录添加版本；明确这是合成优化输入。
  记录online相对默认的耗时/RSS/磁盘增长，解释metadata、逐帧revision与可选完整快照物化成本；不预设未测的实时预算。
- [ ] Step 5：写在线使用、全部字段/单位/错误语义、只读重开、部分提交、回调所有权和离线导出示例；实际报告链接日志和摘要。
  标明当前仍没有几何后端、闭环检测和高斯训练；后续P6-02几何/后端、P6-03在线GS、P6-04辅助估计各自先细化计划。
- [ ] Step 6：Google格式/`git diff --check`、当前阶段整体代码审阅、修复重要问题后重跑受影响检查；只提交本任务文档与必要验证改动，提交 `docs: validate online submap contracts across ROS versions`。
  报告实际结果、剩余限制和代码提交；不自动合并/推送或打tag。

## 计划自检与交接

- 范围覆盖：分组/生命周期→Task1；位姿/相机/修正→Task2；持久化/部分提交→Task3；所有权/顺序/参数→Task4；工具/移植→Task5；回归→Task6。
- 类型一致：仅保留一份KeyframePose；mode不改变旧SubmapOptions布局；原始位姿数组以ID为下标；正版本/运行覆盖分别记录。
- 失败覆盖：五项Review Focus各有上述具名用例；不以mock代替真实文件失败或双ROS回放。
- 规模控制：没有GTSAM/CUDA/渲染实现、通用插件/调度框架和多套存储权威；未来队列、Gaussian资产和几何表示在后续阶段实现。
- 2026-11-08 用户切换模型后开始实施；当前完成Task1、2、3、4，后续从Task5继续。完整P6-01验收尚未完成。
