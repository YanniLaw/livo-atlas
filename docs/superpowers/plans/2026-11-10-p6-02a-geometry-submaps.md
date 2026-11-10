# P6-02a Geometry Submaps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户已选当前会话直接实施；本轮仅交付文档，正式编码前通知其切换模型。

**Goal:** 将已提交关键帧增量融合为有来源、可随位姿版本重建的局部体素几何，提供普通 C++ 查询和显式导出。

**Architecture:** 单块 `GeometrySubmapBuilder` 计算统计，多块 `GeometryMapper` 管理版本、缓存与重建。
原关键帧、在线索引和位姿仍是可恢复来源；`OnlineSubmapManager` 在既有串行提交链内组合几何。
几何快照由局部数据和 parent 变换组成，后续 backend / Gaussian mapper 分别消费。

**Tech Stack:** C++17、CMake 3.16、Eigen、现有 PCL common/io 与 OpenCV metadata 工具、GTest、Python 标准库；Noetic / Humble。

**Spec:** [2026-11-10-p6-02a-geometry-submaps-design.md](../specs/2026-11-10-p6-02a-geometry-submaps-design.md)。先读该设计再执行；计划不另建一套坐标或版本规则。

**Status:** Task 1–5 的实现已提交并完成独立审阅修复，其复选框保留原始执行清单；Task 6 已完成。
实际验收范围以[验证记录](../../p6-02a-validation.md)为准。原设计检视基线为
`5e0bf071de3591c62f84364321df44ff1eb5d802`，当前分支 `framework_completion`。

## Global Constraints

- 仅修改 livo-atlas；C++17、Google 风格、普通类直接声明私有成员；禁止 Pimpl 与提前建设插件框架。
- 相邻函数定义之间空一行，包括类内定义；简短注释解释坐标、所有权、修正和失败顺序。
- 保留 VIOManager、Frame、Feature、Pose6D、SubSparseMap、VOXEL_LOCATION。
- ROS1/ROS2 共用普通核心与同一 main；不引入 GTSAM、CUDA、LibTorch、vikit 或新第三方依赖。
- 几何默认关闭；开启要求 keyframes、submaps、online；几何主成员上限 64，支持成员不融合。
- 默认 h=0.2 m、min_points=6、max_plane_rms=0.02 m、max_plane_ratio=0.1；lambda1 下限为 0.01*h*h。
- 默认每块最多 100000 格、每帧最多 1000000 点、驻留块最多 3（最小 2）；超限明确失败，不自动降质。
- 局部变换数值等价阈值 1e-9 m / 1e-9 rad；沿用前端同环境精确回归要求，不放宽旧容差。
- 正式编码前通知用户切换模型；不自动 merge/push/tag；保护用户改动、原始 bag 和旧验证目录。
- 串行 `nice -n 10` 构建 `-j1`、`OMP_THREAD_LIMIT=1`；重构验证不并行运行重负载，不开 RViz。

## Review Focus

1. 非交换 SE(3) 修正、锚点改变或连续微小修正：不能双重施加变换，也不能以逐次比较掩盖累计变化（Task 3）。
2. 同帧重复通知、插入后立即封存、支持帧跨组引用：贡献次数、表示修订和主归属必须准确（Task 1/3/4）。
3. 未加载/已淘汰的旧块经历多个版本：Get 必须按当前目标重建，旧快照仍标旧版本，不能错误复用（Task 3）。
4. 超大 PCD、边界坐标、来源列表和体素上限：在危险分配/转换前检查，不用删点掩盖失败（Task 1/2/3）。
5. 原资产已提交后几何失败、导出失败或读到未索引尾部：保留正确可读前缀，不发布半成品或改写源数据（Task 4/5）。

## 文件与职责

| 文件（相对包根） | 职责 |
| --- | --- |
| 新 `include/livo_atlas/mapping/geometry_submap.h`、`src/mapping/geometry_submap.cc` | 值类型、选项校验、单块 Builder、数值统计与平面有效性 |
| 新 `include/livo_atlas/mapping/geometry_mapper.h`、`src/mapping/geometry_mapper.cc` | 版本目标、主归属、活动统计、封存缓存、按需重建 |
| 修改 `include/livo_atlas/io/keyframe_store.h`、`src/io/keyframe_store.cc` | 受点数限制的 body 云读取，复用已有 PCD 检查 |
| 修改 `src/core/online_submap_manager.h/.cc`、`include/livo_atlas/core/livo_system.h`、`src/core/livo_system.cc` | 几何组合与查询；已有失败保护与回调顺序 |
| 修改 `include/livo_atlas/core/system_options.h`、`src/core/system_options.cc`、`src/core/livo_config.cc` | 一份默认值、校验和参数映射 |
| 新 `include/livo_atlas/io/geometry_exporter.h`、`src/io/geometry_exporter.cc`、`tools/export_geometry.cc` | 可复现分块导出与安装 CLI |
| 修改 `cmake/core.cmake`、`cmake/tests.cmake`、`cmake/ros1.cmake` | geometry 目标、测试和安装/静态导出顺序；检查 ros2 导出能自动覆盖新目标 |
| 新下列具名测试，修改双 ROS node_options_test、installed_consumer | 行为与外部消费；不为简单字段读写增加测试 |
| 修改两份 baseline 脚本，新阶段验证工具和文档 | 显式开关、实际节点计时、配对回放和资源证据 |

## 执行环境与基线

实施时先检查 `git status --short`、当前分支、HEAD；保留新出现的用户改动。
代码基线变动则记录真实 SHA 并核对本计划依赖，不重置到本文 SHA。先冻结候选前的参考源码和环境清单。
验证目录在包外单独建立，已有同名目录时使用新后缀，禁止覆盖：

```bash
export LIVO_GEOM_WS=/home/yanni/livo-atlas_ws
export LIVO_GEOM_PACKAGE="$LIVO_GEOM_WS/src/livo-atlas"
export LIVO_GEOM_VALIDATION="$LIVO_GEOM_WS/validation/P6-02a-20261110"
export OMP_THREAD_LIMIT=1
cmake -S "$LIVO_GEOM_PACKAGE" -B "$LIVO_GEOM_VALIDATION/core-shared" \
  -DLIVO_ATLAS_ROS_VERSION=0 -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$LIVO_GEOM_VALIDATION/install-shared"
```

异机只替换 workspace、验证目录、Sophus 实际前缀与允许 CPU 集；构建步骤与配置保持可追溯。
完整命令/版本/摘要写入本阶段日志，不引用旧报告的“通过”作为新实现成绩。
主机旧 CTest 不能依赖 `--test-dir`；使用 `(cd BUILD && ctest ...)`。

### Task 1: 可复现的单块几何统计

**Files:** 新 geometry_submap.h/.cc、`tests/mapping/geometry_submap_test.cc`；修改 cmake/core.cmake、cmake/tests.cmake、cmake/ros1.cmake。

**Interfaces:** 按设计的字段定义 GeometryOptions、GeometryVoxelKey、GeometryVoxel、GeometrySource、GeometryData、GeometrySubmap、GeometryCacheStats。
新增 `void ValidateGeometryOptions(const GeometryOptions&)` 和
`GeometrySubmapBuilder(GeometryOptions)`、`void AddKeyframe(uint64_t, const PointCloud&, const RigidTransform&)`、
`shared_ptr<const GeometryData> Snapshot() const`。Builder 不分配 representation_revision。
新增安装目标 `livo_atlas::geometry`；io/core/头文件检查与 catkin 导出传递依赖它。

- [ ] Step 1：写 RED `AccumulatesKnownPlaneAndSources`：h=0.5；两帧各四点
  `(x,y,z)` 中 x/y 分别取 0.125、0.375，z=0.25，identity 变换、强度统一为 8。
  断言唯一格 `(0,0,0)`、count=8、mean=(0.25,0.25,0.25)、C=diag(0.015625,0.015625,0)、
  intensity=8、normal=(0,0,1)、来源恰好 `{0,1}`。数值误差≤1e-12。
- [ ] Step 2：写 RED `UsesFloorAndChecksIndexRange`：h=0.5，输入 -0.5、0、0.5 及各自
  `nextafterf` 两侧，断言实际 float 转 double 后的 floor；覆盖 ±0、极小 h 导致指数越界、非有限输入。
  写 `RejectsPartialInsertion`：合法状态后遇到一帧中部 NaN、空云、重复/逆序 ID、超格/点数上限，
  原快照、原统计和来源不变；新帧合法重试可成功。
  写 `LimitsSourceFrames`：64个递增主帧成功，第65个拒绝，原统计不变；无论options.enabled为何值均执行该上限。
- [ ] Step 3：写 RED `PreservesDegenerateStatisticsWithoutNormal`：重合点、直线、少于6点、
  两个分离平面的组合分别不提供法向，仍保留统计；提供满足/越过 rms 和 ratio 阈值的确定样本。
  写 `SnapshotsAreOwnedAndDeterministic`：后续插入、Builder 析构不改变旧快照；键和来源排序稳定，
  合并后统计与按相同 ID/点顺序一次性输入结果一致。
- [ ] Step 4：配置并构建 `geometry_submap_test`，记录因新 API 缺失而失败的 RED；实现
  double Welford、无符号哈希、受限候选和 Eigen 自伴求解。注释区分点分布 C 与估计不确定度。
  不转用前端 normal/curvature，不增加射线、体素金字塔或反融合数据结构。
- [ ] Step 5：运行以下目标及旧 submap/pose 测试，全部通过后提交 `feat: add voxel geometry submap statistics`。

  ```bash
  nice -n 10 cmake --build "$LIVO_GEOM_VALIDATION/core-shared" --target geometry_submap_test -- -j1
  "$LIVO_GEOM_VALIDATION/core-shared/geometry_submap_test"
  ```

### Task 2: 不解码图像的受限 body 云读取

**Files:** keyframe_store.h/.cc、`tests/io/keyframe_store_test.cc`。

**Interfaces:** 新增
`PointCloud::ConstPtr KeyframeStore::ReadBodyCloud(std::uint64_t id, std::uint64_t max_points) const`。
显式参数 max_points，0 非法；返回原存储 body 云，不施加姿态，不改变原 `Read` 的图像完整性语义。

- [ ] Step 1：写 RED `ReadBodyCloudDoesNotDecodeImages`：测试副本删除 PNG 后新方法成功并与删除前
  Read.cloud 的 XYZI/顺序相同；原 Read 仍失败。未知 ID、非法标定/云元数据、PCD 缺失仍失败。
- [ ] Step 2：写 RED `ReadBodyCloudChecksLimitBeforePayloadAllocation`：伪造巨大 cloud_points /
  PCD POINTS、头与元数据数量不一致、截断数据和非有限 XYZI；均在合适边界拒绝。
  阈值等于实际点数成功，小1失败；不靠实际分配巨大数组验证。
- [ ] Step 3：构建运行 keyframe_store_test 记录 RED；抽出/复用现有元数据和 PCD 检查函数，
  原完整 Read 继续完成完整图像校验；共享代码不能使原 malformed-file 用例变宽松。
- [ ] Step 4：keyframe_store_test、keyframe_test、submap_store_test 全部 GREEN，提交
  `feat: read bounded keyframe body clouds without images`。

### Task 3: 版本化几何、选择性重建与有界缓存

**Files:** 新 geometry_mapper.h/.cc、`tests/mapping/geometry_mapper_test.cc`；修改共用 CMake。

**Interfaces:** 设计中的 `GeometryMapper(GeometryOptions, CloudReader)`、
`Update(const vector<OnlineSubmap>&, shared_ptr<const PoseSnapshot>, const Keyframe*)`、
`GeometrySubmap GetSubmap(uint64_t)`、`GeometryCacheStats GetStats() const`。
公开层不包含 OnlineSubmapUpdate/core 头。测试 reader 返回小型内存云并计数；真实读取使用 Task 2。

- [ ] Step 1：写 RED `IncrementalEqualsFreshBuild`：逐条交付修订、一次性交付最新封存修订，
  两种 mapper Get 的格/来源完全相同，数值一致；相同修订重送不加点、不加表示修订。
  写 `SupportFramesAreReferencesOnly`：两组的 primary 各唯一，support 来自前组；
  两组 point_count 总和等于主观测总点数，不因 support 增加；支持成员元数据仍可读。
- [ ] Step 2：写 RED `RigidCorrectionReusesLocalData`：整体旋转90°绕 z 后平移(3,-2,1)，
  再施加一个绕 x 的非交换修正；parent 变换由选定锚点派生，`data.get()`、表示修订和 reader 调用次数不变，
  全局点仅被变换一次。检查 v0 odom、v1/v2 map 标记。
- [ ] Step 3：写 RED `RelativeCorrectionRebuildsOnlyOwner`：改变一个主成员的局部平移及旋转，
  只有归属块 Get 触发 full_builds/表示修订增长；它作为其他块 support 时不使其他块重建。
  锚点单独改变、封存块改变、先淘汰再连续接受两次修正也得到同一当前几何。
  写 `SmallCorrectionsCompareAgainstBuiltSources`：多次各 0.75e-9 m 的单成员改动累计超过1e-9，
  触发重建，不能只比较相邻通知。
- [ ] Step 4：写 RED `GrowingFrontierAndSkippedVersionsStayConsistent`：v1/M3→v1/M4 不重建旧块，
  新成员正确插入；从v1直接到v3按最终快照构建；回退version/覆盖、相同version改旧位姿、缺成员姿态拒绝。
  成员跳修订允许前缀扩展，重写锚点、seal后追加、相同revision不同内容和主归属有洞拒绝。
  支持列表一旦建立不可改，状态不可倒退；revision必须等于primary_count加seal标记。
  Mapper只检查support为前块主成员的有序后缀，不校验配置指定数量；原Store继续执行完整配置验证。
  写 `IndependentMapperEnforcesMemberLimit`：一次性传入65主成员拒绝，64成功；旧数据源max_keyframes>64
  但实际成员数≤64可消费。Builder和Mapper同用一个64帧常量。
- [ ] Step 5：写 RED `CacheEvictsSealedDataAndReloadsCurrentVersion`：max_resident=2，
  活动组保持、历史组按 Get 次序淘汰；resident计数不超限，重建按主ID顺序逐云读取，
  no image retention；调用者保留旧快照时内容不变，并明确它不计入内部驻留上限。
  写 `FailedRebuildNeverRetagsOldData`：reader 在第二帧抛错/超过容量；Get失败，无错误的新标签结果，
  表示修订只在成功提交后增长；修复 reader 后独立 mapper 可重试。
- [ ] Step 6：运行 `geometry_mapper_test` RED；实现单活动 Builder、封存快照 LRU 和元数据目录，
  局部参考取实际构建 sources；同一次 Update 的插入/seal 按序校验，避免 seal 丢最后一帧。
  只保留一个临时重建候选，纯位姿登记不遍历历史 PCD。所有正常输入校验先于状态变更。
- [ ] Step 7：mapper/builder/旧 pose_snapshot/submap_pose_change 测试 GREEN；读真实短段数据
  进行一次 SDK 冒烟，不运行 bag；提交 `feat: maintain versioned geometry submap cache`。

### Task 4: 接入共用系统与双 ROS 参数

**Files:** 上表 core 文件、`tests/core/geometry_system_test.cc`、
`tests/ros1/node_options_test.cc`、`tests/ros2/node_options_test.cc`、共用 CMake。

**Interfaces:** SystemOptions::geometry；新增
`GeometrySubmap LivoSystem::GetGeometrySubmap(std::uint64_t id)`、
`GeometryCacheStats LivoSystem::GetGeometryStats() const`，由私有 OnlineSubmapManager 转发。
Manager 持有可选 `unique_ptr<GeometryMapper>`，这是实际可选组件，不是隐藏实现壳。
其 CloudReader 捕获生命周期更长的 KeyframeStore 引用。
Get disabled 抛 logic_error，未知 ID 抛 out_of_range；正常 Finish 后 Get 可用。

- [ ] Step 1：写 RED `GeometryOffPreservesExistingModes`、`GeometryRequiresOnlineKeyframes`：
  关闭时默认/旧分组/online 的状态与原始资产精确保持；开启模式不合法在创建输出前失败。
  两端真实参数入口覆盖默认值、开启、全部覆盖值、错误类型、NaN、负数、容量和64成员边界。
- [ ] Step 2：写 RED `GeometryFollowsCommitAndPrecedesNotification`：回调只保留已拥有的通知，
  不重入系统；Process返回后 Get，元数据/姿态与通知一致。覆盖 LO/LIO/LIVO、时间阈值切组、
  数量触发插入后seal、空和重复Finish。删除原输入缓存后快照仍可用。
- [ ] Step 3：写 RED `GeometryFailureStopsFurtherMutations`：原帧和索引成功后人为设定体素容量过小，
  无该次成功回调，Process/ApplyPoseUpdate/非空回调注册/Finish 重抛原错；GetGeometrySubmap也拒绝，
  GetPoseSnapshot保持现有语义，旧资产仍可重开。独立非法Get ID不应毒化运行。
  写 `PoseUpdateDoesNotFeedBackIntoFrontend`：已知组内修正后继续输入，与无修正参考的前端数组精确一致。
- [ ] Step 4：运行 RED 后实现共用配置/接线。Notify 在有 geometry 或外部 callback 时仅物化一次姿态快照，
  先更新几何再通知；都没有时保持 P6-01 按需快照快路径。旧外部回调签名不变。
  Get 的真实数据/容量失败按设计终止保护，非法参数查询仅拒绝该调用。
- [ ] Step 5：增加几何专属计时：chrono 测 Update、reader、完整重建和 Snapshot 的耗时；
  GeometryCacheStats 增加对应最近操作秒数（`last_update_seconds`、`last_cloud_read_seconds`、
  `last_full_build_seconds`、`last_snapshot_seconds`，未发生为0）。成功 Update 后输出一行带
  `geometry_update_seconds=` 及这些字段的诊断，关闭时无该行。不混入旧 LIO/VIO 计时序列。
  核心另测 `geometry_total_seconds=`，包含几何所需完整PoseSnapshot物化与mapper.Update，
  不含外部回调；性能门槛使用这个总量。last_*按设计在每次Update/Get清零，内部多次阶段累计。
  测试计时字段有限非负，不以墙钟阈值做 GTest；日志对应点与统计定义在使用文档中说明。
- [ ] Step 6：普通 system 全套和两端参数/实际节点测试 GREEN，构建与启动记录真实输出，提交
  `feat: integrate optional geometry mapping in shared core`。

### Task 5: 显式几何导出、回放入口和外部消费

**Files:** geometry_exporter.h/.cc、tools/export_geometry.cc、`tests/io/geometry_exporter_test.cc`、
`tests/tools/geometry_cli_test.py`；修改 tools/run_baseline.py、tools/run_ros2_baseline.py、
tests/installed_consumer/main.cc；新 `tests/installed_consumer/geometry_scenario.h`；共用 CMake。

**Interfaces:** 新
`void ExportGeometry(const std::filesystem::path& dataset, const std::filesystem::path& output, std::uint64_t pose_version, const GeometryOptions& options)`。
CLI 为设计所列格式，所有选项重复/未知/不合法退出2、输入/写入错误退出1、成功0。
baseline 新 `--geometry-submaps`，自动配置 keyframes/submaps/online 和 geometry.enabled；
与显式 `--submap-mode keyframe_groups` 冲突时拒绝，省略模式的默认值不算显式冲突。
现有参数默认行为不变，summary 增加 geometry_enabled、geometry 独立计时统计。

- [ ] Step 1：写 RED `ExportUsesSelectedPoseAndKeepsSourcesUnchanged`：两个已封存块和一个active尾块，
  比较 v0 与已知 v1 的 metadata、局部云、来源和 parent；核对源目录所有字节未变。
  有未索引原帧时按已索引前缀导出，报告尾部数，不补写索引；未知正版本失败。
  覆盖来源块超过64主成员时拒绝，以及源分组配置上限超过64但实际各块≤64时成功。
- [ ] Step 2：写 RED `ExportPublicationIsAllOrNothing`：输出已存在、在源内部或经符号链接指向源内部拒绝；
  用可写目录中被普通文件阻挡的路径构造真实写失败，确认无最终目录/成功manifest、源完整；
  不用root权限下不起效的chmod测试冒充失败验证。
  固定CSV列、排序、double精度、空数据集、无有效平面的标志和来源表分别检查。
- [ ] Step 3：运行 RED 后实现逐块导出、checked-close、同父目录pending+rename；
  不增加在线自动快照写盘、GeometryStore、全局去重或ROS几何话题。
- [ ] Step 4：安装消费者生成 `--geometry-fixtures DIR` 的确定数据集，通过公开 API 执行
  逐帧Update、整体/局部修正、Get旧快照、缓存淘汰/重建、Finish；fixture可供CLI测试使用。
  Python标准库测试入口：

  ```bash
  python3 "$LIVO_GEOM_PACKAGE/tests/tools/geometry_cli_test.py" \
    --executable "$LIVO_GEOM_VALIDATION/install-shared/bin/livo_atlas_export_geometry" \
    --fixtures "$LIVO_GEOM_VALIDATION/geometry-fixtures"
  ```

  CSV检查全部中心/对称协方差元素/来源表，不能只检查文件存在。YAML可由现有 C++ OpenCV 测试校验，
  不为Python工具新加PyYAML。重复导出到两个新目录的数值内容一致，源路径等记录项按契约比较。
- [ ] Step 5：给baseline添加上述显式开关和诊断解析，覆盖默认、自动online、显式冲突和错误输入；
  不以mock参数测试代替Task6真实ROS回放。安装并运行移动前缀后的消费者、旧CLI/COLMAP回归，
  提交 `feat: export and inspect reproducible submap geometry`。

### Task 6: 跨环境验收与交付

**Files:** 新 `docs/geometry-submaps.md`、`docs/p6-02a-validation.md`，修改 docs/README.md、
docs/core-build.md、docs/online-submap-dataset.md；新证据仅在本阶段 validation 目录。

**Interfaces:** 消费全部公开 API、已安装CLI和真实节点，产出实际构建/测试清单、源/库摘要、
同环境配对结果、数学/缓存/资源证据及未覆盖范围。

- [x] Step 1：shared 全套构建、ctest、公开头编译、安装消费。static 与无 OpenMP 在独立目录
  分别配置/运行；static用 `-DBUILD_SHARED_LIBS=OFF`、无OMP用 `-DLIVO_ATLAS_USE_OPENMP=OFF`。
  三种模式均验证 geometry/core 的公开头与安装后外部链接，不依赖源码include或构建树目标。

  ```bash
  nice -n 10 cmake --build "$LIVO_GEOM_VALIDATION/core-shared" -- -j1
  (cd "$LIVO_GEOM_VALIDATION/core-shared" && ctest --output-on-failure)
  cmake --install "$LIVO_GEOM_VALIDATION/core-shared"
  cmake -S "$LIVO_GEOM_PACKAGE/tests/installed_consumer" -B "$LIVO_GEOM_VALIDATION/consumer-shared" \
    -DCMAKE_PREFIX_PATH="$LIVO_GEOM_VALIDATION/install-shared"
  nice -n 10 cmake --build "$LIVO_GEOM_VALIDATION/consumer-shared" -- -j1
  "$LIVO_GEOM_VALIDATION/consumer-shared/installed_consumer"
  ```

- [x] Step 2：Noetic/Humble 独立候选副本编译，执行普通测试、实际ROS/DDS测试、启动参数保护和
  SDK/catkin/ament移动安装消费。沿用[ROS构建说明](../../ros-builds.md)的方法，
  不直接运行会覆盖旧目录的历史验证脚本。迁移机器缺依赖时记录再安装，不能把跳过写成通过。
- [x] Step 3：冻结实施前的同环境基线，分别串行回放：默认关闭、旧分组、online几何关闭、online几何开启。
  先新增开启模式30 s冒烟，再每种模式完整136 s请求覆盖bag；两端新增开启模式再做3 s旧产品检查。
  比较参考与候选四数组/发布语义及原始资产；几何开启还与候选online关闭作配对，分离模块成本。

  ```bash
  python3 "$LIVO_GEOM_PACKAGE/tools/run_baseline.py" \
    --workspace "$LIVO_GEOM_VALIDATION/noetic-ws" --bag "$LIVO_GEOM_WS/Retail_Street.bag" \
    --output "$LIVO_GEOM_VALIDATION/noetic-geometry-full" --duration 136 --rate 1 \
    --cpu-affinity 0 --image-queue-size 32 --semantic-output --geometry-submaps
  ```

  CPU0仅本机例子，执行前确认允许集；两端同条件固定CPU。Humble使用已经核验的同源ROS2转换bag，
  实际节点、YAML和安装库必须属于候选副本。记录源码/实际加载库摘要，不能混用Pimpl前后二进制。
  正常完整参考为1351 LIO、1350 VIO、172关键帧；如启动变体出现，用冻结参考同条件复现并单列。
  旧四数组、原始517文件、成员/位姿索引及短段57产品按同环境精确比较，几何导出独立列项。
- [x] Step 4：在真实关键帧副本上构造已知整体和单成员修正，验证几何Get/导出及源摘要；
  旧OnlineSubmapStore不能重开追加，需在新副本按既有Builder/Store重新创建索引与版本，不能改原记录。
  整体修正验证数据内容不变、局部修正验证所属块变化；无真实回环算法或真值时不报告回环/绝对精度。
- [x] Step 5：报告0.2/0.3/0.5 m的格数、有效平面比例、来源覆盖、CPU/RSS/导出大小与耗时。
  对默认0.2 m检查 geometry_total_seconds的p95≤50 ms/最大≤200 ms、重建p95≤500 ms、峰值RSS增量≤128 MiB、
  1倍速无新增丢帧。计时区分纯刚性登记、追加、重建、读云和Snapshot；不得拿旧LIO/VIO内部计时代替新增耗时。
  使用固定大小小云生成至少1000个封存块并循环查询/修正，验证resident上限、无持有全部云，
  报告元数据增长；它是缓存压力测试，不是真实长距离耐久证明。
- [x] Step 6：写实际使用与验证文档，注明版本号仅mapper生命周期内有效、主成员融合、空间重叠、
  正版本尾帧外推、无图像完整性检查的ReadBodyCloud、单格单平面局限和同步Get阻塞边界。
  运行Google格式、函数间空行人工检查、git diff --check，完成阶段独立代码审阅；
  修复重要问题后重跑受影响验证，提交 `docs: validate geometry submaps across ROS versions`。

## 计划自检与编码交接

- 几何/退化/坐标→Task1；安全来源读取→Task2；版本/来源/缓存→Task3；提交与双ROS→Task4；导出/外部消费→Task5；实际回归/预算→Task6。
- 每个新API只有一处负责定义；pose权威沿用现有PoseSnapshot，几何sources记录实际局部构建输入。
- 表示修订仅成功提交推进；seal和纯整体修正不推进；换进程不假装接续旧编号。
- 默认开启前没有自动几何工作；即使开启也不反馈改变前端滤波、匹配地图、发布/TF。
- 本轮文档完成后向用户交付，正式编码前按其要求通知切换模型；不因写好计划自动开始Task1。
