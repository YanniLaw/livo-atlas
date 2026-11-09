# Remove Pimpl Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 移除当前全部 8 处 Pimpl，让接口直接对应方法实现，并保留原行为。

**Architecture:** 将 Impl 成员和辅助方法归回所属类的 private 区域，方法在原 `.cc` 直接定义。
保留真实组件的所有权及现有模块边界，不用另一种实现包装替代 Pimpl。

**Tech Stack:** Google C++17，既有 CMake/Eigen/PCL/OpenCV/Sophus/GTest，ROS1 Noetic/ROS2 Humble。

**Spec:** [移除 Pimpl 设计](../specs/2026-11-10-remove-pimpl-design.md)。

**状态：范围已由用户确定；尚未编码，等待用户切换模型。** 沿用当前会话直接实施，不再要求选择执行方式。

## Global Constraints

- 仅 livo-atlas；当前分支 refactor2，读取时 HEAD 为 `37412f4`，已有两份路线文档修改必须保留。
- Google C++17，无新增依赖、物理库、算法接口或数据格式。
- 全部 8 处移除；不保留 Frontend/Exporter 的 Impl，不改名包装。
- 原公开函数签名、删除复制及移动约束、算法公式、成员构造/销毁和错误语义保持。
- 保留 VIOManager、Frame、Feature、Pose6D、SubSparseMap、VOXEL_LOCATION。
- 普通核心不引入 ROS；保留同一 main、两个 ROS 版本和既有职责目录。
- 布局变化要求整套库及消费者重编译，禁止旧新 ABI 混用；参考运行库整体冻结。
- 不切分支、不合并、不推送、不改 tag；不修改其他包、bag 或旧验证证据。
- 新证据放工作区 `validation/Pimpl-20261110/`，构建与回放串行 `-j1`、nice 10、`OMP_THREAD_LIMIT=1`，无 RViz。

## Review Focus

1. 视觉对象借用地图/相机：析构顺序保持，重复构造处理销毁不产生悬空访问。Task 1。
2. 文件打开/写入/关闭失败：仍报告具体路径，Finish 幂等，失败后拒绝后续写入。Task 2。
3. 在线原始帧写失败：ApplyPoseUpdate/新回调注册仍拒绝推进，已提交快照仍可读。Task 3。
4. ROS 回调捕获与定时器：捕获实际 this，节点退出前停止相关资源，发布结果不变。Task 4。
5. 安装头出现完整值类型：普通头独立编译，迁移后的 SDK 消费者无 src 私有头或偶然间接依赖。Task 5。

## 文件与任务

下面路径相对于包根；只增加本文档和验证说明，不增加新的产品代码文件。
执行前在新验证目录记录 HEAD、已有差异和工具链，按 [P6-01 报告](../../p6-01-validation.md)
复用验证方式并改用新输出目录，不运行会覆盖旧证据的脚本。

### Task 1: Frontend 与 CloudColorizer 直接实现

**Files:** `include/livo_atlas/frontend/frontend.h`、`src/frontend/frontend.cc`、
`include/livo_atlas/mapping/cloud_colorizer.h`、`src/mapping/cloud_colorizer.cc`。

**Interfaces:** 现有 Frontend 的 Process/state/IMU 查询和 CloudColorizer::Process 原签名保持。

- [ ] 记录现有 frontend_test、cloud_colorizer_test 及地图生命周期检查的参考结果。
- [ ] 将字段及辅助方法迁入 private，原 Impl 构造/析构体迁为所属类实现，删除转发。
  Frontend 值成员直接包含 PCL VoxelGrid 和 PointWithCovariance 的定义，真实指针组件允许前置声明。
- [ ] 重建并运行 `OMP_THREAD_LIMIT=1 ctest --test-dir <新普通构建目录> --output-on-failure -R '^(frontend_test|cloud_colorizer_test)$'`；
  复用既有地图生命周期检查核对视觉/体素根释放顺序，所有原断言通过才继续。
- [ ] 检查改动范围及 Google 格式，提交本任务对应文件。

### Task 2: LivoExporter 直接持有文件资源

**Files:** `include/livo_atlas/io/livo_exporter.h`、`src/io/livo_exporter.cc`。

**Interfaces:** `ValidateExportOptions`、构造、Write、Finish 原签名保持；
private TextFile 含 `std::filesystem::path path` 和 `std::ofstream stream`，
private static `Check(const TextFile&)`、`Open(const std::filesystem::path&, TextFile*)`、`Close(TextFile*)`。

- [ ] 先运行原 livo_exporter_test，记录文件失败、独立实例及重复 Finish 的参考结果。
- [ ] 按设计迁移成员、文件辅助函数和方法，移除 Impl；保留资源初始化、写入精度及关闭检查顺序。
- [ ] 执行 `OMP_THREAD_LIMIT=1 ctest --test-dir <新普通构建目录> --output-on-failure -R '^livo_exporter_test$'`，
  尤其核对 ReportsSpecificFileOnActualTextCloseFailure 和 ReportsPngCloseFailureAndKeepsTheWriterFailed。
- [ ] 检查改动与格式，提交本任务对应文件。

### Task 3: LivoSystem 直接组合

**Files:** `include/livo_atlas/core/livo_system.h`、`src/core/livo_system.cc`。

**Interfaces:** AddLidar/AddImu/AddImage、ProcessNext/ProcessAvailable、PropagateLatest、Finish、
SetOnlineSubmapCallback、GetPoseSnapshot、ApplyPoseUpdate 原签名保持。OnlineSubmapManager 保持现有私有模块接口。

- [ ] 先运行现有四组 system 测试，确认 RawSaveFailureBlocksOnlineMutations 参考通过。
- [ ] 迁移构造、字段和方法，逐一合并转发与 Impl 逻辑；保留现有 submap_error_ 防护和成员顺序。
  KeyframeStore/SubmapStore/OnlineSubmapManager 可前置声明，析构在 `.cc` 完整定义处实现。
- [ ] 执行 `OMP_THREAD_LIMIT=1 ctest --test-dir <新普通构建目录> --output-on-failure -R '^(livo_system_test|keyframe_system_test|submap_system_test|online_submap_system_test)$'`，
  所有现有错误、所有权、隔离和在线版本断言通过。
- [ ] 检查改动与格式，提交本任务对应文件。

### Task 4: 两端节点和发布器直接实现

**Files:** `include/livo_atlas/ros{1,2}/livo_node.h`、`src/ros{1,2}/livo_node.cc`、
`include/livo_atlas/ros{1,2}/livo_publisher.h`、`src/ros{1,2}/livo_publisher.cc`，其中花括号表示两个现有目录。

**Interfaces:** 两端 LivoNode 的构造/ProcessNext/ReportInputs，及各自 LivoPublisher 的现有方法保持。

- [ ] 在各自环境运行既有 ROS 节点/发布参考测试，记录结果。
- [ ] 两端同步迁移成员和方法，增加实际使用类型的直接包含；保留成员顺序、回调捕获、发布设置和时间戳处理。
- [ ] 用 Noetic 真实 rostest 和 Humble 的 ros2_node_test 验证订阅、定时器、图像、TF 和退出行为；
  命令及环境沿用 [双版本说明](../../ros-builds.md)，使用新验证工作区。
- [ ] 检查对称职责、同一 main 和格式，提交本任务对应文件。

### Task 5: 整体验收及文档

**Files:** 更新 `docs/cpp-style.md`、`docs/core-build.md`、`docs/README.md`；创建 `docs/pimpl-removal-validation.md`。
复用 `tests/installed_consumer/`、现有行为测试和两个 baseline 工具；只有发现真实行为缺口时才补对应测试。

- [ ] 在新目录分别配置 shared/static 普通构建：`LIVO_ATLAS_ROS_VERSION=0`、`LIVO_ATLAS_BUILD_TESTS=ON`、Release；
  `nice -n 10 cmake --build <目录> -- -j1`，然后 `OMP_THREAD_LIMIT=1 ctest --test-dir <目录> --output-on-failure`。
- [ ] 单独编译所有安装公共头，尤其 Frontend、LivoSystem、Exporter；检查普通头不含 ROS 或 src 私有路径。
- [ ] 两种安装均移动前缀后重新配置、构建并运行 tests/installed_consumer；Noetic/Humble 也重建全部依赖这些类的库和节点。
- [ ] 各环境串行回放现有 Retail_Street.bag：默认、旧分组、online 完整段及开启产品的短段，
  与该环境冻结参考比较四轨迹数组、发布摘要和既有导出资产，沿用原精确判定，不放宽容差。
- [ ] `rg -n 'class Impl|struct Impl|impl_|pimpl_' include src` 无匹配（预期退出码 1），人工确认没有改名包装；
  Google 格式与 `git diff --check` 通过。记录实际结果和局限，不以旧测试数字代替本次实测。
- [ ] 更新文档并完成自查，提交验证说明及最终任务状态；不修改已完成阶段的历史证据。

纯结构整理没有新增行为时不强行添加 RED 测试。若发现行为问题，先用最小行为测试复现，
再修复，并将新增影响纳入回归；不借此次任务调整算法数值。
