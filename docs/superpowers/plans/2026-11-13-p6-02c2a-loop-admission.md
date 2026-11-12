# P6-02c2a Loop Admission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Preserve the user's current-branch and inline-execution preferences.

**Goal:** 将几何通过项转成经过歧义、相关性、图试算和优化后几何检查的只读位姿提案。

**Architecture:** 普通无状态 ProposeLoopUpdate 函数重建临时图，不维护第二份正式后端状态。
C1 负责搜索与配准，GTSAM提供鲁棒因子和原始残差诊断；私有离线读取器提供同视图几何。
正式线程调度、持久化接纳和ROS全局输出留给C2b/C3。

**Tech Stack:** C++17、Eigen、PCL1.10/1.12、GTSAM4.2.2、现有OpenCV YAML，Noetic/Humble。

**Spec:** [C2设计](../specs/2026-11-13-p6-02c2-loop-admission-design.md)。状态：**待用户审阅的实施草案，未编码**。
本轮设计细节是执行者建议。正式编码前先通知用户切换模型，继续沿用当前会话内实施方式。

## Global Constraints

- 只修改livo-atlas；Google C++、函数定义之间空行、必要注释、无Pimpl，单ROS main。
- 两个既有开关同时ON才构建 `livo_atlas::loop_backend`；不开第三个算法选项。
- 旧PoseGraph默认 `loop_huber_delta=0` 保持Gaussian；准入默认1.345、Block、白化6维范数。
- 回环sigma默认0.2m/3°，可配置但会话内固定；C2a强制Huber delta>0；歧义0.5m/5°；相关两端gap<=2或任何主成员交集；每查询最多一条。
- 图残差上限3、历史回环增量0.5；每帧改变量3m/20°；所有边界含等号。
- 最多5候选（配置范围1–5）、5000帧、200已接纳回环；C1几何预算和阈值不放宽。
- 非法输入抛异常；合法拒绝没有proposal。函数不写文件或改变输入；工具仅改变自身内存状态。
- 输入dataset只读，新输出必须在输入外且不存在。零提案exit0，CLI语法错误2，运行失败1。
- 重型任务串行 nice10/-j1，回放CPU0/OMP1，无RViz；现有bag和旧验收只读。

## Review Focus

1. 鲁棒权重很小导致错误约束“目标下降”，却污染整图：Task1/3检查实际未降权残差和历史约束。
2. 不同目标给出互相冲突的位置，最高分仍很好：Task3必须整查询歧义拒绝，不能挑最高分掩盖。
3. 组内变化后只换锚点、不重建体素：Task2/3核对逐源局部位姿，且在预览视图检查Z及Z_pred。
4. 试算拒绝却留下一条边或可提交PoseUpdate：Task3/4逐字段检查输入不变、proposal空、后续结果未污染。
5. 原始数据损坏与正常几何拒绝混淆，或失败报告覆盖输入：Task4检查损坏/失败位置与独占输出。

## 文件与接口

| 文件 | 责任 |
| --- | --- |
| `include/livo_atlas/backend/pose_graph.h`, `src/backend/pose_graph.cc` | 可选鲁棒回环与优化诊断 |
| `include/livo_atlas/backend/loop_closure.h`, `src/backend/loop_quality.cc`, `loop_registration.cc`, `loop_internal.h` | 复用输入验证，新增固定变换检查 |
| `include/livo_atlas/backend/loop_admission.h`, `src/backend/loop_admission.cc` | 普通输入、准入顺序、试算及诊断 |
| `src/backend/loop_selection.h/.cc` | 内部候选排序、歧义和相关性，独立确定性测试 |
| `src/io/loop_proposer.h/.cc`, `tools/propose_loops.cc` | 只读数据适配、顺序模拟和诊断输出 |
| `src/io/loop_options.h/.cc`, `src/io/loop_verifier.cc` | 提取复用现有数字/字段读取助手，保持C1配置兼容 |
| `cmake/loop_backend.cmake`, `CMakeLists.txt`, `cmake/tests.cmake`, `cmake/ros1.cmake`, `cmake/ros2.cmake`, `cmake/livo_atlasConfig.cmake.in`, `cmake/ros2_dependencies.cmake.in` | 条件目标、安装消费和测试注册；沿用现有catkin依赖extras |
| `tests/backend/*`, `tests/io/loop_proposer_test.cc`, `tests/installed_loop_backend_consumer/` | 语义与包边界 |

验证命令约定（编码时运行，先设置本机路径；阶段目录必须新建，禁止复用旧验收build）：

```bash
LIVO_SRC=/path/to/livo-atlas
C2A_BUILD=/path/to/new-validation/core
cmake -S "$LIVO_SRC" -B "$C2A_BUILD" \
  -DLIVO_ATLAS_ROS_VERSION=0 -DBUILD_TESTING=ON \
  -DLIVO_ATLAS_WITH_GTSAM=ON -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
  -DGTSAM_DIR=/path/to/gtsam/lib/cmake/GTSAM \
  -DSophus_DIR=/path/to/sophus-config
nice -n 10 cmake --build "$C2A_BUILD" -- -j1
(cd "$C2A_BUILD" && OMP_THREAD_LIMIT=1 ctest --output-on-failure)
```

预期：RED阶段只有新增契约失败；GREEN为非零测试且全部通过。CMake3.16应进入build运行ctest，
不用可能被忽略的新版命令参数。每个任务提交前运行所列过滤测试及完整suite。

以下名字与字段是跨任务契约；实现者可决定函数内部局部变量和测试fixture组织，不创建通用框架。

### Task 1：鲁棒因子与真实残差诊断

**Consumes:** 现有PoseGraph、LoopConstraint、PoseSnapshot。
**Produces:** `PoseGraphOptions::loop_huber_delta`；`PoseGraphOptimizationError : std::runtime_error`；`FactorResidual`；PoseGraphResult新增
`vector<FactorResidual> odometry_residuals, loop_residuals`。
`FactorResidual`固定字段：from_id、to_id、initial_whitened_norm、final_whitened_norm、
final_rotation_rad、final_translation_m。保留原PoseGraphResult字段/API。

- [ ] 在 `tests/backend/pose_graph_test.cc` 先加测试：delta0保持既有高斯结果；正delta对大离群回环降权；
  prior/odometry不包装；非法delta拒绝；诊断与实际factor的未降权白化误差相同，6维Block而非逐分量。
  非交换旋转和平移用于验证测量方向，不能只测纯平移。
  验证非法配置仍为invalid_argument；有限合法输入触发非有限目标时为PoseGraphOptimizationError，
  数值失败不产生PoseUpdate；未知异常与bad_alloc不能被统一改为正常拒绝。
- [ ] 配置阶段外构建目录，ROS_VERSION=0、GTSAM=ON、LOOP_CLOSURE=ON、BUILD_TESTING=ON。
  编译新增测试，预期缺字段/API的RED；实现只在回环因子上包装Huber。
- [ ] 诊断直接调用原factor误差，按原sigma计算范数，拒绝非有限数值；不要用Robust whiten后的误差。
- [ ] 运行 `ctest --output-on-failure -R pose_graph`（实际build工作目录），预期非零测试全部通过；
  运行完整已有核心suite，确认默认行为未变。提交 `feat: add robust loop factors and graph diagnostics`。

### Task 2：给定变换的几何检查

**Consumes:** C1 GeometrySubmap、LoopMetrics、LoopRegistrationOptions及已实现输入/数值保护。
**Produces:** `LoopGeometryCheck { LoopStatus status; LoopMetrics forward, reverse; }`，以及：

```cpp
LoopGeometryCheck EvaluateLoopGeometry(
    const GeometrySubmap& target, const GeometrySubmap& source,
    const RigidTransform& target_from_source,
    const LoopRegistrationOptions& options = {});
```

- [ ] 新增 `tests/backend/loop_geometry_check_test.cc`：已知三维配准通过；固定错误变换即使重新GICP能修正也必须拒绝；
  平面退化、双向覆盖、NaN/Inf、float有限但平方不安全、预算、sources/视图不符全部有明确断言。
- [ ] 缺API编译RED后，将C1现有验证提取为同一内部实现供两入口调用；公共类型不暴露PCL。
  固定变换及逆只做质量，不调用GICP；复用重叠→残差→退化顺序，不使用人为零闭合误差充数。
- [ ] `LoopStatus::kVerified`在这个接口仅表示固定变换质量通过；预算/点数/质量拒绝沿用已有枚举，
  不可能返回迭代/收敛类状态。非法变换抛异常。空几何为insufficient_points。
- [ ] 运行 `ctest --output-on-failure -R 'loop_'` 与全核心suite；C1历史测试保持通过。
  提交 `feat: validate geometry at fixed loop transforms`。

### Task 3：无状态约束准入与试算

**Consumes:** Tasks1/2、C1搜索/配准、MakePoseSnapshot。
**Produces:** 设计中的AcceptedLoop、LoopAdmissionInput、LoopProposal、LoopAdmissionResult、
LoopAdmissionOptions和 `ProposeLoopUpdate(input, reader, options)`。

选项字段：search、registration、graph；loop_translation_sigma_m=.2、loop_rotation_sigma_rad=3°对应弧度、
max_hypothesis_translation_m=.5、max_hypothesis_rotation_deg=5、correlated_submap_gap=2、
max_factor_residual=3、max_accepted_residual_increase=.5、max_pose_correction_m=3、
max_pose_correction_deg=20、max_keyframes=5000、max_loops=200。默认构造设置graph.loop_huber_delta=1.345；
其余graph值保持原默认。数值必须有限，sigma与预算为正，增量非负；角度在(0,180]。
同时拒绝registration的min_points/min_inliers/min_normal_pairs超过max_voxels的不可达配置；
不能错误地用粗化max_points限制原始体素诊断数量。

`LoopAdmissionStatus`枚举为kNoCandidate、kGeometryRejected、kAmbiguous、kCorrelated、kResourceLimit、
kOptimizationRejected、kConstraintResidual、kPoseCorrection、kGeometryChanged、kProposed。
候选诊断保存candidate及verification；图诊断保存目标/迭代/Task1残差，**不保存PoseUpdate**；
逐边几何诊断保存两端ID、是否新边及optional measurement/prediction检查。
LoopProposal含loop、verification、update；失败时proposal必须空，未执行诊断必须缺失。
公共诊断字段冻结为：`LoopCandidateReport { candidate, verification }`；
`LoopGeometryReport { target_id, source_id, new_constraint, optional<LoopGeometryCheck> measurement, prediction }`；
`LoopTrialDiagnostics { initial_error, final_error, iterations, iteration_limit_reached,
vector<FactorResidual> odometry_residuals, loop_residuals }`。
`LoopAdmissionResult`字段为status、candidates、optional trial、geometry_checks、optional proposal、
optional<string> optimization_error。求解器未返回时只保留optimization_error，不填造trial数值。

- [ ] 内部选择函数 `SelectLoopCandidate(hypotheses, accepted, poses, options)` 的精确内部类型由实现者在
  loop_selection.h定义；测试直接输入普通小型hypothesis，断言两两歧义、全部排序优先级、相等边界、
  无符号ID差、主成员交集、近邻相关、最优相关时不换次优。先缺API RED再实现。
- [ ] `loop_admission_test` 用真实三维体素和实际图测试：合法回环返回update并减少已知漂移；
  pose版本与方向正确；输入所有位姿/约束/几何逐字段不变；多次拒绝后合法输入结果不受污染。
- [ ] 为任务失败条件各设断言：图迭代到限；残差限；历史残差恶化；位姿修正限；
  已接纳项在新预览下失效；新项几何变化；已成功提案不能绕过历史复核；版本/前缀/噪声不一致；预算边界。
  增加C2a拒绝delta0而通用图仍允许0的断言；不靠随机噪声碰运气，使用固定已知变换与明确构造几何。
- [ ] 调用者提供的reader需要依指定快照重建；测试记录请求的version与sources，模拟cache重建改变
  representation_revision仍等价可用，伪造旧sources则异常。固定测量Z与预测Z_pred都须通过Task2。
- [ ] 依设计顺序实现；临时图逐帧加入raw、再历史约束及唯一新边；仅在Optimize调用的边界将
  PoseGraphOptimizationError转成kOptimizationRejected；Task1按设计精确转换数值失败类型。
  reader/I/O异常、未知异常和bad_alloc继续向外传播。
- [ ] 新目标 `livo_atlas_loop_backend` 链接现有backend与loop_closure，导出alias；单开/全关不存在目标。
  运行新增选择/准入测试与全suite，预期非零且全部通过；提交 `feat: propose loop updates with admission checks`。

### Task 4：只读顺序试算工具

**Consumes:** Task3、现有KeyframeStore/OnlineSubmapStore/GeometryMapper与YAML助手。
**Produces:** 安装工具 `livo_atlas_propose_loops`，只写新输出目录。

- [ ] 在 `loop_proposer_test` 创建小型真实数据集，覆盖零候选、全拒绝、一次提案、接纳后再次查询新视图；
  metadata损坏、输出已存在/在输入内、非法配置、先写一项再损坏后续资产。必须验证已有记录保留、
  complete=0和失败原因，原资产字节不变；补足C1延期测试在新消费者中的风险。
- [ ] 实现私有 `void ProposeLoopDataset(const std::filesystem::path& dataset,
  const std::filesystem::path& output, const LoopAdmissionOptions& options)`；构造完整元信息与原始位姿，
  从version0按source ID顺序调用Task3；所有查询固定为整个N帧数据集和全部封存元信息。
  测试future关键帧确实在图中、但future子地图不会入选；N超预算不自动改前沿。接受提案时先构造新快照和新accepted列表，全部成功后交换内存状态。
  reader持有独占的当前/预览几何缓存；按指定快照建立几何，不能重用不匹配的cached sources。
- [ ] 输出 `summary.yml`、`queries/ID.yml`、`proposals/VERSION.yml`，schema=1；记录完整选项、
  source范围、全部候选诊断、旧/新版本、完整PoseUpdate、constraint/成员来源、RSS另由验证脚本采样。
  summary含complete、查询/提案数和状态计数。失败写failure.yml，绝不把部分运行标记完整。
- [ ] 配置入口 `LoopAdmissionOptions ReadLoopAdmissionOptions(const std::filesystem::path& path)`
  在loop_proposer.h声明，严格解析四组，复用C1内部字段助手；禁止未知键、非数值与不可精确表示的整数。
  重构助手后C1配置测试及实际CLI必须继续通过；不扩展dataset持久化格式。
- [ ] 运行工具测试和全suite；提交 `feat: evaluate loop admission on recorded datasets`。

### Task 5：跨平台、实数据与文档验收

**Consumes:** 最终冻结源码、已存在官方bag及C1同平台基线。
**Produces:** docs/loop-admission.md、docs/p6-02c2a-validation.md及独立阶段证据。

- [ ] SDK四种开关组合；双开shared/static/no-OpenMP；安装消费者实际调用Task3并检查OFF目标缺席。
  两种单开要保留原PoseGraph或C1能力，旧Gaussian测试保持原默认结果。
- [ ] Noetic/Humble shared/static 双开构建、真实节点测试和安装消费；同平台默认全关配置作对照。
  冻结source副本和实际加载库；不得覆盖任何旧阶段证据。
- [ ] 复用同一703文件/172帧/13封存块数据集，两端顺序试算。记录每查询的版本、候选、提案/拒绝、
  各因子与几何指标、图复制/构造与求解时间、RSS、原文件摘要。零提案不是失败，不固定接纳数。
- [ ] 双端普通节点双开/全关各完整与短导出回放；比对输入、四数组、发布语义、资产和旧导出。
  未启用在线后端的节点不得自动调用准入函数，跨平台只做各自同环境严格比较。
- [ ] 一次整分支独立审阅；重要问题先失败复现再修复并跑相关suite；更新设计、计划和文档索引。
  检查文档链接与git diff --check，保存实际证据后提交 `docs: validate loop admission proposals`。

## 自审与后续边界

Tasks1/2分别提供图/几何诊断；Task3独占准入规则和失败顺序；Task4验证实际文件边界；Task5验证包兼容。
C2b的资产读取视图、所有者接纳日志、封存与排空生命周期尚未实现，不能从本计划推断在线安全已证明。
实现者应先修正文档中未决的重要问题再开工；独立设计检查的三项问题已回写：固定离线前沿、求解异常类型、可配置sigma与强制Huber。
