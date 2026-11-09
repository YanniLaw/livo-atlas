# P5-01关键帧快照与存储实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立默认关闭、两端共用的关键帧选择/独立快照/存储重读及优化位姿版本基础。

**Architecture:** mapping拥有普通Keyframe/KeyframeSelector，io拥有KeyframeStore，LivoSystem组合。
相机/点云原语继续复用现有依赖；新keyframes库避免core与io循环。原始前端结果与运行接口保持。

**Tech Stack:** Google C++17、CMake3.16、Eigen/PCL/OpenCV/Sophus、Noetic/Humble，无新依赖。

**Spec:** docs/superpowers/specs/2026-10-26-p5-keyframes-design.md

## Global Constraints

- 仅修改livo-atlas，在已授权refactor2直接实施，基线c84368f；不改master/tag，不合并或推送。
- 保留VIOManager/Frame/Feature/Pose6D/SubSparseMap/VOXEL_LOCATION；不改变公式、常量、所有权、同步/量化顺序。
- keyframes默认关闭；不加GTSAM/回环/子地图融合/CUDA/训练/线程；现有ROS运行接口及默认结果保持。
- nice/-j1；编译与bag串行，无RViz；证据P5-01-20261026，容器/tmp/livo_atlas_p5_01。
- 旧证据/原Log只读，vikit当前缺失保持缺失。用户已选择本会话实施与主动本机验证。

## Review Focus

1. 云源LIO时刻、有效图像时刻与当前位姿时刻不同时，时间与坐标关联是否仍准确且不影响估计。
2. cv::Mat非连续内存与PCL别名在后续前端更新后，保存快照是否独立且只复制选中帧。
3. 已有数据集、重复ID/版本、资产缺失或写入中失败是否保护原文件并报告正确上下文。
4. 64位ID与浮点元数据跨序列化能否精确往返，损坏/错误形状是否在分配和使用前受到所需验证。
5. 默认关行为及完整core的shared/static/无OMP/ament安装、两端实际节点是否使用同一最终实现。

### Task 1: 关键帧数据、坐标与选择策略

**Files:** 新mapping/keyframe.{h,cc}、mapping/keyframe_selector.{h,cc}；frontend_result.h/frontend.cc；
tests/mapping/keyframe_test.cc、keyframe_selector_test.cc；cmake/core.cmake/tests.cmake/ros1.cmake。

**Interfaces:** Keyframe包含设计中的字段，CaptureKeyframe(uint64_t,const FrontendResult&)->Keyframe；
ValidateKeyframe(const Keyframe&)->void；TransformKeyframeCloud(const Keyframe&,const RigidTransform&)->PointCloud。
KeyframeOptions{enabled=false,translation_threshold=.5,rotation_threshold_deg=10,time_threshold=2}；
ValidateKeyframeOptions(const KeyframeOptions&)->void；KeyframeSelector(FrontendMode,const KeyframeOptions&)，bool Select(const FrontendResult&)。
FrontendResult新增double cloud_timestamp=0与optional<double> image_timestamp；新livo_atlas::keyframes目标。

- [x] Step1: 写行为测试后实际编译RED：ROI图像逐字节及云深复制；90度绕Z旋转、t=(1,2,3)、world=(1,3,3)得到body=(1,0,0)，
  新parent旋转单位、t=(10,0,0)重建(11,0,0)；timestamp10.25/cloud10.2/image10.24分别保持。
- [x] Step2: 写选择测试：默认关/首帧、LIVO仅Visual与LO/LIO门槛；相对上次选中累积位移>=.5、旋转>=10度、间隔>=2秒的独立触发，
  同时刻不重选，回退重设选择基准；NaN/负阈值、无效变换与图像不推进状态。先看到缺少头/符号失败。
- [x] Step3: 按原世界云和原状态只生成独立body快照，保存实际相机位姿；加输出元数据，不改同步器。
  编译keyframes物理库并补catkin/安装传递依赖，含公共头检查。
- [x] Step4: 全部普通测试GREEN，核对旧152行为仍在，Google格式和diff检查，提交。

Expected: 新接口RED→GREEN；选中快照独立且坐标/时间字面值正确，旧数值测试通过。
Commit: feat: capture independent keyframes and select motion updates。
Task gate: validation/P5-01-20261026/check_phase.py model。

### Task 2: 数据集保存、重读与优化位姿版本

**Files:** 新io/keyframe_store.{h,cc}、tests/io/keyframe_store_test.cc；cmake/core.cmake/tests.cmake。

**Interfaces:** KeyframeStore(path,calibration)创建新集；KeyframeStore(path)打开已存在集；
Append(const Keyframe&)、Read(uint64_t)->Keyframe、ids()->const vector<uint64_t>&、calibration()->const Calibration&；
KeyframePose{id,map_from_body}；WritePoses(uint64_t,const vector<KeyframePose>&)、ReadPoses(uint64_t)->vector<KeyframePose>。
旧集禁止Append，允许写新优化版本。原始位姿在Keyframe中，版本0不写文件。

- [x] Step1: 写实际I/O测试并RED：BGR/无图像两种快照，旋转/标定/双精度时间和超32位版本ID精确往返；
  新版本重建世界点(11,0,0)，原帧文件摘要及原body云保持。
- [x] Step2: 写保护/损坏测试并RED：已有根目录及哨兵不覆盖，ID不连续/重复，未知schema，缺图/坏PCD/字段或点数不符，
  校准图像尺寸不符，重复/缺ID的版本与既有版本不覆盖；.pending目录忽略；写入中失败保留已提交帧并阻止后续写。
- [x] Step3: 按设计实现schema1、固定资产名、严格数值解析与17位字符串；PCD readHeader后才读数据；PNG验证BGR/标定尺寸。
  使用临时资产目录和rename提交，错误带路径，不自动删除既有数据。新store源归现有io库并依赖keyframes。
- [x] Step4: 全部普通测试GREEN、公共头与格式检查，提交。

Expected: 实际写/读/版本修正与失败保护全部GREEN，前端状态不受影响。
Commit: feat: persist keyframes and version optimized poses。
Task gate: check_phase.py store。

### Task 3: 共用系统与参数集成

**Files:** system_options.h/.cc、livo_config.cc、livo_system.cc、tests/core/livo_system_test.cc、
tests/ros1/node_options_test.cc、tests/ros2/node_options_test.cc、tests/installed_consumer/*。

**Interfaces:** SystemOptions.keyframes；四个keyframes.*参数；数据集exporter.directory/keyframes。
LivoSystem仍使用原ResultCallback，只有新功能开启时选择、快照并Append。

- [x] Step1: 写真实系统测试并RED：同一LO/LIO/LIVO序列开/关的状态、云、时间和更新次数一致；
  默认无keyframes目录、开启得到可重读快照；后续多次更新不改首帧；已有目录启动不截断mat哨兵。
- [x] Step2: 扩展两端既有参数行为断言，四键默认关闭并读取同样值；直接SystemOptions非法阈值在任何I/O前失败。
  安装消费者通过完整core实际调用保存/打开/版本修正接口，保持旧三模式场景。
- [x] Step3: PrepareSystemOptions先验证，Store先于Exporter构造；按原消费链加入选择和保存，Append成功后递增ID。
  不增加ROS私有流程或库依赖，不改变发布与已有导出；失败向上报告。
- [x] Step4: 普通全套、Noetic全套与真实ROS、Humble全套GREEN，安装消费者构建运行通过，提交。

Expected: 默认关与启用均保持前端原行为，新目录可重读；所有功能仅在完整普通core组合一次。
Commit: feat: record keyframes through the shared LIVO system。
Task gate: check_phase.py integration。

### Task 4: 官方bag、安装迁移与数据格式文档

**Files:** 新docs/p5-01-validation.md、数据格式/读取说明及当前README索引，tools/keyframe数据集检查工具（仅若实际验收需要）。
验证辅助在包外新证据，不改冻结参考。

**Interfaces:** schema1跨电脑重读与版本化几何；实际节点源码+新运行库摘要绑定。

- [x] Step1: shared/static/无OMP独立SDK及Humble ament安装移动后实际LO/LIO/VIO和存储消费通过，所有普通头独立编译。
- [x] Step2: Noetic/Humble完整单元、真实ROS/DDS与启动保护、Python工具回归；默认关/非法选项无副作用。
- [x] Step3: 两端各短3秒与全136秒bag，开启P5；检查所有帧资产可读、有效时间/校准/有限位姿/图像与局部几何关联，
  冻结P4-naming四数组与发布精确一致、短段各57旧产品逐字节一致。保存输入与实际源码/节点/库摘要。
- [x] Step4: 更新格式、参数、启用/重读和跨电脑命令；核对原Log/无vikit/tag/master保持，提交完整验收。

Expected: 四次开启P5的真实回放保留前端结果，关键帧数据可消费，所有安装模式通过。
Commit: docs: validate portable keyframe datasets across ROS versions。
Task gate: check_phase.py acceptance。

## 阶段收尾

一次fresh整阶段独立评审；Important/Critical一次RED→GREEN修复并跑相关全套，Minor登记；无再次评审。
只归档本计划工作区，完整历史bundle及空bare恢复核对全部引用，保留用户refactor2分支和工作区。
