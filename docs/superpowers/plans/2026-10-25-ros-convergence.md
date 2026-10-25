# ROS 接入收敛实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 让同一入口构造同一个普通运行层，两套 ROS 接入共用处理流程和参数 schema。

**Architecture:** 普通 MappingSession 统一输入状态/估计/输出组合；六类 typed parameter 读取及名称描述连接两个真实ROS配置源，公共清单只维护一份。两套 MappingAdapter 接收现成 Session，目录职责对应，保留原执行器/订阅策略。

**Tech Stack:** Google C++17、CMake3.16、Noetic、Humble；现有 Eigen/PCL/OpenCV/Sophus；不增加依赖。

**Spec:** `docs/superpowers/specs/2026-10-25-ros-convergence-design.md`

## Global Constraints

- 仅修改 livo-atlas，不修改 rpg_vikit，不合并/推送。
- 先文档后编码，Google C++17；普通算法无ROS依赖或版本条件。
- 不修改 IMU/LIO/VIO、地图、相机、同步器、预处理和输出/writer 的估计实现。
- main创建一次Session；适配器不拥有/创建Frontend、同步器、writer。
- Noetic `[0, 2^32)` 秒，Humble `[-2^31, 2^31)` 秒；保留两次时间量化和coverage顺序。
- Noetic spinOnce/5000Hz空闲等待、原订阅队列；Humble单线程executor/1ms drain/4ms传播/QoS保持。
- 原 YAML / Noetic参数路径 / Humble点号名称、话题/schema保持；统一拒绝非法配置和非有限IMU。
- 新证据 `validation/P4-convergence-20261025`；候选只写 `/tmp/livo_atlas_convergence`，冻结依赖/bag只读。
- 编译nice -n10/-j1，回放与编译串行，无RViz；一次整阶段独立评审。

## Review Focus

1. 共享运行层被两端注入：生命周期销毁顺序、回调视图期限、异常/Finish和实例隔离。
2. 时间修正包含两次舍入：coverage用中间值，范围和秒进位正确，拒绝坏数据后继续有效序列。
3. 两个typed配置源：默认值/整数浮点标定/相机私有命名空间/只读参数/remap；错误在文件副作用前失败。
4. 空云/回退/丢图/初始化窗口：单步与drain共用处理正文，不因某次未更新而阻塞或发布旧视觉云。
5. 安装和真实运行：普通runtime可独立shared/static/无OMP消费，同环境bag/输出严格一致。

## 文件归属

按spec路径创建 runtime/{mapping_session,options,parameter_reader,mapping_options} 公开头、三个实现。
两个ros目录最终都只保留 mapping_adapter/parameters/measurement_conversions/result_publisher 的 .cc 与对应规范头。
测试普通运行语义放 `tests/runtime`；ROS转换配对留各ROS测试，参数/接入测试使用真实ROS API。
更新 `cmake/{core,ros1,ros2,tests}.cmake`、共同main、安装消费者及当前docs/README。
旧阶段文档/证据不改；实际辅助脚本在新证据目录复制，所有参考场景指向P4-layout冻结产物。

### Task 1: 普通共享运行层

**Files:** runtime公开头/实现；移除 `src/ros2/mapping_session.*`；更新ROS2使用方/CMake；`tests/runtime/{fixtures.h,session_test.cc}`、`tests/ros2/{fixtures.h,session_test.cc}`。

**Interfaces:** Consumes 原Frontend/Preprocess/Synchronizer/Output/Writer；produces
`MappingSession(SessionOptions)`、`AddLidar/AddImu/AddImage`、
`ProcessNext(const ResultCallback&) -> SynchronizationResult`、`ProcessAvailable`、
`PropagateLatest`、`Finish`，普通target `livo_atlas::runtime`。

- [x] **Step 1: 新普通消费者先失败。** 写runtime测试：LO通过单步产生有限快照并保持anchor；driver fix修正后coverage尚不足时返回waiting，后续IMU覆盖后ready；拒绝回退/非有限数据后有效序列不变。实际mode0配置/编译失败于新runtime接口不存在。
- [x] **Step 2: 抽出运行实现。** 原ROS2 session移至普通命名空间；ProcessAvailable调用单份ProcessNext；范围作为普通TimestampRange配置；PrepareSessionOptions集中现有普通校验/派生值。ROS2立即使用新库，不留下复制实现/转发头。
- [x] **Step 3: 迁移边界测试。** 原8项session测试分为普通语义与3个ROS转换配对；普通测试无cv_bridge/ROS。补unsigned范围、进位越界、两个实例传播/Hilti、初始化无发布、回退前空云校验、无发布消费等真实边界。覆盖原ROS1组合测试矩阵。
- [x] **Step 4: 验证并提交。** mode0全套原138+新runtime测试、普通头检查；Humble原转换配对和session行为不变。保存XML/命令/源码清单。

Expected: 新API缺失RED，普通runtime测试GREEN，原普通138项仍通过；Humble转换配对通过。
Commit: `refactor: share the mapping session outside ROS adapters`。
Task gate: 新证据 `check_convergence.py runtime` 检查已运行的全套普通XML、无ROS编译输入和原算法摘要。

### Task 2: 公共参数清单与typed读取器

**Files:** `runtime/{parameter_reader,mapping_options}.h`、`src/runtime/mapping_options.cc`、两端 `parameters.h/.cc`、ROS参数测试。

**Interfaces:** Consumes Task1 SessionOptions/PrepareSessionOptions；produces
`ParameterReader::{Boolean,Integer,Number,String,NumberArray,IntegerArray,Name}`，
`LoadMappingOptions(ParameterReader&) -> MappingOptions`；两端NodeParameters均有`.mapping`，
ROS2额外保留QoS。`ReadParameters(Node&) -> NodeParameters`用于Task3。

- [x] **Step 1: 写实际读取测试并观察失败。** 两端真实Node读取同一组覆盖值：模式、相机、外参、体素、预处理、输出、时间偏移；校验字面期望值与默认值。ROS1私有相机命名空间、ROS2只读/合法remap覆盖必须运行。
- [x] **Step 2: 公共schema。** 参数键只列一份；默认取options字段，整浮数值/两组外参/相机模型装配共用；输出目录默认规则共用。节点与普通配置分离，不增加YAML依赖。
- [x] **Step 3: 两个读取器。** ROS1 XmlRpc保留int/double数组/私有相机名称映射，参数key点号到斜杠；ROS2保留read_only声明/数值动态类型/QoS。两端先检查话题解析再创建Session。
- [x] **Step 4: 错误无副作用GREEN。** 类型错误、非有限/负协方差、非法话题与输出/地图/预处理边界，对新目录与既有哨兵目录都拒绝；两端普通配置一致。全套原参数/启动约束继续通过。

Expected: 公共schema缺失RED，实际读取/错误保护GREEN；合法原YAML得到一致普通配置。
Commit: `refactor: share mapping configuration across ROS versions`。
Task gate: `check_convergence.py parameters` 检查两端新参数测试XML与启动保护结果。

### Task 3: 对称适配器与同一manager入口

**Files:** `ros{1,2}/mapping_adapter.*`、共同main、两个ROS CMake；移除旧LIVMapper/独立calibration/voxel readers、ROS2 mapper_node/private头；更新测试与迁移文档。

**Interfaces:** Consumes Task1/2；produces 两端
`MappingAdapter(Node&, const NodeParameters&, MappingSession&)`、
`ProcessNext() -> SynchronizationResult`、`ReportInputs()`；适配器所有权和输入接口对应。

- [x] **Step 1: 实际接入先失败。** ROS1与DDS测试从真实发布输入推进到有限odom/cloud/TF，校验接收计数；新适配器缺失应使编译失败。保留现有发布schema测试。
- [x] **Step 2: 注入同一个运行层。** main共同路径读取参数、构造Session一次、构造选中适配器；Noetic保留原spinOnce单步，Humble定时drain相同单步。正常退出共同Finish/报告计数。
- [x] **Step 3: 清理重复职责。** 两端只留四类源/头；合并ROS1发布到mapper库，去掉ROS2私有session库；普通runtime导出到SDK/catkin/ament。旧ROS1公开内部状态测试改为普通语义测试+真实消息验证，不添加测试专用getter。
- [x] **Step 4: 两端全套GREEN。** 全部既有算法/转换/发布测试、普通runtime与配置测试、真实ROS/DDS和启动检查；检查同一TU/私有宏/无ROS普通编译输入、各算法源/头保持不变。

Expected: 新接入API缺失RED，真实消息与原schema GREEN，两个适配器都不创建算法组件。
Commit: `refactor: inject one mapping runtime into paired ROS adapters`。
Task gate: `check_convergence.py adapters` 检查真实ROS结果、最终编译输入/源码绑定。

### Task 4: SDK、官方bag和文档验收

**Files:** `tests/installed_consumer` 的实际runtime消费示例、当前构建/架构/风格文档、阶段验证报告；辅助脚本只在新证据目录。

**Interfaces:** Consumes Task1–3；produces 安装可用runtime、跨电脑复现文档与最终软件验收。

- [x] **Step 1: 安装消费者先失败再GREEN。** 补普通Runtime的LO/LIO/连续VIO场景；mode0 shared/static/无OMP均实际执行，Humble ament安装执行，无ROS链接/旧头/源树路径依赖。
- [x] **Step 2: 串行回放。** 两端各3s产品与完整136s；Noetic使用新catkin副本，Humble使用实际colcon安装节点。四数组、语义发布、短段57产品严格对照各自P4-layout；输入与实际程序/库SHA绑定。
- [x] **Step 3: 更新交付文档。** 新职责图/目录、共享参数schema与合法差异、旧API迁移、11个原ROS1组合测试的覆盖映射和实际新测试数量、跨电脑构建与证据路径。
- [x] **Step 4: 提交并验收。** 原Log31/vikit不变，所有检查与最终源码一致，没有以新阈值放宽回归。

Expected: 三种普通SDK与Humble安装成功；两端各1351位姿、四数组/发布精确一致、短段各57产品精确一致。
Commit: `docs: verify converged ROS adapters and shared runtime delivery`。
Task gate: `check_convergence.py acceptance`。

## 阶段收尾

完成软件验收后执行一次fresh-context整阶段独立评审，范围从dae47ef至最终实现，
检查spec/plan/ledger与五项Review Focus；重要问题一次RED→GREEN修复，minor记录。
再归档执行记录、生成包含全部引用/历史的bundle并在空bare仓库恢复验证最终提交。
保留当前分支和工作区；收尾记录与软件任务分开，不把尚未执行的恢复标记完成。
