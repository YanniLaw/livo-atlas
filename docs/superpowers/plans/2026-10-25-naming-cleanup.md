# 命名与职责整理实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实施已确认命名和模块归类，保留六个指定类型名及全部有效数据行为。

**Architecture:** 完整普通core加共享common定义；两端LivoNode接入同一个LivoSystem。
逐个迁移旧算法调用后删除internal兼容包装；CloudColorizer负责着色，LivoExporter负责文件导出。

**Tech Stack:** Google C++17、CMake3.16、Noetic、Humble，现有Eigen/PCL/OpenCV/Sophus，不增加依赖。

**Spec:** docs/superpowers/specs/2026-10-25-naming-cleanup-design.md

## Global Constraints

- 仅修改livo-atlas，当前feature分支直接实施，不合并/推送。
- VIOManager/Frame/Feature/Pose6D/SubSparseMap/VOXEL_LOCATION保留名称、拼写、大小写。
- 保留ROS节点名、可执行名、launch/参数键/话题/frame/schema；保持原执行策略。
- 不改公式/常量/状态顺序/所有权/量化和时间修正顺序，不拆VoxelMapManager。
- nice/-j1，编译与回放串行，无RViz；新证据P4-naming-20261025，容器/tmp/livo_atlas_naming。
- 旧证据/依赖只读；Log31与rpg_vikit保持原样。

## Review Focus

1. 保留的六个类型跨命名空间、std::hash、前向声明与模板实例是否仍指向同一类型。
2. 去掉兼容宏/别名是否改变矩阵类型、临时值、初始化顺序、重力/时间常量或表达式结合顺序。
3. 空/非空RGB云指针是否严格保持旧has_cloud的含义，包含LIVO的LIO阶段空消息和无发布消费。
4. COLMAP迁入导出器后，相机缩放、原始/去畸变图像、无效配置副作用与失败生命周期是否正确。
5. 完整core目标是否造成循环/ROS泄漏或静态导出缺失，安装迁移与实际回放是否使用最终程序。

### Task 1: 公共核心入口与接入命名

**Files:** core/common目录迁移；app/livo_main.cc；两端livo_node/node_options/livo_publisher；measurements；io/livo_exporter；CMake与调用方。

**Interfaces:** LivoSystem(SystemOptions)，AddLidar/AddImu/AddImage、ProcessNext/ProcessAvailable/Finish；
NodeOptions.livo、LivoConfig.system、SystemOptions.exporter；Imu/TimedImage/TimedPointCloud与FilterState。
完整target livo_atlas::core；临时保留output组件直到Task3。

- [ ] Step 1: 迁移现有安装消费者到core/livo_system.h和LivoSystem，使用冻结旧SDK编译，应因新头缺失失败。
- [ ] Step 2: 按spec统一目录、类型、配置成员和函数名；删除原core聚合目标，以完整core替代runtime；接入库改node。
- [ ] Step 3: 更新现有数值/系统/转换测试和所有include/CMake，保留表达式与外部接口。
- [ ] Step 4: mode0全部测试/公开头通过，Noetic节点与Humble构建通过；记录源码和变更映射，提交。

Expected: 新入口消费RED，完整普通core及两端构建GREEN。
Commit: refactor: name the shared LIVO core and ROS node interfaces。
Task gate: validation/P4-naming-20261025/check_naming.py core。

### Task 2: 公共定义与旧兼容层消化

**Files:** 删除include/livo_atlas/internal；新common/terminal_colors.h、mapping/point_with_covariance.h、frontend/imu/pose6d.h；既有算法头/源、对应测试/CMake。

**Interfaces:** 六个保留名位于livo_atlas；算法使用规范类型/常量与显式std/Eigen/Sophus名称；
保留Pose6D布局及FramePtr所有权。其余采用审查清单，Frame坐标方法使用明确名称。

- [ ] Step 1: 将原Euler/Reset兼容测试和PCL内存测试迁移到公共定义；去掉仅验证废弃别名的静态断言，数值/内存行为断言保持。
- [ ] Step 2: 拆出真实定义，逐个替换旧别名/宏、全局using namespace和全局算法类型；保留六个名字及std::hash行为。
- [ ] Step 3: 清理实际未引用的DS_POINT/指针别名，按归属移动传播/视觉校验和ROS1测试；更新枚举/方法/成员的明确命名。
- [ ] Step 4: mode0全部原行为测试/头检查、两端完整测试GREEN，核对六个类型与机械变更范围，提交。

Expected: 无internal目录或转发头，所有原数值/生命周期断言GREEN。
Commit: refactor: consolidate shared types and remove legacy wrappers。
Task gate: check_naming.py common。

### Task 3: 着色与导出职责分离

**Files:** mapping/cloud_colorizer.*、io/livo_exporter.*、core/system_options、LivoSystem、两端LivoPublisher、tests/mapping与tests/io。

**Interfaces:** ColoredCloud别名归sensor/point_types；CloudColorizer::Process(result)返回ConstPtr，
nullptr等价旧has_cloud=false；非空空云保持发布语义。Publisher.Publish/Exporter.Write接收该指针；无PreparedOutput。
CloudColorizerOptions只含camera/mode/scan_interval/minimum_color_distance；ExportOptions拥有COLMAP配置。

- [ ] Step 1: 迁移已有着色和导出测试到新接口，保留字面RGB/图像/文件/计数期望；先编译失败于新类/签名不存在。
- [ ] Step 2: 保留原累积/采色正文，移为CloudColorizer；删除COLMAP去畸变分支；用空指针表示无云。
- [ ] Step 3: 导出器在文件操作前建立已校验的COLMAP相机，Write时按原有效云门槛调用同一去畸变函数；保持记录及命名/格式/计数。
- [ ] Step 4: 更新系统/发布/安装消费者，移除output目录和目标；全套数值、发布schema、无副作用与跨帧测试GREEN，提交。

Expected: 新实际接口RED，原着色/导出/系统/发布行为GREEN，只有mapping/colorizer与io导出职责。
Commit: refactor: separate cloud colorization from reconstruction export。
Task gate: check_naming.py products。

### Task 4: 安装、官方bag与迁移文档

**Files:** 当前构建文档/README、命名迁移报告、既有安装消费示例；新证据辅助脚本。

**Interfaces:** 完整core安装可单独消费；各真实节点绑定最终源码与实际运行库。

- [ ] Step 1: shared/static/无OMP及Humble ament安装移动后，实际LO/LIO/连续VIO通过；所有公开普通头独立编译。
- [ ] Step 2: Noetic/Humble全部单元、真实ROS/DDS、启动保护通过；Python工具回归、外部接口清单保持。
- [ ] Step 3: 串行两端3s/136s回放，精确对照各自P4-convergence四数组/发布/短段57产品，保存输入与节点/库SHA。
- [ ] Step 4: 更新迁移表与跨电脑说明；Log31/vikit不变，提交并运行最终验收。

Expected: 各1351位姿/57短段产品精确一致，安装与源码/运行文件绑定通过。
Commit: docs: verify naming and responsibility cleanup across ROS versions。
Task gate: check_naming.py acceptance。

## 阶段收尾

一次fresh整阶段独立评审，范围32b5892至软件最终提交；重要问题一次RED→GREEN修复，
minor记录。之后归档本计划工作区，创建--all完整历史bundle并空bare恢复，绑定最终提交。
保留当前分支/工作区；不把尚未执行的归档/恢复提前标为完成。
