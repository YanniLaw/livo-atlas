# livo-atlas 重构文档

按下面的顺序阅读、实施：

1. [目标架构与阶段验收](superpowers/specs/2026-10-11-refactor-architecture-design.md)：重构范围、模块边界、迁移顺序及未来 SLAM/3DGS 接口。
2. [P1-01 执行计划](superpowers/plans/2026-10-11-p1-01-core-state.md)：数学与状态隔离的文件边界、兼容接口及数值验收。
3. [P1-02 执行计划](superpowers/plans/2026-10-18-p1-02-calibration.md)：普通标定类型、ROS1 参数入口、相机接线及验收步骤。
4. [P1-03 执行计划](superpowers/plans/2026-10-18-p1-03-measurements.md)：内部测量、ROS1 解码、IMU 值输入与回归步骤。
5. [ROS1 基线复现](validation.md)：在另一台 Noetic 电脑上创建独立构建副本、回放 bag、保存证据。
6. [C++ 编码约定](cpp-style.md)：Google 风格、依赖管理和控制实现规模的约束。
7. [P0 实测记录](p0-baseline.md)：原版问题、构建修复、已运行的检查及数值差异。
8. [P1-01 实测记录](p1-01-validation.md)：独立核心、状态兼容与配对 bag 数值证据。
9. [P1-02 实测与复现](p1-02-validation.md)：标定边界、真实启动检查、相机所有权及完整 bag 配对结果。
10. [P1-03 实测与复现](p1-03-validation.md)：图像/点云所有权、普通 IMU 输入、审阅修复、完整 bag 对照与离线迁移。
11. [P2-01 同步器计划](superpowers/plans/2026-10-18-p2-01-synchronizer.md)及[实现与复现](p2-01-validation.md)：普通同步核心、ROS1 成对缓冲、时间边界和 bag 对照。
12. [P2-02 Frontend 计划](superpowers/plans/2026-10-24-p2-02-frontend.md)及[实现与复现](p2-02-validation.md)：单一前端状态、真实处理流程、结果消费及发布独立性。
13. [P2-03 输出边界设计](superpowers/specs/2026-10-24-p2-03-output-design.md)、[执行计划](superpowers/plans/2026-10-24-p2-03-output.md)及[实现与复现](p2-03-validation.md)：普通输出处理、独立文件写入与 ROS1 发布。
14. [P3 核心独立设计](superpowers/specs/2026-10-24-p3-independent-core-design.md)、[执行计划](superpowers/plans/2026-10-24-p3-independent-core.md)、[实测记录](p3-validation.md)及[跨电脑构建说明](core-build.md)：普通相机/图像函数、core/Noetic 构建入口、外部消费与评审修复。
15. [P4 Humble 设计](superpowers/specs/2026-10-24-p4-humble-design.md)、[执行计划](superpowers/plans/2026-10-24-p4-humble.md)、[双版本使用说明](ros-builds.md)和[实测记录](p4-validation.md)：共用核心、ROS2接入、完整DDS回放及跨环境差异归因。
16. [统一入口与职责目录修正](superpowers/specs/2026-10-24-p4-unified-entry-layout-design.md)、[实施计划](superpowers/plans/2026-10-24-p4-unified-entry-layout.md)：同一 main、算法文件分类和规范 include 路径；当前目录见[源码组织](core-build.md#源码组织与-include-迁移)，结果见[修正验证记录](p4-layout-validation.md)。
17. [接入进一步收敛设计](superpowers/specs/2026-10-25-ros-convergence-design.md)、[实施计划](superpowers/plans/2026-10-25-ros-convergence.md)及[实测与迁移](p4-convergence-validation.md)：main注入同一普通MappingSession、共用参数清单、对称ROS接入职责与独立runtime安装。
18. VIO 独立修复：[缓存均值](vio-cached-mean-fix.md)、[图像采样边界](vio-sampling-bounds-fix.md)。
19. [命名与职责整理设计](superpowers/specs/2026-10-25-naming-cleanup-design.md)、[实施计划](superpowers/plans/2026-10-25-naming-cleanup.md)、[迁移清单](naming-audit.md)及[当前验证](p4-naming-validation.md)：core/common、LivoSystem与对称ROS节点、旧兼容层移除、着色和导出职责归类；保留六个指定类型名。
20. [P5-01 关键帧设计](superpowers/specs/2026-10-26-p5-keyframes-design.md)、[实施计划](superpowers/plans/2026-10-26-p5-keyframes.md)、[数据集与使用说明](keyframe-dataset.md)及[验证记录](p5-01-validation.md)：独立快照、目录式存储与原始/优化位姿版本。
21. [P5-02 子地图设计](superpowers/specs/2026-10-29-p5-submaps-design.md)、[实施计划](superpowers/plans/2026-10-29-p5-submaps.md)、[子地图说明](submap-dataset.md)和[验证记录](p5-02-validation.md)：稳定分组、持久成员索引及按逐帧版本重建几何。

这些文档描述目标设计。实际完成情况以源码、测试和验证报告为准；阶段表中的验收条件不代表已经通过。
当前已完成 P0 本机构建修复，以及 P1-01 的数学/状态隔离、直接测试、配对 bag 回归和独立审阅问题闭环。
P1-02 已完成普通标定类型、ROS1 参数入口、相机接线、直接测试、完整回放及独立审阅问题闭环。
P1-03 已完成内部测量类型、ROS1 转换、IMU 值输入、预处理独立构建、审阅问题闭环及最终完整回放；验证包括 78 项单元测试、2 项真实 ROS 回调、13 项 ASan 预处理、40 项启动检查和两组导出值零差异配对。
P2-01 已完成普通 C++ 同步器、ROS1 成对缓冲和独立审阅；98 项单元、5 项真实 ROS、40 项启动检查、两组导出值零差异对照及默认并行回放通过，另有 1 项延期的 IMU 回调补测建议。P2-02 已完成前端状态/处理所有权、ROS1 结果消费及审阅修复；106单元、9真实ROS、40启动、地图生命周期泄漏检查和最终完整回放通过，配对导出值零差异。P2-03 完成发布/I/O 分离及独立 writer：132 单元、11 真实 ROS、50 启动检查、精确 bag/导出对照通过。P3 已去除 vikit 并提供独立核心：150 单元、shared/static 核心各130项、12真实ROS、50启动、23公开头检查通过；三种安装迁移消费及外部catkin消费通过，136秒回放精确一致，57个导出文件逐字节一致。整阶段评审问题已闭环，无延期 Minor。P4 已完成 Noetic/Humble 同源接入及一次独立评审修复：Noetic158单元/12真实ROS/50启动，Humble176单元/16实际启动、5项Python工具测试通过；shared/static核心各138项及25公开头通过。两套环境均完整回放1351位姿，Noetic对P3严格一致，Humble同环境离线/DDS严格一致，跨环境差异已量化。下一步为关键帧快照与存储边界，再接 GTSAM/重建导出。

上一轮入口修正统一main并完成职责目录和规范头迁移，其Noetic158/Humble176等历史结果见[入口修正报告](p4-layout-validation.md)。

上一轮进一步收敛为一份普通MappingSession、一份参数清单和两个各含四类职责的ROS适配器。
Noetic172项单元/8项真实ROS/50项启动、Humble188项C++/16项启动通过；普通shared/static各152项与40个公开头检查通过。
三种普通SDK和ament安装实际消费runtime成功；两端全包各1351位姿、四数组与发布精确保持，短段各57个产品逐字节一致。
详细迁移和证据见[接入收敛报告](p4-convergence-validation.md)。

当前命名整理已采用core/common、LivoSystem和两端LivoNode，移除internal兼容层并拆分着色/导出职责，保留六个指定类型名。
普通152、Noetic172单元/8真实ROS/50启动、Humble188单元/16启动及39个普通头通过；
三种普通SDK和ament安装消费通过，两端完整bag各1351位姿、四数组/发布精确保持，短段各57文件逐字节一致。
当前入口与跨电脑迁移见[命名整理报告](p4-naming-validation.md)。历史报告中的runtime等名称保留其原始记录。

P5-01 已加入默认关闭的关键帧选择、独立快照、数据集保存/重读和优化位姿版本。
普通180、Noetic200单元/8真实ROS、Humble216项及42个普通公开头通过，三种SDK及ament迁移消费通过。
两端开启关键帧的完整bag各1351位姿、172个带图像关键帧；四数组/发布保持精确一致，短段各57个原有产品逐字节一致。
Humble数据集经Noetic构建的C++读者完整消费。格式和操作见[数据集说明](keyframe-dataset.md)，
具体证据见[P5-01报告](p5-01-validation.md)。

P5-02 已完成默认关闭的子地图分组、成员索引和选定位姿版本下的几何重建，补齐前轮输入错误路径信息。
普通201、Noetic221单元/8真实ROS、Humble237项及44个普通头通过；SDK和ament迁移消费通过。
两端完整172个关键帧各形成9组，四数组/发布及原有关键帧资产保持一致，短段各57旧产品逐字节一致。
实际副本的逐帧位姿修正改变组内几何且不修改原资产；使用与证据见[子地图说明](submap-dataset.md)和[P5-02报告](p5-02-validation.md)。

当前原版参考提交：`ca556e5ad6706ba20dc8dff91010faae546b7528`。
2026-10-11 的原版采集结果位于工作区 `validation/P0-noetic-20261011/`。
构建兼容性修复的采集结果位于 `validation/P0-build-fix-20261011/`。
验证产物在包外，迁移电脑时需要单独复制；bag 和编译产物不进入源码仓库。
