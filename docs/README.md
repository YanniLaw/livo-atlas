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

22. [P5-03 离线导出设计](superpowers/specs/2026-10-30-p5-colmap-export-design.md)、[实施计划](superpowers/plans/2026-10-30-p5-colmap-export.md)、[使用说明](colmap-export.md)和[验证记录](p5-03-validation.md)：按选定位姿版本导出居中针孔图像、相机与带颜色LiDAR种子，保留原数据。
23. [P5-04可靠性设计](superpowers/specs/2026-10-31-p5-reliability-design.md)、[实施计划](superpowers/plans/2026-10-31-p5-reliability.md)和[验证记录](p5-04-validation.md)：检查图片写入与关闭、定位大小核数值差异和图像历史淘汰，补齐视觉缓存所有权与重复/耐久验证。
24. [完整 SLAM 与在线 3DGS 子地图调研](online-3dgs-submap-research.md)：截至 2026-11-08 的论文和官方实现核对、候选架构、闭环地图修正及当前接口差距；属于研究建议，尚未实施。
25. [在线子地图设计](superpowers/specs/2026-11-08-online-submaps-design.md)：共享观测、局部坐标、生命周期、版本化位姿与高斯修正契约；首个实施范围限定为 P6-01 数据与接口基础。
26. [P6-01 实施计划](superpowers/plans/2026-11-08-p6-01-online-submaps.md)：在线分组、位姿快照、不可变索引、拥有观测交付及双 ROS 验收的六个任务；已完成，证据见下方验证记录。
27. [P6-01 实施进度与验证](p6-01-validation.md)：P6-01最终验收、三种普通构建、双ROS完整bag及已知边界；[在线子地图使用说明](online-submap-dataset.md)介绍配置、检查和离线兼容出口。
28. [P6-01 之后的阶段路线](superpowers/specs/2026-11-08-online-submaps-design.md#p6-01-之后的阶段路线)：先完成几何子地图，再依次接入 GTSAM 位姿图、真实回环、在线 Gaussian mapping 和可选高斯辅助估计；各阶段编码前先细化设计与验收。
29. [移除 Pimpl 设计](superpowers/specs/2026-11-10-remove-pimpl-design.md)、[实施计划](superpowers/plans/2026-11-10-remove-pimpl.md)及[验证记录](pimpl-removal-validation.md)：移除现有 8 处 Pimpl，恢复普通类的私有成员与直接方法实现；双 ROS、普通 SDK 及配对 bag 已完成验收。
30. [P6-02a 几何子地图设计](superpowers/specs/2026-11-10-p6-02a-geometry-submaps-design.md)、[实施计划](superpowers/plans/2026-11-10-p6-02a-geometry-submaps.md)、[使用说明](geometry-submaps.md)与[验证记录](p6-02a-validation.md)：主成员体素统计、条件有效的面元、版本修正、有限缓存和显式导出已实现，双 ROS 回归与本阶段验收已完成。
31. [P6-02b GTSAM 位姿图设计](superpowers/specs/2026-11-11-p6-02b-pose-graph-design.md)、[实施计划](superpowers/plans/2026-11-11-p6-02b-pose-graph.md)、[使用说明](pose-graph.md)和[验证记录](p6-02b-validation.md)：可选批量优化、模拟回环和版本化几何交付已完成阶段验收。
32. [P6-02c 真实回环设计](superpowers/specs/2026-11-12-p6-02c-loop-closure-design.md)、[C1 实施计划](superpowers/plans/2026-11-12-p6-02c1-loop-verification.md)、[使用说明](loop-verification.md)和[验证记录](p6-02c1-validation.md)：C1 先验辅助几何验证与只读工具已完成阶段验收；C2a 准入与 C2b 在线接纳见后两项，全局当前位姿见 C3a，优化历史轨迹与地图留待 C3b。

33. [C2 回环准入与在线接纳设计](superpowers/specs/2026-11-13-p6-02c2-loop-admission-design.md)、[C2a 实施计划](superpowers/plans/2026-11-13-p6-02c2a-loop-admission.md)、[使用说明](loop-admission.md)及[验证记录](p6-02c2a-validation.md)：无状态约束准入、鲁棒图试算、优化后几何复核和离线工具已完成阶段验收；在线调度与提交见 C2b。
34. [C2b 在线回环设计](superpowers/specs/2026-11-14-p6-02c2b-online-backend-design.md)、[实施计划](superpowers/plans/2026-11-14-p6-02c2b-online-backend.md)及[使用说明](online-loop-backend.md)：固定读取前沿、单 worker、所有者持久接纳和停止后检查已完成；主验收14组构建与8次官方回放、审阅修复后的7组受影响构建复验通过，见[验证记录](p6-02c2b-validation.md)。
35. [C3a 全局 TF 与当前位姿设计](superpowers/specs/2026-11-14-p6-02c3a-global-pose-design.md)、[实施计划](superpowers/plans/2026-11-14-p6-02c3a-global-pose.md)、[使用说明](global-pose-output.md)和[验证报告](p6-02c3a-validation.md)：常数规模修正、成对 TF 和当前全局位姿已实现；14组构建、12组真实入口、12次官方回放及整阶段独立审阅完成；原验收器Minor已[补验关闭](p6-02c3a-followup-validation.md)，新版自动检查版本解释顺序。优化历史轨迹与版本化地图留待 C3b。
36. [C3b 历史轨迹与几何地图设计](superpowers/specs/2026-11-15-p6-02c3b-history-design.md)、[C3b-01 计划](superpowers/plans/2026-11-15-p6-02c3b1-keyframe-trajectory.md)、[使用说明](keyframe-trajectory.md)与[验证记录](p6-02c3b1-validation.md)：版本化逐帧轨迹、双 ROS 消息、有限后台输出已完成；14组构建、22个安装消费者及10次官方回放通过，两项重要问题闭环；C3b-02 几何显示在后续阶段实施。

37. [C3b-02 几何地图显示设计](superpowers/specs/2026-11-15-p6-02c3b2-geometry-map-design.md)、[实施计划](superpowers/plans/2026-11-15-p6-02c3b2-geometry-map.md)、[使用说明](geometry-map-output.md)和[验证记录](p6-02c3b2-validation.md)：私有几何构建、有限片段缓存、流式显示采样、版本来源和双 ROS 话题已完成；构建矩阵、官方 bag、RViz 和独立审阅结果见验证记录。

38. [首帧传感器暂存设计](superpowers/specs/2026-11-15-startup-sensor-buffer-design.md)、[实施计划](superpowers/plans/2026-11-15-startup-sensor-buffer.md)、[使用说明](startup-sensor-buffer.md)与[验证记录](startup-sensor-buffer-validation.md)：首帧前有界暂存与旧行为开关已实现，双 ROS 受控顺序及完整 bag 对照和独立审阅已完成。

39. [P6-03a 重建观测设计](superpowers/specs/2026-11-15-p6-03a-reconstruction-view-design.md)、[实施计划](superpowers/plans/2026-11-15-p6-03a-reconstruction-view.md)、[使用说明](reconstruction-view.md)和[验证记录](p6-03a-validation.md)：为在线高斯建图准备拥有数据的去畸变图像、稀疏深度和版本化相机位姿。

40. [P6-03b GPU 可行性设计](superpowers/specs/2026-11-16-p6-03b-gpu-feasibility-design.md)、[实施计划](superpowers/plans/2026-11-16-p6-03b-gpu-feasibility.md)、[实验说明](p6-03b-gpu-probe.md)与[验证记录](p6-03b-validation.md)：独立 SDK 观测与 GPU 离线训练链路已在 Noetic 主机和真实关键帧上验证；不代表在线 3DGS SLAM 已实现。

41. [P6-03c 在线 Gaussian 设计](superpowers/specs/2026-11-16-p6-03c-online-gaussian-design.md)、[P6-03c1 实施计划](superpowers/plans/2026-11-16-p6-03c1-online-gaussian.md)、[使用说明](online-gaussian.md)与[验证记录](p6-03c1-validation.md)：单活动子地图的有界 GPU 训练和版本化预览已接入共用 ROS 入口；功能边界、平台范围和官方 bag 成对比较见验证记录，不代表完整在线 3DGS SLAM。

42. [P6-03c2 多子地图与模型恢复设计](superpowers/specs/2026-11-17-p6-03c2-gaussian-submaps-design.md)、[C2a 实施计划](superpowers/plans/2026-11-17-p6-03c2a-gaussian-state.md)、[资产使用说明](gaussian-state.md)和[验证记录](p6-03c2a-validation.md)：C2a 独立 Gaussian 工作集保存、CPU 检查、GPU 恢复和继续训练已完成；80 项 Python/CUDA、62 项普通核心 CTest、真实来源冷恢复及独立审阅问题闭环通过，在线多块和闭环一致性见后续 C2b。

43. [P6-03c2b 在线 Gaussian 子地图设计](superpowers/specs/2026-11-17-p6-03c2b-online-gaussian-submaps-design.md)与[实施计划](superpowers/plans/2026-11-17-p6-03c2b-online-gaussian-submaps.md)及[验证记录](p6-03c2b-validation.md)：单 GPU/CPU 确认回退、历史资产/目录、刚性重用/组内重建、持久提交与公平调度已实现；四项独立审阅问题已复现修正，最终同源官方短/完整配对与 C1 回归、13 块 GPU 冷恢复和受控真实观测通过；退出仍保留明确标记的历史 needs_rebuild，不代表完整在线 3DGS SLAM。

44. [P6-03c3a Gaussian 最终收敛设计](superpowers/specs/2026-11-18-p6-03c3a-gaussian-finalization-design.md)、[实施计划](superpowers/plans/2026-11-18-p6-03c3a-gaussian-finalization.md)和[验证记录](p6-03c3a-validation.md)：可选终端 Finalize 已接入共用 ROS 入口，候选官方 bag 的 13 块确认内容全部 current；默认模式保持，最终审阅及复验进行中。
