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

44. [P6-03c3a Gaussian 最终收敛设计](superpowers/specs/2026-11-18-p6-03c3a-gaussian-finalization-design.md)、[实施计划](superpowers/plans/2026-11-18-p6-03c3a-gaussian-finalization.md)和[验证记录](p6-03c3a-validation.md)：可选终端 Finalize 已接入共用 ROS 入口，14 次成组官方回放、13 块 GPU 冷恢复及独立审阅修正后的最终复验通过；默认模式保持，已有确认内容全部 current，仍不代表完整在线 3DGS SLAM。

45. [P6-03c3b 最终快照联合渲染设计](superpowers/specs/2026-11-18-p6-03c3b-snapshot-joint-rendering-design.md)、[实施计划](superpowers/plans/2026-11-18-p6-03c3b-snapshot-joint-rendering.md)、[使用说明](online-gaussian.md#最终快照联合渲染)与[验证记录](p6-03c3b-validation.md)：只读联合一至两块，按最终相机和绑定处理坐标/遮挡；审阅修正后 148 项 Python/CUDA、76 项工具和 8 个真实快照案例通过，最终全量复验及执行记录归档完成；Unicode 诊断问题后续补验关闭（149 项 Python/CUDA）。

46. [P6-03c3c1 在线联合渲染设计](superpowers/specs/2026-11-18-p6-03c3c1-online-joint-rendering-design.md)、[实施计划](superpowers/plans/2026-11-18-p6-03c3c1-online-joint-rendering.md)、[使用说明](online-gaussian.md#在线联合渲染c3c1)和[验证记录](p6-03c3c1-validation.md)：同 worker 只读联合最多两块、严格 owner 版本接纳和四对文件预览已接线；审阅修正后的71项核心、174项Python/CUDA、92项工具及Humble74项CTest通过；真实输入边界前52次双块贡献跨11组，归档清理完成，阶段保留低于1GiB。

47. [P6-03c3c2 空间候选与双ROS预览推进设计](superpowers/specs/2026-11-18-p6-03c3c2-spatial-rendering-design.md)、[实施计划](superpowers/plans/2026-11-18-p6-03c3c2a-spatial-rendering.md)和[验证记录](p6-03c3c2a-validation.md)：C3c1可靠性收尾及空间候选已实施，共用ROS入口与worker；双ROS矩阵和官方数据对照通过，审阅必要修复、全量复验及归档完成；最终55次双块贡献跨12组，其中24次非相邻。C3c2b图像发布单独推进。

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
普通205、Noetic225单元/8真实ROS、Humble241项及44个普通头通过；SDK和ament迁移消费通过。
两端完整172个关键帧各形成9组，四数组/发布及原有关键帧资产保持一致，短段各57旧产品逐字节一致。
实际副本的逐帧位姿修正改变组内几何且不修改原资产；使用与证据见[子地图说明](submap-dataset.md)和[P5-02报告](p5-02-validation.md)。

P5-03 已提供普通C++离线COLMAP导出和安装CLI，支持原始/优化位姿版本、居中去畸变图像与带颜色LiDAR种子。
普通230、Noetic250单元/8真实ROS、Humble266项及45个普通头通过；SDK/ament迁移消费与两端完整bag精确回归通过。
官方COLMAP/3DGS格式读取代码实际消费六组导出；使用、范围与未定因回放异常见[导出说明](colmap-export.md)和[P5-03报告](p5-03-validation.md)。

P5-04 已补齐历史图像写入/关闭错误，配置有界发布历史，修复两条视觉分配所有权路径；实测结果见[可靠性报告](p5-04-validation.md)。
两端各三次正常参考的完整1倍速回放严格通过，另两次启动变体由冻结旧库精确复现并单列保留。
原版负坐标视觉索引不一致导致重复建点和图像保留，本轮明确尚未修复；正坐标固定工作集连续处理不能证明一般地图内存有界。
整阶段独立评审无新增重要回归，1项模块启动Minor延期；完整历史bundle及恢复核对见报告。

P6-02c2a 已完成离线约束准入、鲁棒图试算及优化后几何复核；6 组 SDK、8 组 ROS 构建、8 次 bag 回放与两端离线工具通过验收。
完整回放对同平台 C1 严格一致；同一 172 帧数据集两端各产生 2 个提案，703 个输入文件保持不变。
使用方式、独立审阅修复和延期测试见[准入说明](loop-admission.md)与[验证报告](p6-02c2a-validation.md)。

P6-02c2b 已完成有界在线调度、持久接纳、停止排空和共用 ROS 入口。
完整官方回放中 Noetic/Humble 各接纳2次，原始前端结果保持同平台严格一致；两项重要审阅问题均有失败复现和修复验证。
用法见[在线接纳说明](online-loop-backend.md)，版本、构建矩阵和数据证据见[验证报告](p6-02c2b-validation.md)。

当前原版参考提交：`ca556e5ad6706ba20dc8dff91010faae546b7528`。
2026-10-11 的原版采集结果位于工作区 `validation/P0-noetic-20261011/`。
构建兼容性修复的采集结果位于 `validation/P0-build-fix-20261011/`。
验证产物在包外，迁移电脑时需要单独复制；bag 和编译产物不进入源码仓库。

48. [P6-03c3c2b 双ROS有版本预览](p6-03c3c2b-ros-preview.md)与[验证记录](p6-03c3c2b-validation.md)：同一main、共享转换、有界CPU发布已完成；74项核心、196项Python/CUDA、124项工具及Humble78项CTest通过；审阅11项必要修复、最终官方短段/完整bag发布OFF/ON对照和证据归档完成。

    后续补验覆盖全部空间 READY 的相机字段和镜像诊断，125 项工具测试通过。[接收证据补验](p6-03c3c2b-validation.md#接收证据补验2026-11-19)

49. P6-03c4a 已建立固定最终位姿下的留出视角画质基线。Retail_Street 的 25 个留出视角完成评估，23 个配对视角的汇总 PSNR 从 8.98 升至 10.95 dB。两项审阅问题修复后完成重复实测。[验证与复现](p6-03c4a-validation.md) · [设计](superpowers/specs/2026-11-19-p6-03c4a-held-out-rendering-design.md) · [实施计划](superpowers/plans/2026-11-19-p6-03c4a-held-out-rendering.md)

50. P6-03c4b 完成同种子预算的覆盖优先采样对照。Retail_Street 的 23 个双块留出视角汇总 PSNR 增加 0.657 dB，覆盖率增加 3.065 个百分点，共同深度 L1 增加 7.8 mm，满足在线试验筛选线。[验证与复现](p6-03c4b-validation.md) · [设计与实测依据](superpowers/specs/2026-11-22-p6-03c4b-coverage-guided-seeding-design.md) · [实施计划](superpowers/plans/2026-11-22-p6-03c4b-coverage-guided-seeding.md)

51. P6-03c4c 已接入 worker 的 `--seed-policy`。Retail_Street 的 Noetic 短段与完整双策略对照均通过。完整回放两侧的 13 块子地图全部持久化，162 个配对暖机视角的训练耗时差中位数为 +1.18 ms。[验证与复现](p6-03c4c-validation.md) · [设计与预检](superpowers/specs/2026-11-22-p6-03c4c-online-seeding-design.md) · [实施计划](superpowers/plans/2026-11-22-p6-03c4c-online-seeding.md)
