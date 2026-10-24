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
16. VIO 独立修复：[缓存均值](vio-cached-mean-fix.md)、[图像采样边界](vio-sampling-bounds-fix.md)。

这些文档描述目标设计。实际完成情况以源码、测试和验证报告为准；阶段表中的验收条件不代表已经通过。
当前已完成 P0 本机构建修复，以及 P1-01 的数学/状态隔离、直接测试、配对 bag 回归和独立审阅问题闭环。
P1-02 已完成普通标定类型、ROS1 参数入口、相机接线、直接测试、完整回放及独立审阅问题闭环。
P1-03 已完成内部测量类型、ROS1 转换、IMU 值输入、预处理独立构建、审阅问题闭环及最终完整回放；验证包括 78 项单元测试、2 项真实 ROS 回调、13 项 ASan 预处理、40 项启动检查和两组导出值零差异配对。
P2-01 已完成普通 C++ 同步器、ROS1 成对缓冲和独立审阅；98 项单元、5 项真实 ROS、40 项启动检查、两组导出值零差异对照及默认并行回放通过，另有 1 项延期的 IMU 回调补测建议。P2-02 已完成前端状态/处理所有权、ROS1 结果消费及审阅修复；106单元、9真实ROS、40启动、地图生命周期泄漏检查和最终完整回放通过，配对导出值零差异。P2-03 完成发布/I/O 分离及独立 writer：132 单元、11 真实 ROS、50 启动检查、精确 bag/导出对照通过。P3 已去除 vikit 并提供独立核心：150 单元、shared/static 核心各130项、12真实ROS、50启动、23公开头检查通过；三种安装迁移消费及外部catkin消费通过，136秒回放精确一致，57个导出文件逐字节一致。整阶段评审问题已闭环，无延期 Minor。下一阶段为 P4 ROS2 Humble 接入。

当前原版参考提交：`ca556e5ad6706ba20dc8dff91010faae546b7528`。
2026-10-11 的原版采集结果位于工作区 `validation/P0-noetic-20261011/`。
构建兼容性修复的采集结果位于 `validation/P0-build-fix-20261011/`。
验证产物在包外，迁移电脑时需要单独复制；bag 和编译产物不进入源码仓库。
