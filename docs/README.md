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
10. [P1-03 实测与复现](p1-03-validation.md)：图像/点云所有权、普通 IMU 输入、初始化修复及完整 bag 对照。
11. VIO 独立修复：[缓存均值](vio-cached-mean-fix.md)、[图像采样边界](vio-sampling-bounds-fix.md)。

这些文档描述目标设计。实际完成情况以源码、测试和验证报告为准；阶段表中的验收条件不代表已经通过。
当前已完成 P0 本机构建修复，以及 P1-01 的数学/状态隔离、直接测试、配对 bag 回归和独立审阅问题闭环。
P1-02 已完成普通标定类型、ROS1 参数入口、相机接线、直接测试、完整回放及独立审阅问题闭环。
P1-03 的内部测量类型、ROS1 转换、IMU 值输入、预处理独立构建及完整回放已完成，独立审阅待闭环。
完整前端与 ROS 接入层的隔离仍按架构阶段表推进，下一步是 P2 的同步器、前端调度及发布/I/O 分离。

当前原版参考提交：`ca556e5ad6706ba20dc8dff91010faae546b7528`。
2026-10-11 的原版采集结果位于工作区 `validation/P0-noetic-20261011/`。
构建兼容性修复的采集结果位于 `validation/P0-build-fix-20261011/`。
验证产物在包外，迁移电脑时需要单独复制；bag 和编译产物不进入源码仓库。
