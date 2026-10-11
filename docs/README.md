# livo-atlas 重构文档

按下面的顺序阅读、实施：

1. [目标架构与阶段验收](superpowers/specs/2026-10-11-refactor-architecture-design.md)：重构范围、模块边界、迁移顺序及未来 SLAM/3DGS 接口。
2. [P1-01 执行计划](superpowers/plans/2026-10-11-p1-01-core-state.md)：数学与状态隔离的文件边界、兼容接口及数值验收。
3. [ROS1 基线复现](validation.md)：在另一台 Noetic 电脑上创建独立构建副本、回放 bag、保存证据。
4. [C++ 编码约定](cpp-style.md)：Google 风格、依赖管理和控制实现规模的约束。
5. [P0 实测记录](p0-baseline.md)：原版问题、构建修复、已运行的检查及数值差异。

这些文档描述目标设计。实际完成情况以源码、测试和验证报告为准；阶段表中的验收条件不代表已经通过。
当前已完成 P0 的本机构建修复与正常退出验证，P1 的结构迁移尚未实施。

当前原版参考提交：`ca556e5ad6706ba20dc8dff91010faae546b7528`。
2026-10-11 的原版采集结果位于工作区 `validation/P0-noetic-20261011/`。
构建兼容性修复的采集结果位于 `validation/P0-build-fix-20261011/`。
验证产物在包外，迁移电脑时需要单独复制；bag 和编译产物不进入源码仓库。
