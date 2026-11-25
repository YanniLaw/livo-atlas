# 几何准备与位姿交付延迟 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 完成用户批准的下一步：分析已有四次无探针回放，核查缓存和版本失效，补齐优化位姿交给 LivoSystem 的完整延迟。

**Architecture:** 复用现有 diagnostics.stage_timing、GeometryCacheStats 与 PoseGraph 发布/回执边界；保持串行工作队列与候选依赖图。

**Tech Stack:** C++17、GLOG、GTest、Noetic / Humble、Python 标准库分析脚本。

**Spec:** [本轮方案与结果](../../geometry-delivery-timing.md)，基于用户已批准的分析、必要小改动与完整交付延迟核查。

## Global Constraints

只修改 livo-atlas，沿用当前开发分支；不新增功能编译宏、运行参数、公共 SDK 字段或归档字段。
保持匹配参数、点序、数值行为、缓存驻留预算、异常传播与停止顺序。
关闭阶段计时时，新路径不读时钟、不获取额外缓存统计、不打印诊断。
遵循 Google C++，函数定义之间空一行；不使用 Pimpl，不引入共享缓存或新的线程。
沿用本地测试与提交授权；不推送、不合并；Humble 容器恢复原来的停止状态。

## Review Focus

普通关键帧、封口、外部位姿更新不能冒充回环位姿交付。
回执前回调必须已完成，回调抛异常不能产生成功交付指标。
时间戳必须随发布保存并在回执时消费，不能读取下一次查询的派发时间。
关闭开关不新增时钟或统计开销；跨线程访问遵循已有 mutex 保护。
几何读取、构建、快照互相包含，缓存请求次数不可当成构建次数；短序列的小样本尾部不作总体推断。

### Task 1: 归因与诊断覆盖

Files: docs/geometry-delivery-timing.md、src/loop_closure/loop_query.cc、tests/backend/loop_task_test.cc。

- [x] 汇总已有四次完整回放的各阶段累计与 P50/P95/P99、前端延迟，核对31次构建/51次读取。Expected: 不混加父子阶段，结论与原日志一致。
- [x] 先补缓存诊断开关测试，要求成功读取报告 submap、pose_version、reader_reset、built、cloud_reads、evictions、resident_submaps。Expected: 旧代码缺少诊断而失败。
- [x] 只在计时开启时采集 GetSubmap 前后的现有统计并记录每次读取，不改变缓存算法。Expected: 测试通过，关闭时无新增日志，既有数值/候选测试保持一致。

Interfaces: 成功的 MakeGeometryReader 调用输出一条 loop_geometry_cache 日志；已有 loop_geometry_read 等计时语义保持不变。

### Task 2: 优化位姿交付边界与验证

Files: src/pose_graph/pose_graph.h、src/pose_graph/pose_graph.cc、tests/core/loop_backend_test.cc、docs/loop-stage-timing.md、docs/engineering-roadmap.md。

- [x] 使用真实 LivoSystem/LoopBackend 回归测试计时开关、回调内尚未交付、成功回执只记录一次、拒绝或失败回调无成功记录。Expected: 新指标在旧实现上缺失而失败。
- [x] 为私有 Publish 增加可选回环派发时间；仅接纳的回环提交携带它，mutex 保护下保存/消费。回执时在恢复队列前采样，解锁后记录 loop_pose_delivery_latency 和 loop_pose_publication_wait。Expected: 覆盖派发至 owner 应用及回调完成，不包含 ROS/GPU 输出后的消费；普通提交不产生样本。
- [x] SDK 全套、Noetic/Humble 相关测试，完整 Retail_Street 回放一次。Expected: 原始资产、数学诊断与归档保持一致，13次查询中仅2次接纳产生两个新指标；缓存日志精确解释读取与构建。
- [x] 独立源码复核、保存分析脚本和可复现文档，本地提交。Expected: 无重要未解决问题；若没有明确收益证据，本轮不追加缓存优化。

Interfaces: 两任务仅共享现有 stage_timing 开关，无数据依赖；新指标是包含关系，不与查询计算相加。证据目录 validation/geometry-delivery-20261126。

结果：两类缺失诊断测试均先失败后通过。独立复核的派发时间边界问题由250毫秒提交延迟探针 RED→GREEN 后修复；新运行缺失诊断的验证器也先复现误通过再修正。SDK76/76、Noetic4/4、Humble6/6，最终完整回放数值/资产一致、归档consistent；51次读取31次构建20命中，3块驻留；两次 owner 完整延迟673.632/793.195毫秒。无查询内重复构建证据，不追加缓存改造。见 [最终结果](../../geometry-delivery-timing.md)。
