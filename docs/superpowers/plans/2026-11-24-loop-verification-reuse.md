# 回环验证内部剖析与局部复用

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 根据内部计时减少已确认的重复计算，保持所有验证判定与数值结果。
**Architecture:** 通过内部入口传递已有 stage_timing 开关，公共 SDK 入口保持原签名。计时划分准备、正反向 GICP 和正反向质量计算。复用限定在一次候选/一次质量计算中，不跨任务共享可变结构。
**Spec:** docs/loop-stage-timing.md 的下一步；用户已授权当前会话直接实施和本地提交。

## 约束与复核重点

Google C++ 风格，函数间空行，不引入 Pimpl、新功能宏或参数。不放松任何质量阈值。
覆盖缓存命中、失败查询、重复最近邻、空点集和并发候选；保持 float 搜索坐标与 reciprocal 判定。
时间相互包含时说明范围；计时关闭时不额外读时钟。公共 VerifyLoop/EvaluateLoopGeometry 入口不变。

### Task 1: 内部计时与实测

- [x] 测试已有开关能控制新增细分计时，先观察失败。
- [x] 修改 loop_internal.h、loop_registration.cc、loop_quality.cc、loop_query.cc、loop_admission.cc，传递内部计时参数，保持公开接口。运行相关 SDK 测试。
- [x] 完整 bag 定位耗时，记录准备、正反配准及质量计算，核对资产与候选诊断。

### Task 2: 数据决定的局部复用

- [x] 按实测选择一个重复计算来源。优先检查质量计算中同一目标点的反向最近邻是否重复查询；若配准占主导，核对 PCL 协方差复用。记录选择及代价。
- [x] 先添加解析场景或调用计数的失败用例，再实现局部复用；不采用近似搜索或修改坐标计算顺序。
- [x] SDK、Noetic、Humble 相关测试与完整 bag 配对验证，记录耗时与内存；独立复核后本地提交。

证据目录 validation/loop-verification-reuse-20261124。沿用当前分支和现有构建目录。完整 bag 只能代表当前短序列。

结果见 [验证记录](../../loop-verification-reuse.md)。公开 API、匹配参数和质量阈值保持，源码复核无重要问题。
