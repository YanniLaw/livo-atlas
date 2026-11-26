# 留出边界与运行证据实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans. Work inline with one final fresh read-only reviewer.

**Goal:** 完成上阶段遗留的双语言边界一致性及每查询运行证据。
**Architecture:** 保留现有协议和单worker，只补充验证与报告字段。
**Tech Stack:** C++17/OpenCV、Python/NumPy/Torch/gsplat、Noetic/Humble。
**Spec:** ../specs/2026-11-26-evaluation-boundaries-and-evidence-design.md

## Global Constraints

仅修改livo-atlas；Google C++风格；函数之间空一行；不引入Pimpl或新依赖。
在已有授权的framework_completion执行，本地提交，不推送/合并。

## Review Focus

- 齐次行四个分量及浮点边界必须一致验收，不能扩大旋转/身份容差。
- 同面积不同尺寸仍须拒绝；只在spatial要求与retained尺寸一致。
- 推理计时/峰值不得额外重置CUDA状态或改变训练随机数。
- joint缺省应明确为空，错误查询不能生成成功报告。
- 每份报告仍受已有SHA回执及报告大小/磁盘限制保护。

## Task 1：双语言协议边界

**Files:** src/gaussian/evaluation_protocol.cc, tools/3dgs_probe/evaluation_protocol.py,
tests/3dgs_probe/test_evaluation_protocol.py。
**Interfaces:** ValidateGaussianEvaluationMessage / validate_evaluation_message保持原签名。

- [x] 添加测试：齐次行各分量±5e-10通过、±2e-9拒绝；spatial尺寸4x12/4x6与8x6不匹配拒绝。
- [x] 原始socket发送同一header/body，比较两种校验器；先观察RED。
- [x] C++采用1e-9齐次行容差，双方增加spatial逐维尺寸校验，再GREEN。
- [x] 运行协议相关回归和SDK82项测试。

## Task 2：报告证据及交付

**Files:** tools/3dgs_probe/online_evaluation.py, tests/3dgs_probe/test_online_evaluation.py,
docs/gaussian-online-evaluation.md。
**Interfaces:** 报告增加software、inference.memory_peak_scope、inference.single/joint；RPC不变。

- [x] 添加实际CUDA评分测试，要求软件字段、真实timings/峰值、累计scope和缺省joint；观察RED。
- [x] 复用已有infer_blocks返回值，单/双块分别记录，不调用reset_peak_memory_stats。
- [x] 对双块路径检查记录，训练状态/RNG保持原值；报告SHA和预算原有测试继续通过。
- [x] 完整Python/CUDA回归、双ROS编译、单次独立审查；修复重要问题并补回归。
- [x] 更新复现文档和证据、本地提交，保留分支。
