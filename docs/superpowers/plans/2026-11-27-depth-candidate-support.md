# Depth Candidate Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 对比全量训练候选与已选种子的空洞支持，隔离采样和窗口代理。
**Architecture:** 纯 CPU 离线诊断，复用冻结模型及原生准备/选块工具。
**Tech Stack:** Python、NumPy、现有 SDK。
**Spec:** docs/superpowers/specs/2026-11-27-depth-candidate-support-design.md

## Global Constraints

当前 framework_completion 分支、本地提交；不改训练器、默认值、核心或 ROS。
四份诊断 Retail97/CBD9 seed0/1，不新增训练/渲染。r0/2/4；未知深度单列。
两块、每块≤32主观测、64000点/投影块；64MiB输出，1GiB磁盘预留含准备临时空间。

## Review Focus

- 所有候选来自训练角色，原始资产/最终位姿/相机/选块/旧模型强绑定，不能误用 heldout。
- 非单位坐标、float32 初始化和分块投影保持一致；采样与保留是不同的嵌套对比。
- 分母固定、深度类别互斥、未知保持未知；不能把邻域计数当可见性或因果证明。
- 输入/原生运行库/输出/待发布报告在最后写入后重验；预算含临时空间。

### Task 1: CPU candidate support diagnostic

**Files:** tools/3dgs_probe/diagnose_depth_candidates.py; tests/3dgs_probe/test_depth_candidates.py.
**Interfaces:** candidate_points(view,camera)->float32 points; accumulate(points,camera,K,query)->radii maps;
candidate_summary(query,alpha,support)->fixed-denominator statistics;
CLI --diagnostic-directory --map-directory --keyframes --prepare-executable --selector-executable --output.

- [ ] RED tests: nonidentity backprojection/masks, >64000 points, hand-counted sampling/retention/unknown partitions and invalid subset rejection.
- [ ] Implement bounded CPU functions and CLI; integration fixture using existing source/selector formats, no training; tests for input mutation, budget and late artifact/report mutation preventing success.
- [ ] Focused tests and full Python/CUDA suite expected all pass/no skips; commit; task-done focused tests.

### Task 2: Four frozen analyses, audit and documentation

**Files:** docs/depth-candidate-support.md; docs/gaussian-coverage-diagnostics.md; docs/engineering-roadmap.md;
local validation/depth-candidate-support-20261127.
**Interfaces:** Task1 CLI, counts.npz, report.json and immutable previous four reports.

- [ ] Freeze protocol then run four sequential CPU analyses; expect original query/selection and zero new training/rendering.
- [ ] Independently reprepare sources, backproject candidates and recompute with bincount; verify all support maps, subset comparisons, statistics, receipts/budgets. Expected four passes.
- [ ] Document counts/denominators/radius sensitivity and scoped conclusions; commit; task-done independent audit.
- [ ] One fresh read-only final whole-change review; Important fixes once RED→GREEN/full suite; archive own ledger, preserve current branch/local commits.
