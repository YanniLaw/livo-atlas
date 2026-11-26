# Gaussian Seed Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline. Steps use checkbox syntax.

**Goal:** 有界记录真实种子的纹理/历史投影支持及优化参数轨迹，完成双场景开发诊断。
**Architecture:** 独立离线工具复用原模型，通过工具内观测子类拦截真实预渲染/初始化；训练器不变。
**Tech Stack:** Python/NumPy/PyTorch/gsplat、既有原生准备器。
**Spec:** docs/superpowers/specs/2026-11-27-gaussian-seed-diagnostics-design.md

## Global Constraints

- 当前framework_completion分支，仅livo-atlas；在线模型、训练、配置默认和ROS接口不变。
- 每次一个不超过32帧的组、最多1000追踪种子及11个快照；64MiB输出、至少1GiB磁盘保留。
- 只使用train观测；无额外渲染或随机调用；稀疏深度缺失保持unknown。
- 一致阈值max(0.05m,2%参考深度)仅诊断；来源和数组摘要校验。
- Retail6/98、CBD0/6；resample0/25与replace25，各seed0/1；不做新默认或唯一因果结论。

## Review Focus

- 非单位相机、负z/越界、image无效、无稀疏深度时支持计数正确且不越界。
- 像素对应实际初始化顺序，未入选uniform不被当作实际被删除高斯。
- 无预渲染与空cohort不伪造覆盖/均值，目标驱逐/非train必须拒绝。
- 观测复制数组不改变原模型参数、优化器、训练调度或输入；输出预算与失败状态正确。
- 使用heldout选择代表组只构成开发诊断，多视图支持不等于真正贡献或唯一因果。

### Task 1: Bounded diagnostics tool

**Files:** tools/3dgs_probe/diagnose_gaussian_seeds.py; tests/3dgs_probe/test_diagnose_gaussian_seeds.py.
**Interfaces:** seed_features(view,pixels,history), cohort_summary(features,selected,uniform), parameter_changes(initial,current), TraceModel(device,tracked_id,options), run(args).

- [x] CPU failing tests: hand-computed gradient/projection/depth/color support, masks/behind camera, empty cohorts and parameter deltas.
- [x] CUDA failing tests: actual selected pixels/init mapping, owned snapshots, unchanged model/schedule, invalid target and output handling.
- [x] Observe RED; implement tool without changing production training/model files; focused tests expected OK.
- [x] Full Python/CUDA suite expected all pass, then commit and ledger completion.

### Task 2: Two-scene diagnosis and evidence

**Files:** docs/gaussian-seed-diagnostics.md; docs/engineering-roadmap.md; local validation/gaussian-seed-diagnostics-20261127.
**Interfaces:** Task1 CLI/NPZ/summary; reference map/split/config from prior Retail/CBD studies.

- [x] Freeze protocol and run12 sequential traces; expected real calls/20steps/retained target and aligned1000seed records each.
- [x] Independently rehash sources/config/artifacts and recompute feature/cohort/parameter statistics from NPZ; expected identical and all train IDs.
- [x] Document measured results and proxy/selection limits, run git diff --check; commit and ledger completion.
- [ ] One fresh read-only final review, fix Important/Critical via RED→GREEN and full suite; preserve branch and local commits.
