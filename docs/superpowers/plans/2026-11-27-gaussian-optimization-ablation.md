# Gaussian Optimization Ablation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 验证固定均值、固定尺度对replace25%覆盖/单块SSIM/共同深度的影响。
**Architecture:** 复用零学习率YAML和原训练器，扩展严格的比较验收模式；双场景同预算实验。
**Tech Stack:** Python/NumPy/PyTorch/gsplat及既有原生准备器/选块器。
**Spec:** docs/superpowers/specs/2026-11-27-gaussian-optimization-ablation-design.md

## Global Constraints

- 当前framework_completion分支，仅livo-atlas、本地提交；无训练器/在线默认/ROS/编译参数变化。
- 三条件只有一个means或log_scales正学习率变0；覆盖比例0.25和replace固定，全组从初始化冻结。
- Retail150调用3000步，CBD85调用1700步，各三条件两个seed；1GiB磁盘保留，GPU4096MiB/.4顺序执行。
- 相同观测/损失调度/最终位姿/参考选块，不固定逐调用种子ID；解释总效应与采样反馈。

## Review Focus

- 新比较模式仅接受means/log_scales，正数到0；两项同时变、反向/非零、其他优化器/YAML类型变化必须拒绝。
- 默认/比例/方式模式保持原限制与历史归档兼容，三种显式模式互斥，标签/报告真实反映干预。
- 零学习率跨插入与窗口驱逐仍保持初始化参数，其他参数允许更新，不能仅信任配置声明。
- 六次深度交集及无覆盖null、训练隔离/源和产物校验保持有效。
- 动态alpha改变种子选择属于总效应，不宣称固定种子纯效应或默认改进。

### Task 1: Strict optimizer intervention comparison

**Files:** tools/3dgs_probe/compare_gaussian_sampling.py; tests/3dgs_probe/test_compare_gaussian_sampling.py; tests/3dgs_probe/test_online_model.py.
**Interfaces:** compare(...,frozen_parameter=None); --frozen-parameter {means,log_scales}; comparison_scope=learning_rates.<name> and learning_rates baseline/candidate metadata.

- [x] Add failing tests for accepted one-rate freeze, invalid directions/names, extra ratio/method/other rate/YAML type changes, and mutually exclusive modes.
- [x] Verify existing zero learning rates via real CUDA across12 insertions/eviction and initialized values; this is characterization of existing support.
- [x] Observe comparison RED, implement minimal explicit mode; run focused and full suite, expected all pass; commit and ledger.

### Task 2: Two-scene full-prefix study

**Files:** docs/gaussian-optimization-ablation.md; docs/engineering-roadmap.md; local validation/gaussian-optimization-ablation-20261127.
**Interfaces:** existing evaluate_gaussian_sampling plus Task1 comparison mode; three conditions per scene/seeds0/1.

- [x] Freeze protocol/config snapshots;12 fresh sequential evaluations, expected unchanged calls/steps/selection/splits.
- [x] Run8 bounded real-data frozen-parameter witnesses and8 controlled comparisons; independently rehash/recompute inputs/metrics, per-scene six-way common depth and quotas/retention counts.
- [x] Inspect diagnostics and document measured effect including negative outcomes/seed-selection feedback; verify diff and commit.
- [x] One fresh read-only final review, Important fixes RED→GREEN and full suite; preserve local branch/worktree.
