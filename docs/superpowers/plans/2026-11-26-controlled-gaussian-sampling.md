# Controlled Gaussian Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 固定观测与训练预算比较两种高斯采样策略，并输出可核验的空洞/交接视角诊断。

**Architecture:** 复用 train_group、冻结快照、原生空间选择和 float 样本评分。独立离线入口不改变在线工作流；对比器验证控制变量并重算指标。

**Tech Stack:** Python / NumPy / PyTorch / gsplat / 既有原生准备器与选块器。

**Spec:** docs/superpowers/specs/2026-11-26-controlled-gaussian-sampling-design.md

## Global Constraints

- 仅改 livo-atlas；按用户要求在当前 framework_completion 开发分支完成。
- 配置复用 config/gaussian.yaml；不改变 SLAM、默认采样、容量或优化器算法。
- 每观测 20 步、窗口 10、每观测最多 1000 个高斯；仅 train 观测训练。
- 验证输入及完整训练调度；输出新目录、预算与至少 1 GiB 余量；失败无成功报告。
- 两策略各两次、使用同一 172 帧快照和冻结选块计划；不复制 bag。

## Review Focus

- 预冻结角色与 canonical camera 外参必须匹配，留出数据不能参与训练。
- YAML 优化器设置与训练调度改变时，对比器必须拒绝归因比较。
- 不同选块、参考像素或篡改 float 样本不得静默比较。
- 无共同覆盖/无深度/无邻居需显式报告，不能用各自掩码冒充相同深度样本。
- 失败、中断或磁盘预算不足不得发布成功报告；图像误差固定量程并区分诊断与证明。

### Task 1: Fixed-budget runner

**Files:** Modify tools/3dgs_probe/evaluate_gaussian_submaps.py; Create tools/3dgs_probe/evaluate_gaussian_sampling.py; Create tests/3dgs_probe/test_evaluate_gaussian_sampling.py

**Interfaces:** Consumes load_evaluation_snapshot, EvaluationSelector, prepare_group, train_group; produces run(args) and report.json plus exact samples.

- [x] Write failing tests for frozen roles/group/query plan, canonical train admission, configurable training options, real CUDA schedule, budget and failure publication.
- [x] Run focused tests and observe missing behavior fail.
- [x] Implement runner; extend train_group with optional WorkerOptions, preserving legacy default.
- [x] Run focused tests, then the whole tests/3dgs_probe suite; expected all pass.
- [x] Commit runner with tests; ledger completion.

### Task 2: Verified comparison and visual diagnostics

**Files:** Create tools/3dgs_probe/compare_gaussian_sampling.py; Create tests/3dgs_probe/test_compare_gaussian_sampling.py

**Interfaces:** Consumes two Task 1 outputs; produces comparison report with shared-mask depth, per-view errors, diagnostics and matching budgets.

- [x] Write failing tests for altered schedules/config/selection/reference/artifacts, exact metric reconstruction, shared-mask depth, empty masks and worst-view PNG output.
- [x] Observe RED, implement bounded report/sample loading and comparison using existing measure and training checks.
- [x] Run focused and full Python/CUDA suite; expected all pass.
- [x] Commit comparator; ledger completion.

### Task 3: Controlled experiment and evidence

**Files:** Create docs/controlled-gaussian-sampling.md; Update docs/spatial-heldout-comparison.md and docs/engineering-roadmap.md; evidence under workspace validation/controlled-gaussian-sampling-20261126.

**Interfaces:** Runs Task 1 for uniform/coverage seeds 0/1, Task 2 for each pair; records commands, schedules, sample hashes, metrics and limits.

- [x] Run all four full frozen-snapshot experiments sequentially and audit 150 calls/3000 steps, common selection, poses and train-only lineage.
- [x] Compare both pairs and inspect worst-view diagnostic images; expected reports complete and all numerical controls match.
- [x] Document observed quality/depth/resource tradeoffs, single-scene limitation and one recommended next optimization.
- [x] Run final full Python/CUDA suite and git diff --check; expected pass.
- [x] Commit evidence guide and obtain one fresh read-only final review; fix Important/Critical findings via RED→GREEN and full suite.
