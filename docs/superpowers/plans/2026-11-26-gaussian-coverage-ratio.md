# Gaussian Coverage Ratio Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从共享 YAML 控制空洞种子预留比例，并完成 0/25/50% 固定预算敏感性验证。

**Architecture:** 在现有初始化/在线模型中传递一个 WorkerOptions 数值，默认兼容原算法；比较器通过显式模式只放行比例这一控制变量。复用固定预算训练、原生选块与精确浮点评分。

**Tech Stack:** Python/NumPy/PyTorch/gsplat，现有原生准备器/空间选块器。

**Spec:** docs/superpowers/specs/2026-11-26-gaussian-coverage-ratio-design.md

## Global Constraints

- 仅修改 livo-atlas，使用当前 framework_completion 分支；默认 uniform / coverage比例0.5不变。
- 比例取有限 [0,1]，零值跳过覆盖预渲染；不改变窗口10、每观测上限1000和每次训练20步。
- 参数来自 config/gaussian.yaml；精确归档配置、输入、位姿、选块、完整训练调度。
- 六次全前缀实验，GPU顺序运行；不复制bag；输出新目录且保留至少1GiB空闲。

## Review Focus

- 零比例必须使用相同均匀初始化且没有覆盖预渲染开销；全空洞/小定额/比例1不越界或重复。
- YAML 拒绝非法比例；初始化和预渲染记录必须使用同一参数，旧默认记录仍可校验。
- 允许比例变化不能隐藏学习率、深度权重、其他 YAML 或原始配置摘要变化。
- 原始配置归档不得在 GPU 运行中变化而获成功认证，默认比较继续拒绝不同配置。
- 六次共同深度掩码、按像素/窗口聚合和资源统计须清楚，不能据开发验证集宣称泛化或新默认。

### Task 1: YAML ratio and sampler propagation

**Files:** Modify tools/3dgs_probe/{worker_config,train,online_model}.py and config/gaussian.yaml; tests/3dgs_probe/{test_train,test_online_model,test_worker_config}.py.

**Interfaces:** WorkerOptions.coverage_uncovered_ratio: float=.5; initialize_gaussians(..., uncovered_ratio=.5); _reserved_seed_count(..., uncovered_ratio=.5).

- [ ] Add failing tests for config validation/defaults, 0/.25/.5/1 exact seed arrays, zero-prepass CUDA behavior and nonzero quota propagation.
- [ ] Observe RED, implement minimal parameter propagation and comments.
- [ ] Run focused tests and full Python/CUDA suite plus worker config suite; expected all pass.
- [ ] Commit and ledger completion.

### Task 2: Controlled ratio comparison

**Files:** Modify tools/3dgs_probe/{evaluate_gaussian_sampling,compare_gaussian_sampling,compare_gaussian_holdout}.py; add tests to corresponding sampling/holdout tests and sampling_fixtures.py.

**Interfaces:** compare(..., allow_coverage_ratio_change=False); new archived config.yaml binds report config_sha256; explicit mode normalizes only coverage_uncovered_ratio.

- [ ] Add failing tests for strict default rejection, allowed ratio-only change, other-config/optimizer/tampered config rejection, zero and fractional seed evidence.
- [ ] Observe RED, implement archived config verification and ratio-aware legacy seed checks.
- [ ] Run focused and full Python/CUDA suite plus worker config suite; expected all pass.
- [ ] Commit and ledger completion.

### Task 3: Six-run sensitivity experiment

**Files:** Create docs/gaussian-coverage-ratio.md; update docs/controlled-gaussian-sampling.md and docs/engineering-roadmap.md; local evidence validation/gaussian-coverage-ratio-20261126.

**Interfaces:** run existing evaluator for coverage ratios0/.25/.5 seeds0/1; compare each nonzero against matching seed zero with explicit flag; independently audit all six.

- [ ] Execute six sequential runs; expected150calls/3000steps/21queries/19joint each, shared inputs/poses/selection/capacity.
- [ ] Recompute all metrics and six-way common depth, inspect worst-view maps; expected all proof checks pass.
- [ ] Record observed tradeoffs, resource scope and unchanged defaults; run final suites and git diff --check.
- [ ] Commit docs, obtain one fresh read-only final review, fix Important/Critical findings via RED→GREEN and green full suite.
