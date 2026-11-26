# Gaussian Seed Replacement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增加保持uniform位置的定额替换方式，并验证其相对旧补洞方法的覆盖/纹理/深度取舍。

**Architecture:** 在原WorkerOptions和初始化器上增加一个枚举，复用在线预渲染和训练。比较器只在显式模式允许这一枚举变化，兼容旧配置；固定预算实验验证实际效果。

**Tech Stack:** Python/NumPy/PyTorch/gsplat，既有原生准备器和选块器。

**Spec:** docs/superpowers/specs/2026-11-27-gaussian-seed-replacement-design.md

## Global Constraints

- 只修改livo-atlas，使用当前开发分支，默认uniform/resample/0.5不变。
- N种子中最多替换floor(N*ratio)个已覆盖位置，只加入尚未入选的空洞位置，保持总数与其余位置。
- 不改变窗口10、每观测1000上限、每次20步、损失/优化器/位姿/选块与留出隔离。
- YAML参数、真实计数证据、类型敏感比较；归档、旧记录和原子发布校验保持有效。
- 十次GPU顺序实验，不复制bag/模型；开始时检查总磁盘预算，保留至少1GiB空闲。

## Review Focus

- 全空洞/无空洞/不足空洞/不足可移除点/比例0与1/候选不足时不重复、不越界、保留未替换位置。
- 在线初始化和计数证据必须使用同一方式和比例；alpha覆盖并非纹理冗余证明。
- 方式比较只能改变一个枚举，比例/优化器/YAML类型变化、缺失归档必须拒绝。
- 历史配置缺字段只能补resample，不能把真实replace记录降级；默认及比例模式不能放行方式变化。
- 指标需使用共同查询/共同深度，位置保留不等于覆盖保留；单场景结果不得升级为默认或泛化结论。

### Task 1: Replacement sampler and worker evidence

**Files:** tools/3dgs_probe/{worker_config,train,online_model}.py; config/gaussian.yaml; tests/3dgs_probe/{test_worker_config,test_train,test_online_model}.py.

**Interfaces:** WorkerOptions.coverage_sampling='resample'; initialize_gaussians(...,coverage_sampling='resample'); _replacement_seed_count(...); replace-only uniform_uncovered/replaced_seeds evidence.

- [ ] Write failing tests for exact replacement positions/bounds/defaults and real CUDA initialization/evidence, including zero bypass.
- [ ] Observe RED; implement enum, deterministic set replacement and shared count helper.
- [ ] Run focused and full Python/CUDA suite, expected all pass.
- [ ] Commit implementation and ledger result.

### Task 2: Exact controlled method comparison and legacy records

**Files:** tools/3dgs_probe/{evaluate_gaussian_sampling,compare_gaussian_sampling,compare_gaussian_holdout}.py; tests/3dgs_probe sampling/holdout fixtures and tests.

**Interfaces:** compare(...,allow_coverage_sampling_change=False); mutually exclusive CLI flags; archived options supply only missing historical coverage_sampling='resample'.

- [ ] Add failing tests for allowed enum-only change, wrong ratio/optimizer/type rejection, default/ratio-mode rejection, replacement counts and old archived options.
- [ ] Observe RED; implement explicit method comparison and count validation without weakening existing guards.
- [ ] Run focused and full suite, expected all pass; commit and ledger result.

### Task 3: Ten-run paired study and documentation

**Files:** docs/gaussian-seed-replacement.md; docs/gaussian-coverage-ratio.md; docs/engineering-roadmap.md; local validation/gaussian-seed-replacement-20261127.

**Interfaces:** existing evaluator runs five conditions seeds0/1; same-ratio method comparison, zero-based ratio comparisons and independent ten-run audit.

- [ ] Run ten sequential complete-prefix experiments, expected150calls/3000steps/21queries/19joint each and unchanged budgets.
- [ ] Verify all source/pose/config/float artifacts, quotas/retention counts, ten-way common-depth metrics and worst-view diagnostics.
- [ ] Document positive or negative outcome and limitations; verify full suite and git diff --check; commit docs.
- [ ] Obtain one fresh read-only final review and fix Important/Critical findings via RED→GREEN plus full suite.
