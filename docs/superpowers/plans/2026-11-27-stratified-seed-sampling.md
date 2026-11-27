# Stratified Seed Sampling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 验证固定1000点预算下二维分层分布的支持和画质取舍。
**Architecture:** 可选worker参数、纯NumPy分层器及现有训练/诊断/单变量比较入口；不改核心SLAM。
**Tech Stack:** Python/NumPy/PyTorch/gsplat、现有SDK。
**Spec:** docs/superpowers/specs/2026-11-27-stratified-seed-sampling-design.md

## Global Constraints

当前framework_completion分支，本地提交；缺省flat，uniform/resample/.5不变。
有效点精确min(1000,candidates)、窗口10/每块10000/每调用20步，留出隔离及最终位姿/选块固定。
四条件×双场景×seed0/1，共1880调用37600步；不复用权重、不新增依赖。
开始≥8GiB，研究≤6.5GiB，预留≥1GiB及准备空间；GPU顺序4096MiB/.4，OMP/BLAS1。

## Review Focus

- 稀疏/高度不均匀/细长/候选少于预算/余数容量必须保证唯一、精确预算和有效位置。
- 在线覆盖证据、实际点和诊断重放使用同分布；历史缺字段只代表flat。
- 新比较只允许flat→stratified；禁止其他控制项/类型/YAML别名变化或旧模式放行。
- 静态固定反馈代理与真实训练反馈分开解释，所有产物和来源强绑定、共同深度分母完整。
- 四条件完整前缀/原生选块/训练角色与资源预算均核验；不得因正结果改变默认或夸大泛化。

### Task 1: Optional stratified sampler and worker integration

**Files:** tools/3dgs_probe/{seed_sampling,worker_config,train,online_model,diagnose_gaussian_seeds,diagnose_depth_candidates,evaluate_gaussian_sampling}.py;
config/gaussian.yaml; tests/3dgs_probe/test_stratified_sampling.py.
**Interfaces:** stratified_pixels(pixels,count,shape); WorkerOptions.seed_distribution='flat';
_seed_pixels(...,seed_distribution='flat'); initialize_gaussians(...,seed_distribution='flat').

- [x] RED hand-calculated grid/quotas, skinny/sparse/saturation/invalid cases, flat equivalence and replacement positions; YAML valid/invalid/historical defaults.
- [x] Implement minimal NumPy allocator and parameter; same base distribution in coverage evidence/initializer/replay; normalize missing historical flat only.
- [x] Real CUDA zero-rate insertion/eviction, actual centers and prepass evidence; focused and full Python/CUDA expected no failures/skips; commit/task-done focused tests.

### Task 2: Controlled distribution comparisons

**Files:** tools/3dgs_probe/compare_gaussian_sampling.py; tests/3dgs_probe/test_compare_gaussian_sampling.py.
**Interfaces:** compare(...,allow_seed_distribution_change=False); mutually exclusive CLI flag;
comparison_scope='seed_distribution', seed_distributions={baseline:'flat',candidate:'stratified'}.

- [x] RED accepted distribution-only pairs for uniform/coverage, implicit flat, conflicting flags, other knobs/types/aliases, legacy/default/ratio modes rejecting true changes.
- [x] Implement typed normalized controls and labeled diagnostics without weakening existing guards; focused/full suite expected all pass/no skips; commit/task-done focused.

### Task 3: Frozen static support and sixteen-run study

**Files:** docs/stratified-seed-sampling.md; docs/engineering-roadmap.md; docs/depth-candidate-support.md;
local validation/stratified-seed-sampling-20261127.
**Interfaces:** existing evaluate_gaussian_sampling and compare CLI, frozen source/pose/config/selection receipts;
static support uses previous captured masks without training, real study uses natural per-condition feedback.

- [x] Freeze protocol and source/tool/runtime/software receipts before scoring; static support study on two queries; sequential 16 full-prefix runs, expected1880calls37600steps, disk within budget.
- [x] Compare each distribution pair and same-distribution uniform→coverage; independently audit all assets/configs/training lineage/quotas and recompute float metrics, eight-way depth intersection and coverage retention per scene.
- [x] Document positive/negative effects, radius/feedback/depth limits, resources/reproduction; commit/task-done audit.
- [ ] One fresh final read-only whole-change review; Important fixed once RED→GREEN/full suite, relevant evidence revalidated; archive own scratch and keep current branch.
