# Gaussian Means Rate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 在同一replace25%预算下核验正均值学习率减半的质量/覆盖/共同深度取舍。
**Architecture:** 保留训练器，扩展严格只读单学习率比较；复用可验证旧档，四次新训练。
**Tech Stack:** Python、NumPy、PyTorch/gsplat及既有原生准备器/选块器。
**Spec:** docs/superpowers/specs/2026-11-27-gaussian-means-rate-design.md

## Global Constraints

- 当前framework_completion分支自主实施/本地提交；不改训练器/模型/在线默认/ROS/编译开关。
- means0.0005→0.00025，其余学习率不变，replace25%；Retail150调用3000步，CBD85调用1700步。
- 每场景seed0/1，四次新训练、八份旧原率/零率引用；同训练源与库版本/源资产强绑定。
- 新六次共同深度不得与旧交集混用；无覆盖null。总效应包含alpha选择反馈。
- 初始空闲8GiB、保留1GiB；GPU4096MiB/.4，OMP/BLAS1，训练顺序运行；不复制旧资产。

## Review Focus

- 正率减小与冻结模式严格分离，零/增大/相同/双参数及其他配置差异必须拒绝，模式互斥。
- YAML锚点只隔离指定路径，键和值类型及无关默认字段变化不能被掩盖。
- 旧档复用必须绑定实际代码/库/输入/身份，目录名不能充当seed或条件的证据。
- 深度共同像素、空集null、完整训练隔离及替换/驻留证据必须重算，不能只复制汇总。
- 新比较的全部PNG摘要与发布结果完整；不挑选重复/视图，不由负结果修改预注册率或默认。

### Task 1: Strict reduced-positive-rate comparison

**Files:** tools/3dgs_probe/compare_gaussian_sampling.py; tests/3dgs_probe/test_compare_gaussian_sampling.py.
**Interfaces:** compare(...,reduced_learning_rate=None); --reduced-learning-rate means|log_scales;
comparison_scope=learning_rates.<name>, learning_rates.parameter/baseline/candidate.

- [x] Add RED tests for both positive-rate fields and CLI acceptance; reject zero/equal/increase/names/mode conflicts, second rate/config changes, YAML aliases/key types/default spelling; default/freeze modes reject positive-rate interventions.
- [x] Implement shared one-rate verification with distinct freeze/decrease validation and accurate labels; preserve old outputs.
- [x] Run focused and full Python/CUDA suite, expected all pass, no skips; re-certify genuine old method/freeze comparisons byte-identical; commit and ledger.

### Task 2: Four fresh evaluations with verified reused controls

**Files:** docs/gaussian-means-rate.md; docs/engineering-roadmap.md; docs/gaussian-optimization-ablation.md;
local validation/gaussian-means-rate-20261127.
**Interfaces:** original evaluator and Task1 comparison; protocol.json maps original full/zero archives plus new half-rate runs.

- [x] Freeze protocol/config/source/old-report fingerprints before training; verify reuse admission; run four new evaluations and four comparisons. Expected calls/steps/splits/selection unchanged.
- [x] Independent audit of all12 full/half/zero archives, raw metrics, per-scene six-way depth, training IDs/quotas, identities and all comparison PNGs. Expected succeeded; no-common depthnull.
- [x] Inspect diagnostic figures, record positive or negative results and resources/reproduction without default change; verify diff, commit and ledger.
- [x] One fresh whole-change final read-only review; fix Important issues once RED→GREEN/full suite, revalidate relevant artifacts; preserve branch/local commits.
