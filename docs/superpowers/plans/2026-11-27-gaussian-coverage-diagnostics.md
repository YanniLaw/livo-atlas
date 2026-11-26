# Gaussian Coverage Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 用实际初始/保留/训练后中心和渲染解释局部与联合空洞的观测关系。
**Architecture:** 纯CPU投影统计加有界离线观察CLI，复用原生准备器/选择器和现有模型，不改训练器。
**Tech Stack:** Python、NumPy、PyTorch/gsplat及既有SDK工具。
**Spec:** docs/superpowers/specs/2026-11-27-gaussian-coverage-diagnostics-design.md

## Global Constraints

- 当前framework_completion分支/本地提交；不改原训练器/算法/默认YAML/ROS/编译参数。
- Retail97选5/6(27调用540步)，CBD9选0(11调用220步)，seed0/1，四次共76调用1520步。
- 原率coverage/replace25%，每帧1000/窗口10/每块10000/每调用20步；两块各最多32观测。
- 半径0/2/4像素，z>1e-6，深度容差max(.05m,.02z)；未知深度单列；中心邻域不等于高斯足迹。
- 新输出<=min(64MiB,YAML预算)，保留>=1GiB并计准备临时空间；GPU4096MiB/.4，OMP/BLAS1顺序。

## Review Focus

- 非单位相机、半像素边缘/画幅外近邻、遮挡代理与无标签必须正确，不能膨胀深度类别跨表面。
- 保留初始参数逐点对应origin，观察训练不能改变种子或训练调度，窗口驱逐保持10000上限。
- heldout/query角色、原生空间选择/最终位姿与标定/源资产强绑定，单块与联合不能混用。
- 参数/数组/图像预算提前计算，写入失败或空间下降不得发布成功报告，所有输出和源需重验。
- 互斥分类和覆盖迁移分母必须完整；开发查询/稀疏参考不能作唯一因果或全图瓶颈结论。

### Task 1: Bounded coverage diagnostic tool

**Files:** tools/3dgs_probe/gaussian_coverage.py; tools/3dgs_probe/diagnose_gaussian_coverage.py;
tests/3dgs_probe/test_gaussian_coverage.py.
**Interfaces:** projected_support(points,camera_from_points,K,query,radius)->count maps;
coverage_summary(query,initial,trained,support)->JSON statistics;
CaptureModel(OnlineGaussianModel), train_observed_group(...); CLI standard map/keyframes/prepare/selector/config/keyframe-id/random-seed/output.

- [ ] Add RED CPU tests with hand-computed nonidentity/border/radius/front/behind/mixed/unknown counts and coverage partitions; reject invalid shapes/cameras/points/radius.
- [ ] Implement minimal bounded projection/statistics, then observation class/CLI. Add CUDA equivalence againsttrain_group through12 insertions/eviction; actual initial/retained correspondence, same lossIDs/steps, no heldout training.
- [ ] Add CLI tests for heldout/selection/source constraints and budget/partial publication; run focused and full Python/CUDA, expected all pass/no skips; commit and ledger.

### Task 2: Two-query real-data diagnosis and audit

**Files:** docs/gaussian-coverage-diagnostics.md; docs/engineering-roadmap.md; docs/gaussian-means-rate.md;
local validation/gaussian-coverage-diagnostics-20261127.
**Interfaces:** Task1 CLI with frozen query97/9, seeds0/1, source full-rate sceneYAML; split/poses/NPZ/report/PNGs.

- [ ] Freeze protocol/config/source fingerprints before four sequential runs; expected six trained submaps/76calls1520steps, exact selected IDs, no default change.
- [ ] Independently prepare raw sensor views and reconstruct actual selected seeds, final origin/window counts, camera transforms and per-pixel densities/statistics; re-render saved parameters on GPU, verify all receipts/budgets/source bindings. Expected all pass and null/unknown remains explicit.
- [ ] Inspect overlays, document positive/negative observations, denominators, radius sensitivity, retention proxy and local/joint limits; save reproduction/size and commit.
- [ ] One fresh whole-change final read-only review; fix Important once RED→GREEN/full suite and revalidate relevant experiments; archive own scratch, preserve branch/local commits.
