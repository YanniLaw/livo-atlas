# Common Coverage Texture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline.

**Goal:** 固定覆盖区域比较纹理误差，分解覆盖周转，并补齐来源复核。
**Architecture:** 新增纯NumPy度量、通用归档验证和CPU CLI；既有评分/训练源码不变。
**Tech Stack:** Python/NumPy、既有原生快照读取器；无新增依赖。
**Spec:** docs/superpowers/specs/2026-11-27-common-texture-design.md

## Global Constraints

当前framework_completion、本地提交；不训练/渲染/改默认、不动vikit或C++/ROS。
每场景八份共同覆盖alpha>=.05，四条件×seed0/1；最多2场景/32查询/100万像素。
四邻域RGB差异RMS阈值.02、strong/weak/unknown；SSIM完整11×11区域窗口按中心参考梯度分组。
SSE/L1与像素四类可加，PSNR不可加；空/完美值显式null及原因。
输出<=64MiB、磁盘预留>=1GiB；旧研究数据只读，旧验证脚本备份后增强。

## Review Focus

- 窗口有效性和中心梯度分组不能混淆，零窗口/未知梯度不能伪装成零误差或有效SSIM。
- 四类/梯度类别必须分割固定分母，误差贡献与全图一致，不能合计dB或混用每对交集。
- 每次split及静态上游全部产物须独立复查，报告字节、当前样本和来源链不能错绑。
- 输入变化、JSON/数组超预算及发布后段篡改必须拒绝成功报告，最后一次检查紧邻rename。
- 诊断相关性与因果/泛化分开，旧全图评分与新all-region统计一致，默认及训练源码保持不变。

### Task 1: CPU metric, archive verifier and CLI

**Files:** tools/3dgs_probe/{texture_metrics,study_verification,diagnose_texture}.py;
tests/3dgs_probe/{test_texture_metrics,test_texture_diagnostic}.py.
**Interfaces:** gradient_masks(rgb,image_mask)->three masks;
texture_metrics(reference,rgb,regions,gradients)->region/cohort sufficient statistics;
coverage_partition(reference,a,b,image_mask)->four masks/RGB statistics and contributions;
aggregate_stats(rows)->pooled statistics;
verify_splits_and_static(study,coverage,candidates)->protocol/receipts;
load_study_inputs(...)->runs/snapshots/receipts; verify_inputs(inputs)->None;
CLI --study-directory --coverage-directory --candidate-directory --output.

- [ ] Write RED metric tests with hand-counted 1×4 four-state error sums, 13×13 gradient/mask counts, naive 2D-window SSIM, complete-window/center-bin and null/perfect cases; implement minimal metrics and observe GREEN.
- [ ] Write RED archive tests for per-run split, capture NPZ/PNG/config/poses, candidate counts/receipt chain mutation using real small frozen fixtures; implement bounded same-byte JSON/receipt checks.
- [ ] Write RED CPU CLI tests for fair common masks, source metric identity, sufficient-stat aggregation, input/output overlap/budget and late source/metrics/pending mutation; implement streaming aggregation/atomic publication.
- [ ] Focused tests plus full Python/CUDA expected pass/no skips; commit; task-done focused metric+archive discovery.

### Task 2: Frozen sixteen-result analysis and independent audit

**Files:** docs/common-coverage-texture.md; docs/engineering-roadmap.md; docs/stratified-seed-sampling.md;
local validation/common-texture-20261127; existing local stratified verify_study.py (backed up before augmentation).
**Interfaces:** Task1 source helper/CLI, metrics.json/report.json; same archived16 runs and static ancestors, no new models.

- [ ] Freeze methods/input/implementation fingerprints before scoring; append generic missing checks to old verifier and run it; analyze all16 source results, expected0 new training/render calls and bounded output.
- [ ] Independently recompute reference gradients, four-state sums, eight-way coverage and SSIM via sliding-window weighted sums; check all raw metrics, counts, masks, source/outputs/budgets; preserve null cases. Expected all16 sources and8 distribution pairs consistent.
- [ ] Inspect fixed-query overlays, document weighted decompositions/texture-cohort counts/radius-independent limits and reproduction; commit; task-done fresh evidence verifier.

## Final review and finish

One fresh read-only whole-change review; Important fixes in one RED→GREEN/full-suite pass, relevant evidence
revalidated, no rereview. Record all rulings/deferred minors; archive only this plan's scratch, keep current branch.
