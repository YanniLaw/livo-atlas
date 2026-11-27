# Stratified Additions Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for inline execution.

**Goal:** 只改变反馈替换时补入点的分布，并验证两个场景的覆盖与纹理取舍。

**Architecture:** 在现有采样器增加一个 YAML 取值，复用分层函数、受控比较和共同区域度量。

**Tech Stack:** Python、NumPy、现有 CUDA worker 与原生留出准备工具。

**Spec:** docs/superpowers/specs/2026-11-27-stratified-additions-design.md

## Global Constraints

现有开发分支，仅本地提交。静态重放先于正式编码和训练。
使用 flat/全量分层的可信归档与相同冻结参考。4 次新增训练、470 调用、9400 步。
输出上限 2 GiB，保留 1 GiB，GPU 空闲至少 4096 MiB，分配器比例 0.4。
既有默认值、C++/ROS、优化器及历史归档保持原样。

## Review Focus

- 同一掩码下补点分层不能改变基底、移除 ID、q 或有效像素约束。
- 零反馈、零配额和小候选集的 fallback，YAML 与直接入口的 replace 约束。
- 比较器接纳真实的新模式，拒绝无效策略、反向对照与额外设置差异。
- 隔离的六组共同掩码、完整窗口、深度分母及历史归档的身份正确。
- 从冻结反馈代理和真实前缀分别解释观察，在线回放采用预先记录的进入条件。

### Task 1: Replay and integrate the sampler

**Files:** tools/3dgs_probe/{train,worker_config,compare_gaussian_sampling}.py;
tests/3dgs_probe/{test_stratified_sampling,test_compare_gaussian_sampling}.py;
local validation/stratified-additions-20261127/replay_additions.py.

**Interfaces:** 验证 `seed_distribution=stratified_additions` 仅适用于 replace。
`_seed_pixels(mask, limit, uncovered_mask, uncovered_ratio, coverage_sampling, seed_distribution)`
维持接口。比较器复用 `allow_seed_distribution_change=True`。

- [ ] 从现成四份捕获和原生视图，使用旧独立采样器计算期望并保存重放证据。预期 76 批，旧基准像素和投影支持一致，新条件移除位置与数量一致。
- [ ] 给 4×4 固定空洞例写断言，旧 flat 选 [0,1,2,12]，新条件选 [0,1,3,12]；覆盖无反馈和候选不足、合法 YAML、拒绝 resample。比较器用真实小归档验证 coverage 接纳、uniform 拒绝。观察 RED。
- [ ] 向现有取值列表与 replace 的补入分支加最小改动，并扩充受控比较的取值判断和诊断图标签。
- [ ] 跑专项并在 76 批真实重放核对产品输出，预期逐像素相同。完整 Python/CUDA 回归后提交。

### Task 2: Evaluate four runs and record results

**Files:** local validation/stratified-additions-20261127/{run_study,audit_results}.py;
docs/stratified-additions.md、docs/engineering-roadmap.md、docs/common-coverage-texture.md.

**Interfaces:** 使用冻结的四条件研究路径、采样评分 CLI、比较器、共同区域度量。
新条件归档命名 `{Retail|CBD}-stratified_additions-coverage-{0|1}`。

- [ ] 锁定八份对照收据、实验配置、工具和比较口径。串行运行四次评分并保存 primary flat 对照报告。预期 4 次成功、470 调用/9400 步，输出范围内。
- [ ] 从 float 样本重查新报告和比较控制，按查询生成六组固定交集重算纹理及深度。输出分种子统计与误差，核对四类覆盖周转可重构全图。
- [ ] 若达到预定进入条件，用 Retail 相同的 10 秒前缀做 flat 与 stratified_additions 的 Noetic 短段回放。验证 worker 使用 coverage/replace、有实际替换并持久封口。
- [ ] 记录两模式的结果、预定回放条件是否满足、资源与复现命令。验证最终证据后本地提交。

## Final Review

一次只读整阶段审查。发现重要问题时用一个修复回合加实际回归验证。结束时保留当前分支，
归档本计划执行记录。
