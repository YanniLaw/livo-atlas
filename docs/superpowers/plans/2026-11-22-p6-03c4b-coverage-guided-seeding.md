# C4b Coverage-Guided Seeding Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans in the current session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在相同种子预算下比较均匀采样与覆盖优先采样对留出视角的影响。

**Architecture:** 在线模型的训练函数接受可选采样策略，C4a 离线工具显式选用。一次预渲染提供
待补区域，比较工具用两个运行的稀疏深度记录复算共同像素误差。

**Tech Stack:** 现有 C++17 SDK 准备工具、Python、NumPy、PyTorch、gsplat。

**Spec:** `docs/superpowers/specs/2026-11-22-p6-03c4b-coverage-guided-seeding-design.md`。

## Global Constraints

- 只改 livo-atlas。实验参数位于既有独立评估入口，默认 `uniform`。
- 固定原封口快照、训练/留出划分、3 cm 初始尺度、0.5 初始不透明度、10 帧窗口和每帧 20 步。
- 每帧最多一次预渲染及 1,000 个种子，每块最多 10,000 个高斯，单次双块最多 20,000 个高斯。
- 每帧最多 1M 像素，顺序使用一个 GPU 模型，`OMP_NUM_THREADS=1`、`OMP_THREAD_LIMIT=1`、`OPENBLAS_NUM_THREADS=1`。
- 留出观测专用于评分。使用新输出目录和 C4a 的只读封口输入。

## Review Focus

- 半数配额遇到少量候选、空集、全覆盖、全未覆盖或上限大于候选数时的采样边界。
- 预渲染看到的参数与实际插入后留下的参数一致。失败前的模型可完整验证。
- C4a 默认入口和训练历史兼容，变更策略后实际高斯总数保持一致。
- 同一留出 ID 的相机、帧尺寸或稀疏标签不同时比较拒绝。缺失、空集及坏的记录可明确诊断。
- 汇总 PSNR 按有效像素重新加权，共同深度误差按共同样本数加权。首块只参与单块评分。

### Task 1: 有界覆盖优先采样

**Files:** `tools/3dgs_probe/train.py`、`tools/3dgs_probe/online_model.py`、
`tests/3dgs_probe/test_train.py`、`tests/3dgs_probe/test_online_model.py`。

**Interfaces:** `initialize_gaussians(train_views, max_per_view, *, uncovered_mask=None)`。
`uncovered_mask` 为当前单训练视角的 `bool[H,W]` 掩码，取样与有效深度掩码相交。
`OnlineGaussianModel.train_candidate(self, view, *, seed_policy='uniform')`。
`coverage` 时 `metrics['seed_selection']` 记录 `prepass_seconds`、`depth_candidates`、
`uncovered_candidates` 和 `reserved_uncovered`。跳过预渲染时耗时和未覆盖候选数为 `null`，
采用均匀回退时预留数为 0。

- [x] 加入失败测试。构造 3×4 掩码，有效展平索引 `[0,1,2,3,4,6,7,10,11]`、待补 `[6,7,11]`、
  上限 4 时，均匀选 `[0,2,4,7]`，覆盖优先选 `[0,3,6,7]`。检查索引唯一、规模相同、回退与
  旧坐标/颜色计算一致。验证非法掩码或策略被拒绝。
- [x] GPU 用例验证空模型、种子数足以用尽候选时跳过预渲染。构造满窗口，使最早来源帧的高斯
  独占一个区域，确认预渲染的参数集合排除该帧，实际淘汰继续由 `_append` 完成。故意触发
  预渲染失败，核对来源、窗口和参数未发生突变。
- [x] 核对 RED。实现确定性预算采样与训练前的无梯度预渲染。先验证策略及输入，在 `_append`
  之前生成种子。CPU 仅接收本帧 alpha。既有 `train_candidate(view)` 的返回格式沿用原路径。
- [x] 在固定环境运行 `"$LIVO_PYTHON" -m unittest -v test_train test_online_model`，期望 GREEN。
  本机复用 C4a 的工具环境和 CUDA 夹具，保持 `PYTHONPATH` 含 `tools/3dgs_probe` 与 `tests/3dgs_probe`。

### Task 2: 留出实验与共同像素比较

**Files:** `tools/3dgs_probe/evaluate_gaussian_submaps.py`、
`tests/3dgs_probe/test_evaluate_gaussian_submaps.py`，新增
`tools/3dgs_probe/compare_gaussian_holdout.py` 和相应定向测试。

**Interfaces:** `evaluate_gaussian_submaps.py` 增加 `--seed-policy {uniform,coverage}` 和
`--save-depth-samples`。`train_group(..., *, seed_policy='uniform')` 传至任务 1。
启用深度记录时，每个查询写 `views/<ID>/depth-samples.npy`，在对应查询报告写入
`depth_samples_sha256`。一行表示一个有效 LiDAR 像素，按展平索引升序，结构化 dtype 为
`pixel_index:<u4, observed_z_m:<f4, single_z_m:<f4, joint_z_m:<f4, single_covered:?, joint_covered:?`。
预测 z 在未覆盖处置零，首组的联合列全零且覆盖标志为假。记录长度为有效深度像素数，可为零。
`compare_gaussian_holdout.py --baseline DIR --candidate DIR --output NEW_JSON` 读取两组报告与记录。

- [x] 写失败测试。固定同一查询相机和 4 个 LiDAR 像素，基线覆盖 `[0,1]`，候选覆盖 `[1,2]`。
  共同像素为 `[1]`，使用该像素的两侧误差计算共同 L1。分别核对新覆盖、丢失、均未覆盖
  的计数各为 1，且空交集报告 `null`。试验错误相机、标签、索引、重复 ID 或损坏记录的拒绝行为。
- [x] 核对 RED。按报告身份和实际 `split.json` 哈希核对划分、实际监督计划、最终位姿、查询集合
  与相机。读取深度记录前检查文件哈希，使用 `np.load(..., mmap_mode='r', allow_pickle=False)`，
  核对固定 dtype、行数、严格递增索引、像素上界和有限正值的有效 z。每次仅比较一个查询。
  写成功 JSON 前完成所有匹配核对。聚合按像素数，从交集上的绝对误差和推导 L1。
- [x] 接入策略并按需记录深度，以同一查询实际渲染的 `alpha >= 0.05` 产生覆盖标志。新增端到端
  GPU 用例核对记录与评分一致、留出专用于评分及渲染失败时成功报告缺失。运行定向与现有
  `tests/3dgs_probe` 套件，使用 `LIVO_REQUIRE_GPU=1`。
- [x] 用同一归档逐次运行下方两种策略，再用比较工具合并结果。预期 13 组、147 次训练确认、
  每种策略 2,940 步、25 个单块结果及 23 个联合结果。按 C4a 实测重复波动核对基线。
- [x] 保存配对结果、逐帧新增/丢失的深度样本、实际种子统计、资源、代码和输入哈希。用设计稿的
  筛选线判断是否适合继续在线试验。按真实结果封存对照，独立审阅后补齐复验及使用记录。

```bash
export OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0
for policy in uniform coverage; do
  "$LIVO_PYTHON" tools/3dgs_probe/evaluate_gaussian_submaps.py \
    --keyframes "$LIVO_BASELINE/review-full-on/keyframes" \
    --map-directory "$LIVO_BASELINE/g-review-full-on" \
    --prepare-executable "$LIVO_PREPARE_BIN" \
    --seed-policy "$policy" --save-depth-samples --output "$LIVO_PHASE/$policy" || exit 1
done
"$LIVO_PYTHON" tools/3dgs_probe/compare_gaussian_holdout.py \
  --baseline "$LIVO_PHASE/uniform" --candidate "$LIVO_PHASE/coverage" \
  --output "$LIVO_PHASE/comparison.json"
```

变量及依赖沿用 C4a 的复现说明。两次运行都用同一个准备器，本机应得到既有划分哈希
`0bafdb25a4b27ecb72053ae0c6f4f3d7be5706428b724356ecd7f485607c0c2e`。
[运行环境和路径](../../p6-03c4a-validation.md#在另一台电脑复现)
