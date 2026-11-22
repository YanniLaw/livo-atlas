# P6-03c4a 留出视角评估与复现

`evaluate_gaussian_submaps.py` 在封口快照的最终位姿上，从空模型逐组训练，再对同一个留出
相机渲染本组和本组加前一组。训练沿用 20 步和 10 帧活动窗口。汇总中的两列使用相同查询。
[设计](superpowers/specs/2026-11-19-p6-03c4a-held-out-rendering-design.md) ·
[实施计划](superpowers/plans/2026-11-19-p6-03c4a-held-out-rendering.md)

## Retail_Street 实测

输入为 C3c2b 归档的 172 帧、13 个封口子地图。统一采用位姿版本 2，含 169 帧优化前缀和
3 帧按锚点修正的末端。共确认 147 个训练视角、2,940 个优化步骤，评分 25 个留出视角。
首组的 2 个视角只有单块结果，其余 23 个均有配对结果。

| 查询 | 数量 | PSNR 单块 → 双块 | alpha 覆盖率 单块 → 双块 |
| --- | ---: | ---: | ---: |
| 全部配对 | 23 | 8.98 → 10.95 dB | 79.20% → 90.52% |
| 每组第二帧 | 12 | 8.71 → 11.33 dB | 77.61% → 92.20% |
| 每组中间帧 | 11 | 9.31 → 10.56 dB | 80.94% → 88.70% |

PSNR 从有效图像像素加权的 MSE 推导。覆盖率使用 `alpha >= 0.05`，RGB 评分包含未覆盖像素。
23 个配对视角的 PSNR 都提升，逐视角提升约 0.27 到 6.55 dB。第 19 帧的双块渲染补足了
部分墙面和店招，仍可看到点状空洞。

深度 L1 单块为 0.1934 m，双块为 0.2035 m。各自的可评分深度样本从 264,005 增至 290,707，
这两个深度误差使用各自覆盖的稀疏深度像素。留出针对 Gaussian 初始化和优化，查询位姿来自
完整 SLAM 估计。该表测量固定前端和最终位姿下的画质。

两次执行使用相同划分和实际监督 ID。独立复核了每步监督、最终高斯来源、相机、PNG 和汇总
算式，基于原始 YAML 复算的相机最大矩阵元素差为 `5.44e-13`。逐视角最大重复波动为
0.000590 dB PSNR、0.00556 个百分点覆盖率和 0.000325 m 深度 L1。

本机使用 RTX 4060 Laptop、Python 3.10.22、NumPy 1.26.4、Torch 2.3.0+cu121、
gsplat 1.5.3、Pillow 10.4.0、PyYAML 6.0.2。第一次运行约 26 秒，进程峰值 RSS 约
1.12 GiB，PyTorch 训练峰值 allocated 约 96.5 MiB。每组复制渲染参数后记录的模型 allocated
为 0，保留的 allocator 缓存另有统计。

## 在另一台电脑复现

准备归档的 `review-full-on/keyframes` 和 `g-review-full-on`，保留前者的原始图像、点云、成员
记录和位姿，后者保留 `map.json`、`finalization.json`、`status.json` 和状态资产。GPU 环境
沿用已有版本并使用 PyYAML 6.0.2。准备工具由普通 SDK 构建，可复用已有的
`livo_atlas_prepare_3dgs_views`。 [GPU 环境](p6-03b-gpu-probe.md) · [准备工具构建](gaussian-state.md#环境和数据准备)

将以下变量设为机器上的绝对路径。从包根目录执行命令，输出子目录须为新目录。

```bash
export LIVO_REPO=/path/to/workspace/src/livo-atlas
export LIVO_PYTHON=/path/to/3dgs-env/bin/python
export LIVO_PREPARE_BIN=/path/to/prepare-build/livo_atlas_prepare_3dgs_views
export LIVO_BASELINE=/path/to/P6-03c3c2b-20261119
export LIVO_PHASE=/path/to/new-c4a-validation
mkdir -p "$LIVO_PHASE"
cd "$LIVO_REPO"
export OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0
"$LIVO_PYTHON" tools/3dgs_probe/evaluate_gaussian_submaps.py \
  --keyframes "$LIVO_BASELINE/review-full-on/keyframes" \
  --map-directory "$LIVO_BASELINE/g-review-full-on" \
  --prepare-executable "$LIVO_PREPARE_BIN" --output "$LIVO_PHASE/held-out"
```

`split.json` 绑定原始观测、地图元数据、最终位姿和准备器的哈希。`report.json` 在来源复核通过后
发布，包含实际训练历史、渲染块、相机、指标和资源。`views/<ID>/` 保存参考、单块和存在时的
双块 RGB。另一台电脑若重新编译准备器，可按观测哈希、划分和相机字段比对数据一致性，同时记录
新的二进制哈希。

本机证据目录为 `validation/P6-03c4a-20261122`，含两次报告、逐视角图片、运行日志、
独立审阅和校验清单。

## 验证记录

最终运行源码为 `07b4ed4`。独立审阅检查了候选 `f45b495`，提出两项重要问题。用定向用例
复现后，评估器增加了导出相机与该帧原始相机的核对，并要求 RGB、深度及掩码来自该帧的
固定资产路径。修复后的两次完整实测见 `final-holdout/` 和 `final-repeat/`。对应划分的 SHA256 为
`0bafdb25a4b27ecb72053ae0c6f4f3d7be5706428b724356ecd7f485607c0c2e`。

复验包括 204 项 Python/CUDA 测试、125 项工具测试，以及原快照 2,082 个归档文件的哈希。
`final-holdout-audit.json` 保存两次运行的独立核对和重复波动。跨电脑命令在本机环境核对，
另一台电脑可使用同一划分和逐视角报告核对数值。
