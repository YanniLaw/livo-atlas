# P6-03c4b 覆盖优先采样验证

评估器用同一份 Retail_Street 封口快照比较 `uniform` 和 `coverage`。两次运行均使用最终位姿
版本 2，包含 169 帧优化前缀和 3 帧按锚点修正的末端。训练使用相同的 13 组、147 帧和
2,940 步。每帧为 1,000 个种子，每块最多保留 10 帧和 10,000 个高斯。
[设计与补测](superpowers/specs/2026-11-22-p6-03c4b-coverage-guided-seeding-design.md) ·
[实施计划](superpowers/plans/2026-11-22-p6-03c4b-coverage-guided-seeding.md)

## 留出视角

| 查询 | 数量 | PSNR 均匀 → 覆盖优先 | alpha 覆盖率 均匀 → 覆盖优先 |
| --- | ---: | ---: | ---: |
| 单块 | 25 | 8.781 → 8.930 dB | 77.426% → 80.091% |
| 双块 | 23 | 10.945 → 11.602 dB | 90.523% → 93.588% |
| 双块，每组第二帧 | 12 | 11.328 → 11.923 dB | 92.197% → 94.616% |
| 双块，每组中间帧 | 11 | 10.563 → 11.277 dB | 88.696% → 92.466% |

PSNR 由有效图像像素加权的 MSE 推导，覆盖率使用 `alpha >= 0.05`。双块的 23 个视角均提升
PSNR 与覆盖率。单块有 17 个视角的 PSNR 提升，最差的一个视角下降约 0.60 dB。

| 双块 LiDAR 像素 | 数量 | 均匀深度 L1 | 覆盖优先深度 L1 |
| --- | ---: | ---: | ---: |
| 共同覆盖 | 287,089 | 0.2001 m | 0.2079 m |
| 仅覆盖优先可评分 | 9,514 | — | 0.3128 m |
| 仅均匀可评分 | 3,617 | 0.4717 m | — |
| 均未覆盖 | 6,183 | — | — |

双块满足设计中的三条在线试验筛选线。PSNR 提升 0.657 dB，超过 0.20 dB。覆盖率增加
3.065 个百分点，超过 0.50 个百分点。共同深度 L1 增加 7.8 mm，低于 20 mm 上限。当前
线上默认使用 `uniform`。筛选结论来自这份数据的固定留出对照，在线调度与吞吐需要单独实测。

## 资源与复核

RTX 4060 Laptop 上有 134 帧执行训练前预渲染，其中 30 帧在满窗口排除最早来源。预渲染
中位数 1.113 ms，P95 1.687 ms，累计 0.162 s，实际预留 66,755 个种子。这些预留配额
占用现有每帧预算。包内评估耗时为 25.98 s 和 26.09 s，峰值 RSS 约 1.13 GiB，PyTorch
峰值 allocated 约 96.5 MiB，峰值 reserved 112 MiB。资源数字来自各一次顺序运行。

独立脚本核对两次实际训练日程、高斯来源、相机和深度记录。重跑均匀策略与 C4a 原实测相比，
逐视角最大差异为 0.000448 dB、0.0065 个百分点覆盖率和 1.18 mm 深度 L1。深度和覆盖率
的最大差异略高于 C4a 两次运行之间见到的 0.325 mm 与 0.0056 个百分点。
[C4a 基线与重复实测](p6-03c4a-validation.md)

## 在另一台电脑复现

使用 C4a 的依赖环境、归档和准备器。在包根目录执行，两个评估输出目录及比较文件均使用新路径。
[环境与路径变量](p6-03c4a-validation.md#在另一台电脑复现)

```bash
cd "$LIVO_REPO"
mkdir -p "$LIVO_PHASE"
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

本机的划分 SHA256 为 `0bafdb25a4b27ecb72053ae0c6f4f3d7be5706428b724356ecd7f485607c0c2e`。
`report.json` 保存实际监督顺序、种子分配、资源与逐视角评分。可选的
`views/<ID>/depth-samples.npy` 保存行优先像素索引、观测与预测相机 z、覆盖标记，以报告内的
相对路径和 SHA256 绑定。比较工具使用 CPU，从这两份报告和记录重算汇总。

证据位于工作区 `validation/P6-03c4b-20261122`，包含报告、留出渲染 RGB、稀疏深度、独立
复核、回归日志、审阅记录和哈希清单。

## 验证记录

两次评估使用 `704a1d4`。最终比较器为 `d7e231b`，逐视角核对稀疏深度 L1 与评分报告，
逐训练帧核对候选数、预渲染状态和实际配额。两种不一致均有失败复现和修复用例。用最终
比较器重新读取两份评估产物，`final-comparison.json` 与审阅前的结果逐字节一致。
`final-audit.json` 记录独立复算和三项筛选结果。

最终通过 215 项 Python/CUDA 测试和 125 项工具测试。C3c2b、C4a 和 C4b 补测的
2,082、190 和 10 个归档文件哈希全部匹配。比较器在隐藏 CUDA 设备且进程未加载 Torch
和 gsplat 的环境中复现了同一份 JSON。跨电脑复现时可从归档的 `split.json` 核对输入，
重新编译准备器时记录新的二进制哈希。
