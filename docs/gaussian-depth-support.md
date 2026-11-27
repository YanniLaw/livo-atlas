# 离线多帧深度支持评分

`evaluate_depth_support.py` 对已经生成的最终快照留出评分加上 LiDAR 多帧支持统计。它从原始
关键帧重投影相邻云，在各策略共同覆盖的同一批有效标签上计算有符号误差、L1 和 RMSE。
单份封口地图可以生成绝对指标，配对输入还会给出增量。入口使用 CPU 和已有的 NumPy、PyYAML
环境。[最终快照评分](gaussian-heldout-scoring.md) · [多帧支持口径](multiview-depth-support.md)

## 使用

每个 `--run` 依次填写显示名称、封口地图目录、关键帧目录和最终浮点评分目录。可以传入一至四份
结果，并用 `--pair` 指定基准和候选。所有输入参与共同掩码，单块和联合使用各自的掩码。

```bash
python tools/3dgs_probe/evaluate_depth_support.py \
  --run flat /data/flat/gaussian /data/flat/keyframes /data/flat/final-scores \
  --run additions /data/additions/gaussian /data/additions/keyframes /data/additions/final-scores \
  --pair flat additions --output /data/depth-support.json
```

输出文件使用已有父目录下的新路径。`--exclude-query ID` 可重复指定，适用于另有用途或在原评分中
跳过的冻结查询。报告记录有效相机、选块、每份归档的选择策略和内容收据。输入使用同一冻结划分、
物化位姿、相机和有效选块，归档移到其它绝对路径后可以按新位置传入。

本帧复算使用原始标定和 KeyframeStore 写出的二进制 PCD。邻帧使用冻结相机与最终位姿，默认
检查 `id±1/±2`、2/4 px 和 `max(0.20 m, 0.03 × z_ref)`。规则与[多帧复验设计](superpowers/specs/2026-11-27-multiview-depth-support-design.md)
一致。报告中 `views` 保存逐视角分母、掩码摘要与支持像素数，`modes` 中汇总数值按像素数加权。
`radii` 下的 `reference` 组表示参考层重复支持且近层未重复。配对指标位于 `deltas`，例如
`modes.joint.radii.2.reference.deltas.flat:additions`，使用米和平方米。空组保留像素数和空指标。

## 真实归档与局部误差

新入口分别读取 Retail 四份和 CBD 两份保存的浮点评分。31 个查询、58 个共同掩码、各支持组
计数以及全部配对指标与上一阶段 CPU 结果逐项一致。主观察组整体改善后，再复查三个仍有局部
增量的视角。表中数值是参考层有重复支持的共同标签上，候选相对 `flat` 的净平方误差变化。

| 视角和模式 | 2 px 主组 (m²) | 两种半径均满足主组 (m²) | 2 px 组内最大的 1% 像素占正增量 |
| --- | ---: | ---: | ---: |
| Retail 1 单块首对 | +461.91 | −90.20 | 76.5% |
| Retail 1 单块反序 | +364.51 | −210.79 | 77.5% |
| CBD 57 单块 | +152.31 | −27.51 | 85.2% |
| CBD 57 联合 | +235.05 | +17.62 | 85.6% |
| CBD 73 单块 | +320.43 | +109.77 | 84.8% |
| CBD 73 联合 | +270.99 | +34.85 | 72.1% |

Retail 1 号的 315 个像素在 4 px 范围新增重复近层支持。该批像素在两对分别贡献
+552.11 和 +575.30 m²，解释该视角在更严格支持条件下结果反转。CBD 57 号对这一条件也敏感。
CBD 73 号在两种半径交集内仍有净增量，单块、联合正增量中分别有 89.4%、91.2% 来自深度
向远处移动。20 m 以上的参考标签承担了该交集内最大的净增量。图上红色表示相对基准移近，蓝色
表示移远，均为平方误差增加像素。增大邻域会引入更多投影观察，所得竞争标记用于评价局部可见性。
随后互换了 73 号使用的两个子地图并复算实际射线权重。[CBD 73 号远深度归因](cbd-73-depth-attribution.md)

本机结果位于 `validation/offline-depth-support-20261128/`。`run_archives.sh` 记录两场景命令，
`retail.json` 和 `cbd.json` 是新工具报告，`diagnose_views.py --verify` 重查两个报告与历史结果、
主要增量像素和三张诊断图。`diagnosis.json` 保存坐标、两个范围的票数、深度区间和各组误差和。
本机 463 项 Python/CUDA 回归通过，`run_regression.sh` 记录使用的原生夹具，`study-files.sha256`
记录脚本、报告和测试日志的摘要。

[设计](superpowers/specs/2026-11-28-offline-depth-support-design.md) ·
[实施计划](superpowers/plans/2026-11-28-offline-depth-support.md)
