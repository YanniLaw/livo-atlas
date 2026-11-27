# 仅对补入点分层的固定预算对照

全部种子做二维分层时，旧实验的联合覆盖提高，共同覆盖区域的弱梯度 SSIM 下降。
这次保留 `flat` 的选点基底，只改变覆盖反馈要补入的点，检验能否减少这个代价。
[分层采样基准](stratified-seed-sampling.md) · [共同覆盖诊断](common-coverage-texture.md)

## 配置和边界

```yaml
gaussian:
  worker:
    seed_policy: coverage
    seed_distribution: stratified_additions
    coverage_sampling: replace
    coverage_uncovered_ratio: 0.25
```

已有的 replace 规则从有效深度中取最多 1000 个 flat 基底点。预渲染的 alpha 低于 0.05
时，按原算法计算替换额度。25% 配置下，每个满额观测最多替换 250 个位置。
在同一批候选和同一反馈掩码上，新模式的基底、移除位置和数量均与 flat 相同，
补入集合来自空洞中的有效候选，用已有二维分层算法选取。没有覆盖反馈或替换额度为零时，
输出 flat 基底。配置读取会拒绝 stratified_additions 与 resample 的组合。

## 冻结反馈重放

使用 Retail97 和 CBD9 各两份捕获，共 76 批已归档的覆盖掩码。列表版参考采样器先生成独立
期望，产品采样器再逐像素对照。70 批发生替换，全部保留了相同的移除 ID 和替换额度。
17452 个补入位置中，有 15773 个与 flat 的补入位置不同。没有使用新的训练或渲染来生成反馈。

下表是最终空洞在 4 像素邻域内有保留帧初始化种子支持的比例，范围来自两次种子。
此指标统计固定查询上的投影邻居，训练后的覆盖、纹理和深度由留出评分评估。

| 场景/模式 | flat | 仅补入点分层 | 全量分层 |
| --- | ---: | ---: | ---: |
| Retail/single | 3.83%–4.06% | 13.27%–13.70% | 31.97%–32.35% |
| Retail/joint | 3.91%–4.09% | 13.53%–14.06% | 31.97%–32.27% |
| CBD/single | 6.55%–6.63% | 15.00%–15.06% | 31.87%–32.30% |

## 实验协议

Retail 使用 150 次训练调用和 21 个留出查询，CBD 使用 85 次调用和 12 个留出查询。
每个场景各跑 seed 0 和 1，合计 470 次训练调用和 9400 步。对照使用原研究中同种子的
`flat-coverage` 和 `stratified-coverage`，共八份。每次从空模型开始，使用同一冻结 split、
位姿、相机、空间选块、训练顺序、窗口和优化参数。真实训练中的覆盖反馈由各自的模型生成。

每场景、每查询、每模式分别在六份结果的共同覆盖上重算纹理和深度，全图覆盖保留完整图像分母。
弱梯度由参考 RGB 阈值 0.02 分组，SSIM 度量使用完整的 11×11 窗口。
覆盖新增与丢失以同种子的 flat-coverage 为参照，误差按像素数或完整窗口数汇总。
进入 Noetic 在线回放的条件提前固定，要求两个场景的两个种子都满足联合覆盖高于 flat，
且共同弱梯度 SSIM 高于全量分层。共同深度和覆盖丢失随结果一并核对。

## 留出评分结果

四次均成功。下表每项先汇总单次运行的查询，再对两个种子取均值。覆盖和全图 SSIM
使用各自的完整图像，弱梯度 SSIM 与深度使用本实验的六份共同交集。丢失比例以同种子
flat 已覆盖的像素为分母，将两个种子的像素数合计后计算。

| 场景/模式 | 分布 | 覆盖 | 全图 SSIM | 共同弱梯度 SSIM | 共同深度 L1/m | 原覆盖丢失 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Retail/single | flat | 83.3341% | 0.229185 | 0.331770 | 0.218426 | — |
| Retail/single | 仅补入点分层 | 84.0704% | 0.233193 | 0.335271 | 0.214955 | 4.001% |
| Retail/single | 全量分层 | 86.7780% | 0.223251 | 0.300693 | 0.217184 | 4.631% |
| Retail/joint | flat | 92.5251% | 0.290429 | 0.350328 | 0.233722 | — |
| Retail/joint | 仅补入点分层 | 92.6490% | 0.294046 | 0.353836 | 0.231714 | 1.988% |
| Retail/joint | 全量分层 | 94.1293% | 0.286977 | 0.332485 | 0.232064 | 2.183% |
| CBD/single | flat | 64.5368% | 0.159683 | 0.314815 | 0.198681 | — |
| CBD/single | 仅补入点分层 | 64.8967% | 0.163724 | 0.332119 | 0.189197 | 6.559% |
| CBD/single | 全量分层 | 66.4447% | 0.156521 | 0.306338 | 0.187072 | 16.765% |
| CBD/joint | flat | 71.5032% | 0.159550 | 0.275084 | 0.218576 | — |
| CBD/joint | 仅补入点分层 | 71.5894% | 0.162160 | 0.285939 | 0.208458 | 5.968% |
| CBD/joint | 全量分层 | 73.9392% | 0.154800 | 0.260545 | 0.206990 | 12.824% |

仅补入点分层在 Retail、CBD 的联合净覆盖增益分别为 0.124 和 0.086 个百分点。
共同弱梯度 SSIM 相对 flat 分别增加 0.003508 和 0.010855，共同深度 L1 分别降低约
2.01 和 10.12 mm。这一模式保留了更多原覆盖，但仍有覆盖周转。
以 CBD 联合结果为例，原覆盖丢失比例从全量分层的 12.82% 降到 5.97%。

| 场景/种子 | 联合覆盖增量 vs flat（百分点） | 共同弱梯度 SSIM 增量 vs 全量分层 |
| --- | ---: | ---: |
| Retail/0 | +0.1382 | +0.021408 |
| Retail/1 | +0.1094 | +0.021294 |
| CBD/0 | +0.1242 | +0.025360 |
| CBD/1 | +0.0483 | +0.025428 |

四个配对达到预定进入条件。联合弱梯度完整窗口数在 Retail 为 2441148，CBD 为 709753，
共同深度标签数分别为 232763 和 83861。各场景的六份结果共享各自的分母。
联合净覆盖的增幅较小，本轮主要改善是共同区域的结构保持与覆盖丢失。

## Noetic 短段验证

在达到留出评分门槛后，用 Retail_Street 的同一 10 秒前缀分别启动 flat 和仅补入点分层。
复用该 bag 的已验证 Noetic 节点二进制，在独立工作区副本运行当前 Python worker。
两次均处理 5 个关键帧、执行 5 次训练，4 帧各替换 250 个点。节点和 worker 正常退出，
各封口并持久提交 1 个子地图，持久模型与关键帧身份通过读取校验。
这组回放验证配置、在线训练和封口的接线，完整 bag 的吞吐和尾延迟留给长段对照。

第一次短段准备的 socket 目录权限过宽，worker 按原有私有目录要求拒绝启动。
将本轮测试目录创建为 0700 后重试，失败日志保留在 `noetic-smoke-attempt-ipc`。
本机用 `noetic_smoke.py` 从本阶段已核验的评分结果启动短段试验。

## 回归与存储

完整 Python/CUDA 回归 457 项通过，无跳过。专项用例覆盖精确补点、零替换回退、
配置约束及单变量比较。四次模型评分、在线短段、对照图和审计记录合计约 1.4 GiB。
旧研究报告继续绑定其冻结源码提交，四次新训练使用本阶段产品源码和同一组旧模型参考。

## 跨电脑复现

用[原分层研究](stratified-seed-sampling.md#固定协议与比较)的方式生成本机冻结参考和控制组。
在每个场景的 flat-coverage YAML 副本中替换 `seed_distribution` 取值，分别用种子 0 和 1 运行：

```bash
python tools/3dgs_probe/evaluate_gaussian_sampling.py \
  --map-directory /data/reference/gaussian \
  --keyframes /data/reference/slam/keyframes \
  --prepare-executable /sdk/bin/livo_atlas_prepare_3dgs_views \
  --selector-executable /sdk/bin/livo_atlas_select_gaussian_neighbor \
  --config /data/stratified-additions.yaml --seed-policy coverage --random-seed 0 \
  --output /data/stratified-additions-0

python tools/3dgs_probe/compare_gaussian_sampling.py \
  --baseline /data/flat-coverage-0 --candidate /data/stratified-additions-0 \
  --allow-seed-distribution-change --output /data/compare-additions-0
```

比较器校验两份记录的配置、样本、视角和训练收据。第二个场景和种子使用各自的冻结参考、记录
及新输出目录。评分样本预计约 1.2 GiB，本轮验证目录上限 2 GiB，额外保留 1 GiB 可用空间。
CUDA 检查沿用最少 4096 MiB 空闲显存和 0.4 分配器比例。

本机输入清单、重放、训练命令、对照图、收据和统计保存在
`validation/stratified-additions-20261127/`。阶段里的 `run_study.py` 和 `run_audit.py` 分别执行串行训练和六份共同覆盖评分，
审计入口预加载本阶段的同名驱动以区分旧研究。用 `run_audit.py --verify` 重查来源和收据。

[设计](superpowers/specs/2026-11-27-stratified-additions-design.md) ·
[实施计划](superpowers/plans/2026-11-27-stratified-additions.md)
