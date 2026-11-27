# 固定预算二维分层采样对照

2026-11-27。[上一轮候选支持诊断](depth-candidate-support.md)发现输入候选与实际种子的近邻支持
差距，本轮增加可选二维分层分布并验证其实际训练效果。默认仍为uniform/flat/resample/.5。
16次训练及独立审计已完成：分层提高覆盖与PSNR，但SSIM和原有覆盖保留存在损失，
共同深度的收益也取决于场景和是否使用覆盖反馈。本轮保留新参数供对照，不升级默认。

## 参数与算法

```yaml
gaussian:
  worker:
    seed_policy: coverage
    seed_distribution: stratified  # flat（缺省）或stratified
    coverage_sampling: replace
    coverage_uncovered_ratio: 0.25
```

`seed_distribution`决定点在图像上的分布，`seed_policy`决定是否用历史覆盖反馈，
`coverage_sampling`决定反馈点如何置换/预留。三者相互独立，均为运行参数。
示例是实验候选配置；启动完整在线系统的其他设置见[在线Gaussian](online-gaussian.md)。
本轮验证模型层插入/淘汰与离线完整前缀，不等于重新完成新参数的双ROS全bag在线验证。

flat沿用有效像素行优先列表的等间隔取样。stratified按整幅图宽高比建立网格，为每个非空格
分配等额点数；稀疏格先饱和，再把余额分给仍有容量的格。余数在活跃格ID中确定性均匀选取，
格内取行优先中位分位数，最后按像素ID排序。网格仅由预算与图像尺寸确定，没有新增网格尺度参数。
每帧总数精确为min(1000,有效候选数)，无重复且只来自有效image/depth mask；不访问留出图像内容。

同一分布用于基准点、replace的移除/新增子集及resample各子集，配额公式不变。
历史字段uniform_uncovered表示当前分布基准点中的未覆盖数，并非强制使用flat。
覆盖证据、实际初始化和诊断重放统一使用该分布。窗口10帧、每块10000点、每调用20步保持一致。
缺少新字段的历史记录只按flat解释；带stratified配置却删掉报告字段的记录不能降级为旧归档。

## 固定协议与比较

两套冻结参考不重新回放bag、不复制完整地图；每条件从空模型重新训练，未复用旧模型权重。
所有源图像/点云/帧记录、split、最终位姿、原生准备器及选块运行库均绑定摘要。
参考spatial选块固定，不按新模型权重重新选择邻居。

| 场景 | 前缀帧 | train | 预选查询 | 每次训练调用/步数 |
| --- | ---: | ---: | ---: | ---: |
| Retail | 172 | 150 | 21（19联合） | 150 / 3000 |
| CBD | 97 | 85 | 12（10联合） | 85 / 1700 |

每场景flat-uniform、stratified-uniform、flat-coverage、stratified-coverage四条件，seed0/1，
共16次、1880调用、37600步。统一原学习率、损失和replace25%配置；uniform通过评估CLI的
`--seed-policy uniform`显式覆盖YAML的策略，不做覆盖预渲染。这样同一分布的两个策略共享同一
配置快照，跨分布比较只改变seed_distribution。

每种子分别比较同策略flat→stratified和同分布uniform→coverage。全局RGB/SSIM使用有效图像像素，
覆盖为image_mask内alpha≥.05；深度按同场景八次运行的共同覆盖标签交集计算，场景之间不共用
交集或绝对误差基准。每条件先汇总查询，再对两个种子的指标取描述性均值，不给显著性结论。

静态部分只用原Retail97、CBD9的固定最终空洞和历史训练覆盖掩码，对四种选点方案重建支持，
没有训练或渲染。它是固定反馈的代理；真实前缀训练中各条件的反馈掩码会自然分化，两者分别解释。
r0/2/4、floor(u+.5)、z>1e-6和max(.05m,.02z)沿用上一轮，未知深度继续单列。

## 实际训练结果

以下每条件先汇总同场景查询，再取seed0/1的描述性均值。深度列使用本场景八次共同交集。

| 场景 | 分布/策略 | 联合PSNR(dB) | 联合SSIM | 联合覆盖 | 单块SSIM | 联合共同深度L1(m) | RMSE(m) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retail | flat-uniform | 11.309025 | 0.287666 | 89.0014% | 0.235965 | 0.233134 | 0.950332 |
| Retail | stratified-uniform | 12.348050 | 0.286159 | 92.6570% | 0.230983 | 0.228219 | 0.939389 |
| Retail | flat-coverage | 12.025906 | 0.290429 | 92.5251% | 0.229185 | 0.234666 | 0.919054 |
| Retail | stratified-coverage | 12.618288 | 0.286977 | 94.1293% | 0.223251 | 0.232708 | 0.917027 |
| CBD | flat-uniform | 8.900947 | 0.168777 | 67.4706% | 0.170152 | 0.182712 | 0.873023 |
| CBD | stratified-uniform | 9.172963 | 0.159774 | 71.4146% | 0.162627 | 0.184609 | 0.874487 |
| CBD | flat-coverage | 8.994585 | 0.159550 | 71.5032% | 0.159683 | 0.223183 | 0.928052 |
| CBD | stratified-coverage | 9.168491 | 0.154800 | 73.9392% | 0.156521 | 0.208010 | 0.894357 |

分层减flat的配对变化如下；每行只改变分布，其他控制项固定。

| 场景/策略 | 联合PSNR差(dB) | 联合覆盖差(百分点) | 单块SSIM差 | 联合SSIM差 | 共同L1差(mm) | 共同RMSE差(mm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retail/uniform | +1.0390 | +3.6556 | -0.004981 | -0.001507 | -4.92 | -10.94 |
| Retail/coverage | +0.5924 | +1.6042 | -0.005934 | -0.003451 | -1.96 | -2.03 |
| CBD/uniform | +0.2720 | +3.9440 | -0.007525 | -0.009003 | +1.90 | +1.46 |
| CBD/coverage | +0.1739 | +2.4359 | -0.003162 | -0.004750 | -15.17 | -33.70 |

两种子在四对条件中均呈现单块/联合SSIM下降。uniform下共同深度在Retail改善、CBD略恶化；
coverage下分层相对flat的共同L1/RMSE在两场景均改善。不过CBD的stratified-coverage共同L1
仍高于两种uniform条件，不能只从coverage内部的改善认定它是整体最优。

联合共同深度分母：Retail为226421/254967个有效参考标签，CBD为79726/136346；
单块分别为221905/281620、81878/162748。比较器每份报告的深度使用该对的两次交集，
正文表使用同场景八次交集，两者不混用。也不与此前不同条件集合的交集绝对误差直接比较。

### 原有覆盖保留

以下以同种子flat-uniform的联合覆盖作共同基准。丢失比例为两个种子的丢失数之和除以
基准覆盖数之和，衡量原覆盖被移除，区别于净覆盖增长。

| 场景 | 目标 | 原有覆盖丢失像素seed0/1 | 丢失比例 |
| --- | --- | --- | ---: |
| Retail | flat-coverage | 40631 / 39431 | 0.731% |
| Retail | stratified-uniform | 143127 / 143156 | 2.612% |
| Retail | stratified-coverage | 96230 / 96422 | 1.758% |
| CBD | flat-coverage | 53589 / 52836 | 2.434% |
| CBD | stratified-uniform | 310105 / 310096 | 14.184% |
| CBD | stratified-coverage | 263547 / 262991 | 12.042% |

Retail两次基准覆盖为5479565/5479571像素，CBD为2186343/2186265。分层新增的覆盖更多，
但也失去了更多原有覆盖；CBD的stratified-coverage丢失约12.04%，高于flat-coverage约2.43%。
因此净覆盖增加并不等于保留了既有重建。

## 静态支持与解释边界

静态研究固定旧flat/coverage模型的最终空洞和反馈掩码，以下分母为各模式的最终空洞数；
每列是两个种子各自比例的均值。4像素内有保留初始化种子近邻的比例：

| 模式 | flat/uniform | stratified/uniform | flat/coverage | stratified/coverage |
| --- | ---: | ---: | ---: | ---: |
| Retail/single | 4.643% | 28.054% | 3.944% | 32.162% |
| Retail/joint | 4.131% | 27.537% | 4.001% | 32.121% |
| CBD/single | 5.534% | 26.219% | 6.591% | 32.085% |

已知深度空洞中，stratified/coverage的4像素一致近邻比例约35.56%–36.75%，flat/coverage约
3.81%–4.98%。静态空洞约97%没有查询深度，不能把这些几何结论套用到未知部分。
结果也依赖半径：Retail联合flat/coverage在r0/r2/r4为0/0.103/4.001%，分层为
0.854/10.231/32.121%。邻域不是高斯足迹或可见性证明。

分母特意选择旧模型空洞，偏向缺少旧种子的位置，这是不对称的支持代理。它不能独立证明
全图点数不足或真实表面可直接补齐；实际训练的PSNR/SSIM/深度以完整预选查询重新检验。
全图RGB误差包含空洞，PSNR提高也不能单独证明已有纹理更清晰。已查看预选Retail97/seed0
的联合RGB和alpha对照：分层增加点状支持，招牌文字和连续表面仍明显缺失，alpha覆盖不等于纹理恢复。

仅两个反复使用的开发数据集，无轨迹真值；本轮不证明ATE/RPE、盲测泛化或在线P95。
后续已完成[固定共同覆盖纹理诊断](common-coverage-texture.md)：补齐全图收益的四类分解及参考梯度分组，再据此设计单变量干预。
本轮不继续分布/比例网格搜索，也不把stratified自动设为默认。

## 资源、恢复与验收

完整16次/1880调用/37600步和16份配对比较均通过：源资产/最终位姿/原生选块/配置与软件
强绑定，训练角色/20步调度/窗口/origin与替换配额一致；所有NPZ/PNG摘要核对，浮点评分、
八次共同深度及两次配对交集均重新计算。静态304批选点由独立逐格实现重放，并用bincount
核验投影统计；另有200组随机布局与产品分层器逐点一致。

审查前最终Python/CUDA回归435/435；审查修正后 **438/438**，无跳过，141.616秒。
分层/配置/真实CUDA专项8项、比较器51项通过。新参数尚未重新跑双ROS完整bag链路，模型层已验证11次插入和窗口淘汰。

成功运行训练耗时7.20..23.80秒，总评估22.41..50.63秒；节点不是按这些离线耗时实时运行。
worker RSS峰值最高约1336.82MiB，PyTorch allocated峰值约96.52MiB、reserved112MiB，
分配器数字不等于整机显存。CPU静态/完整审计与部分训练有重叠，耗时仅描述，不作性能归因。

此前一次显存前置检查失败，另一次停止等待时发生启动竞争并留下中断输出，均单独归档为
attempt-gpu-shortage和attempt-interrupted-release，未计入16个成功条件。上述1880调用/37600步仅统计16份成功条件，中断尝试可能已执行部分训练，未混入条件统计。
用户释放显存后，
先验证前4份成功报告、所有产物与来源，再从第5次续跑；协议、参数与训练源始终未改。
工作区含这些失败/中断记录约5.3GiB，低于6.5GiB预算，未复制bag或每条件完整模型。

一次独立只读审查发现并修正两项重要校验问题：新分布模式不能顺便补齐其他YAML字段默认值；
两侧训练收据的ID/步数/点数及origin必须为真正整数，不能让bool/float通过Python数值相等规则。
三个回归用例先失败后通过，完整438项回归通过。修正只涉及比较器校验，采样/训练/优化器源码字节未改。
修正后的16份比较报告及全部诊断图与原结果逐字节一致；没有重新训练。
review-verification.json将修正后的两份校验器源码指纹与原实现、全部原比较收据绑定，其他函数AST
及所有其他产品源码字节保持一致；完整独立数据审计按该指纹重新核验。

本阶段原先延后的每次split.json与静态上游产物重查，已在[共同覆盖诊断](common-coverage-texture.md)
补齐：仓库提供通用检查，本机verify_study.py备份后调用；原有源码与数据检查保持，原始研究报告未改。

## 跨电脑复现

```bash
python tools/3dgs_probe/evaluate_gaussian_sampling.py \
  --map-directory /data/reference/gaussian --keyframes /data/reference/slam/keyframes \
  --prepare-executable /sdk/bin/livo_atlas_prepare_3dgs_views \
  --selector-executable /sdk/bin/livo_atlas_select_gaussian_neighbor \
  --config /data/Retail-stratified.yaml --seed-policy coverage --random-seed 0 \
  --output /data/Retail-stratified-coverage-0

python tools/3dgs_probe/compare_gaussian_sampling.py \
  --baseline /data/Retail-flat-coverage-0 --candidate /data/Retail-stratified-coverage-0 \
  --allow-seed-distribution-change --output /data/distribution-comparison
```

比较开关与比例、覆盖方式、学习率比较互斥。本研究使用同策略 flat→stratified 对照，其他输入、YAML
值及类型、别名路径、配置/样本收据和调度均须一致。保留既有输出完整性、磁盘及发布检查。
所有输出用新目录；其他场景和种子替换相应路径/配置，不能复用Retail的split给CBD。

工作区 `validation/stratified-seed-sampling-20261127/` 保存协议、配置、16套评分样本、原生来源及
工具/软件指纹、静态选点、独立审计、共同深度汇总和对照图。开始至少8GiB可用、总量不超过6.5GiB，
原生准备临时空间另计并预留至少1GiB；GPU顺序4096MiB最小空闲/.4分配器比例，OMP/BLAS1。
源码实现`524fe05`、单变量比较`1b07440`，审查校验修正`2983bc9`；协议及工具指纹在首次评分前冻结。

[设计](superpowers/specs/2026-11-27-stratified-seed-sampling-design.md) ·
[实施计划](superpowers/plans/2026-11-27-stratified-seed-sampling.md)
