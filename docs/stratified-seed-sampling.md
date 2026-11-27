# 固定预算二维分层采样对照

2026-11-27。[上一轮候选支持诊断](depth-candidate-support.md)发现输入候选与实际种子的近邻支持
差距，本轮增加可选二维分层分布并验证其实际训练效果。默认仍为uniform/flat/resample/.5。
本页结果表待16次固定协议训练与独立审计结束后填写；当前不能作为新默认的推荐依据。

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

## 结果与验收

代码与比较器的完整Python/CUDA回归 **435/435**，无跳过，179.101秒；分层/配置/真实CUDA
专项8项、单变量比较48项通过。另用独立逐格分配实现核对200组随机布局，与产品分层器一致。
四份静态支持诊断的304批选点、坐标、邻域和深度类别均由独立实现重放核验通过。

当前完整前缀实验完成 **4/16**：Retail seed0的四条件，各150调用/3000步，合计600调用/12000步。
第5次在训练开始前因空闲显存低于4096MiB被拒绝；原始4文件失败目录与日志保存于
`attempt-gpu-shortage/`，没有更改资源阈值或实验参数。已完成样本重新通过来源、配置、评分收据和
完整训练调度核验，`resume-admission.json`记录其摘要。剩余12次、共同深度汇总、最终研究审计及
整轮独立代码审查均待完成，尚无16次的画质/深度结论。

静态旧反馈掩码下，4像素内有保留种子近邻的空洞比例由flat约4%–7%提升至分层约26%–32%。
这是固定空洞/反馈条件下的支持代理，不是实际耦合训练的画质改善证明。新策略尚未升级默认。
本机恢复脚本先重新核验已完成结果和协议，随后仅在空闲显存≥4096MiB时续跑，轮询间隔30秒。
如等待到期，释放显存后重新执行工作区 `resume_study.py`；不要重跑会要求新协议目录的
`run_study.py`，也不要删除已核验的4次结果。

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

新比较开关与比例、覆盖方式、学习率比较互斥，只允许同策略flat→stratified，其他输入、YAML
值及类型、别名路径、配置/样本收据和调度均须一致。保留既有输出完整性、磁盘及发布检查。
所有输出用新目录；其他场景和种子替换相应路径/配置，不能复用Retail的split给CBD。

工作区 `validation/stratified-seed-sampling-20261127/` 保存协议、配置、16套评分样本、原生来源及
工具/软件指纹、静态选点、独立审计、共同深度汇总和对照图。开始至少8GiB可用、总量不超过6.5GiB，
原生准备临时空间另计并预留至少1GiB；GPU顺序4096MiB最小空闲/.4分配器比例，OMP/BLAS1。
CPU静态独立审计与早期训练有重叠，资源耗时仅描述，不作受控CPU延迟或在线P95结论。

[设计](superpowers/specs/2026-11-27-stratified-seed-sampling-design.md) ·
[实施计划](superpowers/plans/2026-11-27-stratified-seed-sampling.md)
