# Fixed-budget Stratified Seed Sampling

## Intent and scope

上一轮只发现输入候选与实际种子的近邻支持差距，不能证明增点有效。本轮加入可选二维分层分布，
先验证支持分布，再用双场景完整前缀训练检验覆盖、纹理、深度和资源取舍。用户已授权当前分支直接
实施、测试和本地提交；不推送、不改 vikit。既有默认 uniform/flat/resample/.5 保持有效。

## Approaches and choice

可直接增加点数，但会改变显存和训练预算，暂缓。三维体素采样需要额外尺度选择，并可能压缩远景，
也暂缓。实施者选择二维分层：只依赖训练视图的有效像素，不访问heldout，不增加调参维度。

`gaussian.worker.seed_distribution: flat | stratified`，缺省flat。和seed_policy、coverage_sampling
相互独立；不是编译宏。只增加Python worker参数，核心SLAM接口与ROS无变化。

分层算法：n=min(limit,候选数)，按整幅图宽高比选择cols=min(width,n,max(1,round(sqrt(n*w/h))))、
rows=min(height,ceil(n/cols))。有效像素归入规则网格。为非空格分配同等点额，容量不足格先饱和，
剩余额度在仍有容量的格之间均匀分配；余数在格ID中确定性均匀选取。各格内按行优先顺序的中位
分位数取点，结果按像素ID排序。总数必须精确为n、无重复、仅来自候选；空集返回空。
不随机、不按查询画质调网格。最多100万像素，分配/排序有界，不引入新依赖。

同一分布用于基准点、覆盖替换的移除/新增点及resample各子集；配额公式不变。
在线覆盖预渲染的uniform_uncovered（历史字段，表示基准点的未覆盖数）必须来自相同分布。
实际初始化、诊断重放和覆盖计数一致；10帧、1000点/帧、10000点/块、20步/调用不变。

比较器增加互斥的 `--allow-seed-distribution-change`，只接受同策略flat→stratified，其他配置、
选块、调度、软件及源资产完全一致。历史记录只补缺省flat；缺字段不能把真实stratified归档降级。
旧方式/比例/学习率模式不能放行分布变化，YAML类型与别名仍作为控制项。

## Frozen experiment

条件：flat-uniform、stratified-uniform、flat-coverage、stratified-coverage；统一replace25%配置，
uniform不执行覆盖预渲染。seed0/1。先在原Retail97、CBD9固定旧查询/空洞/预渲染掩码上比较支持，
这是固定反馈掩码的静态代理，不能代替实际耦合训练。

随后全部16次同版本完整前缀训练：Retail172帧中150train/21query，每次150调用3000步；
CBD97帧中85train/12query，每次85调用1700步；总1880调用37600步。原生spatial参考选块与
最终位姿固定，所有实验从空模型开始，不复用训练权重。四条件的共同深度按同场景两种子共8次
交集计算，逐条件汇总后对种子取描述性均值；提供每种子成对分布比较及同分布uniform→coverage。
观察RGB/SSIM、alpha、共同深度及uniform覆盖保留，并记录初始化支持和训练资源。

开始前冻结源/软件/工具/配置/协议，至少8GiB可用磁盘；预计新增不超过6.5GiB，保留至少1GiB且
计入原生准备临时空间。不复制bag或保存每条件完整模型。GPU顺序、OMP/BLAS1、最小空闲4096MiB、
分配器比例.4；不停止用户服务。资源不足则保留完整失败证据，不能以跳过的条件作结论。

验收：手算分层/稀疏/细长/饱和/配额测试，真实CUDA初始化和计数测试，单变量比较拒绝项，完整
Python/CUDA无跳过回归；16次产物/源/配置/调度/共同指标独立审计。任何结果都不自动升级默认；
仅两个无轨迹真值的开发数据集，不能外推ATE/RPE、在线P95或泛化结论。
