# 保留均匀位置的高斯补洞

用户认可保留 uniform 初始采样位置、定额替换补洞的方向，并授权本会话直接实施、验证和本地提交。
只修改 livo-atlas，在当前 framework_completion 分支完成；沿用普通 worker、配置和固定预算工具。

## 方案与规则

当前 coverage 先预留空洞点，再在剩余候选上重新均匀采样，会改变大量原均匀位置。
新方式先构造同一 uniform 集合，再只替换一部分已覆盖位置。alpha覆盖只是启发式，
不能据此保证被移除点在纹理/几何上冗余，也不能保证最终图像不退步。

新增 `gaussian.worker.coverage_sampling: resample`，枚举 resample / replace。
默认 resample 保持旧规则；仅影响 seed_policy=coverage，不增加编译宏、线程或新框架。
保留 coverage_uncovered_ratio，replace 时表示**最大替换比例**。

令 V 为按行排序的有效深度像素，N=min(limit,|V|)，U为原uniform的N个位置，H为空洞像素。
C=U\H 是可移除位置，A=H\U 是尚未入选的空洞。
q=min(floor(N*ratio),|C|,|A|)。分别在C/A按原有规则均匀选q个位置，结果为排序后的
(U\remove)∪add。这样总数恒为N、位置不重复、保留N-q个原位置，U中原有空洞点全部保留。
q=0/无模型/小候选集/全空洞/无空洞均保持uniform；ratio=0跳过覆盖预渲染。
仅约束初始化像素位置；训练仍会优化高斯均值，不能把初始位置保留当成最终覆盖保证。

## 证据与兼容

replace 的 seed_selection 在已有字段上增加 uniform_uncovered 和 replaced_seeds；
reserved_uncovered 固定0，避免把替换数误称为旧预留数。无预渲染时前者为空、后者0。
校验器验证计数上下界及上述q公式；CPU实际数组与CUDA冻结均值测试验证位置保留。
历史记录缺少coverage_sampling时仅补默认resample，仍按类型精确核对其余配置。

比较器增加显式 --allow-coverage-sampling-change，与已有比例变化开关互斥。
该模式只允许resample→replace、比例相同；仅移除该枚举键后精确比较其余YAML/WorkerOptions。
默认比较和比例比较继续严格拒绝额外变化，旧格式配置仍可读。

## 试验与边界

使用同一172帧、150train/22heldout/21query的冻结参考、最终位姿及原生空间选块。
五个条件各重复两次：uniform等价的coverage/resample/0，以及resample、replace分别在25%/50%。
十次GPU顺序运行，每次150次训练/3000步、窗口10、每观测最多1000、每块最多10000高斯。
两种方式在相同比例下配对，另验证resample相对0的比例对照；不允许自动忽略其他配置。
独立重算十次共同像素深度、单块/联合指标、覆盖增减、位置保留计数、资源与最差视角图。
约4GiB精确浮点评分输出，不复制bag或模型；保留至少1GiB空闲，开始前需有足够总预算。
默认uniform、resample和0.5比例不变。现有bag是开发验证，不据此主张泛化或精确在线延迟。
