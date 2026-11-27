# Common Coverage Texture Diagnosis

复用分层采样研究的16份归档，解释覆盖/PSNR提高与SSIM下降的关系。用户已授权当前分支直接
实施、验证和本地提交。只增加CPU诊断与输入复查，不训练、不渲染、不改变采样器/优化器/默认配置。

## Method choices

只比较全图指标无法分离填洞与原有区域的变化；只用每对自己的交集也不宜横向比较四条件。
采用同场景八份运行的固定共同覆盖：image_mask且所有alpha>=.05。每个模式单独计算，同场景
查询汇总后再对seed0/1取描述性均值。单块/联合不混合，缺少联合的查询不计入联合分母。

另对同策略、同种子的flat→stratified分成共同覆盖、新增覆盖、丢失覆盖、共同空洞四类，
精确分解RGB L1/SSE。每类SSE差除以3×有效图像像素数是全图MSE变化的可加贡献；不相加PSNR。
四类RGB计数与误差须重构全图，报告浮点残差。该分解不是因果证明。

参考梯度只用参考RGB：中心与上下左右四邻域的平方差跨RGB通道求和除以6再开方，
固定阈值.02分strong/weak；中心或邻居无效/越界则为unknown。这样细密交替纹理也不会被
中心差分抵消。三类完整分割有效图像，模型输出不能改变分组。

全图与固定共同覆盖两区域均统计RGB L1/MSE/PSNR及SSIM，分别给all/weak/strong/unknown。
SSIM沿用11×11、sigma1.5、C1=.01²、C2=.03²和原有卷积/通道累加顺序：只有整个窗口
属于该区域才有效，按窗口中心的参考梯度分组，而非要求整个窗口都属于strong。
没有像素/完整窗口返回null及原因，完美RGB的PSNR返回null并标记perfect，不输出Infinity。

既有evaluation_metrics.py保持不变；新诊断复用其Gaussian卷积，展开逐窗口公式，测试与
独立二维窗口参考核对，并逐项验证新全图统计等于原评分。选择少量诊断公式展开以保留旧归档
和生产评分源码指纹；不引入新的训练或评分定义。

## Source verification and publication

CLI输入为study-directory、coverage-directory、candidate-directory、output。支持1..2场景，
固定四条件×seed0/1，每场景最多32查询、每图最多100万像素；真实研究为Retail/CBD16份。
重验原研究final evidence/audits/protocol指纹、每次report/config/poses/split/全部NPZ/PNG及
训练角色/调度/配置控制项；源snapshot及原始资产仍须匹配冻结摘要。只使用已有选块，不重选。

补齐旧复查缺口：每次split原始字节等于参考split；静态capture/depth报告、它们广告的全部
产物（含NPZ/PNG/config/split/poses/counts）及static selections/参考采样器/计数器均依收据验证。
JSON解析与摘要取自同一段有界字节。源路径通过显式根目录绑定，历史路径作为来源信息保留；
源snapshot路径需仍可访问，跨电脑可重新生成相同研究布局，不新增路径映射框架。

通用复查放入仓库，并让本机旧verify_study.py补调用缺失检查；原脚本先备份，不改变旧研究
数据/报告/协议/最终证据，也不放宽其原有源码检查。新分析单独冻结自己的输入和实现指纹。

按查询流式读八份样本，输出metrics.json及report.json，不复制输入NPZ或模型。总输出<=64MiB，
保留>=1GiB磁盘。报告发布前重验所有输入/输出，最后核对待发布JSON字节再原子rename；失败
保留failure.json且无成功report。独立CPU审计用不同窗口卷积重算，并保存有限对照图。

验收包括手算四类误差可加性、梯度/掩码边界、完整窗口/梯度中心、空集/完美值、错误输入拒绝，
split与静态上游损坏/发布期间变化测试；完整Python/CUDA回归、16份真实归档分析、独立审计。
默认不变，结果仅是这两个开发场景的离线诊断，不外推在线延迟、轨迹精度或新场景表现。
