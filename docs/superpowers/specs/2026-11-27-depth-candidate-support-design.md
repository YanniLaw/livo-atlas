# Depth Candidate Support Design

在上一轮固定的 Retail97、CBD9 查询和 seed0/1 上，对比全部有效训练深度候选与实际采样种子。
复用四份已审计模型与渲染；新增训练、渲染调用均为零。保持当前分支、本地提交，不改默认参数、
训练器、ROS、算法核心或 vikit。

## 数据与统计

- 读取 coverage diagnostic 的 report/samples/config/split/poses，校验全部收据；重新加载原始
  snapshot，用同一个原生 spatial selector 验证选块及运行库，用原生 preparer 重建训练观测。
- 只用 split 中的 train 帧；最终位姿、相机外参、内参、源图像/深度和 anchor 均须强绑定。
- 在初始化的 float32 子地图坐标中反投影全部有效 depth/image mask 像素，保留每帧计数。
  实际 all_initial/retained_initial 使用已有捕获值；检查保留关系与输入支持的逐像素包含关系。
  数值边界破坏包含关系时拒绝发布，不静默修正坐标。
- 四组：input_all、input_retained、all_initial、retained_initial。分别比较采样
  (input_all→all_initial、input_retained→retained_initial) 与保留
  (input_all→input_retained、all_initial→retained_initial)，不串成错误的单一因果链。
- 沿用 r=0/2/4、floor(u+.5)、z>1e-6、max(.05m,.02z)。固定最终 alpha<.05 的空洞分母，
  每组统计附近点与已知深度的 consistent/front_only/behind_only/mixed/none；无深度单列。
  每个对比同时给全部空洞损失、已知深度空洞一致支持损失；所有结果只是支持代理。

## 边界与交付

CPU 分块投影每块≤64000点，每组≤32个主观测，每次≤2个子地图；不分配全数据集候选集合。
输出≤64MiB，保留≥1GiB并计入 preparer 临时空间。只在重新验证全部输入、原生 selector、
输出收据和待发布 JSON 后原子发布成功报告。失败保留可识别的局部目录，不发布成功报告。
输出聚合 counts.npz、带收据的 report.json；本地研究保存协议与独立重算审计。

验收：CPU 手算/多块/非法输入/发布期间变更测试，完整现有 Python/CUDA 回归；四次真实分析、
独立原始观测反投影和不同计数算法审计；文档写清分母、半径敏感性和未知深度限制。
本任务不据此改变种子预算、窗口、默认策略，也不声称更多点必然改善 RGB/深度或在线 SLAM。
