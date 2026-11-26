# Gaussian 留出隔离实施计划

依据：[已确认设计](../specs/2026-11-26-online-heldout-evaluation-design.md)。本阶段只实现
训练前冻结划分和训练来源证明，后续评分、SSIM 和独立查询仍按设计另行实施。

## 约束与接口

- 保留实时 SLAM 的完整关键帧、成员和位姿；只筛选 Gaussian 训练输入。
- owner 校验冻结 ID、角色、原始时间戳和标定身份，不读取图像文件或做 GPU 工作。
  worker 在任何播种/损失调用前，核对重建观测的 RGB、深度、掩码、内参与相机外参摘要。
  深度在固定 body 坐标系投影，因此位姿图修正不改变输入摘要。
- manifest 保留源图像/点云文件摘要、前端环境和数据来源；在线输入匹配针对实际传入
  Gaussian 的重建观测。未训练的留出像素将在后续查询入口再做完整摘要检查；本阶段
  不声称已验证留出查询的像素，也不产生画质成绩。
- 新 `holdout_split_v1` capability 显式传输 manifest；启用时持久状态使用 schema 2，
  严格绑定 split_id 和完整训练 lineage；关闭功能继续使用原有消息和 schema 1。
- 不增加 GPU worker；不复用旧的无来源证明模型。路径统一在 config/gaussian.yaml。

## Task 1：冻结 manifest 与纯数据校验

先写失败测试，再实现 CPU-only manifest 库和准备工具。使用既有 reconstruction-view
准备工具逐批读取，临时数据立即清理；冻结完整前缀、源文件/环境/标定摘要、角色、
组成员与最多两个查询。规范 JSON 生成 split_id，拒绝重复身份、篡改和不完整范围。
测试普通输入、跨角色重复图像/时间、内容改变、未知 ID、范围和有界载荷。

## Task 2：worker 与持久状态的完整训练来源

先测试拒绝留出/改变输入且模型不变。协商 capability 后加载 manifest，在普通训练、
所有保留窗口、播种、回环 rebuild 和 restore 前校验；记录实际训练调度的完整 ID
集合（包含已移出窗口的早期 ID）。schema 2 保存带摘要的来源证明；恢复时拒绝旧
schema、不同划分、污染/缺失 lineage，拒绝候选后的回滚恢复原证明。默认路径不变。

## Task 3：native admission 与配置

先测试配置与 admission。启动时读取有界 manifest，owner 保留完整地图语义并检查
每帧元数据；留出帧不覆盖待训练帧。异步发送前再次检查训练角色；握手校验对端返回
的 split_id/schema/capability。未知或不匹配数据使 Gaussian 明确失败，不能静默重分。

## Task 4：回归与短段真实验证

运行 CPU、协议、GPU 状态恢复与 SDK/ROS 构建的相关检查。用已有 Noetic 同环境
数据先冻结短前缀，再从空模型回放同一段 Retail_Street；审计持久状态全部历史及
保留窗口，确认与 heldout 不相交、STOPPED 正常。保留小型证据与明确限制，避免新增
完整 bag/大量训练资产。最终 fresh-context 审查，修复重要问题后本地提交。
