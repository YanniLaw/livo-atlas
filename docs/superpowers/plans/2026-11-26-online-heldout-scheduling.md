# 在线封口留出评分实施计划

依据：[已确认的在线留出评估设计](../specs/2026-11-26-online-heldout-evaluation-design.md)。
在当前授权的 framework_completion 分支按 executing-plans/TDD 执行，最终做一次独立审查。

## 范围与接口

- `evaluation.enabled` 继续控制训练隔离；增加 `evaluation.online_enabled=false` 明确启用
  在线评分。复用现有 worker、mailbox、资源预算、C++ temporal/spatial 投影选块。
- 封口只登记 manifest 预选查询和时间，至多64条元数据；无图像队列、第二个 GPU 模型或线程池。
- 派发捕获准确的 map_sequence、pose frontier、content/asset/binding 和独立 heldout 相机。
  消息 `heldout_evaluate` 与 capability `heldout_evaluation_v1` 独立于训练相机预览。
- 共用 source 编解码/检查，但保留预览“最后保留训练相机”的额外约束。评分不发 ROS READY，
  返回较旧但来源自洽的 as_of 仍可存为历史成绩。
- worker读取已提交前缀中的原始 heldout帧、重建/传输/推理/写盘都在外围串行线程；owner只登记元数据。
  C++ 读取观测，Python用 frozen split 规范化 f4标签/K/C_B、校验来源并复用既有 measure/infer。
- 内容的 decision、map确认、checkpoint/prune先完成；最多两个内容尝试后给予评分机会，
  评分与预览共享该机会并交替，不能连续占用训练轮次。同步争用不能偷偷丢掉评分机会。
- 正常停止取消未派发查询、记录 not_in_scope；在途工作受控完成后记录，再继续 Finalize/STOPPED。
  worker产物/owner状态分开：`evaluations/views/<ID>/` 保存 immutable as_of 与float样本，
  `evaluations/online-status.json` 为有界完整计划状态。错误/取消不能伪装成最终评分成功。
- 在线启用要求持久化、训练隔离和 joint render 引擎（无需 ROS预览发布）；默认行为和旧握手不变。
  最终快照工具仍保留既有 temporal 自动选择；本阶段 spatial 自动选块在在线 C++ 路径实现。

## Task 1：独立来源选择与已提交前缀读取

- [x] RED/GREEN：heldout 不在训练窗口也可选择；preview仍拒绝；历史 owner 只选过去块。
- [x] 提取共用 source 验证/编码；新增独立评价请求类型，使用 frozen Camera。
- [x] 共用现有 spatial 投影逻辑，不复制算法；测试非相邻块、回环/刚体变换与错误来源。
- [x] 扩展 KeyframeReadView 读取有界完整帧，不枚举正在追加的 dataset；保留已提交前缀语义。

## Task 2：独立协议、只读 worker 与资源

- [x] RED/GREEN：严格 hello/capability/limits 及 evaluate/error/ok 帧、字节边界和双语言回环。
- [x] 共用已确认 source 加载/lease/绑定校验，留出 RGB/depth与完整 lineage 校验。
- [x] 只读单/双块推理，记录 online_snapshot 的封口/派发/as_of及评分开销；浮点产物可复算。
- [x] 磁盘预算不足/CUDA OOM等为评分失败，训练状态和 RNG不变；未知能力不能降级。

## Task 3：owner 调度、完整状态及退出

- [x] RED/GREEN：封口触发、内容/预览公平性、try_lock争用、修复/确认前不派发。
- [x] 派发只持一个评分请求；结果按捕获来源验收，不按最新地图丢弃历史样本。
- [x] 状态写盘走相同 worker；未完成/失败/取消/not_in_scope覆盖全部预选查询。
- [x] 参数统一读取 config/gaussian.yaml，ROS1/ROS2共享；Stop/Finalize保留STOPPED与原预算。

## Task 4：回归与真实基线

- [x] Python/CUDA、SDK/双ROS构建、受控公平性/退出进程检查；独立审查并修复重要问题。
- [x] 先用已有数据冻结约35秒前缀，新模型从空运行，至少一个组在结束前封口；审计as_of、
  全部训练历史、浮点样本、状态终态和源数据。磁盘只保留必要小型产物，不复制 bag。
- [x] 文档明确在线历史回看和最终只读评分的区别、空间策略与实测限制；本地提交，不推送。
