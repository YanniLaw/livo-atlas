# Gaussian 留出查询与评分实施计划

> 使用 superpowers:executing-plans 在当前授权的开发分支执行；先写失败测试，最终做一次
> 独立审查。本阶段不接入在线 mailbox，不重新训练模型。

**Goal:** 对已停止并封口的在线 Gaussian 资产执行独立留出查询，生成可复算的评分。
**Spec:** [在线留出评估设计](../specs/2026-11-26-online-heldout-evaluation-design.md)。
**Architecture:** 新的独立查询入口接收冻结 manifest 与最终快照，不放宽 retained-camera
预览接口；复用现有坐标变换和双块 rasterizer。先完成最终只读评分，后续在线调度调用
相同查询/指标原语。逐查询及总报告明确 `phase=final_snapshot`。
**Tech:** 现有 Python 3.10、NumPy、Torch/gsplat、C++ reconstruction-view 准备器。

## Global constraints

- 仅修改 livo-atlas；不新增 CUDA worker、训练模型、Pimpl 或编译宏。
- owner/前端和原有 ROS READY 协议不变。关闭 evaluation 的既有路径不变。
- 查询必须是 manifest 预选 heldout，原始文件及重建观测均匹配；所有选中资产有相同
  split_id 和完整 lineage。相机用独立 C_B 和最终绝对位姿推导，不能借用训练相机。
- 配置仍放 config/gaussian.yaml；固定指标口径不开放调参。磁盘预算不足不能静默删查询。
- 工具默认按 temporal 的 owner+上一可用块评分。空间自动选块属于在线接入下一阶段；
  当前配置请求 spatial 时明确拒绝，不能偷偷退回 temporal。查询原语本身接受显式
  一至两块，测试覆盖非相邻双块；后续复用现有 C++ spatial 选择器，不另写一份策略。
- 全部预选查询必须有成功/明确跳过/失败终态；不能把最终评分称为到达时即时预测。

## Review focus

- heldout 不在保留窗口仍可查询，训练视图不能冒充 heldout；旧状态不能补造证明。
- 回环修正只改变查询和块的相对坐标，不改变 RGB/深度标签；快照变化不得发布成功报告。
- 空 alpha 仍计 RGB 误差；没有深度、没有完整 SSIM 窗口时写 null 和原因。
- 保存 float32 RGB/alpha/depth、参考 RGB/depth/masks，PNG 不用于精确复算。
- 预算、来源变化和 CUDA 错误须有清晰失败记录；不碰原始模型文件。

## Task 1：纯指标与汇总

Files: tools/3dgs_probe/evaluation_metrics.py,
tests/3dgs_probe/test_evaluation_metrics.py。

- [x] 先写失败测试：同图/变色、空覆盖、无深度、小图、掩码孔洞、非有限输入。
- [x] 实现 RGB MSE/PSNR、11x11 σ1.5 RGB SSIM、覆盖和深度累积；按像素/窗口加权汇总。
- [x] 与独立逐窗口公式及可用的 scikit-image 官方实现核对，不新增运行依赖。

## Task 2：独立查询与来源检查

Files: tools/3dgs_probe/heldout_query.py；最小修改 holdout_split.py、gaussian_map.py；
tests/3dgs_probe/test_heldout_query.py。

- [x] 失败测试：未留出/未预选查询、污染资产、错误相机、修改源文件/位姿/资产。
- [x] 分离冻结重建观测检查与训练角色检查；保留原训练入口的拒绝语义。
- [x] 复用最终地图验证并逐状态调用 split 验证，不复制整个地图参数缓存。
- [x] 构造明确 as_of / 内容 / 资产 / 相机来源，双块只读渲染前后检查输入不变。

## Task 3：最终快照评分工具

Files: tools/3dgs_probe/evaluate_heldout_snapshot.py、config/gaussian.yaml；
tests/3dgs_probe/test_evaluate_heldout_snapshot.py。

- [x] 先测配置、预算、缺失模型、失败终态与拒绝在线/未封口输入（CUDA 初始化前）。
- [x] 全部预选查询使用当前准备器逐组准备并逐查询校验；从已保存模型直接渲染。
- [x] 读取 evaluation.max_output_mib/min_free_disk_mib；输出浮点样本、逐查询/汇总报告。
- [x] 从保存样本独立复算；执行一次受控双块 CUDA 查询并验证参数/优化器/RNG 不变。

## Task 4：真实验证、文档与本地提交

- [x] 对上阶段 noetic-verified 最终资产评分，不重放/复制 bag、不改变冻结划分。
- [x] 核对原始资产和前端文件前后摘要，重复只读推理记录波动与资源；保留少量产物。
- [x] 回归原有训练隔离、预览/状态工具，独立审查并修复重要问题；记录已实现与剩余阶段。
- [x] 按用户授权本地提交，保留当前分支，不推送或合并。
