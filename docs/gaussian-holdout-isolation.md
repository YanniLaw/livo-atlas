# 在线 Gaussian 留出数据隔离

第一阶段已实现训练前冻结划分、双端接纳检查和完整训练历史。它为后续在线留出评分
建立来源边界；当前没有在线留出渲染、PSNR/SSIM 成绩或最终状态只读评分工具。
[设计](superpowers/specs/2026-11-26-online-heldout-evaluation-design.md) ·
[实施计划](superpowers/plans/2026-11-26-gaussian-holdout-isolation.md)

## 数据与配置

先用相同 ROS、前端配置和依赖环境产生关键帧数据集，再准备 manifest，最后启动新的
Gaussian 会话。准备阶段不读取旧 Gaussian 权重。Noetic 与 Humble 的关键帧身份分别
准备，不能将两个环境的同一个 ID 当成同一观测。

manifest 冻结连续前缀：`id % 8 == 1` 为 heldout，其余为 train。每个 primary group
按 ID 最多选两个 heldout 查询；未选中的 heldout 同样禁止训练。全局 SLAM 仍使用完整
图像和 LiDAR 流，primary 成员与位姿也保持完整，heldout 可以成为子地图锚点。

使用当前源码编译的 `livo_atlas_prepare_3dgs_views`；旧二进制若没有导出
`parent_from_body` 字段，需要重新编译。准备工具构建步骤见
[状态工具文档](gaussian-state.md#环境和数据准备)。在包根目录运行：

```bash
"$LIVO_PYTHON" tools/3dgs_probe/prepare_holdout_split.py \
  --dataset /absolute/path/to/reference/keyframes \
  --preparer /absolute/path/to/livo_atlas_prepare_3dgs_views \
  --input-file /absolute/path/to/Retail_Street.bag \
  --frontend-profile 'Noetic; revision=...; Eigen=...; PCL=...; OpenCV=...; frontend settings=...' \
  --frame-count 5 \
  --output /absolute/path/to/new-split.json
```

`LIVO_PYTHON` 使用已有的 Python 3.10 / NumPy / Pillow / PyYAML 环境；准备过程不需要
GPU。`frontend-profile` 是来源记录，请填写实际版本及前端设置；它不会自动探测另一台
电脑的驱动或编译器。工具流式计算 bag 摘要、逐批准备视图并删除临时载荷，不复制 bag。
前缀需要 2～512 帧；运行超出冻结范围会明确失败，应在训练前选择正确范围。

节点的设置统一在 `config/gaussian.yaml`：

```yaml
gaussian:
  enabled: true
  persist_submaps: true
  evaluation:
    enabled: true
    split_file: /absolute/path/to/new-split.json
```

同时按既有流程启用 keyframes 和 online submaps，配置新的输出目录与 worker socket。
两个 ROS 回放脚本也提供 `--gaussian-evaluation-split /absolute/path/to/new-split.json`，
它要求 `--gaussian-persist-submaps`。普通启动流程见 [在线建图](online-gaussian.md)。

## 检查边界与兼容性

manifest 绑定 bag、准备器、标定/分组配置和逐帧原始记录、PNG、PCD 的 SHA-256，
以及实际 Gaussian 重建观测的 RGB、深度、掩码、内参和独立相机外参。规范 JSON
生成 `split_id`。跨 train/heldout 的重复 ID、时间或图像会被拒绝。

owner 启动时读取有界索引，逐帧核对 ID、时间、标定身份和分组；heldout 不覆盖 pending
训练帧。异步发送前再次核对训练角色。完整像素/深度摘要在 worker 接纳训练前检查，
不进入前端 owner 循环。worker 也要求目标包含冻结前沿下该组的完整有序成员，不能
改变锚点或省略成员。图像缺失、输入超限或无可用深度不能作为普通跳过后继续验收成功。

RGB 浮点数组和深度按实际训练值绑定。外参先按 native 路径投影至 SO(3)，外参/内参
分别规范到 8/10 位小数并统一零的符号，以容纳导出和在线矩阵计算的舍入差异。
原始 PNG/PCD 文件摘要提供来源绑定；在线摘要比较的是实际用于 Gaussian 的重建观测，
不声称逐字节比较了在线原始 PCD。未参与训练的 heldout 像素将在后续独立查询入口
检查；本阶段不声称已经验证或评分这些查询像素。

新能力通过 `holdout_split_v1` 明确协商，manifest 由 hello body 传给 worker，因此
worker 不必使用与节点相同的文件系统路径。两端必须返回匹配的能力、`split_id` 和
state schema。开启时，输出目录保存 `split.json`，确认/持久状态使用 **schema 2**，
保存完整 lineage 和其 SHA-256。lineage 包含实际播种及损失调度的所有 ID，已经移出
十帧窗口的历史 ID 也不能丢掉。拒绝候选会恢复确认态的 lineage；重建保守保留历史
并只读取已证明的训练观测。恢复检查还覆盖保留视图、Gaussian origins 与完整成员。

启用隔离时，无证明的 schema 1、不同划分或污染历史均不能恢复。关闭功能时，原有
握手、schema 1 资产与训练路径保持兼容，不给旧资产伪补来源证明。这不是完整 SLAM
session 重启恢复功能。

## 本机验证

证据目录：`validation/holdout-isolation-20261126`（工作空间根目录下）。

- SDK：81 个 CTest；Python/CUDA 状态、协议、训练和隔离回归：101 项；双 ROS 回放
  CLI：41 项。新增失败用例覆盖未请求的能力升级、缺失图像/深度/超限点云、错误锚点、
  不完整成员、未知前沿、近似旋转和带符号零。
  SDK 中需额外 GPU 环境变量的集成用例沿用原有跳过条件；实际 CUDA 覆盖来自上述
  Python 测试及本次短段回放。
- Noetic：19 项共享参数测试、7 项退出相关 CTest。Humble：21 项共享参数测试、
  9 项退出/隔离相关 CTest；本阶段未做 Humble 的真实留出 GPU 回放。
- Noetic 10 秒真实回放使用预先冻结的 5 帧前缀：heldout 为 `[1]`，实际训练、保留
  窗口及完整 lineage 均为 `[0,2,3,4]`。四次训练共 80 步，模型 allocated 峰值约
  61.3 MiB，reserved 约 86 MiB；进程峰值 RSS 约 1.08 GiB。
- `noetic-verified-gaussian/finalization.json` 为 succeeded，STOPPED 为真，missing、
  dirty、needs_rebuild 均为零。帧 0～4 的记录、图像、点云共 15 个文件与同环境未启用
  evaluation 的前次回放逐字节一致。回放后只整理 C++ 格式及失败时的诊断文字，
  原回放快照保持原样。
- 本轮冻结 `split_id`：`e39a25d42b1f24a0095a2adc51a6c1e7ad2b477aff51be91c142b310591ad668`。

短段中没有实际回环重建；重建和恢复隔离由 GPU/状态用例覆盖。本次结果不能代替完整
bag、长距离或跨环境验证，更不能用现有训练预览指标代替留出画质成绩。
