# 最终在线资产的留出评分

本阶段使用在线过程中已经训练并封口的 Gaussian 权重，对冻结的留出相机评分。
不重新训练模型，也不改变优化器、参数或训练 RNG。它是最终快照的历史回看，
在线封口触发的任务见[在线留出评估](gaussian-online-evaluation.md)；两者都不是图像到达时的预测结果。

[训练隔离](gaussian-holdout-isolation.md) ·
[总体设计](superpowers/specs/2026-11-26-online-heldout-evaluation-design.md) ·
[实施计划](superpowers/plans/2026-11-26-gaussian-heldout-scoring.md)

## 复现流程

输入需要上阶段启用 `gaussian.evaluation.enabled` 后产生的完整目录：

- Gaussian 目录：`split.json`、`map.json`、`status.json`、成功的 `finalization.json`
  和引用的 `states/<asset_id>/` 三文件资产。
- 同环境的 keyframes 目录：保留原始帧记录、PNG、PCD、分组记录和最终位姿。
- 当前版本的 `livo_atlas_prepare_3dgs_views` 以及已有 Python/CUDA 环境，构建方法见
  [状态工具](gaussian-state.md#环境和数据准备)。

应先正常停止在线节点和 worker。工具要求封口成功、`worker_stopped=true`、地图持久且
没有待修复资产；不会对正在修改的在线目录给出成功报告。原有 retained-camera 预览
接口保持原来的限制，本工具使用独立的留出查询入口。

从包根目录执行，替换为本机绝对路径，输出目录必须不存在：

```bash
export PYTHONPATH="$PWD/tools/3dgs_probe:$PWD/tests/3dgs_probe"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
"$LIVO_PYTHON" tools/3dgs_probe/evaluate_heldout_snapshot.py \
  --map-directory /absolute/path/to/gaussian \
  --keyframes /absolute/path/to/keyframes \
  --prepare-executable /absolute/path/to/livo_atlas_prepare_3dgs_views \
  --config config/gaussian.yaml \
  --output /absolute/path/to/new-scoring-output
```

`LIVO_PYTHON` 使用已有 Python 3.10 / NumPy 1.26 / Torch / gsplat 环境。准备器只导出观测；
其临时 train/validation 分组名是 bundle 格式要求，不会触发训练。模型始终来自已保存
资产。工具记录当前和冻结时的准备器摘要，重建观测仍须匹配冻结身份。

资源设置统一放在 `config/gaussian.yaml`：

```yaml
gaussian:
  joint_render_policy: temporal
  evaluation:
    enabled: false
    split_file: ""
    max_output_mib: 256
    min_free_disk_mib: 1024
```

这里 `enabled` 控制在线训练隔离；显式调用评分工具以资产中已归档的证明为准，
不要求为评分再次启动在线功能。`split_file` 非空时还须与归档划分一致。
GPU 设备与显存预算继续读取 `gaussian.worker`。

`gaussian.joint_render_policy`选择temporal或spatial。temporal与在线保持一致：仅加入
直接前一组，缺少该模型时保留单块，不跨组跳选。spatial从已证明的最终参数用既有
CPU元数据提取器重建extents，调用在线共用的C++投影/排序内核；可选非相邻的历史组，
只考虑owner之前的组。无可见候选明确记录单块及原因，不静默改策略。

spatial需构建并安装`livo_atlas_select_gaussian_neighbor`，显式补充：

```text
--selector-executable /absolute/sdk/bin/livo_atlas_select_gaussian_neighbor
```

未给出时默认查找准备器同目录，但文档中的probe构建与SDK可能在不同目录，应显式指定。
请求/回执分别限制512KiB/64KiB；子进程执行期间用硬文件大小限制约束输出，且有超时。
报告保存工具本体、解析到的工程动态库SHA和组合runtime身份，查询前后及发布前复查。
工具启动环境冻结；重建动态库、更换解析路径或工具变化会使结果失败，不能沿用旧身份。
空间评分按最多两块预留输出，完整Retail_Street建议显式配置1024MiB或更高预算。
详见[完整配对验证](spatial-heldout-comparison.md)。

## 身份、指标与产物

评分先验证完整最终分组与冻结划分，再验证所有资产的 split_id、完整 lineage、
保留观测和 Gaussian origins。查询只能是 manifest 预选的 heldout，RGB/深度/掩码、
标定和时间须匹配。参考 RGB/深度统一为拥有存储的 float32，内参和独立 C_B 使用
manifest 的规范值，再与最终位姿组合。这样同一身份不会因传入 float64 的微小差值
产生不同分数。位姿修正只改变相对坐标，不改写参考标签。

| 指标 | 固定口径 |
| --- | --- |
| RGB PSNR | `[0,1]` RGB，在全部参考有效图像像素累计 MSE；空洞仍计误差 |
| RGB SSIM | 三通道平均，11×11 Gaussian、σ=1.5、K1=.01/K2=.03、population covariance；仅完整有效窗口，无缩放 |
| 图像覆盖 | `alpha >= .05`，分母为全部参考有效图像像素 |
| 深度 L1 / RMSE | 相机 z、单位米；在有效 LiDAR 标签与覆盖交集上计分，另报参考数、使用数和覆盖率 |

封口后还可以从已有浮点评分复算相邻点云的支持度，以及共同覆盖区域的逐视角深度统计。
[离线多帧深度支持评分](gaussian-depth-support.md)

SSIM 依据[作者的单尺度定义](https://ece.uwaterloo.ca/~z70wang/research/ssim/)，本项目明确
选择 RGB 均值和完整掩码窗口；并与 [scikit-image 的官方参数说明](https://scikit-image.org/docs/stable/api/skimage.metrics.html#skimage.metrics.structural_similarity)
对应实现核对。它不是作者默认灰度/缩放流程的成绩。完美 RGB 的 PSNR 写 null 并置
`rgb_perfect=true`；无深度交集或无完整 SSIM 窗口写 null 和原因，不输出 NaN/Infinity。

输出包含：

- `report.json`：完成标记，含每个预选查询的终态、来源/内容/资产、as_of 前沿、相机、
  逐查询指标和累积量；总体 PSNR 从加权 MSE 推导，SSIM 按有效窗口加权。
- `split.json`：本次使用的冻结划分。
- `views/<ID>/samples.npz`：float32 参考 RGB/深度、掩码，以及 single/joint 的浮点
  RGB、alpha、深度。PNG 仅供查看。报告记录样本文件大小和 SHA-256。
- 单块总体、配对单块和联合块汇总分开，不能直接比较样本集合不同的均值。

只有 `report.json` 的成功状态表示本轮评分完成。模型缺失明确跳过，不能补用 heldout
训练。预算或准备器预检失败时，不创建输出目录；若归档划分可验证，stderr 输出完整
查询计划的结构化失败记录。开始后发生失败或来源变化时，全部结果作废、移除待发布
标记，并保留失败报告；磁盘不可写时仍通过 stderr 返回失败计划。

预算先覆盖所有查询与临时准备空间，执行中及发布前重查空闲余量，不静默删掉查询。
`scorer_rss_peak_bytes` 仅为 Python 评分进程峰值，不含 native 准备器子进程。

## 本机基线与限制

证据目录为工作空间下 `validation/heldout-scoring-20261126`。使用上阶段
`noetic-verified` 的 5 帧 / 1 块资产；留出 ID 1，模型完整训练历史为 `[0,2,3,4]`。
同一最终模型重复只读推理，RGB/alpha/z 最大差值均为 0；全部样本指标可复算，原模型
及前端文件摘要保持不变。独立 CUDA 用例还确认活跃模型的参数、Adam 状态以及
CPU/CUDA/NumPy RNG 在双块评估前后不变。

| 留出 ID 1，单块 | 实测 |
| --- | --- |
| RGB PSNR | 8.07227 dB |
| RGB SSIM | 0.271273 |
| 图像覆盖 | 78.7127% |
| 深度 L1 / RMSE | 0.122282 / 0.547475 m |
| SSIM 有效窗口 | 312,658 |

SSIM 的 8 组随机参考对照最大差约 `7.4e-17`；实际掩码图像对照约 `7.8e-16`。
scikit-image / SciPy 仅用于独立验证，不是新增运行依赖。

本轮 Python/CUDA 全量回归 272 项、SDK CTest 81 项、回放工具 41 项通过。
SDK 中原有条件化 GPU 集成用例沿用跳过条件；实际 CUDA 覆盖来自上述 Python 用例
及本次真实只读评分。C++ 和 ROS 接入代码本轮没有改变。

这是单个留出视角、四帧训练后的短段基线，不能用于证明完整 bag、空间选块或在线
画质提升；深度是 LiDAR 参考一致性，不是绝对几何真值。真实数据本轮只有单块；
非相邻双块与回环后的坐标一致性由受控 CPU/CUDA 用例覆盖。
