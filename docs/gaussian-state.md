# Gaussian 工作集保存与恢复

P6-03c2a 为已确认的 Gaussian 工作集提供独立资产：模型参数、保留视图、来源关键帧和
局部坐标可一起保存，在不加载 GPU 库时检查，再在新的模型实例中恢复和继续训练。
它保留最近最多 10 个视图，覆盖范围与 C1 工作集相同。

本阶段工具用于离线验证；不会自动保存运行节点的全部子地图，也不恢复 ROS、前端或后端
session。在线多块索引与闭环修正属于后续 C2b。方案见
[设计](superpowers/specs/2026-11-17-p6-03c2-gaussian-submaps-design.md)和
[实施计划](superpowers/plans/2026-11-17-p6-03c2a-gaussian-state.md)，实测见
[验证记录](p6-03c2a-validation.md)。

## 环境和数据准备

CPU 检查需要 Python 3.10 和 NumPy；GPU 往返还使用现有 PyTorch、Pillow 和预编译 gsplat。
本机验证组合为 NumPy 1.26.4、PyTorch 2.3.0+cu121、gsplat 1.5.3，环境准备沿用
[GPU 实验说明](p6-03b-gpu-probe.md)。不需要给默认 C++ 核心或 ROS 节点添加 GPU 库。

GPU 启动要求至少 4 GiB 空闲显存，PyTorch 分配比例为总显存 40%；缺少 GPU 或预编译
`gsplat.csrc` 明确失败，不执行 CPU 假训练或隐式 JIT。该比例不约束其他进程和驱动分配。

先用已安装的普通 SDK 构建独立准备工具；路径按当前电脑替换，编译单任务运行：

```bash
export LIVO_REPO=/path/to/workspace/src/livo-atlas
export LIVO_PYTHON=/path/to/3dgs-env/bin/python
export LIVO_SDK=/path/to/ordinary-sdk-install
export LIVO_PHASE=/path/to/new-validation-directory
mkdir -m 700 "$LIVO_PHASE"
cmake -S "$LIVO_REPO/tools/3dgs_probe" -B "$LIVO_PHASE/prepare-build" \
  -DCMAKE_PREFIX_PATH="$LIVO_SDK" -DBUILD_TESTING=ON
nice -n 10 cmake --build "$LIVO_PHASE/prepare-build" -j1
```

输入是之前保存的原始关键帧数据集。下面用连续 ID 0–11 做一个 probe 分组，训练帧与
留出帧分开；该分组标记不等于数据集的正式在线子地图编号。先生成并核验来源文件的
`source.sha256`，至少涵盖 `dataset.yml`、准备器读取的 frame 元数据和所选图像/点云。
该清单中的路径相对于数据集根目录，文件本身保存在验证目录。

```bash
export LIVO_DATASET=/path/to/saved-keyframes
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
LIVO_SOURCE_SHA=$(sha256sum "$LIVO_PHASE/source.sha256" | cut -d' ' -f1)
"$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views" \
  --dataset "$LIVO_DATASET" --output "$LIVO_PHASE/bundle" \
  --train-ids 0,1,2,4,5,6,8,9,10,11 --validation-ids 3,7 \
  --source-sha256 "$LIVO_SOURCE_SHA" --max-pixels 1000000 --max-cloud-points 100000
```

新导出视图包括 `parent_from_body`。旧 bundle 仍可供原离线训练使用，但本工具不会猜测
缺失的 body 锚点；遇到旧格式应重新导出。单个 probe 最多 10 个训练、2 个留出视图，
所有输入 ID 合起来必须连续，锚点是第一个主成员；每个视图最多 1,000,000 像素。

## 保存、释放和恢复验证

```bash
cd "$LIVO_REPO"
env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 \
  timeout --signal=TERM --kill-after=10 180s nice -n 10 "$LIVO_PYTHON" \
  tools/3dgs_probe/state_tool.py roundtrip --bundle "$LIVO_PHASE/bundle" \
  --output "$LIVO_PHASE/roundtrip" --anchor-id 0 --submap-id 0
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
```

工具按 ID 升序训练前 N−1 个训练帧，每帧训练后确认，再保存状态。随后删除原模型并检查
张量分配量下降，读取资产、建立新 GPU 模型，比较参数、观察和渲染，再用最后一帧完成
20 步继续训练。留出帧不参与高斯初始化、损失或 Adam；这里不将留出渲染质量作为通过条件。
CUDA context 和 allocator 缓存仍可存在，“冷恢复”具体指原模型已释放后创建新实例。

输出目录必须不存在且与输入互不包含：

```text
roundtrip/
  state/
    manifest.json
    gaussians.npz
    observations.npz
  before.png
  restored.png
  continued.png
  report.json
```

报告记录 `capture_kind: offline_probe`、来源摘要、模型身份、参数/观察逐位一致性、模型
释放前后 allocated 字节数、重载渲染误差、继续训练的梯度/更新、内存与软件版本。
浮点 RGB/alpha 最大绝对差门限 `1e-6`，相机 z 深度 `atol=1e-5 m, rtol=1e-6`，
RGB8 最大差 1；这些是同环境恢复正确性检查，不是跨 GPU 确定性或画质承诺。

## 无 GPU 检查与恢复 API

检查资产的 schema、摘要、受限数组和来源不需要导入 torch/ROS：

```bash
env -u PYTHONPATH "$LIVO_PYTHON" tools/3dgs_probe/state_tool.py inspect \
  --state "$LIVO_PHASE/roundtrip/state"
```

Python 模型接口位于 `tools/3dgs_probe/online_model.py`：
`capture_state(context)` 只捕获非空、已确认的模型；`from_state(state, device)` 返回新的
模型，使用保存的 log/logit 原值，四元数保持不可训练，Adam 初始为 None，下一帧按原规则
建立新优化器。`render_preview(keyframe_id)` 返回保留视图的独立 RGB8 图像。
CPU 读写接口及普通 `GaussianState` dataclass 在 `gaussian_state.py`。

context 由调用方提供完整主成员位姿、身份和独立 `source_cameras`。旧视图的前沿/成员
修订可以早于捕获值，但会话、子地图、generation、位姿版本和相机关系必须一致。
来源关系不能从当前窗口反向充当完整历史，保存的 generation 也不能直接授权新在线会话。
精确字段及坐标约定见设计文档。

## 格式、上限与失败

资产参数是 S 局部系中的均值、wxyz 四元数、对数尺度、RGB logit、透明度 logit 和来源
uint64；观察保存原始 float32 RGB、掩码和米制相机 z 深度。首个主成员的 body 定义 S，
外部 `parent_from_submap` 独立保存；不重复施加 LiDAR 外参或 0.5 像素主点转换。

当前限制为前沿/主成员最多 512 帧、保留 10 视图、10,000 个高斯、每来源最多 1,000 个。
文件大小上限分别为清单 512 KiB、参数 1 MiB、观察 192 MiB。只接受固定字段、小端数值、
无压缩/无 pickle NPZ，先核对摘要、ZIP 目录与 NPY 头，再分配数组；这些不等于进程 RSS 上限。

写入只创建新目录，`manifest.json` 最后发布。此前失败不会留下最终清单；清单发布后若
目录同步失败，返回非零并报告 `commit_uncertain`，已经可见的完整资产保留，不自动回滚或
覆盖。CPU reader 的成功只证明当前内容可读，不能反推此前已确认断电持久性。
普通失败报告 `failed`；进程被外部强制终止时可能没有完整报告。部分目录不作为有效资产，
再次试验选新目录。运行中不要改写资产文件。
