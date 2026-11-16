# P6-03c2a：可恢复 Gaussian 模型资产实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans for the previously chosen inline execution. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保存一个已确认 Gaussian 工作集，使它在不导入 GPU 库时可检查，并能在新 GPU
实例中恢复相同参数、渲染和继续训练。

**Architecture:** JSON 身份与来源清单，配合无压缩、无 pickle 的 NPZ 参数及观测文件。
复用独立 Python/gsplat 模型；只给离线观测准备工具增加实际 body 位姿元数据。
本计划不接在线目录或 ROS 参数，多子地图在线调度属于 C2b。

**Tech Stack:** Google C++17、已有普通 SDK/OpenCV、Python 3.10、NumPy 1.26.4、
PyTorch 2.3.0+cu121、gsplat 1.5.3、unittest。

**Spec:** [C2 设计与 C2a 实施边界](../specs/2026-11-17-p6-03c2-gaussian-submaps-design.md)。
基线为 `2708aef`；设计与本计划先写入仓库，正式编码前通知用户切换模型。

## Global Constraints

- 只修改 livo-atlas；不改变前端、共用 main、ROS1/ROS2 接入、C1 IPC 和默认构建依赖。
- C++ 用 Google C++17 风格、函数实现间空一行，说明坐标和生命周期；不用 Pimpl。
- 候选未确认或模型为空不能捕获。存档不是完整在线 SLAM session，不自动重连。
- 单视图 ≤1,000,000 像素；最多 10 个视图、10,000 个高斯、每来源 ≤1,000 个高斯；
  前沿 ≤512，主成员 ≤512。每个新视图仍训练 20 步，恢复后重建 Adam。
- 清单 ≤512 KiB；gaussians.npz ≤1 MiB；observations.npz ≤192 MiB；NPY 头 ≤4 KiB。
  接受 ZIP_STORED 和 NPY 1.0/2.0、C 顺序、小端数值，不接受 pickle/object/压缩成员。
- GPU 要求预编译 gsplat.csrc、起跑空闲显存 ≥4 GiB、PyTorch 分配比例 40%；缺 GPU
  明确失败。CPU 格式读写/inspect 不导入 torch、CUDA 或 ROS。
- 资产输出要求此前不存在，清单最后提交；拒绝覆盖、符号链接和不完整资产。
  清单重命名为可见性提交点；之后目录同步失败报 CommitUncertainError，保留可读资产，
  CLI 返回非零的 commit_uncertain；不回滚已可见文件。
  CLI 输入与输出互不包含；数据与验证结果写包外新目录，保留旧证据。
- 渲染往返固定误差：浮点 RGB/alpha `atol=1e-6, rtol=0`，z 深度
  `atol=1e-5 m, rtol=1e-6`，RGB8 最大差 1；不通过时排查而非放宽门限。

## 文件职责与共享接口

- `tools/3dgs_probe/prepare_views.cc`：新导出视图附带原始父系 `parent_from_body`。
- `tools/3dgs_probe/bundle.py`：验证并返回该可选字段；旧 bundle 仍可读。
- 新 `tools/3dgs_probe/gaussian_state.py`：纯 CPU dataclass、域校验、受限格式读写。
- `tools/3dgs_probe/online_model.py`：捕获独立 CPU 状态、新 GPU 实例恢复、确认模型预览。
- 新 `tools/3dgs_probe/state_tool.py`：无 GPU inspect；离线真实数据往返及继续训练验证。
- `tests/3dgs_probe/`：已有准备工具测试，以及新增格式、模型恢复和 CLI 测试。
- 新 `docs/gaussian-state.md`、`docs/p6-03c2a-validation.md`：用法、限制和实际证据。

`GaussianState` 为普通 dataclass：`metadata: dict`、`parameters: dict[str, np.ndarray]`、
`origin_ids: np.ndarray`、`views: list[dict]`。parameters 是 5 个原始模型参数，不包含
origin IDs；metadata 包含身份、完整 primary 局部 body 位姿和来源，不包含
`files/views/gaussian_count`。视图字典包含各自元数据与 RGB/掩码/深度/K/局部相机位姿。
writer 派生完整 manifest，reader 拆回相同内存结构；不把 GPU tensor 放进 dataclass。

```python
# gaussian_state.py — only standard library and NumPy imports
def validate_state(state: GaussianState) -> None: ...
def write_state(path: Path, state: GaussianState) -> None: ...
def read_state(path: Path) -> GaussianState: ...
class CommitUncertainError(OSError): ...

# online_model.py
def capture_state(self, context: dict) -> GaussianState: ...
@classmethod
def from_state(cls, state: GaussianState, device) -> "OnlineGaussianModel": ...
def render_preview(self, keyframe_id: int) -> np.ndarray: ...  # owned RGB uint8
```

context 按 Spec 提供 metadata 去掉 `schema/kind/representation/optimizer/software` 后的
字段，再加独立 `source_cameras`，变换为 JSON 可表达的列表。该独立关系来自变换前的
source camera/body 数据，不得从待校验的 T_C_S 和 T_S_B 反推后自行通过。
context 对完整 primary 负责，捕获器只校验可检测的结构、身份与来源冲突；旧视图的
frame_count/membership_revision 允许落后，其他字段按 Spec 责任表核对，不静默覆盖冲突。
reader 返回拥有数据的数组；捕获和恢复不能复用可被原模型训练/reset 修改的存储。

## Review Focus

1. 未确认候选或过期会话身份被当作可恢复在线地图：Task 3 禁止 pending 捕获；Task 4
   固定 offline_probe，报告不得宣称已在线持久化。
2. 合法大小 ZIP 内夹带巨型 NPY、压缩/重名成员或 object dtype：Task 2 在数组分配前
   核对固定字段、NPY 头、shape 和实际字节数，并测试失败顺序。
3. 非单位 body 锚点造成双重外参、相机位姿方向或 0.5 偏移错误：Task 1/4 使用非平凡
   旋转平移，检查局部变换等式及原始像素/深度/K 保持一致。
4. 捕获数组借用模型缓冲、恢复重复 log/logit 或错误带回 Adam：Task 3 检查独立性、
   参数逐位相同、optimizer 为 None、下一次 append 重建优化器。
5. 提交前失败留下可见清单、既有目录被覆盖或输入资产被改写：Task 2 区分提交前错误与
   提交后同步不确定，Task 4 核验真实输入前后哈希及失败 CLI 的退出状态。

## 环境与验证变量

实施时建立新目录，例如 `validation/P6-03c2a-YYYYMMDD/`，不预先创建输出资产目录。
保留当前 Python 环境，串行 `nice -n 10` 编译/运行；编译前检查分区至少 10 GiB 空闲。

```bash
export LIVO_REPO=/path/to/workspace/src/livo-atlas
export LIVO_PHASE=/path/to/workspace/validation/P6-03c2a-YYYYMMDD
export LIVO_PYTHON=/path/to/isolated-env/bin/python
export LIVO_SDK=/path/to/installed-ordinary-sdk
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
# 测试进程中显式使用：
export PYTHONPATH="$LIVO_REPO/tools/3dgs_probe:$LIVO_REPO/tests/3dgs_probe"
export LIVO_FIXTURE_BIN="$LIVO_PHASE/prepare-build/p603b_make_fixture"
export LIVO_PREPARE_BIN="$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views"
cd "$LIVO_REPO"
```

本机可复用 `validation/P6-03b-20261116/env/bin/python`，但其他电脑必须按
[GPU 环境说明](../../p6-03b-gpu-probe.md)准备并核验；不硬编码这些绝对路径到产品代码。

### Task 1: 为离线观测保留 body 位姿依据

**Files:** 修改 `tools/3dgs_probe/prepare_views.cc`、`bundle.py`、
`tests/3dgs_probe/make_fixture.cc`、`test_bundle.py`。

**Interfaces:** 每个新导出视图增加可选 `parent_from_body`：行优先 16 个 float64 数值，
是该原始视图的 `T_parent_B`。`load_bundle(path) -> list[dict]` 原签名不变；字段存在则
验证 SE(3) 并返回 `<f8[4,4]`，不存在则不创建猜测值。新工具必须要求此字段。

- [x] **Step 1: 写 RED。** 新增 `test_optional_body_pose_preserves_old_bundle` 和
  `test_nonidentity_body_pose_matches_source_and_camera`。默认 fixture 的 body 为单位阵，
  删去新字段后的旧 bundle 仍可读；非有限/非 SE(3) 的新字段拒绝。
  fixture 新增仅测试用的 `--nonidentity-body` 可选参数：Rz(90°)、平移 `(4+id,-2,1)`，
  同时调整 camera_from_odom 保持 camera_from_body 和原始像素/深度不变。
- [x] **Step 2: 运行 RED。** 在未改实现时构建原准备工具并运行
  `$LIVO_PYTHON -m unittest discover -s tests/3dgs_probe -p test_bundle.py -v`。
  Expected: 新字段/fixture 能力缺失导致新用例失败，已有用例保持通过。
  构建命令为 `cmake -S tools/3dgs_probe -B "$LIVO_PHASE/prepare-build"
  -DCMAKE_PREFIX_PATH="$LIVO_SDK" -DBUILD_TESTING=ON`，随后
  `nice -n 10 cmake --build "$LIVO_PHASE/prepare-build" -j1`。
- [x] **Step 3: 实现。** 导出时使用同一个已读 Keyframe 的 `odom_from_body`，不再次读取
  新的位姿视图；增加可选字段读取，保留旧 CLI 参数与 bundle schema。
- [x] **Step 4: GREEN。** 单任务构建与运行 Step 2 全部用例；检查准备工具 `ldd`
  无 ROS/vikit/CUDA/torch 依赖。Expected: 实际 C++→Python 数值往返通过。
- [x] **Step 5: 提交。** `git diff --check`，提交来源元数据及测试。

### Task 2: 纯 CPU 资产契约与提交边界

**Files:** 新建 `tools/3dgs_probe/gaussian_state.py`、
`tests/3dgs_probe/test_gaussian_state.py`。

**Interfaces:** 产出共享 `GaussianState`、`validate_state`、`write_state`、`read_state`。
精确字段、dtype、上限、路径和提交规则以 Spec 为准。纯数据格式与 C1 IPC 独立版本化。

- [x] **Step 1: 写 RED。** `test_state_roundtrip_preserves_parameters_views_and_primary_poses`
  使用 8×6、2 个来源、非单位 parent_from_submap 和 camera_from_body；逐位比较 5 个
  参数、origin IDs、RGB/掩码/深度，检查恢复清单没有绝对输入路径依赖。
  增加 `test_cpu_read_does_not_import_torch` 的独立子进程断言。
  写数据驱动拒绝测试：未知 schema/字段/策略、错 dtype/shape、N>10,000、第 11 个视图、
  巨大 NPY shape、压缩/重名 ZIP、object dtype、错摘要、截断/缺文件、非法来源或 SE(3)、
  坏掩码深度。对超限头测试断言没有进入 `numpy.load` 或大数组分配。
  `test_failure_before_commit_never_publishes_manifest` 注入提交前写入、close、文件 fsync、
  rename 错误；`test_directory_sync_failure_reports_uncertain_readable_commit` 验证提交后
  任一目录 fsync 失败抛 CommitUncertainError、资产可读且未回滚；
  `test_existing_symlink_and_partial_outputs_are_not_overwritten` 保留已有标记文件。
- [x] **Step 2: 运行 RED。**
  `$LIVO_PYTHON -m unittest discover -s tests/3dgs_probe -p test_gaussian_state.py -v`。
  Expected: 模块/接口缺失，而非测试数据自身格式错误。
- [x] **Step 3: 实现。** 验证小型元数据，再验真实文件/ZIP/NPY 边界，最后读数组。
  文件摘要分块；输出独占 mkdir，最终清单为提交标志。NPZ 用固定命名、无压缩数值数组，
  reader 使用 `allow_pickle=False`；NumPy 1.26 的 savez 不传此参数，先校验数值 dtype。
  公共 reader 返回自有数组，提交前错误不留下最终清单，提交后失败按不确定状态报告。
- [x] **Step 4: GREEN。** Step 2 全部用例通过；运行 bundle 回归，验证格式模块导入
  不加载 GPU/ROS。记录最大合法数组 payload 测试与实际 RSS，不把 payload 上限称为 RSS。
- [x] **Step 5: 提交。** 提交格式、CPU 校验与故障测试。

### Task 3: GPU 捕获、恢复与确认模型渲染

**Files:** 修改 `tools/3dgs_probe/online_model.py`，新建
`tests/3dgs_probe/test_model_state.py`。

**Interfaces:** 消费 Task 2 的 dataclass/校验；实现共享 `capture_state(context)`、
`from_state(state, device)` 和 `render_preview(keyframe_id)`。CPU 格式模块不得反向导入模型。

- [x] **Step 1: 写 RED。** `test_capture_rejects_empty_and_pending_candidates`：训练候选
  后捕获失败，`decide(True)` 后成功；`decide(False)` 后空模型仍拒绝。
  `test_capture_owns_arrays_after_model_training_and_reset`：捕获后继续训练/reset，捕获
  数组和 context 不变。`test_restore_preserves_raw_parameters_and_renders`：保存/释放/
  重载，参数逐位一致、RGB8 最大差 ≤1，浮点渲染遵守全局门限。
  `test_restore_resets_adam_and_continues_bounded_training`：恢复时 optimizer 为 None，
  第 11 帧按原规则淘汰，10,000 高斯上限、20 步及非零梯度/更新均成立。
  `test_bad_state_is_rejected_before_cuda_allocation` 先做 CPU 验证，
  `test_missing_gpu_fails_without_cpu_fallback` 明确失败。
  `test_capture_checks_independent_camera_relation` 在源关系不变时只改 context 中合法的
  body 姿态，必须拒绝；`test_capture_accepts_older_frontiers_but_rejects_identity_conflicts`
  覆盖正常旧前沿/成员修订和错误 session/generation/版本/锚点。测试不宣称能从窗口证明
  调用方给定的历史 primary 完整。
- [x] **Step 2: 运行 RED。**
  `LIVO_REQUIRE_GPU=1 CUDA_VISIBLE_DEVICES=0 $LIVO_PYTHON -m unittest discover -s tests/3dgs_probe -p test_model_state.py -v`。
  Expected: 新接口缺失；GPU 可用性已独立确认，不能把 skip 计通过。
- [x] **Step 3: 实现。** 用 detach/cpu/copy 捕获已确认参数与视图；结合完整 primary
  body 位姿和独立 source_cameras 检查实际训练的局部相机位姿。from_state 完整 CPU 校验后新建 GPU 张量，
  不二次参数化、不恢复 Adam；保留 quats 的不可训练语义。预览只读已确认模型。
- [x] **Step 4: GREEN。** Step 2、现有 `test_online_model.py`、`test_online_worker.py`
  和 `test_rasterizer.py` 全部通过。记录 allocated/reserved、RSS 与参数/渲染差值；
  对保留原模型与释放后恢复分别验证，不允许同时保留两个 GPU 模型来假装冷恢复。
- [x] **Step 5: 提交。** 提交 GPU API 与真实 CUDA 测试。

### Task 4: 独立工具、真实来源往返与跨电脑说明

**Files:** 新建 `tools/3dgs_probe/state_tool.py`、`tests/3dgs_probe/test_state_tool.py`、
`docs/gaussian-state.md`、`docs/p6-03c2a-validation.md`；更新 `docs/README.md`。

**Interfaces:** `inspect --state DIR` 不导入 GPU 模块；
`roundtrip --bundle DIR --output NEW_DIR --anchor-id ID --submap-id ID` 延迟导入模型。
返回码 0 仅用于全部检查成功；新输出根保存 report.json/渲染，`state/` 为三文件资产。
输入缺 body 位姿、非连续 probe 主成员或锚点不是首成员，创建输出前拒绝。

- [x] **Step 1: 写 RED。** `test_inspect_works_when_torch_import_is_blocked`、
  `test_roundtrip_reports_success_and_cold_restore`、
  `test_old_bundle_bad_anchor_and_overlapping_output_are_rejected`、
  `test_missing_gpu_and_corrupt_state_return_nonzero`、
  `test_commit_uncertain_keeps_asset_and_reports_nonzero`。检查明确的 offline_probe、
  origin_session_id、来源摘要、参数逐位一致、渲染误差与继续训练结果。
- [x] **Step 2: 运行 RED。**
  `$LIVO_PYTHON -m unittest discover -s tests/3dgs_probe -p test_state_tool.py -v`。
  Expected: 工具尚不存在或对应 CLI 未实现。
- [x] **Step 3: 实现。** 用 Task 1 的真实 body 位姿构建 anchor 局部 context，并在
  修改相机表达前由原始相机/body 位姿生成独立 source_cameras。
  以训练 ID 升序训练前 N−1 帧并逐次确认，捕获后释放实例与 GPU 张量，再恢复/渲染，
  用剩余末帧继续 20 步；留出帧只用于主成员元数据与评估。输出软件、身份、误差、
  梯度、资源、耗时及成功/失败/commit_uncertain 状态；不把存档叫作完整 SLAM 或在线多块地图。
- [x] **Step 4: GREEN 与真实验收。** 全部相关 Python 测试通过；从现有官方 bag 的
  已提交关键帧数据集重新导出 ID 0–11、训练 `0,1,2,4,5,6,8,9,10,11`、留出 `3,7`。
  来源文件清单在准备和训练前后核验，结果在包外新目录。执行下方命令，实际 cold
  restore 与继续训练通过；记录数组/渲染差值及缺 GPU/损坏文件的非零退出。
  不修改节点，无需为本任务重跑所有 ROS 构建和完整 bag。
- [ ] **Step 5: 文档与提交。** 写跨电脑环境、inspect/roundtrip 用法、恢复层级与预算，
  记录实际来源 SHA-256、软件、硬件、路径和完整日志。运行 Google 格式、头/依赖和文档
  链接检查、受影响的完整测试。提交后对 C2a 实现范围独立审阅一次，重要发现先 RED
  再修复，再复验；保留当前分支，不合并/推送/打 tag。

```bash
# 在已建立 LIVO_PHASE 且准备工具已用新源码构建之后；路径由当前电脑提供。
# LIVO_DATASET 中所选原始文件先列入 source.sha256，并在此处核验。
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
LIVO_SOURCE_SHA=$(sha256sum "$LIVO_PHASE/source.sha256" | cut -d' ' -f1)
"$LIVO_PREPARE_BIN" --dataset "$LIVO_DATASET" --output "$LIVO_PHASE/bundle" \
  --train-ids 0,1,2,4,5,6,8,9,10,11 --validation-ids 3,7 \
  --source-sha256 "$LIVO_SOURCE_SHA" --max-pixels 1000000 --max-cloud-points 100000
env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 \
  timeout --signal=TERM --kill-after=10 180s nice -n 10 "$LIVO_PYTHON" \
  tools/3dgs_probe/state_tool.py roundtrip --bundle "$LIVO_PHASE/bundle" \
  --output "$LIVO_PHASE/roundtrip" --anchor-id 0 --submap-id 0
env -u PYTHONPATH "$LIVO_PYTHON" tools/3dgs_probe/state_tool.py inspect \
  --state "$LIVO_PHASE/roundtrip/state"
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
```

## Self-review

来源 body 位姿由 Task 1 提供；格式、预算及提交边界由 Task 2 实现；模型确认与冷恢复由
Task 3 实现；CLI 和实际数据证据由 Task 4 接上。同一名称/参数化/像素中心贯穿所有任务。
C2b 多块驻留、在线索引和实际闭环修正没有列为 C2a 已覆盖项。
执行仍采用当前会话原生逐项实施、最后一次独立整阶段审阅的方式；不新增实现代理链。
