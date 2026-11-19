# C4a Held-out Gaussian Rendering Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans with the current session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在固定封口位姿上生成可复验的单块与双块留出画质基线。

**Architecture:** 一个离线 Python 命令读取现有封口数据和划分，用 SDK 准备器生成观测。
逐组运行现有增量模型，并用同一组相机和掩码比较单块与双块。

**Tech Stack:** 现有普通 C++17 SDK 准备工具、Python、NumPy、PyTorch 和 gsplat。

**Spec:** `docs/superpowers/specs/2026-11-19-p6-03c4a-held-out-rendering-design.md`。

## Global Constraints

- 只改 livo-atlas。复用 C++ 导出器、bundle schema、训练模型及联合光栅器。
- 输入为已停止写入并封口的快照。最多 512 帧、每组 3 到 32 帧；每帧最多 1M 像素和 500,000 点。
- 完整划分先于训练。留出不进入初始化、梯度或已保存模型的复用。
- 一个 GPU 训练模型，当前及上一组 CPU 参数。每帧 20 步，训练窗口 10 帧，单次渲染最多两块、20,000 高斯。
- `OMP_NUM_THREADS=1`、`OMP_THREAD_LIMIT=1`、`OPENBLAS_NUM_THREADS=1`，串行使用 GPU。
- 输出使用新证据目录，与两个输入目录互不包含。仅保存报告、必要图像与验证日志。

## Review Focus

- 同一采集时间或原图跨集合出现时，必须在 GPU 初始化前拒绝（任务 1）。
- 一次闭环修正涉及旋转和平移，训练与查询都须采用正确的最终相机（任务 1）。
- 导出器返回多余/缺少的帧、错误 split 或不属于冻结前沿的相机时，中止（任务 1）。
- 窗口发生淘汰后，实际训练记录仍须保留每次监督 ID，不能仅靠最终来源证明留出（任务 2）。
- 渲染失败、无覆盖区域与首块没有邻居都不能生成虚假的双块收益（任务 2）。

### Task 1: 固定划分和绑定最终相机

**Files:** 新增 `tools/3dgs_probe/evaluate_gaussian_submaps.py`、
`tests/3dgs_probe/test_evaluate_gaussian_submaps.py`。复用 `gaussian_map.py`、`bundle.py`。

**Interfaces:** `split_primary_ids(first: int, count: int) -> tuple[list[int], list[int]]`
返回升序训练和留出 ID。`camera_from_final_parent(view: dict, final_parent_from_body: np.ndarray) -> np.ndarray`
返回 4×4 float64 位姿。`prepare_group(prepare, keyframes, output, group, source_sha256)` 调用已有 exporter，
返回按 ID 的 CPU 观测。`group` 至少包含 `submap_id`、`anchor_id`、`train_ids` 和 `held_out_ids`。

- [ ] 写测试。`test_split_excludes_held_out_ids` 核对 `(168, 4)` 产生训练 `[168, 170, 171]`
  和留出 `[169]`，`(0, 18)` 留出 `[1, 9]` 并保留其余 16 帧。`test_rebind_camera_after_loop`
  用非恒等旋转和平移对照已知最终相机，同时检查原图/深度视图未被修改。
  `test_input_collisions_and_bundle_mismatches_are_rejected` 覆盖跨集采集时间、原图哈希重复及导出 ID/标记偏差。
- [ ] 运行 `"$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_evaluate_gaussian_submaps.py -v`，
  观察缺失接口或对应断言先失败。
- [ ] 读取 `map.json` 前沿并用 `_inspect_gaussian_map` / `inspect_gaussian_finalization` 核对最终位姿、
  资产及封口成员。要求实际权威 records 与最新 pose 文件。先用原始 `frame.yml` 的 `image_timestamp`
  和 `image.png` 哈希全局核对身份，冻结划分与来源。逐组准备，核对 bundle 清单与 split 的精确对应，
  `parent_from_body` 缺失或 bundle 位姿版本混杂时报错。使用设计稿中的相机合成顺序。
- [ ] 运行定向测试及已有 `test_bundle.py`，预期均通过。普通 SDK 准备器用既有方式安装后构建，
  见 [构建命令](../../gaussian-state.md#环境和数据准备)，也可复用预检查使用的同版二进制。

### Task 2: 逐组训练、成对渲染和真实数据报告

**Files:** 同一评估脚本及测试，新增阶段验证记录；复用 `OnlineGaussianModel` 和 `joint_renderer`。

**Interfaces:** 单命令 `evaluate_gaussian_submaps.py --keyframes PATH --map-directory PATH
--prepare-executable PATH --output NEW_DIRECTORY`。任务 1 的 `prepare_group` 按组返回观测。
输出 `split.json`、`views/<ID>/{reference,single,joint}.png` 和完成标记 `report.json`。
首组没有 joint 图。`report.json` 包含 split SHA256、位姿与环境、各组实际训练 ID/监督计划/来源、
每个留出 ID 的查询相机、参与块、指标以及 `near_group_start` / `interior` 汇总和资源。

- [ ] 写测试。`test_held_out_observation_never_trains` 用小型真实 CUDA 夹具和带留出的序列，检查
  实际 `training_frame_ids` 与来源无留出 ID。渲染留出后模型参数保持一致，改变留出 RGB 只改变评分。
  `test_first_group_and_missing_depth_are_reported` 核对首组缺邻居、无覆盖深度为 null，
  以及渲染异常时没有成功报告。
- [ ] 在固定 CUDA 环境运行定向测试，先观察缺失行为。用局部系训练全部训练帧、记录实际 ID，
  在退出模型生命周期后用 `infer_blocks` 和 `image_metrics` 计算结果。汇总同一查询的两种方案，
  按像素数汇总 MSE 和覆盖率。逐组临时 bundle，最后核验来源，再发布 `report.json`。
- [ ] 用 `LIVO_REQUIRE_GPU=1` 运行上述定向测试和现有 `tests/3dgs_probe` 套件，核对未跳过 GPU 用例。
  测试需要已有准备器、fixture、wire peer 和 projection probe，保持现有测试环境变量。
- [ ] 在本机用下列路径运行完整基线。预期 13 组、147 次训练确认、25 个留出评分及 23 个成对结果。
  报告或日志须能追溯每次训练和来源，检查首组仅有两个单块结果、输入哈希未改变与实际 GPU 指标。

```bash
export OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0
"$LIVO_PYTHON" tools/3dgs_probe/evaluate_gaussian_submaps.py \
  --keyframes "$LIVO_BASELINE/review-full-on/keyframes" \
  --map-directory "$LIVO_BASELINE/g-review-full-on" \
  --prepare-executable "$LIVO_PREPARE_BIN" --output "$LIVO_PHASE/held-out"
```

本机 `LIVO_BASELINE` 为工作区根目录下 `validation/P6-03c3c2b-20261119` 的绝对路径。
其他电脑可指向同一份封口数据和已核对的独立 GPU 环境。先创建 `LIVO_PHASE`，输出子目录须不存在。
需要 YAML、Pillow、NumPy、PyTorch 和已编译的 gsplat，环境以 [P6-03b](../../p6-03b-validation.md) 为准。

- [ ] 核对划分隔离、最终相机、数字与样例图；记录实际单块/双块变化及资源。保存源提交、输入 SHA、
  `split.json`、运行日志与结果。用同一划分复跑若干样例，报告 GPU 数值波动。
- [ ] 进行阶段审阅，处理影响留出隔离或评分的缺陷。发布复现说明和验收记录。
