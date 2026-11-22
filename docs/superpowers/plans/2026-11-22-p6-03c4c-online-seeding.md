# C4c Online Coverage-Guided Seeding Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan in the current session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在已有持久子地图 worker 中显式选择覆盖优先采样，完成 Noetic 的在线成对验证。

**Architecture:** 策略由独立 worker 的启动参数固定，传入 schema 2 的普通训练和重建。复用
现有协议、确认恢复和 ROS 节点入口。验证脚本独立记录策略、耗时以及各侧实际训练和接收结果。

**Tech Stack:** Python、PyTorch、gsplat、现有 C++17 Noetic 节点和 Unix socket 协议。

**Spec:** [在线覆盖优先采样设计](../specs/2026-11-22-p6-03c4c-online-seeding-design.md)

## Global Constraints

- 实现范围为 livo-atlas。ROS1 Noetic 和 ROS2 Humble 共用现有 worker。线上选项缺省为 `uniform`。
- `coverage` 用于持久子地图 schema 2。沿用现有 wire 字段和状态资产格式。
- 每帧最多一次预渲染、1,000 个种子和 20 步，窗口最多 10 帧，单块最多 10,000 个高斯。
- 固定单 GPU worker、既有 pending/in-flight 上限和超时。回放两侧 OMP/BLAS 为 1、节点 CPU 0、
  图像队列 32、速率 1。每次在独立 socket 和输出路径运行，串行执行。

## Review Focus

- schema 1 配置了 `coverage` 时在初始化模型前报错，默认连接继续沿用原行为。
- 普通训练、rebuild 内每帧和 discard 恢复后的下一次训练均使用该会话策略。
- `reuse` 为零步。覆盖预渲染失败后已有持久资产和 owner 引用可读且身份一致。
- 验证按实际执行的训练策略、帧和窗口作比较。`rebuild` 操作、内部各帧训练、owner 接纳次数有各自的计数。
- 展示用预览的 READY 时间由独立 ROS 订阅者提供，训练或发布计数不能代替该证据。

### Task 1: 接入会话策略

**Files:** `tools/3dgs_probe/online_model.py`、`online_worker.py`、`submap_worker.py`，以及
`tests/3dgs_probe/test_online_worker.py`、`test_submap_worker.py`。

**Interfaces:** 增加 `online_worker.py --seed-policy {uniform,coverage}`，保留 `--socket`。
`serve(socket_path, *, seed_policy="uniform")`，
`serve_connection(sock, model_factory=create_model, transfer_timeout=5.0, *, seed_policy="uniform")`，
`_serve_submaps(..., *, seed_policy="uniform")` 和
`GaussianSubmapWorker(..., *, joint_render_enabled=False, spatial_render_enabled=False, seed_policy="uniform")`。
`_serve_submaps` 的现有布尔参数继续支持旧调用。策略通过 `train_candidate(view, seed_policy=...)` 传入。

- [ ] 在 `test_online_worker.py` 添加非法 CLI 值用例。用私有临时目录中的新 socket、
  `--seed-policy other` 启动进程，断言 argparse 拒绝且 socket 路径未创建。用 `socketpair`
  发送 schema 1 hello 到配置为 `coverage` 的 `serve_connection`，断言 `protocol_error`、
  模型工厂零调用且服务返回失败。保留默认 schema 1 round trip。
- [ ] 在 `test_submap_worker.py` 复用 `server_roundtrip` 增加 `coverage` 的 schema 2 真实 GPU
  用例。记录 `train_candidate` 的 kwargs，断言 coverage 到达模型且候选仍能由现有 wire 解码。
  用该文件现有 `activate`、`view`、`decide`、`checkpoint` 夹具覆盖两种策略，核对 view 20 步、
  reuse 0 步、双帧 rebuild 40 步且两次调用同策略。拒绝 rebuild 后核对恢复的参数来源，再用
  更高 generation 对兼容目标执行 `reuse`，验证下一次合法 view 的采样策略。
- [ ] 添加 `test_coverage_prepass_failure_preserves_durable_reference`。先确认、checkpoint 并
  确认地图引用。在后续候选进入预渲染时注入失败。用含两个有效深度候选的视角和测试内的
  1 个种子上限触发原预渲染，断言 worker 进入 `failed`，候选未确认，原资产哈希、地图引用和
  已确认身份可复核。通过现有 socket 错误处理检查 `worker_error`。
- [ ] 运行 `"$LIVO_PYTHON" -m unittest -v test_online_worker test_submap_worker`，核对 RED。
- [ ] 实现启动参数和传递。worker 启动入口复用模型模块的策略名常量，保持 CPU 评估比较器的
  导入路径。收到 schema 1 的 `coverage` 在模型工厂前返回协议错误。actor 在创建 `states`
  目录前验证配置，持有会话策略，在 `_view` 和 `_activate` 的 rebuild 循环传入。沿用候选
  回复的现有字段，覆盖证据仍由模型的本地 `metrics` 返回。复用 actor 的失败处理。
- [ ] 运行同一定向命令，期望 GREEN。在 phase 保存 RED/GREEN 日志并提交实现。使用
  `LIVO_REQUIRE_GPU=1`、现有 C4b 的 Python/依赖和 fixture，`PYTHONPATH` 包含当前工具与测试目录。

### Task 2: 成对回放与证据

**Files:** 新阶段 `validation/P6-03c4c-<date>/` 的 `worker-measure.py`、`run-replay.sh`、
`compare-online-seeding.py`、回放报告及校验清单。使用说明写入 `docs/p6-03c4c-validation.md`。

**Inputs:** 当前 HEAD 的 Task 1 实现、`Retail_Street.bag`、Gaussian/GTSAM/回环开启的 Noetic
Release 节点、C4b GPU 环境。本机可复制 C3c2b 的 Noetic devel 目录和当前源码快照，先核对
实际文件 SHA256 与 `e0a26e9` 的 native 源码差异。其他机器先用同样功能选项构建独立 Noetic
工作区。预检记录与命令在 `validation/P6-03c4c-preflight-20261122`。

- [ ] 以 C3c2b `worker-measure.py` 为基础复用其入口和峰值资源记录。验证专用 wrapper 在
  `GaussianSubmapWorker.handle` 范围关联 `view` / `activate(rebuild)`、候选和 accept/discard，
  并记录 `train_candidate` 实际收到的策略、当前 `keyframe_id`、位姿版本、图像尺寸、相机矩阵、窗口、
  `steps`、`gaussian_count`、`seed_selection`、单次调用时间。保留 `(session_id, job_id,
  generation, submap_id, train_index)`。整项 `handle` 耗时与内部训练耗时分别记录。两侧使用
  同一测量方法，最多保留 4,096 条摘要记录，溢出则该次性能证据判为不完整。进程结束时在新文件
  写报告，数据为标量、短 ID 列表和时间，无图像或参数张量。
- [ ] 实现 `run-replay.sh short|full uniform|coverage`，沿用预检参数。worker 通过
  `--socket` 和 `--seed-policy` 启动。使用 `tools/run_baseline.py` 的
  `--submap-mode online --loop-closure --semantic-output --gaussian-persist-submaps
  --gaussian-finalize-on-shutdown --gaussian-joint-render --joint-render-policy spatial
  --gaussian-publish-preview`。`--duration` 用 10 或 136，分别覆盖首 10 秒和 135.47 秒完整 bag。
  各运行使用新目录、私有 socket 和绑定本机回环地址的 ROS master 端口。校验 worker 退出码，
  保存输入、native/Python 源码、二进制与依赖版本。
- [ ] 配对校验复用 `compare_gaussian_replays.py` 的 `_ARRAYS`、`_parameters`、`_raw_keyframes`、
  `_stats`、`_finalization` 与 `validate_finalized_coverage`，分别调用 `inspect_gaussian_map`、
  `inspect_joint_renders` 和 `inspect_gaussian_publication`。要求双方配置均启用持久化、空间渲染、
  预览及 Finalize，验证共同数据和设置，只为新 socket/输出位置归一化。每侧检查回放输入计数、
  干净退出、原轨迹与数组、原始关键帧哈希、话题语义和各自状态证据。逐状态读取最终资产，验证
  eligible 集合、current/durable 及完整回放的 13 块、172 帧、2 次回环、位姿版本 2。
- [ ] 对照实际 worker 证据。均匀侧没有预渲染。覆盖侧每次训练具 C4b 字段，按有效候选数和
  窗口验证有无预渲染及预留配额，完整回放至少一次实际预渲染且预留非零。view/rebuild 分组
  汇总耗时、已确认与已丢弃统计。另以 `(submap_id, keyframe_id, pose_version, width,
  height, frame_ids, gaussian_count)` 配对 view。匹配键须在两侧唯一，且相机矩阵相同。报告匹配
  数量和延迟分布，为零时配对分布留空。区分首个 GPU 训练调用。收集两侧首个截止前 READY、
  运行/封口耗时、各级队列和显存/RSS。对短段只要求有效
  READY，完整回放额外使用 `min_dual=6`、`min_groups=3`。异步合并数和高斯资产字节量分别报告。
- [ ] 依次运行下面四次回放。用配对校验器发布结果，保留逐次校验、实际数据、代码和输入哈希。
- [ ] 运行 `"$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -v` 和
  `"$LIVO_PYTHON" -m unittest discover -s tests -p 'test_*.py' -v`。按最终提交和审阅结果
  复核受影响证据，记录真实耗时和资源。复现文档描述两台机器均可运行的 worker/owner 命令，并
  标明当前验证的是 Noetic 在线运行。把这一阶段的索引加到 `docs/README.md`。

```bash
export OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 LIVO_REQUIRE_GPU=1
for length in short full; do
  for policy in uniform coverage; do
    "$LIVO_PHASE/run-replay.sh" "$length" "$policy" || exit 1
  done
done
```

两种策略的配对校验都要通过才能完成在线试验。报告保留两侧实际耗时、计数和样本量，供判断
是否继续扩大验证场景。生产 worker 缺省仍为均匀采样。
