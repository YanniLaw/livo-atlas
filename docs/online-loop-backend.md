# 在线回环接纳

`LivoSystem` 管理前端、子地图归档和回环的生命周期。回环默认关闭。
启用图优化时，`LoopClosure` 计算候选和约束，`Optimizer` 试算位姿，`PoseGraph` 检查提案来源。
ROS1 Noetic 与 ROS2 Humble 使用同一个 `src/app/livo_main.cc`。
启用关键帧归档时，位姿图通过 `WorkQueue` 串行保存关键帧与成员修订，任务由 `ThreadPool` 执行。
接纳优化结果时，同一执行线先写 `prepared`，再写正式位姿并更新已接纳的约束。
[线程池来源与许可](cartographer-scheduler.md)
它仍使用批量 GTSAM LM 试算，当前不包含 iSAM2、全局描述子、全局 TF 发布或 Gaussian 训练。

## 构建与开启

常规[构建](ros-builds.md)通过 `livo_atlas::core` 提供在线后端及 `livo_atlas_check_loops`，
需要兼容的 GTSAM 4.2 和带 registration、kdtree 的 PCL。非标准安装可指定
`-DGTSAM_DIR=/absolute/path/to/gtsam/lib/cmake/GTSAM`。

| `loop_closure.enabled` | `pose_graph.enabled` | 本次运行 |
| --- | --- | --- |
| `false` | `false` | 纯 LIVO |
| `true` | `false` | 检测回环并保存通过几何筛选的匹配 |
| `true` | `true` | 检测回环、试算位姿图，接纳通过复核的更新 |

位姿图使用检测到的回环约束，启用时需要同时开启回环检测。如果 `pose_graph.enabled`
缺省，取 `loop_closure.enabled` 的值。仅检测时显式设置 `pose_graph.enabled: false`。

ROS1 和 ROS2 launch 都读取 `config/backend.yaml`。下列配置会启用检测和优化。[backend.yaml](../config/backend.yaml)

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
  max_keyframes: 20
loop_closure:
  enabled: true
pose_graph:
  enabled: true
  num_background_threads: 4
  max_pending_keyframes: 64
```

共享配置还包含 `loop_closure.search`、`loop_closure.registration`、`loop_closure.admission`、`loop_closure.geometry` 和 `pose_graph` 的调优项。可用 `backend_config` launch 参数选择其他 YAML。

每次使用新的输出目录。启用要求在线关键帧和子地图，每块最多 64 个主成员。
worker 使用 `loop_closure.geometry` 指定的独立几何参数，默认体素大小为 0.2 m，缓存三块。
节点在创建输出目录前校验参数。旧 `loop_closure.options_file` 非空时优先读取兼容的离线选项格式。

## SDK 所有权与停止

SDK 链接 `livo_atlas::core`。启用在线关键帧和子地图后，把 `LoopAdmissionOptions` 交给 `SystemOptions::loop_closure`。已有导出目标 `livo_atlas::online_backend` 仍可链接。
`SystemOptions::num_background_threads` 设置后台线程数，默认 4。ROS1 和 ROS2 从
`config/backend.yaml` 读取同名的 `pose_graph.num_background_threads`，可设为 6。
独立构造的 `LoopBackend` 持有该会话的提交权，沿用 `Observe()`、`Poll()` 和显式 `Finish()`，写入由同一位姿图执行。

```cpp
system_options.loop_closure = LoopAdmissionOptions{};
LivoSystem system(system_options);
system.ProcessNext(result_callback);
system.Finish();
const auto stats = system.GetLoopStats();
```

`ProcessNext()` 报告前端进度，并在等待输入的周期继续收取已提交前沿。`ProcessAvailable()` 在返回前交付本次已入队的关键帧。调用方可以注册 `SetOnlineSubmapDeltaCallback()` 接收增量，或使用 `SetOnlineSubmapCallback()` 读取每次提交的完整快照。系统调用线程在锁外执行回调。回调可以读取当前前沿，后续处理在回调返回后继续。

`SealInput()` 通过位姿图封存输入，授权的 `ApplyPoseUpdate()` 等待正式位姿交付。正常停止依次封存、排空回环和交付回执、完成导出、写 `complete=1` 并加入工作线程。显式封存后仍允许位姿提交。
析构停止并加入工作线程。必要写入或回调失败时，会话保留首个异常与已交付的只读前沿。回环归档的完成摘要保留 `complete=0`。
两端使用壁钟推进处理。暂停仿真时钟时仍可收取在途结果。

## 前沿、预算和原始结果

派发前沿 N 包含全部已提交帧，也包含活动尾块内的帧；任务只携带已封存块的成员信息。
读取视图拥有绝对目录和固定 count，worker 不扫描活体索引、不访问系统或 ROS。
每个任务独立构建几何缓存，当前与预览位姿切换时重建，避免被拒绝的预览污染下一次查询。

接纳时重新检查版本、已接纳历史、原始矩阵/时间戳前缀、有效位姿和封存成员修订。
允许最新前沿 M 大于 N：保留优化前缀 N，新增尾帧按 projected-last-anchor 策略物化。
后端位姿通过 `OnlineSubmapDelta` 通知轨迹、几何和高斯消费者，SDK 完整回调也收到该版本的快照。

任务与结果共享一个载荷槽，另有一个最新待查 ID。新封存查询可以合并旧待查查询，并记录 skipped。
后台线程数为正整数。多个工作线程允许回环计算与归档同时推进，单条执行线拥有图状态和写入权。
候选验证目前在回环任务内依序执行。1 线程用于队列回归，4 和 6 线程覆盖并发归档与正常停止。
`pose_graph.max_pending_keyframes` 限制待写关键帧，默认 64，适用于所有关键帧归档模式。
SDK 可以通过 `SystemOptions::max_pending_keyframes` 设置同一上限。满载时拒收新关键帧、保留已提交前沿并终止会话。控制请求和交付回执仍可推进。`peak_queued_keyframes` 记录待写峰值。
`GetLoopStats()` 的帧数、位姿版本和已接纳回环数反映系统发布进度，后台拒绝及待查等计数随队列推进。
默认预算为 5000 帧、200 条回环；超过帧预算后停止复制后端历史元信息，也拒绝接纳在途提案。
达到回环预算后停止派发，前端资产仍继续保存。没有以裁剪帧替代完整派发图。
这些限额约束后端工作集，不能证明整个前端地图或进程 RSS 有界；求解和 join 没有硬截止时间。

## 文件与只读检查

原始数据保持[在线子地图格式](online-submap-dataset.md)，正式位姿仍由系统写入 `online_submaps/poses/VERSION.yml`。
新增 `keyframes/loop_closure/`：

| 文件 | 含义 |
| --- | --- |
| session.yml | 固定回环求解选项、几何/坐标约定及原 dataset.yml 字节，元信息上限 64 KiB |
| prepared/VERSION.yml | 任务 N、接纳 M、原始 N 帧矩阵和时间戳、成员、约束、诊断、更新及预期 M 帧位姿 |
| queries/TASK_ID.yml | 引擎状态及接纳处置，同一查询重算使用新的 task_id |
| events/SEQ.yml | 合并跳过范围和资源停止 |
| summary.yml | 初始 complete=0，正常排空和系统 Finish 成功后原子替换为 complete=1 |
| failure.yml | 尽力写入的失败原因 |

仅检测模式保留原始位姿版本 0。通过几何筛选的查询记录 `detected` 和选中目标子地图 ID，
摘要包含 `detected` 计数。回放工具使用 `--loop-detection-only` 选择此模式，
使用 `--loop-closure` 运行检测和位姿图优化。

session、prepared、query、event 正式文件不可覆盖。只有 prepared 与匹配的正式 pose 才建立已接纳版本。
pose 发布后通知或诊断失败，文件保留，整个会话终止；summary 或额外确认文件不能替代位姿提交权威。

停止写入后运行，OUTPUT 必须全新且位于输入数据集之外：

```bash
livo_atlas_check_loops /path/to/run/keyframes /path/to/new-check-output
```

| 退出码 | 状态 |
| --- | --- |
| 0 | consistent：正常完成，正式位姿与 prepared、来源和摘要相符 |
| 3 | incomplete：可解释已提交前缀，但未正常完成；允许紧邻末版的孤立 prepared、未封存尾块或 pending |
| 1 | inconsistent / 运行错误：版本缺口、缺配对、来源或记录矛盾、假 complete 等 |
| 2 | 命令行用法错误 |

report.yml 分别给出 committed_version、uncommitted_prepared 和 issues。
检查器核对原始元信息、标定、固定噪声、成员、任务前沿、查询诊断及连续位姿版本，并重算位姿物化到 1e-9 容差。
prepared 与 query 共用诊断结构校验：候选验证、来源修订、有限指标、图试算/残差和几何复查均按实际计算阶段检查，
早期拒绝可以省略尚未计算的内容。已提交任务若存在 query，其处置必须为 accepted，且版本和前沿与 prepared 一致；
这项约束也适用于 complete=0。发布后中断而缺少最终 query 仍可解释为 incomplete，并保留已确认版本。
它不解码图像、不重跑 GICP、不鉴定点云全部字节，也不修复文件或恢复运行。
资产仍要求单 writer 下不可变；当前发布过程没有 fsync，不承诺掉电事务。

## 重复回归

两个既有回放工具新增 `--loop-closure`，自动启用所需在线关键帧/成员存储：

```text
tools/run_baseline.py ... --geometry-submaps --loop-closure
tools/run_ros2_baseline.py ... --geometry-submaps --loop-closure
```

省略该开关仍走默认关闭路径。验证按同平台原始前端结果比较，新增位姿和回环记录另由检查器核对。
构建矩阵、官方 Retail_Street 结果及故障数据集见[本阶段验证报告](p6-02c2b-validation.md)。
