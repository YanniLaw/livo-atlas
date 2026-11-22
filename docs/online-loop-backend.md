# 在线回环接纳

`LivoSystem` 管理前端、封存子地图和回环后端的生命周期。后端默认关闭。
启用图优化时，worker 计算提案，由 `LivoSystem` 检查来源、保存 prepared，再发布位姿。
ROS1 Noetic 与 ROS2 Humble 使用同一个 `src/app/livo_main.cc`。
worker 通过 Cartographer 2.0.0 的单线程 `ThreadPool` 调度回环任务，位姿提交由系统所有者执行。
[线程池来源](../third_party/cartographer/README.md)
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
```

共享配置还包含 `loop_closure.search`、`loop_closure.registration`、`loop_closure.admission`、`loop_closure.geometry` 和 `pose_graph` 的调优项。可用 `backend_config` launch 参数选择其他 YAML。

每次使用新的输出目录。启用要求在线关键帧和子地图，每块最多 64 个主成员。
worker 使用 `loop_closure.geometry` 指定的独立几何参数，默认体素大小为 0.2 m，缓存三块。
节点在创建输出目录前校验参数。旧 `loop_closure.options_file` 非空时优先读取兼容的离线选项格式。

## SDK 所有权与停止

SDK 链接 `livo_atlas::core`。启用在线关键帧和子地图后，把 `LoopAdmissionOptions` 交给 `SystemOptions::loop_closure`。已有导出目标 `livo_atlas::online_backend` 仍可链接。

```cpp
system_options.loop_closure = LoopAdmissionOptions{};
LivoSystem system(system_options);
system.ProcessNext(result_callback);
system.Finish();
const auto stats = system.GetLoopStats();
```

系统在 `ProcessNext()` 中收取后端结果，包括等待输入的周期。外部消费者可以注册 `SetOnlineSubmapCallback()` 读取快照，并遵守回调不重入、不抛异常的约定。启用后端的会话由系统独占位姿提交。

正常停止顺序为封存输入、排空当前及最新待查任务、join、系统 Finish、写 complete 摘要。
SealInput 后输入与处理停止，位姿提交仍可继续到最终 Finish。
析构只停线程并 join，丢弃未接纳结果。失败会阻止后续变更并保留原始异常；只读位姿快照仍可诊断。
两端使用壁钟推进处理。暂停仿真时钟时仍可收取在途结果。

## 前沿、预算和原始结果

派发前沿 N 包含全部已提交帧，也包含活动尾块内的帧；任务只携带已封存块的成员信息。
读取视图拥有绝对目录和固定 count，worker 不扫描活体索引、不访问系统或 ROS。
每个任务独立构建几何缓存，当前与预览位姿切换时重建，避免被拒绝的预览污染下一次查询。

接纳时重新检查版本、已接纳历史、原始矩阵/时间戳前缀、有效位姿和封存成员修订。
允许最新前沿 M 大于 N：保留优化前缀 N，新增尾帧按 projected-last-anchor 策略物化。
后端位姿通过既有 OnlineSubmapUpdate 通知几何和未来消费者，原始前端状态及关键帧资产继续保留。

任务与结果共享一个载荷槽，另有一个最新待查 ID。新封存查询可以合并旧待查查询，并记录 skipped。
默认预算为 5000 帧、200 条回环；超过帧预算后停止复制后端历史元信息，也拒绝接纳在途提案。
达到回环预算后停止派发，前端资产仍继续保存。没有以裁剪帧替代完整派发图。
这些限额约束后端工作集，不能证明整个前端地图或进程 RSS 有界；求解和 join 没有硬截止时间。

## 文件与只读检查

原始数据保持[在线子地图格式](online-submap-dataset.md)，正式位姿仍由系统写入 `online_submaps/poses/VERSION.yml`。
新增 `keyframes/loop_closure/`：

| 文件 | 含义 |
| --- | --- |
| session.yml | 固定全部选项、几何/坐标约定及原 dataset.yml 字节，元信息上限 64 KiB |
| prepared/VERSION.yml | 任务 N、接纳 M、原始 N 帧矩阵和时间戳、成员、约束、诊断、更新及预期 M 帧位姿 |
| queries/TASK_ID.yml | 引擎状态和所有者处置；同一查询重算使用新的 task_id |
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
