# 在线回环接纳

P6-02c2b 将已有 [C2a 准入计算](loop-admission.md)接入在线封存事件。
默认关闭。启用图优化时，一个 worker 计算提案，系统所有者检查来源、保存 prepared，再发布位姿。
ROS1 Noetic 与 ROS2 Humble 使用同一个 `src/app/livo_main.cc`。
worker 通过 Cartographer 2.0.0 的单线程 `ThreadPool` 调度回环任务，位姿提交由系统所有者执行。
[线程池来源](../third_party/cartographer/README.md)
它仍使用批量 GTSAM LM 试算，当前不包含 iSAM2、全局描述子、全局 TF 发布或 Gaussian 训练。

## 构建与开启

常规[构建](ros-builds.md)提供 `livo_atlas::online_backend` 和 `livo_atlas_check_loops`，
需要兼容的 GTSAM 4.2 和带 registration、kdtree 的 PCL。非标准安装可指定
`-DGTSAM_DIR=/absolute/path/to/gtsam/lib/cmake/GTSAM`。

| `loop_closure.enabled` | `pose_graph.enabled` | 本次运行 |
| --- | --- | --- |
| `false` | `false` | 纯 LIVO |
| `true` | `false` | 检测回环并保存通过几何筛选的匹配 |
| `true` | `true` | 检测回环、试算位姿图，接纳通过复核的更新 |

位姿图使用检测到的回环约束，启用时需要同时开启回环检测。旧 YAML 省略
`pose_graph.enabled` 时，取 `loop_closure.enabled` 的值。

在现有传感器、标定配置基础上，ROS1 使用以下参数：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
  max_keyframes: 20
loop_closure:
  enabled: true
  options_file: ""
pose_graph:
  enabled: true
```

ROS2 对应参数使用同一套字段：

```yaml
laserMapping:
  ros__parameters:
    keyframes.enabled: true
    submaps.enabled: true
    submaps.mode: online
    submaps.max_keyframes: 20
    loop_closure.enabled: true
    loop_closure.options_file: ""
    pose_graph.enabled: true
```

每次使用新的输出目录。启用要求在线关键帧和子地图，每块最多 64 个主成员。
`submaps.geometry.enabled` 可以关闭；worker 自己使用固定的 0.2 m 几何表示和三块缓存。
空 options_file 使用 C2a 默认值；非空时读取与离线工具相同的 search、registration、graph、admission 四组。
节点在创建输出目录前校验参数组合。启用回环检测时读取选项文件。

## SDK 所有权与停止

SDK 链接 `livo_atlas::online_backend`。先读取选项，再创建 `LivoSystem`、`LoopBackend` 和接入对象。
应用注册唯一在线通知回调，并在系统处理返回后泵送结果：

```cpp
auto loop_options = config.loop_closure_options_file.empty()
                        ? LoopAdmissionOptions{}
                        : ReadLoopAdmissionOptions(
                              config.loop_closure_options_file);
loop_options.pose_graph_enabled = config.pose_graph_enabled;
LivoSystem system(config.system);
LoopBackend backend(system, loop_options);
system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& update) {
  backend.Observe(update);
});

// 传感器接入、ProcessNext 和 Poll 都在同一个系统所有者线程上。
// 即使 ProcessNext 返回 kWaiting，也调用 Poll。
system.ProcessNext(result_callback);
backend.Poll();

// 正常停止时显式调用；它也完成 system.Finish()。
backend.Finish();
```

Observe 只保留元信息，释放关键帧的图像/点云引用；不写盘、不重入系统，异常交给下次 Poll。
其他消费者可在同一个回调中组合，但也必须遵守原有的不重入、不抛异常约定。
后端只接受零帧、零位姿版本的新系统，并独占该会话的位姿更新；外部 ApplyPoseUpdate 随后会被拒绝。

正常停止顺序为封存输入、排空当前及最新待查任务、join、系统 Finish、写 complete 摘要。
SealInput 后输入与处理停止，位姿提交仍可继续到最终 Finish。
析构只停线程并 join，丢弃未接纳结果。失败会阻止后续变更并保留原始异常；只读位姿快照仍可诊断。
ROS1 开启后台时使用壁钟等待，因此暂停仿真时钟也能继续泵送；ROS2 在原有 wall timer 中调用同一处理后 hook。

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
