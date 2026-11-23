# SLAM 第二阶段实施计划

> 按验证清单逐项实施。涉及运行行为的改动先在现有测试中复现，再修改代码。

这一批完成总设计的 2.1 和 2.2。先测量 ROS 输入到前端与发布各段的耗时，再限制 `/path` 的可视化历史。测量结果用于决定 2.3 的线程交接范围。 [总设计第二阶段](../specs/2026-11-24-slam-engineering-optimization-design.md#2-计时发布和串行处理)

工作环境沿用 C++17、Google C++ 风格、glog、ROS1 Noetic 和 ROS2 Humble。算法与 ROS 计时都使用 `std::chrono::steady_clock`。传感器时间继续用于同步和样本顺序，ROS 发布沿用当前的消息时间语义。

## 任务 1 统一计时字段和回放解析

涉及 `src/ros_common/input_diagnostics.h`、`src/ros1/livo_node.cc`、`src/ros2/livo_node.cc`、`tools/mapping_log.py`、两个回放工具和 `tests/test_baseline_runner.py`。

- [x] 在现有解析测试加入新旧日志样本。断言都能读取 LIO/VIO 更新数和前端耗时，混合输入优先读取结构化更新记录，避免同一批结果重复计数。缺少耗时的旧更新仍计入更新数。
- [x] 从公共日志入口输出 `kind` 和 `frontend_process_seconds`。沿用现有回调传入的 `double elapsed`，取值范围是 `Frontend::Process()`。前端 SDK 调用继续使用同一回调签名。 [实际计时区间](../../../src/core/livo_system.cc#L341)
- [x] 两个回放工具复用 `tools/mapping_log.py` 的解析结果。保留摘要的 `lio_cycles` 和 `vio_cycles`，耗时输出统一为秒，包含样本数、P50、P95、P99、最大值。用阶段一已保存的 `mapping.log` 核对旧日志仍得到 1,352 次 LIO 和 1,351 次 VIO。 [阶段一验收](2026-11-24-slam-phase-1-install-and-input.md#实施与验收记录)

## 任务 2 测量持锁和前后端工作

涉及 `src/core/livo_system.cc`、`src/core/loop_backend.cc`、两端 `livo_node.cc` 和 `livo_publisher.cc`、公共日志入口和回放解析器。分段采样通过 `config/backend.yaml` 的 `diagnostics.stage_timing` 开关启用，默认关闭。现有前端更新计数始终可用。

- [x] 订阅端按 LiDAR、IMU、图像记录转换耗时及转换结束到取得 `system_mutex_` 的等待。系统侧记录 LiDAR 预处理、就绪同步、每次前端调用、关键帧写入和实际位姿提交。回环记录派发到系统取回结果的耗时，沿用已有的配准与求解计时。发布侧记录消息构造和发布调用耗时，传播侧记录相邻传播输出之间的单调时间间隔。
- [x] 分别记录启动暂存和运行队列的最高占用与时戳跨度。保留第一阶段的超限诊断。暂停期间只处理已有工作和后台结果，空轮询保持无分段日志。分位数由回放工具离线汇总。
- [x] 复用现有等待与暂停测试验证有界采样。固定相同输入、限额、CPU 亲和性和 OpenMP 设置，比较开关前后的轨迹和计数。用独立配置启用 `uav.imu_rate_odom` 验证传播间隔，记录阶段性 P95/P99 供 2.3 选定改动范围。 [现有锁内发布](../../../src/ros2/livo_node.cc#L170)

## 任务 3 限制 `/path` 历史

涉及 `config/backend.yaml`、公共 `LivoConfig`、两端 `LivoPublisher` 和参数测试、发布测试及回放检查。

- [x] 新增正整数参数 `publish.path.max_poses`，默认 2000。两端共用参数加载和校验，测试合法覆盖值以及零、负数、非整数报错。
- [x] `/path` 按现有顺序保留最新 `max_poses` 条。沿用 `nav_msgs::Path`、`camera_init`、原有时间语义和发布时机。轨迹文件与版本化 `/keyframe_path` 分别按自己的输出配置保存。
- [x] 双 ROS 发布测试以容量 3 发送 5 个估计，逐项验证最新 3 个位姿的顺序、坐标和时间语义。用较长的合成序列越过容量边界，检查消息位姿数与序列化尺寸进入稳定范围。
- [x] 在回放摘要记录 `/path` 最终位姿数。两端通过节点启动日志识别实际容量，按 `min(lio, max_poses)` 校验。旧节点沿用历史回放约定。 [Humble 路径检查](../../../tools/run_ros2_baseline.py#L42)

## 配对验收

- [x] 从包目录运行 `python3 -m unittest discover -s tests -p test_baseline_runner.py`。Noetic 运行现有 `lidar_callbacks.test`，Humble 运行 `ros2_node_test`。
- [x] 同机先跑 10 秒，再跑完整 `Retail_Street.bag`。按第一阶段记录对照输入计数、LIO/VIO 次数、轨迹文件、原始关键帧及消息载荷。默认容量 2000 应覆盖本包的 1,352 条 `/path` 位姿。另用小容量回放或双端合成输入验证裁剪，同时核对轨迹文件条数。 [配对回放入口](2026-11-24-slam-phase-1-install-and-input.md#任务-4-验证两端节点和实际输入)
- [x] 保存分段时间、锁等待、传播间隔、队列峰值与路径消息大小。用同一受控负载确定 2.3 中优先迁移的持锁工作，再设计系统所有者处理顺序和发布交接。

## 实施与验收记录

- 独立 SDK 的 4 组 CTest、回放工具的 38 项测试、Noetic 的 41 项节点测试和 Humble 的 6 组 CTest 通过。双 ROS 的路径发布测试将容量设为 3，发送 256 个位姿，核对保留顺序、帧名、时戳和序列化尺寸。
- Noetic 在逻辑 CPU 2、OpenMP 单线程下，以 1× 回放对照 10 秒与完整 `Retail_Street.bag`。10 秒组覆盖默认计时、分段计时，以及容量 3 配合 IMU 传播。三组均为 98 次 LIO、97 次 VIO，轨迹和前端日志逐字节一致。
- 完整 SLAM 对照均得到 1,352 次 LIO、1,351 次 VIO。轨迹、前端日志、ROS 点云和图像载荷，以及 172 个原始关键帧的 516 个文件一致。两次都接受 2 次回环并提交位姿版本 2，归档检查通过。末条 `/path` 含 1,352 个位姿，共 112,247 字节。
- Humble 在逻辑 CPU 8 以 1× 运行三组 10 秒 DDS 回放，验证完整输入、相同轨迹和路径尾部。容量 3 时，Noetic 和 Humble 的末条路径消息分别为 280 和 276 字节。两端均采集到了实际 IMU 传播间隔。

完整 Noetic SLAM 回放的部分 P95 如下。锁等待从转换结束计时，测量包含采样日志和回放观测负载。

| 环节 | P95 |
| --- | ---: |
| 图像回调等待系统锁 | 109.7 ms |
| LIO 前端更新 | 75.6 ms |
| 关键帧写入 | 49.4 ms |
| LiDAR 回调等待系统锁 | 20.9 ms |
| ROS LIO 发布调用合计 | 0.059 ms |
| ROS VIO 发布调用合计 | 0.189 ms |

2.3 优先将已转换的传感器数据送入有界入口，由单一系统所有者按入队顺序消费并推进估计。用相同负载比较图像和 LiDAR 的入队等待、队列峰值和 IMU 传播间隔，然后据实测确定发布交接的范围。

本机配对摘要 `phase-2-verification.json`、两端回放与构建测试日志保存在工作区的 `validation/slam-timing-20261124/`。
