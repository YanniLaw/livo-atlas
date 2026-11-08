# P6-01 实施进度与验证

状态：截至 2026-11-08，Task 1–5 完成；完整 bag、构建矩阵和阶段审阅仍待 Task 6。
本报告中的结果不代表整个 P6-01 已验收。实施顺序见
[计划](superpowers/plans/2026-11-08-p6-01-online-submaps.md)，坐标与版本契约见
[设计](superpowers/specs/2026-11-08-online-submaps-design.md)。

## 当前行为

普通核心支持在线分组、成员修订、拥有数据的关键帧通知、冻结位姿快照和尾帧延续。
`submaps.enabled` 默认关闭，`submaps.mode` 默认为 `keyframe_groups`；显式选择 `online`
才创建 `keyframes/online_submaps/`。两套 ROS 通过相同参数清单配置同一个 LivoSystem。

回调在原始关键帧和本次成员修订提交后调用，只登记或入队，不重入系统、不抛异常，
也不通过 cv::Mat/PCL 别名修改资产。非法或过期后端输入仅拒绝该次更新；持久化失败和
意外回调异常会终止本次写运行，后续处理/Finish 保留原异常，最近成功位姿仍可读取。
私有管理器的 CheckRunning/RethrowFailure 在输入处理和关闭 writer 前检查终止状态，
避免后端提交失败后仍继续处理输入或用关闭错误掩盖原错误。

优化仅更新在线地图消费者看到的位姿，不覆盖前端滤波状态、速度、bias、工作地图或
现有 ROS 发布/TF。当前没有 GTSAM、真实闭环或高斯训练。

## Task 4 实测

| 环境 | 实际结果 |
| --- | --- |
| 普通 Release shared 核心 | 29 个 CTest 目标、267 项 GTest，零失败 |
| 普通公开头 | 50 个头独立编译通过 |
| Noetic 独立候选工作区 | 287 项 C++ 测试，零失败 |
| Noetic 实际 ROS | rostest 的 9 项测试通过，包含新增在线参数用例 |
| Humble 容器候选 | 32 个 CTest 目标、305 项测试，含实际 DDS 节点测试，零失败 |
| 两端源码核对 | 容器与本机 216 个源码/配置/测试文件的 SHA-256 一致 |

新增系统用例覆盖 LO/LIO/LIVO 的状态、点云、图像摘要及原始关键帧文件保持一致；
跨帧及析构后的快照有效；原始图像被覆盖后保留的关键帧图像不变；
位姿更新不反馈前端；写入失败不交付半次通知、不发布未提交版本；
成员/位姿版本变化使过期消费者结果失效，以及消费者从 v0 直接计算到 v2 的修正。

补测中曾错误假设“新帧必然修改子地图 1”，实际该组已经封存，新帧进入相邻组时其成员修订
正确保持不变。测试改为选取真正的活动组后通过；生产分组规则没有为该测试改变。

## 证据与复现

证据在工作区 `validation/P6-01-20261108/`，不进入源码仓库，迁移电脑时需另行复制。
构建和测试串行执行，使用 nice 10、`-j1`、`OMP_THREAD_LIMIT=1`。

- `task4-core-verified.log`、`core-shared/test_results/`：最终普通核心结果。
- `task4-noetic-unit-verified.log`、`noetic-ws/build/test_results/`：最终 Noetic C++ 结果。
- `task4-noetic-rostest.log`：实际 ROS 测试；`task4_ros.py noetic` 记录配置和执行命令。
- `task4-humble-verified.log`、`task4-humble-results/`：最终 Humble 结果。
- `task4-summary.json`、`task4-source-manifest.json`、`task4-source-final.tar.gz`：计数与最终源码证据。

Noetic 候选仅包含 livo-atlas，位于本阶段 `noetic-ws/`；Humble 候选位于已有 ros2 容器的
`/tmp/livo_atlas_p6_01_task4/`。原阶段证据、原始 bag 和其他包保持不动。
Task 4 没有进行 Retail_Street.bag 回放、SDK 安装迁移或完整阶段性能验收；这些结果须在后续实际运行后补充。

## Task 5 实测

显式 online CLI 已通过 5 项标准库 unittest 测试：真实冻结 M3/N2 文件物化到 M4、
活动组和未索引尾部、旧输出格式、错误参数/未知版本、两个命名空间共存及只读文件摘要。
实现前原工具在同一组真实数据上有 3 项失败，实现后及移动 SDK 后均全部通过。

普通 shared SDK 安装后移到 sdk-task5-moved/，原前缀不再存在。独立 CMake 消费者
重新配置/编译并运行旧/online LO、LIO、LIVO、两次模拟位姿更新、跨析构资产保留和离线出口。
编译参数没有生产源码 include，ldd 的 12 个自有库全部来自新安装前缀。
确定性副本同时验证了旧分组重建和实际 COLMAP 导出，重复兼容版本写入被拒绝。

两端都使用新增 --submap-mode online 做了官方数据的 3 秒、1 倍速冒烟回放，
CPU0、图像队列32、OMP线程限制1。均产生26次 LIO、25次 VIO、2个关键帧和1个正常封存的在线组；
summary 中 keyframes_enabled/submaps_enabled 为 true，submap_mode 为 online。
Noetic 正常关闭，Humble passed=true 且输入序列化载荷精确；两份在线索引均由移动后的普通 SDK CLI 读取。
这只验证工具接线，不代替完整 bag 配对回归或闭环准确性验证。

节点及库复用 Task 4 已验证产物：本阶段生产 src/、include/ 未变，并逐文件核对 Task 4 摘要。
Noetic 回放使用独立源码和产物副本 task5-replay-ws/，没有放宽工具的源码链接保护。
Humble 复用已核验的 ROS2 转换数据。Humble 的 Python 工具测试实际通过5项；
本机 Python 3.8 的同一测试集合因版本要求跳过，没有把跳过计为成功。

Task 5 普通核心仍为267项测试和50个公开头，全部通过。新增证据包括：
task5-cli-red.log、task5-cli-moved.log、task5-consumer-ldd.log、task5-moved-consumer.log、
task5-ctest.log、task5-noetic-online-smoke/、task5-humble-online-smoke/。
复现命令见包外 task5-commands.md；当前参数、CLI 和兼容出口见[使用说明](online-submap-dataset.md)。
