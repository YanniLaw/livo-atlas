# P6-01 实施进度与验证

状态：截至2026-11-09，Task 1–6完成，P6-01验收通过；保留1项轻微文档边界差异。实施顺序见
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

## Task 6 最终验收（2026-11-09）

最终产品源码为 `f2450d5`，以下均使用审阅修复后的最终源码。

| 配置 | 最终结果 |
| --- | --- |
| 普通 shared / static / 无 OpenMP | 各268项测试、50个公开头、独立安装迁移消费及5项CLI测试通过 |
| Noetic 独立源码工作区 | 288项C++、9项实际ROS、50项实际启动检查通过 |
| Humble colcon/ament 安装 | 306项测试（含DDS）、迁移安装消费、27项实际启动检查通过 |
| 源码与产物 | 两端218个文件摘要与最终源码一致；每次实际加载的14个自有库及节点均已记录 |

两端各完成 online 3秒产品检查，默认/旧分组/online 三种模式各30秒和完整136秒请求，
合计14次回放。四数组 trajectory.tum、mat_pre.txt、mat_out.txt、imu.txt 及发布载荷
均精确匹配冻结同环境参考；30秒场景检查参考前缀。算法参数一致，不放宽数值阈值。
默认关闭关键帧时无新增数据集；启用的完整运行各172个关键帧、517个原始资产文件逐字节一致。
两端短段各57个原产品文件逐字节一致。新增索引单独校验，不混入旧资产比较。
完整运行均有1351次LIO、1350次VIO；速率1、CPU0、图像队列32、OMP线程限制1。
同环境各运行加载的节点和自有库摘要一致。参考为P5-04的 candidate_full_1 / humble_full_2，
未使用已知启动变体替代正常参考。

| 环境/模式 | 峰值RSS/MiB | LIO p95/ms | VIO p95/ms | 回放墙钟/s |
| --- | ---: | ---: | ---: | ---: |
| Noetic 默认 | 790.4 | 44.94 | 23.94 | 137.17 |
| Noetic 旧分组 | 788.4 | 45.00 | 24.37 | 137.37 |
| Noetic online | 792.4 | 44.36 | 24.01 | 137.17 |
| Humble 默认 | 813.3 | 47.75 | 17.35 | 135.56 |
| Humble 旧分组 | 820.3 | 46.79 | 16.88 | 135.63 |
| Humble online | 819.3 | 47.17 | 16.85 | 135.48 |

计时为前端Process打印时间，不含完整I/O/通知开销，也不是训练或渲染FPS。
一条ROS2退出日志插入计时数字中间；汇总仅移除格式明确的signal_handler日志行后解析，原日志保留。
两端online关键帧目录约210.7MiB。采样差异不构成性能提升结论；本次ROS运行没有训练消费者，
不能外推其快照物化、队列或GPU成本。

## 真实观测的几何与持久化契约

安装SDK逐一读取两端全部172张图像和点云：Noetic 3,763,228点、Humble 3,762,300点。
按原始观测重建的成员和修订与原运行一致，分别13/14组，全部封存。分组按各环境原始轨迹计算。
跨环境轨迹最大位置差0.02527m、最大角度差0.06746°；没有作跨环境逐值相等声明。

在独立派生数据集中，先保存冻结M170/N168的模拟优化版本，再追加最后两帧并物化到M172：
冻结文件不变，未优化尾帧标记保持。非平凡Rx90整体修正保留局部姿态与相机相对关系；
第二版只移动一个非锚点观测0.25m，分类仅报告实际引用它的主成员/支持区域。
两份原运行目录全部文件摘要前后不变。合成输入不代表真实闭环精度。

## 独立审阅与实施取舍

一次独立只读审阅覆盖 ad38dc3..64c3f35。重要问题是原始关键帧保存失败后，
ApplyPoseUpdate 可绕过系统终止错误提交新版本。真实pending目录阻断用例先失败，
`f2450d5` 将位姿更新和非空回调注册入口纳入系统错误守卫；只读快照与空回调停用仍可用。
RawSaveFailureBlocksOnlineMutations 和全部最终配置/回放通过，无未处理的重要问题。

延期Minor：实现允许在首个已提交关键帧之前注册回调，文档建议在第一帧/处理前注册。
边界更宽松，不丢失已提交观测；后续单独统一措辞或边界。

本阶段全部实施取舍及其边界如下，原始ledger也保存在验证目录：

1. 共用私有坐标运算头；其中错误会同时影响尾帧与修正分类，已有对应覆盖。
2. 串行所有者检查终止状态；漏检可能允许错误后继续处理，真实I/O失败回归覆盖此风险。
3. Task5仅工具变化时复用同源节点并核对生产源码；Task6重新构建，排除旧产物混用。
4. 长验收前做一次独立审阅；后续修复靠失败用例和完整最终验收验证，不重复审阅。
5. 后端、闭环、Gaussian/GPU延后；不证明未实现模块的精度或性能。
6. 并发调用、回调重入及共享资产别名修改禁止；违反约定会造成竞态或资产污染。
7. 不增加fsync事务及最高修订整段删除检测；仍有丢失最新修订的保证边界。
8. 构建、回放和资源结论以实际证据为准；135秒数据不能外推长距离、长期或在线训练表现。

## 最终证据与后续

证据位于 validation/P6-01-20261108/，完成日期2026-11-09。
acceptance-summary.json 汇总最终结果；各运行的strict-checks.json与runtime-artifacts.json
绑定精确比较及实际加载产物；acceptance-source-manifest.json绑定最终源码。
geometry-noetic.json / geometry-humble.json记录完整资产及模拟修正结果。
acceptance_sdk.py、acceptance_build.py、acceptance_replays.py、runtime_runner.py、
after_replays.py、summarize_acceptance.py保存复现流程。旧证据和原始bag保持只读。

当前完成共享观测、索引和位姿修正基础；P6-02几何/闭环后端、P6-03在线Gaussian mapper、
P6-04高斯辅助估计，各自在实施前细化设计与预算。
