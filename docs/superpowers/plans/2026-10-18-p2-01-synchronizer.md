# P2-01 同步器执行计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 将现有同步与点云切分迁入普通 C++ 库，保持 Noetic 有效输入的窗口及数值结果。

**Architecture:** SensorBuffers 成对保存普通测量；SensorSynchronizer 操作队列及 SynchronizedMeasurements。ROS1 负责锁和日志，原估计器继续更新同步输出中的 last_lio_update_time。先测原实现，再实现核心，最后接线并回放。

**Tech Stack:** C++17、Eigen、PCL common、OpenCV core、GTest、ROS1 Noetic、Python 3。

**Spec:** `docs/superpowers/specs/2026-10-18-p2-01-synchronizer-design.md`。

## Global Constraints

- 只修改 livo-atlas；vikit 保持 6c886c8e5d83997806e00294826d528cea3581dd。
- 使用当前 refactor-fast_livo2 分支原地实施；运行使用独立源码/构建副本。没有推送、合并或重新授权步骤。
- 新代码遵循 Google C++17；保留原消费者字段名的兼容别名，不增加第二份状态。
- 不更改滤波、积分、去畸变公式、相机或点时间单位；保持 double 秒和 curvature 毫秒。
- 不增加线程、事件总线、EOF 排空、全估计器 reset、后端或 3DGS 实现。
- nice -n 10、-j1；构建与回放串行；输出在 validation/P2-01-noetic-20261018，不覆盖先前阶段。
- 基线 1bca9a3d706b2c25afd50a6d197db51e4474d567；控制配对使用 OMP_THREAD_LIMIT=1、OMP_DISPLAY_ENV=TRUE；轨迹和导出状态零容差，不对齐、不裁剪。

## Review Focus

1. 点/IMU恰好在窗口边界时，严格小于和含等号必须不同；跨窗口剩余点不能重复或遗漏。Task 1 原实现与 Task 2 核心测试覆盖。
2. 接收覆盖时间与 ROS 最终舍入时间不一致时，ONLY_LIO 仍使用原 double；LIVO 使用队尾样本时间。Task 1/2 分别验证。
3. 等待时不得消费数据或让后到扫描替换队首；过旧图像只丢弃一帧，并报告可由 ROS1 记录的状态。Task 2 验证。
4. 空/空指针云、单点 ONLY_LIO、内部无效云不能越界或永久堵塞；LIVO 的既有三队列门控必须明确，不得悄悄增加末尾 VIO。Task 2 验证。
5. 成对队列迁移后，坏消息不能清空有效数据，接收端时间修正不能重复，mutex 必须覆盖全部队列操作。Task 3 的真实回调及完整回放覆盖。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| tests/lidar_callback_test.cc | 真实 LIVMapper 同步行为的迁移前后特征测试。 |
| include/livo_atlas/frontend/synchronized_measurements.h | 模式、更新类型和旧同步结构的唯一实现。 |
| include/livo_atlas/sensor/synchronizer.h, src/sensor/synchronizer.cc | 普通队列、同步返回值和同步/切分实现。 |
| tests/frontend/synchronizer_test.cc, tests/frontend/CMakeLists.txt | 核心行为 RED/GREEN、无 ROS 构建和两个头独立包含。 |
| include/common_lib.h | 原枚举常量和类型的兼容别名。 |
| include/LIVMapper.h, src/LIVMapper.cpp | 成对队列、配置同步器、持锁委托和 ROS 日志。 |
| CMakeLists.txt, docs/README.md, docs/p2-01-validation.md | 库/测试接线、结果和跨机复现。 |

### Task 1: 固定原同步行为

**Files:** tests/lidar_callback_test.cc、CMakeLists.txt；独立候选副本与包外验证脚本。

**Interfaces:** 继续调用真实 `LIVMapper::sync_packages(LidarMeasureGroup&)`，通过实际配置选择 LIVO、ONLY_LIO、ONLY_LO；新增用例留在现有 lidar_callback_test 的同一编译单元，复用独立 rostest master。

- [x] **Step 1: 基线与特征测试**
  运行当前 78 项单元和 10 项 Python。添加 3 项测试：100/100.125/100.25 秒与点偏移 0/125/250 ms 验证严格切分和同时间 LIO→VIO；ONLY_LIO 接收覆盖时间为 nextafter(100.125,-inf) 时等待、达到 100.125 时处理；ONLY_LO 无 IMU/图像时输出扫描尾部。
- [x] **Step 2: 独立构建、运行原实现**
  git archive 基线创建 candidate_ws，加入测试；显式 CATKIN_DEVEL_PREFIX，单线程构建 fastlivo_mapping、lidar_callback_test、run_tests。运行 rostest，预期原 2 项回调与新增 3 项同步测试全部通过；算法源码摘要与基线相同。记录 XML、源码及运行库摘要。
- [x] **Step 3: 提交与任务检查**
  Commit: `test: characterize existing sensor synchronization`。任务检查脚本验证 78 单元、5 ROS、10 Python，输出明确 OK。

### Task 2: 实现独立同步核心

**Files:** 两个新公共头、src/sensor/synchronizer.cc、tests/frontend 下测试及 CMake。

**Interfaces:** 采用 Spec 的精确类型、枚举值、构造函数和 TrySynchronize 签名。SensorBuffers 三条队列及 latest_imu_time；输出保留旧结构字段与初值，且不自动推进 last_lio_update_time。

- [x] **Step 1: 测试和可编译空实现**
  除 Task 1 三组行为外，覆盖缺输入等待、曝光偏移、过旧图像阈值、覆盖不足不消费、严格 IMU 窗口、重复等待保留队首、多个扫描切分、剩余点跨窗口、输入云不被改写、VIO 图像所有权、VIO 保留原三队列门控、尾点而非最大偏移作为覆盖、空/空指针云、ONLY_LIO 单点移除和内部无效云。TrySynchronize 空实现只返回 kWaiting，测试不得解引用未产生的输出。
- [x] **Step 2: 运行 RED**
  独立 cmake -S tests/frontend -B <证据>/frontend-build，nice 构建 -j1 后运行测试。预期得到内容/状态断言失败，不把编译或链接错误计为 RED。
- [x] **Step 3: 实现 GREEN**
  直接按原分支实现，保留浮点表达式、顺序、旧就绪门控及剩余云语义。仅增加 Spec 已定义的无效云边界处理；不引入模拟 ROS 或第二份同步算法。
- [x] **Step 4: 全部检查与提交**
  独立测试及两个公共头编译通过，无 ROS/vikit/native/fast-math flags；当前 Noetic/Python/ROS 特征测试仍绿。Commit: `refactor: add standalone sensor synchronizer`。

### Task 3: 接入 ROS1 与数值验收

**Files:** common_lib.h、LIVMapper.h/.cpp、CMakeLists.txt、两组 ROS 测试的输入接线及验证文档。

**Interfaces:** common_lib.h 转发旧名称和枚举常量；Mapper 持有 SensorBuffers/SensorSynchronizer，保留原 last_timestamp_imu 及高频 ROS stamp。全部回调把 P1-03 测量直接入成对队列，sync_packages 持锁委托并将返回状态映射到旧 bool/日志。

- [x] **Step 1: 迁移接线**
  原 5 条平行队列替换成 SensorBuffers；新同步器按原 slam_mode_/exposure_time_init 配置；删除 mapper 中同步算法和 lidar_pushed。两组 ROS 测试只调整缓冲输入/观察接线，Task 1 数值断言不变。
- [x] **Step 2: 全套直接验证**
  主工程与独立候选都构建 fastlivo_mapping/run_tests；执行全部 C++、10 Python、5 ROS、40 启动检查；独立 core/sensor/frontend 及公共头检查。所有实际 XML 用例通过。
- [x] **Step 3: 参考校验及三次候选回放**
  逐一核对 P1-03 最终候选的源码/全部运行库/bag/环境，校对 6 个估计源编译参数；复用其短段/完整记录。候选依次运行 30 s、136 s、默认并行 45 s；预期 296/1351/446 点、输入和更新计数一致、轨迹及 mat_pre/mat_out 导出值完全一致、正常退出。
- [x] **Step 4: 记录与提交**
  写 docs/p2-01-validation.md、更新文档索引，记录实际命令、边界、证据与未完成的 P2-02/P2-03。Commit: `refactor: delegate ROS1 synchronization to the core`。运行最终 verify.sh，全部通过后完成任务。

## 收尾

完成任务后执行一次独立只读审阅，重要问题一次集中 RED→GREEN 修复并运行全套验证，不循环重审。保留所有取舍及代价、执行账本、完整 Git bundle 和 SHA256；当前分支保持本地，不合并、不推送。
