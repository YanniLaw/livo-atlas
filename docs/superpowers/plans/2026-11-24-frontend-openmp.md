# 前端 OpenMP 运行时预算实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 用 YAML 配置每个前端实例的 OpenMP 预算，并让构建按工具链能力选择 OpenMP。

**Architecture:** `LIVO_ATLAS_USE_OPENMP=AUTO|ON|OFF` 只选择工具链能力，默认 AUTO。`frontend.openmp_threads` 和 `FrontendOptions::openmp_threads` 默认 2，必须为正数，1 为串行。预算通过构造函数传给 LIO/VIO，每个并行区使用 `num_threads`，不改变 OpenMP 全局设置。无 OpenMP 的构建仍接受相同 YAML，记录串行回退。

**Tech Stack:** C++17、CMake 3.16、OpenMP、GLOG、Noetic、Humble。

**Spec:** `docs/superpowers/specs/2026-11-24-slam-engineering-optimization-design.md`；`docs/engineering-roadmap.md` 中的线程预算。

## Global Constraints

- 只修改 livo-atlas；保持共享 ROS 入口和已有传感器标定。
- Google C++ 风格，函数实现间空一行；不引入 Pimpl。
- 保留旧 ON/OFF 配置及常见 CMake 布尔值；ON 缺失能力时配置失败，AUTO 缺失时串行。
- OpenMP 导出依赖必须与实际链接一致，支持关闭后在同一构建目录重新配置。
- VIO 按观测顺序汇总误差，保持串行 EKF 终止行为；同机 1/2/4 线程要求原始数据与基线一致。
- 初次对照发现浮点 reduction 使前端最大位置差 2.27 厘米，并放大为优化位姿差异；使用逐点误差槽和顺序汇总消除该差异，增加 O(观测数) 临时空间。

## Review Focus

1. 同进程不同前端预算互不干扰；外部 OpenMP 环境仍可缩小实际团队。
2. 无效参数在输出 I/O 前拒绝，直接 SDK 调用遵循相同约定。
3. 无 OpenMP 的 AUTO 和显式 OFF 不导出 OpenMP 依赖。
4. LIO 并行写入使用独立元素，按输入顺序归并。
5. 回放确实进入多个 OpenMP 工作线程，避免被 OMP_THREAD_LIMIT=1 隐藏。

### Task 1: 构建、预算和实例传递

**Files:** `CMakeLists.txt`、`cmake/{core,ros1,ros2}.cmake`、`include/livo_atlas/frontend/{frontend_options.h,vision/vio.h}`、`include/livo_atlas/mapping/voxel_map.h`、对应实现、`src/core/{livo_config,system_options}.cc`、四份传感器 YAML、现有前端/系统/ROS 参数测试。

**Interfaces:** 新增尾部字段 `int FrontendOptions::openmp_threads = 2`；VIOManager 和 VoxelMapManager 的已有构造函数增加默认线程参数。现有调用继续成立。

- [x] 先添加正数、缺省值、非正数及两种 ROS 参数读取测试，运行并观察失败。
- [x] 实现 AUTO/ON/OFF 探测、预算传递和 GLOG；两个并行区使用实例预算。LIO 用字节标记替换带锁的压缩布尔标记，保持归并顺序；VIO 同样按观测顺序汇总误差。
- [x] 构建并通过相关 SDK、Noetic、Humble 测试；检查 AUTO 缺失、ON 缺失、OFF 和重配置，以及安装后消费者。
- [x] 完整 bag 验证串行基线，实际 2/4 线程验证及实例隔离探针；保存耗时和数值差异。
- [x] 更新构建文档、执行记录，本地提交 `feat: configure frontend OpenMP thread budgets`。

验证使用当前工作区的 `Retail_Street.bag`，证据放 `validation/frontend-openmp-20261124/`；独立构建复用 `validation/system-backend-20261123/sdk-build`。

## 验证记录

实现与验证已完成。SDK 76 个 CTest 目标、Noetic 9 个核心目标和 40 个 ROS 用例、Humble 10 个目标、4 个安装后消费者通过。完整 bag 的五组预算对照及最终程序的补充回放通过，前端资产字节一致，最终位姿差低于 1e-12。实际并发、故障注入、独立复核与待补测试见 [验证记录](../../threading-validation.md)。
