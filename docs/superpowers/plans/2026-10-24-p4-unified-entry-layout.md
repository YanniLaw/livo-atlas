# 统一 ROS 入口与职责目录实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 用同一入口编译 Noetic/Humble 节点，并将剩余旧算法文件按职责分类。

**Architecture:** `src/app/mapping_main.cc` 按目标私有宏选择 ROS 初始化、已有适配器和执行器；`cmake/node.cmake` 只定义一次节点。两种适配器复用现有普通核心，算法只迁移文件和 include 路径。

**Tech Stack:** Google C++17、CMake 3.16、ROS1 Noetic、ROS2 Humble、catkin/ament。

**Spec:** `docs/superpowers/specs/2026-10-24-p4-unified-entry-layout-design.md`

## Global Constraints

- 只改 livo-atlas，rpg_vikit 和 P4 验证证据只读。
- Google C++17；既有算法不整文件格式化、不改公式或类名。
- 版本宏只用于节点入口，普通核心无 ROS 条件。
- mode0 不创建 ROS 节点；模式1/2的节点名、话题、参数与退出语义保持。
- 不在根目录遗留转发头，源码使用方必须迁移 include；安装和消费者同时更新。
- 现有 Humble 容器沿用，新的候选只写 `/tmp/livo_atlas_layout`。
- 新证据 `validation/P4-layout-20261024`，构建 nice -n10/-j1，和回放串行，无 RViz。
- 一次整阶段独立评审，重要问题一次测试修复；不合并/推送。

## Review Focus

1. 错误配置或处理中异常：共用入口保持错误码1，ROS2 shutdown，正常退出执行 Finish。
2. 源码与安装消费者包含新头：头文件自包含，mode0 SDK 无 ROS，旧根头不残留。
3. 包含视觉地图点的不同链接顺序：共享/静态安装消费者都能真正执行 LO/LIO/VIO。
4. 构建环境自动选择与显式 mode0：只有节点带版本宏，两种模式实际编译同一个 TU。
5. 路径变化影响运行时资源：实际安装节点完成 bag 回放，四组数组、发布与产品保持。

## 文件与职责

spec 中的 source/header 映射是本计划的完整迁移表；保持已有库目标及链接图。
`src/app/mapping_main.cc` 负责进程生命周期；`cmake/node.cmake` 负责节点目标与安装。
`frontend/imu`、`frontend/vision`、`mapping`、`sensor` 分别收纳相应原实现；
`ros1/mapper` 收纳 LIVMapper；`internal/legacy` 收纳旧全局别名/宏。
当前构建文档 `docs/core-build.md`、`docs/ros-builds.md` 与 README 记录新布局、入口及迁移规则。
验证脚本仅放新证据目录：复用 P4 runner 的副本，参考路径指向冻结 P4，禁止覆盖其输出。

### Task 1: 同一个实际入口

**Files:** Create `src/app/mapping_main.cc`, `cmake/node.cmake`; remove `src/main.cpp`, `src/ros2/main.cc`; modify `CMakeLists.txt`, `cmake/{ros1,ros2}.cmake`。

**Interfaces:** Consumes 现有 `LIVMapper(ros::NodeHandle&)`、`MapperNode()`/`Finish()`；produces 同名 `fastlivo_mapping`，私有 `LIVO_ATLAS_ROS_VERSION=1|2`，普通库接口不变。

- [x] **Step 1: 验证旧构建不满足同一 TU。** 新证据脚本读取 P4 两份 compile_commands，断言节点源均为 `src/app/mapping_main.cc`，记录旧入口失败；不为目录移动添加产品单元测试。
- [x] **Step 2: 实施统一入口和节点 CMake。** 使用一处 main、共用状态和异常处理，条件分支保留两个版本各自生命周期；节点 RPATH、可选 mimalloc、安装目的地保持。
- [x] **Step 3: 构建两种真实节点并验证编译输入。** 新 Noetic candidate_ws、新 Humble build2，串行 `cmake --build ... --target fastlivo_mapping -- -j1`；断言两种模式同一 TU、宏私有、普通核心不含 ROS 依赖。
- [x] **Step 4: 运行实际启动检查。** Noetic 50 例，Humble 16 例，验证正常 SIGINT 与无效配置的非零退出，输出到新目录。
- [x] **Step 5: 提交。** `refactor: share the mapping entry across ROS versions`。

Expected: 旧编译输入断言失败；新节点编译成功，统一 TU 检查和 66 个启动检查通过。
Task gate: `/usr/bin/python3 validation/P4-layout-20261024/check_layout.py entry`。

### Task 2: 按职责迁移源文件和头

**Files:** spec 中所有迁移路径、全部直接 include 使用方、`cmake/{core,ros1,tests}.cmake`、`tests/sensor/CMakeLists.txt` 和发现的独立测试构建引用。

**Interfaces:** Consumes Task1 节点与旧类/库名；produces 规范新路径，无根转发头；保留全部 C++ 类型/符号。

- [x] **Step 1: 编译新路径消费者确认缺失。** 使用头文件自包含探针包含 `livo_atlas/frontend/imu/imu_processor.h`，旧树报该头不存在。
- [x] **Step 2: 迁移并更新引用。** 执行 spec 映射，更新 include、guard、构建安装与测试路径；必要时补标准自包含 include，不更改算法正文。
- [x] **Step 3: 检查迁移内容。** 对5884f15旧路径和新路径逐文件去除 include/guard 后比较，正文完全相同；实际包根无剩余算法实现或旧转发头。
- [x] **Step 4: 构建全部现有测试和新头。** Noetic158、ROS12、Humble176，mode0安装检查的头文件逐个编译；Python Noetic13和Humble5均通过，Noetic ROS2工具模块按现有条件跳过。
- [x] **Step 5: 提交。** `refactor: group estimator sources and headers by responsibility`。

Expected: 新路径探针先缺头失败，再编译成功；两个版本全部原测试通过，正文比较一致。
Task gate: `/usr/bin/python3 validation/P4-layout-20261024/check_layout.py sources`。

### Task 3: 安装、实际回放与可移植交付

**Files:** `docs/{core-build,ros-builds,cpp-style}.md`、README、本阶段验收记录；验证副本在新证据目录，当前仓库不增加框架。

**Interfaces:** Consumes Task1/2相同目标与新头；produces 可复现文档、完整验收结果与独立历史 bundle。

- [x] **Step 1: mode0三个安装消费。** shared/static均运行138例和所有普通头检查，禁OpenMP安装消费者执行LO/LIO/VIO；Humble ament安装消费者同样通过，无旧根头。
- [x] **Step 2: 官方 bag 回放。** Noetic catkin/Humble真实colcon安装节点各运行3s产品和136s完整窗口，对各自P4同环境结果严格比较 trajectory/mat_pre/mat_out/imu、发布语义及57产品；ROS2回调输入计数和14个运行文件SHA保持绑定。
- [x] **Step 3: 更新当前文档。** 给出单入口流程、新目录、旧include迁移表、三模式构建命令、执行证据、仍存在的legacy内部接口边界；历史P4文档保留原样。
- [x] **Step 4: 提交可审查实现并执行一次整阶段评审。** 比较5884f15..HEAD，评审spec/plan/ledger及五项Review Focus，重要问题一次RED→GREEN修复与相关全套验证。
- [ ] **Step 5: 收尾。** 归档本计划执行记录，新完整历史bundle在空仓库恢复验证；保留当前分支，不合并/推送。

Expected: 各模式安装成功，回放同环境严格一致，源码Log和vikit不变，bundle可恢复最终提交。
Task gate: `/usr/bin/python3 validation/P4-layout-20261024/check_layout.py acceptance`。
