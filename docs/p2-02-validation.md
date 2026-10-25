# P2-02：单一前端状态与处理流程

本阶段按[设计](superpowers/specs/2026-10-24-p2-02-frontend-design.md)和[执行计划](superpowers/plans/2026-10-24-p2-02-frontend.md)，从 LIVMapper 提取实际 Frontend。参考提交为 `f77ae4871b9f72a469918ea6a5617db6067566d8`。本阶段实现、独立审阅问题修复和最终验收均已完成。106项单元、9项真实ROS、10项Python、40项启动、1项地图生命周期泄漏检查，以及61项独立核心/传感器/同步检查和14个公共头通过。修复后的3秒真实残差调试正常退出；最终30秒/完整136秒/默认并行45秒分别296/1351/446点，控制配对时间、计数和三份状态/轨迹导出数组完全一致。全部正式最终记录均在审阅修复后重新采集。

## 已实现的边界

`Frontend` 私有实现持有主滤波状态、传播先验、IMU、VIO、局部体素地图和算法点云，顺序执行首次时间标记、IMU/对齐、LIO/VIO及地图更新。VIO 的状态指针绑定同一个主状态，不复制成第二个滤波器。Frontend 不可复制或移动，内部地址在生命周期内固定；相机先于 VIO 创建、晚于 VIO 析构，继续使用此前的 vikit 分配/析构工厂。

| 入口 | 职责 |
| --- | --- |
| `FrontendOptions` | 普通标定、IMU/VIO/地图及点云设置；值来自原有参数，不增加 YAML 键。 |
| `VoxelMapOptions` | 原配置结构的唯一普通实现；旧 VoxelMapConfig 仅为兼容别名。 |
| `Frontend::Process` | 消费同一份同步输出，原地推进其 last_lio_update_time，完成实际算法处理。 |
| `FrontendResult` | 按值保存预测/估计状态、传感器时间、相对时间及更新状态；点云为只读视图。 |
| `LIVMapper::ProcessMeasurements` | 调用前端，使用结果写原状态/轨迹文件并执行 ROS 发布，更新高频传播的值快照。 |

公开前端头仅含普通 C++/Eigen/PCL/OpenCV 类型；VIOManager、VoxelMapManager 只前置声明。两个借用访问口 visual/local_map 暂供原 ROS 彩色点云/地图输出使用。旧算法内部的 ROS 日志、地图发布、ROOT_DIR 文件和 vikit 仍在，因此整个 Frontend 实现本轮继续依赖 Noetic；公共头可单独编译不等于完整算法已实现无 ROS 构建。后续 P2-03/P3 继续移除这些依赖。

P2-01 的缓冲、同步器及同步输出继续由 Mapper 持有并在同一个 mutex 下操作；Frontend::Process 是成功同步后的顺序调用。时间仍为 double 秒，curvature 仍为毫秒。输入需有序且时间有限；调用者遵守现有标定/图像/网格约束。结果中的状态是独立值；只读点云视图的使用期限至下一次 Process，长期关键帧需另行复制，不能异步保留借用的旧组件引用。

## 发布不再控制前端是否可更新

原 publish_frame_world 会清空前端下一次 VIO 使用的世界云。本轮将消费动作放入 Frontend：成功 VIO 返回只读云视图，内部换成空云；调用者可以跳过全部发布。

专门的真实集成用例先完成 IMU 初始化和一次实际 LIO/VIO，再输入无点 LIO/VIO 窗口，全程不调用发布。末次必须 updated=false。独立副本暂时省略核心消费语句时，其余 8 项通过，只有 `VisualCloudIsConsumedWithoutPublishing` 失败，实际 updated=true；恢复正式实现后重新检查。回归探针的源码、程序和本地库保存在证据目录 `ownership-red/artifacts`，没有用后来的构建覆盖。

## 状态日志与计时

prediction 在 IMU/可选对齐之后、LIO/VIO 更新之前按值捕获；原 mat_pre 在空点云/初始化阶段仍写一行。updated=false 时不发布、不写 mat_out。成功更新按 estimate 写原结果。

ROS 发布从 LIO 地图更新前移到完整处理后；地图更新不改变滤波状态。必须通过导出值与更新数量的零容差比较来验证这一迁移，不能用轨迹对齐或阈值放宽。

LIO 计时现在在接入层外侧测量整个 Frontend::Process，包括 IMU；保留一处 `LIO Mapping Time` 和 Current Total Time 以支持原验证计数。新的时延不能直接与旧分阶段表格比较；VIO 内部原计时暂留后续拆分。新 frontend.cc 本身不调用 ROS、时钟或文件 I/O API，但其调用的旧算法仍可能执行这些副作用。

## 测试与证据

本机证据位于 `validation/P2-02-noetic-20261024/`。迁移前 98 项单元、8 项真实 ROS、10 项 Python 通过，生产源码与固定参考一致；`characterization-artifacts` 保存原源码、运行文件和 XML。真实 IMU 用例使用 header=(100s,100000000ns)、LiDAR头0.975s和整秒补偿，样本最终时间1.1s，覆盖 double 略小于1.1，ONLY_LIO 应等待；再接收1.25覆盖，回退消息不改变有效队列或时间。这补齐了 P2-01 的延期测试。

新 Frontend 的 8 项测试对可编译空 Process 全部得到行为 RED；真实实现后 8/8 GREEN，完整单元增至106。覆盖空组、LO 连续处理和首次锚点、共享状态地址、地图连续性、状态值快照、实例隔离、初始化空云、空视觉和非单位外参。使用真实 IMU、PCL 和地图，不使用算法 mock。构建配置、链接顺序或可写图像引用的编译问题没有计为行为 RED。

真实 Frontend 初始化可能打开旧 IMU 日志，所以需要完成初始化的用例只在独立 ROS 测试副本运行。主工程的直接 Frontend 用例保持初始化前状态，不向源码树写入该日志。最终真实 ROS 共9项，外层 rostest 包装项不重复计数。

原六个估计源 preprocess.cpp、IMU_Processing.cpp、vio.cpp、frame.cpp、visual_point.cpp、voxel_map.cpp 保持不变。P2-01 已采集的短段/完整候选作为本阶段参考，仅在源码、可执行文件、全部8个本地动态库、bag、编译器/依赖及六源归一化参数一致后复用，不声称本轮重新采集参考。新候选分别采集30秒、完整136秒和默认并行45秒，控制配对采用 OMP_THREAD_LIMIT=1、OMP_DISPLAY_ENV=TRUE；对时间、计数、shape、有限性和导出值逐元素比较。

## 链接顺序回归与修复

首次短段在第一轮有实际平面残差的 LIO 后 double free；3秒 gdb 回放重复得到 StateEstimation 内 SIGABRT。六个估计源和参数未变，但传递依赖使可执行文件只直接依赖 laser_mapping，而其依赖顺序让 native vikit 先于估计器。两侧导出同名弱 Eigen 动态矩阵 resize 实现；不同对齐路径的分配/释放不能混用。

修复只在 CMake 显式将 vio/lio/imu_proc 放在 vikit 之前，不改算法或 vikit。相同3秒 gdb回放正常退出，26 LIO/25 VIO，三份导出数据与参考相应前缀逐元素相等。`before-link-order-fix/` 保存失败源码、全部运行文件、ROS/启动结果和两次失败；`link-order-fixed.json` 保存最终ELF依赖，`link-order-gdb-green` 保存通过记录。

这是仍依赖旧 vikit 时的兼容约束，后续构建调整须继续验证真实非空残差及回放；P3解除vikit后再消除此约束。两点无平面的小型单元不能替代这项集成验证。

## 正式回放记录

| 记录 | 轨迹点 | 状态日志 | 结果 |
| --- | ---: | --- | --- |
| 30秒 | 296 | mat_pre 597×19，mat_out 591×20 | 时间、计数、三份导出数组完全一致。 |
| 完整136秒 | 1351 | mat_pre 2707×19，mat_out 2701×20 | 时间、计数、三份导出数组完全一致。 |
| 默认并行45秒 | 446 | 三份导出数据全部有限 | 正常退出，不作逐值确定性结论。 |

完整输入为27447 IMU、1355点云、1355图像，更新为1351 LIO、1350 VIO。

这里的相等指现有导出精度，不表示未导出的全部内部状态逐位相等。参考经过摘要及环境校验后复用，没有重新采集或改写旧工作区。

## 跨机重建与复现

只复制源码、固定版 rpg_vikit 和 bag，创建新的 build/devel。C++17、Google风格，不能跨机复用 CMakeCache。只修改 livo-atlas；vikit 为 `6c886c8e5d83997806e00294826d528cea3581dd`。构建使用 nice -n10/-j1，不与回放并行，无 RViz。

独立公共头和已提取核心检查：

```bash
nice -n 10 cmake -S src/livo-atlas/tests/frontend -B validation/p202-headers
nice -n 10 cmake --build validation/p202-headers -- -j1
(cd validation/p202-headers && ctest --output-on-failure)
```

此入口仅查找 Eigen/PCL common/OpenCV core/GTest，编译同步器和5个公共头；`tests/core`、`tests/sensor` 维持原独立入口，共14个头检查。实际 Frontend 测试需要当前 Noetic 依赖，由主 catkin 构建 `frontend_test`/`run_tests`，不包含在上述无 ROS 链接目标中。

ROS 配置依赖和创建独立源码副本见[通用说明](validation.md)。新候选先显式配置 CATKIN_DEVEL_PREFIX，再以 -j1 构建 fastlivo_mapping、lidar_callback_test、run_tests；真实 ROS 测试只能 source 独立副本运行：

```bash
source validation/p202-candidate/devel/setup.bash
rostest livo_atlas lidar_callbacks.test
```

Python 测试必须在包目录运行：

```bash
cd src/livo-atlas
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
```

启动检查使用 tools/check_calibration_startup.py 的全部40个案例，传入独立 workspace、新输出目录及空闲 master 端口。回放从工作区根目录运行，每次输出必须不存在：

```bash
OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE /usr/bin/python3 \
  src/livo-atlas/tools/run_baseline.py \
  --workspace validation/p202-candidate --bag Retail_Street.bag \
  --output validation/p202-short --duration 30 --rate 0.5 --port 11402
```

之后运行完整136秒，再取消两个OMP控制变量运行默认并行45秒。预期轨迹点数296/1351/446；控制配对的两项最大轨迹误差设为0，另比较mat_pre/mat_out导出数组。`verify.sh` 重跑直接检查并核验已采集的真实XML/摘要/回放；`run_validation.py`/`build_commands.sh` 保存本机实际步骤。

## 独立审阅与生命周期修复

一次只读全阶段审阅覆盖 `f77ae48..187c8b0`，提出1项 Important：新 Frontend 析构没有释放旧 VoxelMapManager 容器中的原始体素树指针。另有新增文件行末空白使整个阶段差异的格式检查失败；该问题按明确的验收门槛提升为 Important，同一批修复，不再进行第二轮审阅。

私有实现析构先销毁视觉借用者，再逐个删除剩余根节点，复用原 VoxelOctoTree 的递归析构，最后清空容器。六个估计源文件/数值公式保持不变。

实际生命周期检查连续创建4个前端、执行真实LO建图并析构。无地图对照也保留12680字节、7处外部TBB/VTK/PCL分配；规则仅匹配这5个已确认的依赖分配站点，不含应用或估计器。应用地图在限定规则后仍 RED：11872字节/32处（8根、8平面、8点向量、8层级向量）。修复后相同检查退出0。报告、无地图对照、规则和程序摘要均保存在 `final-review/`。

本机单独 liblsan 在进入 main 前初始化失败，该错误没有计为泄漏 RED；随后链接集成 ASan/LSan 运行库进行分配/泄漏观察。仅诊断程序带 sanitizer 链接选项，估计器及其所有编译参数不变，因此不声称执行了完整插桩的地址越界检查。此检查也不宣称系统库不存在泄漏。

Linux GCC/Clang 可显式构建诊断程序；普通 run_tests 不运行需要进程线程检查权限的 sanitizer：

```bash
nice -n 10 cmake --build validation/p202-candidate/build \
  --target frontend_lifecycle_check -- -j1
ASAN_OPTIONS=detect_leaks=1 \
LSAN_OPTIONS='suppressions=/absolute/path/to/src/livo-atlas/tests/frontend/map_lifecycle.lsan' \
  validation/p202-candidate/devel/lib/livo_atlas/frontend_lifecycle_check
```

平台依赖改变时须重新检查外部对照，不能随意扩大排除规则。范围格式检查使用 `git diff --check f77ae48` 覆盖已经提交与未提交的整个阶段，而非只检查当前工作区差异。

审阅中的未判断项已逐项裁定：继续保留ROS/vikit过渡实现及借用访问口、已验证的Noetic链接顺序、原数值/输入约定、直接调用前置条件和顺序执行契约。旧ROOT_DIR/静态输出计数器留给P2-03；无残差时原诊断-nan留待单独清理，不能当作有限质量指标。没有据此承诺异步保留对象、多个实例的独立输出文件、任意链接顺序或其他设备/平台/性能/真值结果。新计时定义和发布墙钟时刻按本轮既定取舍保留。完整理由和代价记在执行账本，没有未处理的重要发现，也没有延期Minor。

## 后续范围

P2-03 将迁移 ROS 发布和文件输出，P3 移除 vikit 并验证完整无ROS核心，P4 再实现 Humble 接入。保留原首帧绝对时间锚点、完整时钟回退 reset、disB初值及其他驱动/缺ring约定的独立修复边界。本 bag只覆盖非特征 Livox+IMU+相机，不证明其他设备、特征分支、ARM/Humble、异步安全、1倍实时或真值精度。

本机未安装clang-format；新代码按Google C++17人工核对，并执行git diff --check，不声称自动格式化已通过。

## 离线交付

最终源码完整历史保存在 `validation/P2-02-noetic-20261024/livo-atlas-p2-02-20261024.bundle`，同名 `.sha256` 提供摘要。已在空Git仓库校验其全部前置历史，`completion.json` 记录最终源码提交、验证及归档状态。换机示例：

```bash
mkdir -p src
git clone -b refactor-fast_livo2 \
  validation/P2-02-noetic-20261024/livo-atlas-p2-02-20261024.bundle \
  src/livo-atlas
```

另行带上固定版rpg_vikit和Retail_Street.bag。`execution-record/`保存本阶段账本、任务验证日志和一次审阅差异；审阅报告及修复前完整源码/运行文件/记录保存在`final-review/reviewed-version`，最终结果在阶段根目录。失败或审阅前结果没有覆盖为通过结果。
