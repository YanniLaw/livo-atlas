# 独立核心与 ROS 构建

相机模型、图像基础操作、同步、预处理、前端、文件输出与运行调度均为普通
C++17 库。`cmake/ros1.cmake` 和 `cmake/ros2.cmake` 分别加载 Noetic 与 Humble
接入库；公共 `cmake/node.cmake` 从同一个 `src/app/livo_main.cc` 创建节点。双版本操作见 [ros-builds.md](ros-builds.md)。
`rpg_vikit` 不再是 livo-atlas 的构建或运行依赖；工作区里原有的依赖源码可以保留。

## 依赖与编译选项

- CMake 3.16、C++17 编译器、Eigen3、PCL 的 common/filters/io。
- OpenCV 的 core/imgproc/calib3d/imgcodecs。
- 兼容 `sophus/se3.h` 的旧版 Sophus（原工程所用的非模板版本）。通过
  `Sophus_DIR` 或 `CMAKE_PREFIX_PATH` 指定实际安装配置；不硬编码库名。
- OpenMP 可选。`LIVO_ATLAS_USE_OPENMP=OFF` 同时关闭编译选项和并行宏。
- GTest 仅测试需要。`BUILD_TESTING=OFF` 构建独立库及数据集检查工具。

算法目标使用编译器默认指令集，不加 native ISA 或 fast-math。使用单任务编译，
并在构建完成后回放 bag，避免同时占用大量内存。

## 普通 CMake 工程

在 livo-atlas 包目录执行：

```bash
cmake -S . -B build-core \
  -DLIVO_ATLAS_ROS_VERSION=0 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install-core"
nice -n 10 cmake --build build-core -- -j1
cd build-core
OMP_THREAD_LIMIT=1 ctest --output-on-failure
cd ..
cmake --install build-core
```

静态库增加 `-DBUILD_SHARED_LIBS=OFF`。无需 OpenMP 时增加
`-DLIVO_ATLAS_USE_OPENMP=OFF`。构建入口不会在 mode 0 查找 catkin 或 vikit。
SDK 安装前缀可以在同一平台移动；系统依赖仍需可用，旧 Sophus 的非标准库目录
会由配置记录为运行搜索路径。跨平台应在目标平台重新编译和验证。

外部工程使用：

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_frontend LANGUAGES C CXX)
find_package(livo_atlas REQUIRED CONFIG)
add_executable(my_frontend main.cc)
target_link_libraries(my_frontend PRIVATE livo_atlas::core)
```

配置外部工程时指定 `-DCMAKE_PREFIX_PATH=/实际/sdk/前缀`。公开入口为
`livo_atlas/core/livo_system.h`。`livo_atlas::core` 提供完整输入、同步、估计与输出组合；
仅需自己调度前端时可使用 `livo_atlas/frontend/frontend.h` 和 `livo_atlas::frontend`。
`livo_atlas::camera`、`livo_atlas::colorizer`、`livo_atlas::io` 可以分别链接。
`livo_atlas::keyframes` 提供快照、选择及几何操作，`io` 提供数据集读写；
格式、位姿版本和检查工具见[关键帧数据集](keyframe-dataset.md)。
`common/` 只有共享定义，没有单独的物理库或聚合目标。
`tests/installed_consumer` 实际通过 Frontend 和 LivoSystem 两种接口执行连续
LO/LIO/VIO，并检查视觉跟踪、有限状态和文件写入。它只依赖安装头和导出目标。

## ROS1 Noetic

默认 `LIVO_ATLAS_ROS_VERSION=1`。在 catkin 工作区执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_BLACKLIST_PACKAGES="vikit_common;vikit_ros" -j1
source devel/setup.bash
catkin_make run_tests_livo_atlas -j1
rostest livo_atlas lidar_callbacks.test
roslaunch livo_atlas mapping_avia.launch
```

若工作区还有其他包，按该工作区原有约束调整黑名单。不要删除其他包源码。
节点名、话题和配置入口沿用现有行为，输出目录继续由 `output/directory` 指定。
mode 1 保留 catkin 包配置；独立 SDK 的 `find_package` 配置由 mode 0 安装。

外部 Noetic 工程通过 `${catkin_LIBRARIES}` 消费普通核心时，导出列表包含完整的
核心库及静态链接所需运行库。独立消费示例同时调用公开标定校验和连续 LO/LIO/VIO：

```bash
source devel/setup.bash
cmake -S src/livo-atlas/tests/catkin_consumer -B build-catkin-consumer
nice -n 10 cmake --build build-catkin-consumer -- -j1
build-catkin-consumer/catkin_consumer
```

该示例应在独立 CMake 工程中运行，以验证导出的库文件；它会拒绝借用原工程目标。
各模式的实际验证范围见[本轮记录](p4-naming-validation.md)。

`LIVO_ATLAS_ROS_VERSION=2` 提供 Humble 接入，要求已 source Humble 环境。
其 ament 安装导出同一套 namespaced core target；mode 0 不需要 ROS。
关键帧基础存储已提供；GTSAM 后端、子地图管理和新的 3DGS 导出属于后续阶段。

## 数值与接口约定

相机配置描述原始图像，新模型只缩放一次。为保持迁移可比较，保留旧库的
畸变开关和鱼眼光轴特殊分支；细节见
[参考样本说明](../tests/camera/data/README.md)。原始 native 库的 FMA 差异
单独记录，不用于放宽 bag 和导出比较。

`FrontendResult` 的状态为值快照，图像和点云视图有效期到下一次 `Process`。
直接使用 Frontend 时，文件写入和 ROS 发布由调用者处理，跳过发布也会正常消费视觉帧。
LivoSystem 组合 CloudColorizer 和 LivoExporter；`ProcessNext` 处理一次同步结果，
`ProcessAvailable` 处理到等待输入，回调可为空。调用方串行执行输入、处理和传播，
正常退出调用 `Finish`。回调视图在下一次处理前消费。
ROS 层仅对成功更新打印一次整体 LIO/VIO 耗时；LivoSystem测量耗时，估计仍使用传感器时间。

## 源码组织与 include 迁移

Noetic 与 Humble 的可执行文件都从 `src/app/livo_main.cc` 编译。公共
`cmake/node.cmake` 创建一次 `fastlivo_mapping`；目标私有宏
`LIVO_ATLAS_ROS_VERSION=1|2` 只选择入口里的初始化、适配器、执行器与退出处理。
mode0 提供普通库及离线数据集检查工具。`cmake/ros1.cmake` / `cmake/ros2.cmake` 保留各自接入库和包导出。

```text
src/
├── app/livo_main.cc                  # 唯一 ROS 节点入口
├── core/                            # LivoSystem、配置读取与校验
├── frontend/
│   ├── frontend.cc                   # 估计流程与状态所有权
│   ├── imu/{imu_processor,imu_propagation}.cc
│   └── vision/{vio,frame,image_options}.cc
├── mapping/                         # 体素/视觉地图、着色、关键帧快照与选择
├── sensor/{lidar_preprocessor,synchronizer,extrinsics}.cc
├── camera/                          # 相机模型
├── image/                           # 图像基础操作
├── io/                              # 原有导出与 keyframe_store 数据集
├── ros1/                            # 四对接入源/头
└── ros2/                            # 同样四类职责
```

头文件从 `include/livo_atlas` 进入，按同样职责组织；共享状态、数学和终端颜色在
`include/livo_atlas/common/`。`Pose6D` 位于 `frontend/imu/pose6d.h`，点协方差在
`mapping/point_with_covariance.h`。没有 `runtime/`、`internal/` 或 `output/` 兼容目录。

两个 ROS 目录各含 `livo_node`、`node_options`、`measurement_conversions`、
`livo_publisher` 四对源/头，普通配置由 `core/livo_config` 共用。
visual_point 归 mapping，Frame/Feature 归视觉前端；目录不强制对应单独物理库。

旧 include、类型、目标及配置成员迁移见[命名清单](naming-audit.md)和
[本轮迁移表](p4-naming-validation.md)。六个指定类型名保留，类型归 `livo_atlas`。
外部工程应更新实际调用方，不再使用转发头或废弃别名。

调用方先 `ReadNodeOptions(node)`，创建 `LivoSystem(options.livo.system)`，
再将引用注入 `LivoNode(node, options, system)`。system 先构造、后析构。
两端 main 都依次完成参数/话题校验、创建系统、接入、执行及显式 `Finish()`。

`CloudColorizer::Process(result)` 返回 `ColoredCloud::ConstPtr`；空指针表示没有有效云，
非空空云保留原发布节奏。`LivoPublisher` 负责 ROS 消息与 TF；`LivoExporter` 负责文件，
COLMAP 相机和去畸变图像也由导出器管理。跳过发布不影响同步消费与文件导出。
关键帧从普通结果与 I/O 边界接入，后续 GTSAM/3DGS 可消费独立数据集和位姿版本。
