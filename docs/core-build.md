# 独立核心与 Noetic 构建

P3 把相机模型、图像基础操作、同步、预处理、前端和文件输出作为普通
C++17 库构建。只有 `cmake/ros1.cmake` 加载 ROS 节点、参数、消息转换和发布。
`rpg_vikit` 不再是 livo-atlas 的构建或运行依赖；工作区里原有的依赖源码可以保留。

## 依赖与编译选项

- CMake 3.16、C++17 编译器、Eigen3、PCL 的 common/filters/io。
- OpenCV 的 core/imgproc/calib3d/imgcodecs。
- 兼容 `sophus/se3.h` 的旧版 Sophus（原工程所用的非模板版本）。通过
  `Sophus_DIR` 或 `CMAKE_PREFIX_PATH` 指定实际安装配置；不硬编码库名。
- OpenMP 可选。`LIVO_ATLAS_USE_OPENMP=OFF` 同时关闭编译选项和并行宏。
- GTest 仅测试需要。`BUILD_TESTING=OFF` 可以只构建独立库。

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
`livo_atlas/frontend/frontend.h`；`livo_atlas::core` 聚合前端、同步与预处理，
`livo_atlas::camera`、`livo_atlas::output`、`livo_atlas::io` 可以分别链接。
`tests/installed_consumer` 是实际执行连续 LO/LIO/VIO 的独立消费示例。

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

ROS2 Humble 接入安排在 P4。当前传入 `LIVO_ATLAS_ROS_VERSION=2` 会明确报错，
尚不能宣称支持 ROS2。GTSAM 后端、持久关键帧和 3DGS 导出也属于后续阶段。

## 数值与接口约定

相机配置描述原始图像，新模型只缩放一次。为保持迁移可比较，保留旧库的
畸变开关和鱼眼光轴特殊分支；细节见
[参考样本说明](../tests/camera/data/README.md)。原始 native 库的 FMA 差异
单独记录，不用于放宽 bag 和导出比较。

`FrontendResult` 的状态为值快照，图像和点云视图有效期到下一次 `Process`。
文件写入和 ROS 发布由调用者处理，跳过发布也会正常消费视觉帧。
ROS 层仅对成功更新打印一次整体 LIO/VIO 耗时，核心不读取墙上时间。
