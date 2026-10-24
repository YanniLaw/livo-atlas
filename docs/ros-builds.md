# Noetic 与 Humble 使用说明

同一份源码通过 `LIVO_ATLAS_ROS_VERSION=0|1|2` 选择独立核心、Noetic 或 Humble。
默认从已 source 的 `ROS_VERSION` 推断；没有 ROS 环境时默认 1。显式 CMake 缓存参数优先。
两套 ROS 使用各自的构建目录和依赖；已验证的平台是 Ubuntu20.04/Noetic 与
Ubuntu22.04/Humble。跨系统需要重新编译。

算法库、相机、同步器和文件输出共用；`src/ros1` 与 `src/ros2` 处理各自的参数、消息与发布。
ROS2 的 `mapping_session` 是接入实现中的普通 C++ 组合对象，不是另一套估计器，也不是新公开 SDK。
核心安装与外部 CMake 消费见 [core-build.md](core-build.md)。

两个版本共用 `src/app/mapping_main.cc` 中唯一的 main，按 OpenVINS 的
[统一入口组织方式](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/run_subscribe_msckf.cpp)
选择 ROS 初始化、适配器和执行器。`cmake/node.cmake` 创建一次节点目标，
仅给它定义 `LIVO_ATLAS_ROS_VERSION=1|2`；普通算法库没有这个宏。Noetic 分支创建
LIVMapper，Humble 分支创建 MapperNode 并在单线程 executor 退出后 Finish/shutdown。
`src/ros1` / `src/ros2` 保存各自的接入实现；源码与头文件的职责目录和 include 迁移表见
[源码组织](core-build.md#源码组织与-include-迁移)。

## 依赖

核心需要 Eigen3、PCL、OpenCV 和兼容旧非模板 API 的 Sophus。继续沿用原工程兼容版
Sophus，通过 `Sophus_DIR` 指定其配置目录；不能直接用只提供模板 API 的新版替换。
Noetic/Humble 的实际依赖版本与验证范围见阶段报告。

Humble 接入需要 rclcpp、sensor_msgs、nav_msgs、geometry_msgs、visualization_msgs、
tf2_ros、cv_bridge、pcl_conversions，以及官方
[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。先在独立工作区构建并 source
该驱动的 Humble 安装环境。这里使用其消息接口；算法库不链接硬件 SDK。

本阶段在已有 Humble 容器里使用官方提交
`21445540f0d100dc86a7e6df312dd70bbdb4afdf` 的精确消息定义建立包外消息夹具，
运行了真实 DDS 回放。它验证消息与接入兼容性，物理驱动和硬件仍需实机联调。
`rpg_vikit` 不再是任何构建分支的依赖。

## Noetic

在 Noetic 工作区根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps livo_atlas \
  -DLIVO_ATLAS_ROS_VERSION=1 -DCMAKE_BUILD_TYPE=Release -j1
source devel/setup.bash
roslaunch livo_atlas mapping_avia.launch rviz:=false
```

订阅、参数和活动发布话题沿用现有 Noetic 接口。已有 bag 验证流程见
[validation.md](validation.md)。只构建 livo-atlas 不需要重建旁边保留的 vikit 源码。

## Humble

将这个包放入 Humble 工作区的 `src/livo-atlas`，在工作区根目录执行。
`/path/to/...` 替换为本机实际目录。

```bash
source /opt/ros/humble/setup.bash
source /path/to/livox_driver_ws/install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
nice -n 10 colcon build --packages-select livo_atlas --executor sequential \
  --cmake-args -DLIVO_ATLAS_ROS_VERSION=2 -DCMAKE_BUILD_TYPE=Release \
  -DSophus_DIR=/path/to/compatible-sophus-config -DBUILD_TESTING=ON
source install/setup.bash
```

运行共用 C++ 测试和 DDS 测试：

```bash
export ROS_DOMAIN_ID=74
export ROS_LOCALHOST_ONLY=1
OMP_THREAD_LIMIT=1 colcon test --packages-select livo_atlas --executor sequential \
  --ctest-args --output-on-failure
colcon test-result --verbose
```

选择本机空闲的 DDS domain。构建和回放串行进行；复现严格数值结果时设
`OMP_THREAD_LIMIT=1`，默认并行配置另外作为运行检查。

启动 Avia：

```bash
ros2 launch livo_atlas mapping_avia.launch.py \
  use_sim_time:=true reliability:=reliable output_directory:=/path/to/results/retail
```

launch 直接读取既有 `config/avia.yaml` 与 `config/camera_pinhole.yaml`，不维护另一份标定。
`config:=...`、`camera:=...` 可以替换路径。手写 ROS2 参数文件时使用点号名称，例如
`extrin_calib.extrinsic_R`、`vio.patch_size`、`output.directory`。标定数组支持整数或浮点数。
自定义参数在启动后只读；内建 `use_sim_time` 由 rclcpp 管理。

输入 QoS 默认为 best_effort/volatile；IMU、云、图像队列默认 2000/100/20。
回放 bag 时使用 launch 的 `reliability:=reliable`，大图像的可靠传输已实测核验。
直接启动可执行文件时可用 `--ros-args -p qos.reliability:=reliable`，以及
`qos.imu_depth`、`qos.lidar_depth`、`qos.image_depth`。节点使用单线程 executor。

活动发布话题为 `/cloud_registered`、`/cloud_effected`、`/aft_mapped_to_init`、`/path`、
`/mavros/vision_pose/pose`、`/LIVO2/imu_propagate`、`/planes`、`/rgb_img` 与 TF。
保持旧 frame id 与空云/RGB/BGR8 schema。估计时间来自传感器；常规发布头仍沿用 ROS clock，
后端接入应使用普通 `FrontendResult.timestamp` 的传感器时间契约。

## 将官方 ROS1 bag 转为 rosbag2

迁移工具需要 Python3.10+ 和 `rosbags==0.10.9`，不属于节点或核心的运行依赖。
可在独立 Python 环境安装；Humble 原生回放工具使用系统 ROS Python 环境。

```bash
python3 -m venv .venv-rosbags
.venv-rosbags/bin/pip install rosbags==0.10.9 numpy==1.26.4
.venv-rosbags/bin/python src/livo-atlas/tools/convert_ros1_bag.py \
  Retail_Street.bag Retail_Street.ros2 \
  --livox-msg-dir /path/to/livox_ros_driver2/msg
```

目前工具支持本工程官方 bag 的 Imu、Image、Livox CustomMsg 三类消息。
Livox 类型名变为 `livox_ros_driver2/msg/CustomMsg`，ROS1 Header.seq 移除；
传感器字段、整数时间戳和 bag 记录时间逐消息核验。目录中的 `conversion-summary.json`
记录输入/输出摘要、固定工具版本和官方消息定义摘要。目标目录必须不存在。

正常 ROS2 播放可在节点启动后运行 `ros2 bag play Retail_Street.ros2 --clock --rate 0.5`。
自动验收工具会自行启动节点，通过 Humble 原生序列化发布接口进行 DDS 回放：

```bash
python3 src/livo-atlas/tools/run_ros2_baseline.py \
  --bag Retail_Street.ros2 --output /path/to/new-validation-case \
  --executable install/livo_atlas/lib/livo_atlas/fastlivo_mapping \
  --duration 30 --rate 0.5
```

`--save-products` 保存 PCD、图像和 COLMAP；`--default-threads` 检查默认并行配置。
工具保留输入字节摘要、观察者与节点接收计数、轨迹、发布摘要、文件摘要及退出状态，
失败返回非零。输出目录必须不存在。默认输出目录仍遵循 ROS_HOME/HOME，显式指定目录
便于迁移和保存运行证据。

## 数值复现范围

相同环境和相同输入要求严格一致。Noetic 回归继续对照冻结的 P3 结果。
不同编译器与 Eigen/PCL/OpenCV 版本之间存在实测数值差异，不能用跨系统逐字节相同
作为隐含前提。P4 用同环境离线 session 与实际接入对照定位差异，同时保留跨环境量化结果；
没有修改估计公式或以新容差掩盖接入差异。

GTSAM、持久关键帧和 3DGS 仍属于后续阶段。
