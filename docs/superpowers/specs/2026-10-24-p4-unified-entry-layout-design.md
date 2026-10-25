# P4 修正：统一入口与职责目录

## 用户修正与目标

用户明确指出：期望 OpenVINS 风格的同一个文件入口同时编译 ROS1/ROS2，而现有
`src/main.cpp`、`src/ros2/main.cc` 两个入口不符合意图；frame、IMU 等旧文件也应继续分类。
本阶段在 P4 `5884f15728e3a1dfd394beaa1f3a595db87d35af` 上修正这两点。
继续已授权的当前会话实施、本机验证、代理决定目录；先文档后代码，Google C++17，
只改 livo-atlas，rpg_vikit 和 P4 验证证据只读。

OpenVINS 的 run_subscribe_msckf.cpp 在同一 main 中按 ROS_AVAILABLE 选择 ROS 初始化、
ROS1/ROS2Visualizer 及 executor；适配器仍分开，VioManager 共用。
本工程采用同样的入口组织原则。原 C++ 核心继续被两个适配器复用。

## 选择与边界

选择一个真正的 `src/app/mapping_main.cc`，ROS 版本宏仅在这个入口选择初始化、
适配器和执行器；由公共 `cmake/node.cmake` 创建一次 fastlivo_mapping 目标。
ROS1/ROS2 构建只提供自己的适配库；mode0 不创建 ROS 节点。

不采用两份 main，也不把两个 main 原样隐藏成 RunRos1/RunRos2 再转发。
不为统一入口引入虚接口、运行时 ROS 选择或插件工厂。编译时选择版本即可。
保留各自的订阅/参数/发布实现，普通 Frontend/Synchronizer/Preprocess/Output/Writer 共用。
ROS2 mapping_session 仍是它的接收/时间修正/调度实现；不借本次重写 ROS1 流程。

入口保持现有进程语义：ROS1 laserMapping、ImageTransport 与 mapper.run；
ROS2 laserMapping、SingleThreadedExecutor、明确 Finish 与 shutdown；错误退出码1。
现有普通核心没有 ROS 版本条件。节点名、可执行文件名、参数、话题和输出格式保持。

## 文件归属与命名

```text
include/livo_atlas/
  frontend/imu/imu_processor.h
  frontend/vision/{vio,frame,feature}.h
  mapping/{voxel_map,visual_point}.h
  sensor/preprocess.h
  ros1/mapper.h
  internal/legacy/{common,types,so3_math,color}.h
src/
  app/mapping_main.cc
  frontend/imu/imu_processor.cc
  frontend/vision/{vio,frame}.cc
  mapping/{voxel_map,visual_point}.cc
  sensor/preprocess.cc
  ros1/mapper.cc
  ros2/                    # 已有适配器
```

现有 camera、frontend facade、sensor 数据与同步、mapping options、output、io 等目录保留。
visual_point 是有世界坐标/观测关联的视觉地图点，归 mapping；frame/feature 是视觉处理数据，
归 frontend/vision。IMU propagation 普通函数留 frontend，IMU 处理器归 frontend/imu。

原 common_lib/utils 的全局别名和宏集中到 internal/legacy，明确它们是算法内部过渡接口；
当前保留其内容以避免重写估计数学。新代码继续使用普通 core/sensor API。
ROS1 已生成消息头仍保留 livox_ros_driver 的标准 include 命名空间，保持包类型/MD5。

所有主动维护的源码、测试和构建使用规范新路径；不在根目录遗留转发头。
旧头路径的源码使用方需要更新 include，迁移表写入文档；类名和库目标名不在本阶段重命名。
安装导出、独立消费者和独立测试 CMake 必须同时更新。历史阶段文档与证据保留原记录。

## 验证

1. 直接检查三个 mode 的真实编译输入：mode1/2 的 fastlivo_mapping 都只有同一个入口 TU，
   节点私有宏分别1/2；普通库不带该宏或 ROS includes/libs；mode0 没有节点目标。
2. 目录迁移前后逐文件比较：算法 CPP 除 include 路径外内容相同，头文件仅路径/guard/
   自包含 include 调整；不借重排重写算法或整文件格式化。所有新普通头独立编译。
3. Noetic158、真实ROS12、启动50、Python13；Humble176、实际启动16、Python工具5，
   所有检查用新阶段目录，不改旧工作区/原始产物。
4. mode0 shared/static/无OpenMP 的安装消费，Humble ament 安装消费，确认新头路径可用，
   安装里不残留旧根头；两个 ROS 的实际 colcon/catkin 节点仍可启动、处理和干净退出。
5. 官方 bag 3s产品和完整136s窗口，对各自 P4 冻结同环境参考严格比较四组数组、发布摘要、
   57个产品；ROS2输入字节/节点回调计数及14个程序/库摘要绑定不弱化。

构建 nice -n10/-j1，和回放串行，无 RViz。现有 Humble 容器沿用，新的候选只写
`/tmp/livo_atlas_layout`，读取已固定的 P4 依赖与 bag，不改挂载的其他项目。
新证据 `validation/P4-layout-20261024`。一次整阶段独立评审，重要问题一次测试修复。
完成后文档、完整历史 bundle 与空仓库恢复；不合并/推送。

## 参考

- [OpenVINS 统一入口](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/run_subscribe_msckf.cpp)
- [OpenVINS 分版本构建选择](https://github.com/rpng/open_vins/blob/master/ov_msckf/CMakeLists.txt)
