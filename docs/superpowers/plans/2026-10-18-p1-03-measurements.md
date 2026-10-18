# P1-03 内部测量与 ROS1 转换执行计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 IMU 处理和点云预处理接收不含 ROS 消息的测量，同时保持 Noetic 前端的输入数值、同步与估计结果。

**Architecture:** ROS1 负责消息解码、时间修正和图像编码转换，输出值类型、拥有像素的 OpenCV 图像和 PCL 点云。保留各雷达的原始字段类型，用 C++17 variant 表达实际存在的七种输入；预处理只消费这些类型。现有同步和发布代码继续留在 LIVMapper，P2 再迁移职责。

**Tech Stack:** C++17、Eigen、OpenCV 4、PCL 1.10、ROS1 Noetic、现有 Livox 消息头、GTest、Python 3。

**Spec:** `docs/superpowers/specs/2026-10-11-refactor-architecture-design.md`，特别是第 5、6、7 节。

## Global Constraints

- 本次只修改 `livo-atlas` 包；`rpg_vikit` 保持原样。
- ROS1 验证平台为本机 Noetic；ROS2 目标为 Humble。当前阶段不宣称支持 Humble 或完整前端无 ROS 构建。
- 本轮保留工程现有 C++17 工具链；新增接口遵循 `docs/cpp-style.md` 的 Google C++ 风格。
- 第一轮接口继续使用 `double` 秒，保留原有比较与截断方式。使用传感器 header 时间，不能换成 bag 记录时间。
- `curvature` 保存点内相对时间，单位为毫秒；Livox `offset_time` 的单位为纳秒。保留原有除法精度、筛选、补时和排序顺序。
- IMU 测量保存时间、角速度和加速度数值，不保存 ROS 消息指针。原有 IMU 初始化归一化由算法执行。
- 图像在进入队列前获得可跨回调存活的所有权，保留原 BGR8 输入及 VIO 的灰度转换。
- PCL 使用其实际 `Ptr` 类型。回调把独立点云交给队列；同步和估计依次修改，发布不获得该输入的异步可写别名。
- 保留先 LIO 后 VIO、同一个 19 维状态、IMU 边界样本及原时间窗口；不迁移同步状态机，不改相机模型或滤波公式。
- 构建使用 `nice -n 10`、`-j1`，不与 bag 回放并行。产物保存在包外 `validation/P1-03-noetic-20261018/`。
- 数值参考为阶段开始的 `10e9222b122c16a4ecb36a2b19e30d113ed78d25`；参考与候选均含前阶段已经验证的修复。

## Review Focus

1. IMU 偏移经过 ROS Time 的纳秒舍入后再用于传播；适配器不能归一化加速度，或把高频输出的原时间戳重新由 double 构造。Task 1 的转换测试及 Task 2 的接线和回放覆盖。
2. RGB、灰度和带行填充的图像必须得到内容正确的 BGR8；原消息释放或修改后，队列数据仍有效。失败转换不能部分覆盖输出。Task 1 的真实 cv_bridge 测试覆盖。
3. Livox 点数与向量长度不符、PointCloud2 必需字段缺失/已提供字段类型不符/数据截断必须在解码前拒绝；可选字段缺失保留原 PCL 零默认值与补时回退。不支持的大端数据明确失败。Task 1 的畸形消息及审阅补充测试覆盖。
4. 空点云及有行填充的组织点云不能越界；不同雷达的时间字段、反射率与排序语义不能合并成一种假定。Task 1 的七类型输入和预处理测试覆盖。
5. 回调退出后图像和点云仍由队列拥有，下一次预处理不得覆盖前一帧；IMU 初始化和跨帧保留样本改为值后，更新序列和状态必须不变。Task 1 的所有权测试与 Task 2 的配对完整回放覆盖。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `include/livo_atlas/sensor/measurements.h` | `ImuMeasurement`、`ImageMeasurement`、`PointCloudMeasurement`；单位和修改者约定。 |
| `include/livo_atlas/sensor/point_types.h` | 内部输出 PCL 别名和五种带时间/线号的雷达点布局与字段注册。 |
| `include/livo_atlas/sensor/lidar_scan.h` | Livox 普通点和值容器，以及七种解码结果的 variant 和传感器时间。 |
| `include/livo_atlas/ros1/measurement_conversions.h`, `src/ros1/measurement_conversions.cc` | IMU 字段复制、BGR8 独立所有权、ROS 点云布局校验与解码。 |
| `include/preprocess.h`, `src/preprocess.cpp` | 新 `Process` 入口与普通类型 handler，原筛选/特征/时间公式保持原样；移除未调用的 ROS 发布成员/函数。 |
| `include/utils/types.h`, `include/common_lib.h` | 旧点云名称转发到新别名；MeasureGroup 的 IMU 改为值；移除公共 ROS IMU/TF 包含。 |
| `include/IMU_Processing.h`, `src/IMU_Processing.cpp` | IMU 值输入及跨帧值样本；日志的 ROS 包含暂时局限在实现文件。 |
| `include/LIVMapper.h`, `src/LIVMapper.cpp`, `include/voxel_map.h` | 回调接入、IMU/高频队列值类型、显式 ROS 包含；同步顺序不变。 |
| `tests/measurement_conversions_test.cc`, `tests/sensor/preprocess_test.cc`, `tests/sensor/CMakeLists.txt` | 实际消息转换、预处理、跨调用所有权和无 ROS 预处理构建验证。 |
| `CMakeLists.txt`, `docs/p1-03-validation.md`, `docs/README.md` | 构建接线、精确命令、阶段结果与后续边界。 |

### Task 1: 内部类型、ROS1 解码与普通点云预处理

**Files:** 上表前三项类型头、ROS1 转换、preprocess、utils/types、两个 CMake 入口和两组新测试。

**Interfaces:**
- Consumes: Eigen `Vector3d`、OpenCV `cv::Mat`、PCL `PointXYZINormal`、原七种输入字段布局。
- Produces: `ImuMeasurement { double timestamp; Eigen::Vector3d angular_velocity, linear_acceleration; }`，默认时间和向量为零。
- Produces: `ImageMeasurement { double timestamp; cv::Mat bgr; }`；`PointCloudMeasurement { double timestamp; PointCloud::Ptr cloud; }`，空默认值；点云别名为 `PointCloud = pcl::PointCloud<pcl::PointXYZINormal>`。
- Produces: `LivoxPoint { float x,y,z; uint32_t offset_time; uint8_t reflectivity,tag,line; }`，全部初始化；`LidarScan { double timestamp; variant<vector<LivoxPoint>, pcl::PointCloud<VelodynePoint>, pcl::PointCloud<OusterPoint>, pcl::PointCloud<pcl::PointXYZRGB>, pcl::PointCloud<Xt32Point>, pcl::PointCloud<Pandar128Point>, pcl::PointCloud<RobosensePoint>> points; }`。
- Produces: `ImuMeasurement ros1::ConvertImu(const sensor_msgs::Imu&)`，直接复制已经修正的 header 时间和未缩放的数值。
- Produces: `std::string ros1::ConvertImage(const sensor_msgs::Image&, double time_offset, ImageMeasurement*)`；`std::string ros1::ConvertLivox(const livox_ros_driver::CustomMsg&, LidarScan*)`；`std::string ros1::ConvertPointCloud(const sensor_msgs::PointCloud2&, int lidar_type, double time_offset, LidarScan*)`。空字符串成功；失败不改变输出。
- Produces: `void Preprocess::Process(const livo_atlas::LidarScan&, PointCloudXYZI* output)`，按 variant 类型调用原 handler；空输入清空输出，输出独立于处理器下一帧的缓存。

- [x] **Step 1: 写行为测试和最小可编译空实现**

  IMU 使用 sec=17/nsec=123456789，三个非对称角速度及加速度值；额外测试原 ROS 偏移及驱动修正后的消息，逐值验证不归一化。图像用 RGB 像素 `{1,2,3}` 得 BGR `{3,2,1}`、mono8 的复制三通道、非紧密行及消息释放/修改；坏编码和截断数据不改变输出。
  Livox 使用不同 line/tag/offset_time，保留顺序与整数。拒绝 point_num 不一致。为六种 PointCloud2 布局各构造实际消息并解码，覆盖强度的 uint8/float、时间的 float/double/uint32 和 ring 的 uint8/uint16。含组织点云的行填充、字段乱序、空点云及畸形布局。
  预处理固定 blind_sqr=1、point_filter_num=1、feature_enabled=false，使用手算点偏移：Livox 0/4167ns -> 0/0.004167ms；超 1ms 跳变保留补时分支；Ouster t=2500000 -> 2.5ms 并按时间排序；Velodyne time=2500 -> 2.5ms；XT/Pandar/Robosense timestamp 差 0.0025s -> 2.5ms（比较浮点转换后的明确容差）；L515 保留 RGB 到 normal 与零时间。下一次处理不能改变之前输出。

- [x] **Step 2: 运行 RED**

  单线程构建并运行 `measurement_conversions_test` 和 `sensor_preprocess_test`；空实现应产生可读的数值/内容断言失败，不把编译或链接错误作为 RED。保存日志。

- [x] **Step 3: 实现普通类型与解码，迁移预处理入口**

  保留 PCL 点布局、原 cv_bridge 的 `bgr8` 目标编码。解码前校验字段、尺寸、步长、数据范围，拒绝不支持的大端，失败不提交输出。Livox 只保留原算法实际读取字段，不提前计算点内时间。所有原预处理计算与特征提取分支保留；空输入在索引前返回。新 Process 以 std::visit 直接派发，不增加接口基类。

- [x] **Step 4: 运行 GREEN 及无 ROS 构建**

  Run: 单线程构建和运行所有 catkin C++ 测试以及 `python3 -m unittest discover -s tests -p 'test_*.py' -v`；独立 `cmake -S tests/sensor -B <validation>/sensor-build`，构建后在 build 目录运行 `ctest --output-on-failure`。
  Expected: 全部通过；三个新 sensor 头独立包含、preprocess.cpp 在没有 ROS/vikit 包含或链接目录时可编译运行。数学/标定独立入口仍然通过。

- [x] **Step 5: 提交**

  Commit: `refactor: separate sensor measurements and ROS1 decoding`。本任务同时更新 mapper 的两条点云回调调用新入口，保持每个提交可构建；不为过渡保留 ROS 预处理重载。

### Task 2: 前端 IMU 值接线与完整数值回归

**Files:** 上表 common_lib、IMU、mapper、voxel_map 的显式依赖；`tests/imu_measurement_test.cc`；阶段验证文档及入口。

**Interfaces:**
- Consumes: Task 1 的四项转换、三种测量和 `Preprocess::Process`，签名不变。
- Produces: `MeasureGroup::imu`、mapper 两条 IMU 队列均为 `deque<livo_atlas::ImuMeasurement>`；`ImuProcess` 的跨帧 `last_imu` 为值。图像和点云经 Task 1 转换后再按原有队列和窗口入队。
- Produces: ROS1 mapper 单独保存高频输出用的 `ros::Time`；输入算法仍只使用普通 double 时间。

- [x] **Step 1: 固定接线验收证据**

  先为真实 IMU Process2 添加现状特征测试，在迁移前运行，验证初始加速度范数、静止状态、有限协方差与保留边界样本；迁移后把同一输入夹具改为值测量，保持断言。原算法已经存在，不用人为清空算法制造失败。Task 1 的新转换函数须有行为 RED→GREEN。同步完整行为以阶段开始提交的独立参考为对照。

- [x] **Step 2: 迁移消费者**

  去掉公共头中的 ROS IMU 和 TF 包含；需要 TF 的 voxel_map 显式包含。保持原 IMU 标量运算顺序，时间访问替换为 timestamp。ROS 时间偏移和驱动补偿仍在 ROS1 回调按原 fromSec/toSec 顺序执行。图像转换失败在回调报告并正常释放锁。

  实施补充：旧 `last_prop_end_time` 未初始化。用填充为 0x7f 的对齐存储构造真实 ImuProcess，输入 0 和 0.2 秒的两条 IMU（角速度 0、2 rad/s），旧代码输出 yaw=0，期望 0.2 rad。单独修复为构造/Reset 时清零，不改首次传播的其他公式。保留首次候选的源码、库与三次回放，最终候选重新执行短段、完整和默认并行回放；参考源码与运行库验证摘要后复用其已完成的两次回放。

- [x] **Step 3: 全部测试和真实启动检查**

  Run: 独立核心及 sensor 测试、全部 catkin 测试、全部 Python 测试；对候选执行现有 40 项 `tools/check_calibration_startup.py`。
  Expected: 全部通过；参数行为不变；输入算法接口中没有 ROS 消息。编译器参数保持原基准指令集，不引入 native/fast-math。

- [x] **Step 4: 配对短段、完整 bag 和默认并行回放**

  用 git archive 从参考提交建立 reference_ws，候选建立独立 candidate_ws，不复用其他路径的 CMake 缓存。两者依次构建、依次回放。统一 `OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE`，各运行 30 秒、0.5 倍；通过后各运行 136 秒覆盖完整 bag。候选另运行默认并行 45 秒。
  Expected: 短段 296、完整 1351、默认 446 个轨迹点；完整输入 27447 IMU、1355 点云、1355 图像；输入计数、传感器时间和 LIO/VIO 更新计数相同；轨迹及 mat_pre/mat_out 的导出数值零差异，所有运行正常退出。保存源、bag、可执行文件和全部本地运行库摘要，不做对齐/裁剪/放宽容差。

- [x] **Step 5: 文档与提交**

  在 `docs/p1-03-validation.md` 记录实际命令、结果、证据路径、原各型号时间语义以及仍然存在的 ROS/日志/同步依赖。更新 docs/README 和计划完成状态。
  Commit: `refactor: feed plain measurements to the frontend`。

## 阶段结束

遵循用户已经选择的当前会话直接实施，不再要求阶段确认。完成两项任务后执行一次新的独立只读审阅；重要问题以 RED→GREEN 和全套测试验证。保留执行记录和离线 bundle，不推送或合并。下一阶段为 P2 的同步器、调度与 ROS 发布/I/O 分离。

## 独立审阅后的补充

原“字段缺失全部拒绝”的计划条目与保留原有输入行为的规范冲突，修正如下：

- xyz、intensity 为必需字段；Ouster 的 t、Pandar128/Robosense 的 timestamp、深度相机的颜色字段也必需。
- 各布局的 ring、Velodyne 的 time、XT32 的 timestamp，以及 Ouster 未使用的 reflectivity/ambient/range 可以缺省为零。缺失 ring 维持原单线解释；多线驱动应提供真实 ring。XT32 非特征分支缺时间仍得到零点偏移，特征分支保留 yaw 回退。
- 可选字段只允许省略，已提供时仍严格检查类型、count 和 offset。颜色只接受 PCL 实际支持的 rgb/FLOAT32、rgb/UINT32、rgba/UINT32。
- 对 Velodyne/XT32 yaw 数组、Ouster 稀疏线及全盲区特征线增加边界保护，构造及 set 同步 blind_sqr。测试须先在审阅版本上失败，再验证修复。
- 两种真实 ROS1 雷达回调在转换和筛选成功后才处理时间回退，同时清理点云和时间队列。用独立 ROS master 验证坏帧保留队列及有效回退后的配对，不把全估计器重置混入本阶段。

首次传播的时间起点继续保留先前决定；P2 必须单独验证绝对时间锚点，并定义跨传感器和估计状态的完整重置。仅修复点云队列配对不能替代完整时钟回退处理。
