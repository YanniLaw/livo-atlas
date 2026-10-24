# P4：Noetic / Humble 同源接入

## 意图和范围

在 P3 `973a5cce533da34952bfa33574e2a28de4ead579` 上实现 ROS2 Humble 接入，
继续保留 Noetic 与 mode 0 独立核心。只有 livo-atlas 进入修改范围；rpg_vikit 不变。
用户已授权当前会话直接实施、本机验证与目录安排，继续先文档、再编码，不重复请求批准。
新代码用 Google C++17，避免为未来后端预造框架。GTSAM、关键帧存储和 3DGS 不在 P4。

成功标准：两套 ROS 调用同一份 Frontend/Synchronizer/Preprocess/Output/Writer 源码；
Humble 能实际接收官方 Retail_Street 数据、运行并发布结果；Noetic 回归不退化；
纯核心编译/安装不依赖 ROS。跨电脑操作和验证限制写入仓库文档。

## 方案取舍

采用既定架构的分版本薄接入：mode 0/1/2，ROS1 与 ROS2 各自转换消息、读参数和发布。
ROS2 内部用一个普通 C++ session 组合既有组件，供节点和确定性测试调用；它是接入
实现，不是第二套估计器，也不新增稳定核心门面。ROS1 保留当前入口，只复用抽出的
图像约束校验及高频 IMU 单步传播公式。

没有选择在算法文件内加入 ROS 版本宏：这会重新引入中间件耦合。也没有选择现在
重写整个 ROS1 Mapper 为通用节点框架：已有回归充分的接入不需要大范围变动。

以上属于代理的实施选择，依据用户已有架构方向与执行授权，不声称用户指定了细节。

## 环境与依赖

- 本机 Noetic：Ubuntu 20.04 / GCC9 / Eigen3.3 / PCL1.10 / OpenCV4.2。
- 现有 `ros2` 容器：Ubuntu22.04 / Humble / Eigen3.4 / PCL1.12 / OpenCV4.5，
  已有 colcon、cv_bridge、pcl_conversions、rosbag2_py、SQLite bag 插件、GTest。
- 容器已恢复运行；构建只写 `/tmp/livo_atlas_p4`，不写它挂载的其他工程。
- 兼容旧 API 的 Sophus 从本机已用源码复制到临时依赖目录，在 Humble 原生编译；
  不替换数学实现，不改宿主机的第三方源码。库本身使用自己的配置入口。
- ROS2 adapter 依赖官方 `livox_ros_driver2` 消息类型。验证环境缺少该包时，用
  官方精确 `.msg` 定义在包外构建消息夹具；不宣称验证了物理驱动或 SDK。
  实机用户安装官方驱动。普通 core 不依赖这些消息或硬件 SDK。
- bag 转换是验证/数据迁移工具，使用隔离 Python 环境中的 rosbags，固定实际版本，
  不成为节点的运行依赖。类型映射为 ROS1 Livox → 官方 ROS2 Livox，保留传感器值和时间。

构建 `nice -n 10`、`-j1`，一次只运行一个构建或 bag 场景，无 RViz。Humble DDS 使用
独立 ROS_DOMAIN_ID、localhost 范围，避免与挂载的其他项目通信。P3 原始证据只读。

## 构建及 manifest

`LIVO_ATLAS_ROS_VERSION=0|1|2`：默认从已 source 环境的 ROS_VERSION 推断 1/2，
没有该环境时保持默认 1；显式缓存参数优先。mode 2 限定 Humble，错误环境明确失败。
package.xml 升为 format 3，按 ROS_VERSION 选择 catkin/ament 和接入依赖。
mode 0 不读取或需要 ROS 环境。

新增 `cmake/ros2.cmake`。mode 2 通过 ament 导出同一个 namespaced core target 图，
以及依赖和旧 Sophus shim；不同时安装一个会遮蔽 ament 配置的 SDK Config。
mode 0 维持 lib/cmake SDK，mode 1 维持完整 catkin 库列表。ROS2 节点仍名为
`laserMapping`、可执行文件 `fastlivo_mapping`，安装到 lib/livo_atlas。

普通公共头安装排除 ros1/ros2；各 ROS 分支单独安装自己的头。测试清单共用，
接入专用测试只在对应版本编译。

## 共享的两个普通函数

`ValidateImageOptions(const CameraConfig&, const VisualOptions&) -> std::string`：
移出 ROS1 的相机维度/网格/patch/pyramid 整数和边界校验，首先验证相机再转换整数。
保留原来的约束与 ROS1 错误文本，ROS2 入口可将参数路径转为点号显示。空串成功。
COLMAP 限定 Pinhole 仍由 ValidateOutputOptions 负责。

`PropagateImu(State*, double dt, Eigen::Vector3d acc, Eigen::Vector3d gyro,
double mean_acc_norm)`：从 Mapper 移出原高频传播的十余行公式，保留运算顺序和单位。
调用方保证非空状态、正的初始化均值和有效 dt。ROS1/ROS2 仍各自管理接收队列和游标。
不改 Frontend 的 IMU/LIO/VIO 更新和地图代码。

## ROS2 数据和时间边界

`include/livo_atlas/ros2/measurement_conversions.h` 接受 Humble 的 Imu/Image/
PointCloud2 和 livox_ros_driver2::msg::CustomMsg，返回既有普通测量或 LidarScan。
错误不修改输出：保留 ROS1 的完整缓冲区/步长/字段/可选 ring/time/颜色别名检查；
Livox 检查 point_num，图像深拷贝并转 BGR8。已支持的标准 LiDAR 布局保持原枚举。

时间按 `double(sec) + double(nanosec)*1e-9` 计算，与 ROS1 保持表达式；不以
`double(total_nanoseconds)*1e-9` 替代。返回 ROS stamp 时分解秒并四舍五入纳秒，
处理进位，拒绝非有限、非法纳秒和 int32 秒越界。IMU 偏移及旧驱动整数秒修正保留
ROS1 的纳秒归一化步骤；图像/LiDAR 保持既有偏移契约。估计器不读取 ROS clock。

## ROS2 配置

配置仍取现有 config/*.yaml 和 camera_*.yaml，launch 中读入、展平成点号名称，
标定数值数组统一为 double。避免维护两份 Avia 标定。允许自定义 config/camera 路径。
节点的参数读取器独立于 ROS1，校验数值类型/范围、两组外参、相机、网格和输出选项，
在 session、文件写入和传感器订阅前完成校验。

自定义参数为启动期只读，避免参数显示改变而实际算法配置未改变。内建 use_sim_time
由 rclcpp 管理。运行目录使用现有 ROS_HOME/HOME 规则，不写源树。

输入 QoS 用 sensor-data 的 best_effort/volatile，队列深度可配置：IMU2000、云100、
图像20；reliability 可选 reliable 用于 bag。拒绝非法 profile/零深度。输出为可靠
队列，沿用活动话题和 frame id。记录验证实际使用的 RMW 和 QoS。

## ROS2 session / 节点

私有 `src/ros2/mapping_session.{h,cc}` 不包含 ROS：拥有 Preprocess、SensorBuffers、
SensorSynchronizer、Frontend、OutputProcessor、ResultWriter 和高频传播状态。
配置结构是普通类型，私有头不安装为新 SDK API。

接口：`AddLidar(const LidarScan&)`、`AddImu(ImuMeasurement)`、
`AddImage(ImageMeasurement)`、`ProcessAvailable(ResultCallback)`、
`PropagateLatest() -> optional<PropagationResult>`、`Finish()`。
ResultCallback 同步接收 `const FrontendResult&`、`const PreparedOutput&` 和本次处理耗时；
必须在下次 Process 前消费视图。session 只组合组件，不实现新的滤波/地图算法。

沿用有效输入的接收规则：首个 LiDAR 前忽略 IMU/图像；LiDAR 验证/预处理成功后才
进行回退清队列；IMU 反向时间拒绝；图像重复/过密/反向时间拒绝；Hilti 计数为实例成员。
错误消息由转换层拒绝；无效图像不进入 session，不承诺复刻无效包对 Hilti 计数的影响。
每次处理同步结果直到等待，丢图/坏云状态继续处理，EOF 不强行补最后一个 VIO。

`MapperNode` 仅负责参数、订阅、单线程 executor 定时调度、ROS 发布和退出。
回调转换/入队，1ms wall timer 驱动 session；传播 timer 沿用 ROS1 周期。
wall timer 只调度，估计时间始终来自传感器。SIGINT 后明确 Finish，再释放实例。
不支持 MultiThreadedExecutor 并发访问该 session。

发布 /cloud_registered、/cloud_effected、/aft_mapped_to_init、/path、
/mavros/vision_pose/pose、/LIVO2/imu_propagate、/planes、/rgb_img 和 TF。
保留 LIVO LIO 的空云、VIO RGB schema、跟踪图 BGR8、LIO 的 Euler/TF 四元数路径、
传播的 Eigen 四元数和原 frame id。未使用的 ROS1 占位广告不复制为功能。

## 验证

1. Noetic 现有150项、ROS12、启动50、Python13及核心共享/静态/安装消费继续通过。
   新共享函数以真实数值和坏配置 RED→GREEN；Noetic bag 对 P3 冻结参考仍严格比较。
2. Humble 原生构建完整 core 与节点，共用普通测试；新转换测试覆盖有效/畸形载荷、
   ROI、数值时间边界、输出保持不变。配置覆盖数值数组、只读、坏配置不改文件。
3. 同样的普通测量序列在 session 和 ROS2 转换后入口中产生一致的更新种类、时间、
   状态与点云；覆盖 LO/LIO/连续 VIO、跳过发布、两实例和传播游标。
   host/Humble 的不同数学库结果先严格比较，若有差异记录并定位，不假设跨 ABI 位等价。
4. 真正 DDS 测试订阅发布、TF、时钟和消息 schema。转换 Retail_Street.bag 为 rosbag2，
   比较原/新传感器时间与字段摘要，确认转换只改变协议/类型名。
   Humble 实际执行短3秒导出、30秒和136秒窗口，核验输入计数、位姿数、有限性、
   正常退出与产品。DDS 跨话题到达次序和依赖版本差异须与转换/核心对照区分；
   数值不一致先定位，不直接放宽阈值。默认线程短场景作为健康检查。
5. 构建/回放串行；一次 fresh-context 整阶段评审，Critical/Important 一次测试修复，
   Minor 留档；文档、完整历史 bundle、空仓库恢复与执行记录交付。

验证不能替代物理 Livox 驱动、其他 CPU/操作系统和所有传感器的实测。

## 参考

- [REP149 的同源双 ROS 条件依赖](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0149.rst)
- [ROS2 QoS 设计](https://design.ros2.org/articles/qos_configurability.html)
- [官方 Livox CustomMsg](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomMsg.msg)、[CustomPoint](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomPoint.msg)
