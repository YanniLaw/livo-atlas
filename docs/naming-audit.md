# 命名与职责整理清单

本清单记录已实施的命名与职责迁移；表格左列为整理前名称，右列为当前入口。
仅涉及livo-atlas，采用Google C++17命名并保留现有算法行为；验证见[本轮记录](p4-naming-validation.md)。

## 已实施的整理方向

| 旧名称或位置 | 当前名称或位置 | 职责 |
|---|---|---|
| `runtime/` | `core/` | 普通C++系统入口、调度和组合 |
| 原基础定义 `core/` | `common/` | 共享状态、数学类型与数学运算 |
| `MappingSession` | `LivoSystem` | 组织输入、同步、前端和结果消费 |
| `MappingAdapter` | `ros1::LivoNode` / `ros2::LivoNode` | 各版本ROS节点接口 |
| `ResultPublisher` | `LivoPublisher` | 发布里程计、轨迹、点云、图像、标记和TF |
| `ImuMeasurement` | `Imu` | IMU采样值；缩写按项目Google命名约定处理 |
| `ImageMeasurement` | `TimedImage` | 时间戳和图像 |
| `PointCloudMeasurement` | `TimedPointCloud` | 时间戳和规范化点云 |
| `OutputProcessor` 中的点云处理 | `mapping/CloudColorizer` | 点云累积、投影和着色 |
| `OutputProcessor` 中的COLMAP图像准备 | `io/` | 导出需要的去畸变图像 |
| `internal/legacy/` | 按职责合并后移除 | 消化旧别名与包装，不把兼容大头文件直接搬入common |

构建目标同步迁移：`livo_atlas::core` 现在是承载完整LivoSystem的物理库，
显式依赖前端、同步、预处理、着色与I/O；common只承载共享定义，没有单独物理库。

## 用户明确保留的名称

以下六个类型保留原有名称、拼写和大小写，此约束优先于通用命名建议：

- `VIOManager`
- `Frame`
- `Feature`
- `Pose6D`
- `SubSparseMap`
- `VOXEL_LOCATION`

整理文件归属、命名空间和兼容头时仍保留这些类型名。职责通过目录和注释说明，
例如SubSparseMap仍表示当前视觉跟踪使用的数据，不因此将其定义为后端持久子地图。

## 其他已实施的组件和数据类型

| 旧名称 | 当前名称 | 根据实际职责判断的原因 |
|---|---|---|
| `Preprocess` | `LidarPreprocessor` | 处理雷达点云过滤、按线组织和特征分类 |
| `ImuProcess` | `ImuProcessor` | IMU初始化、传播和点云去畸变；与当前文件名一致 |
| `pointWithVar` | `PointWithCovariance` | 携带多个坐标系中的点和协方差，不是单个标量方差 |
| `Warp` | `PatchWarp` | 图像块仿射变换及搜索层级 |
| `VOXEL_POINTS` | `VisualVoxel` | 体素内的视觉地图点集合及其现有所有权 |
| `VoxelOctoTree` | `VoxelOctreeNode` | 实例表示八叉树节点，包含八个子节点和平面 |
| `orgtype` | `LidarPointAttributes` | 雷达点的距离、邻域几何和特征分类 |
| `PointToPlane` | `PointPlaneMatch` | 点与平面对应关系及残差计算所需数据 |
| `SynchronizationResult` | `SyncStatus` | 返回等待、就绪、丢图、无效点云等状态，没有结果数据载荷 |

内部辅助类型优先留在实际使用的组件中；不为每个小结构体新建文件。
`DS_POINT` 和只有声明、没有使用的旧智能指针别名已移除；实际使用的FramePtr保留。

`State` 已命名为 `FilterState`，表示包含逆曝光、偏置和重力的19维前端滤波状态；
字段、状态顺序和运算语义保留。

## 配置、文件和成员保持对应

| 旧名称 | 当前名称或位置 |
|---|---|
| `SessionOptions` / `runtime/options.h` | `SystemOptions` / `core/system_options.h` |
| `MappingOptions` / `runtime/mapping_options.*` | `LivoConfig` / `core/livo_config.*`；完整应用配置，包含系统配置与接入设置 |
| `NodeParameters` / `ros*/parameters.*` | `NodeOptions` / `ros*/node_options.*` |
| `PreprocessOptions` | `LidarPreprocessorOptions` |
| `ResultWriter` / `io/result_writer.*` | `LivoExporter` / `io/livo_exporter.*`；轨迹、记录、点云、图像与COLMAP文件导出 |
| `OutputOptions` | `ExportOptions` |
| `OutputProcessorOptions` | `CloudColorizerOptions`，仅含相机、模式、间隔与最小着色距离 |
| `PreparedOutput` | 已移除；直接传递彩色云指针，导出器内部准备COLMAP图像 |
| `app/mapping_main.cc` | `app/livo_main.cc` |

文件归属也同步核对：`frontend/imu_propagation.*` 归入 `frontend/imu/`，
`frontend/image_options.*` 的视觉配置校验归入 `frontend/vision/`。
ROS1根测试目录中的节点/转换测试移入 `tests/ros1/`，与ROS2的对应测试保持一致。

新增或实际重命名的类使用对应snake_case头/源；明确保留的类型不会因这一通用规则机械改动文件名。
配置读取函数、成员名、测试和安装消费示例随实际迁移一同更新。
`LivoConfig` 的命名表示完整应用配置；`SystemOptions` 等Options表示具体组件的构造选项。
ROS参数键、话题、消息字段、坐标系及现有运行入口属于外部运行接口，单独于C++名称迁移管理。

## 需要明确职责后再处理的内容

`VoxelMapManager` 同时持有体素地图、建立点面对应、执行LiDAR状态估计和维护滑动地图。
直接改成VoxelMap会把估计职责藏起来。先保留完整职责说明；需要拆分时再明确
LiDAR更新组件与局部体素地图的边界，避免本轮命名整理演变成另一轮估计流程重构。

`SensorSynchronizer`、`FrontendResult`、`RigidTransform`、`Camera`、`PinholeCamera`、
`EquidistantCamera` 和 `VisualPoint` 目前结合其模块位置已能表达含义，不需要为统一前缀而改名。

`MeasurementGroup` 已改为 `UpdateMeasurements`，强调它对应一次前端更新；
`SynchronizedMeasurements` 已说明同步含义，不必单纯为了缩短而换成Packet等更模糊的名称。
其 `lio_vio_flg`、`pcl_proc_cur`、`pcl_proc_next` 已改为 `update_kind`、`current_cloud`、`next_cloud`，
保留时间切分、游标更新和消费顺序。

`Frame` 的旧坐标缩写已调整：`w2c` 返回的是像素，`w2f` 才返回相机三维坐标。
保留Frame类名；前者已改为 `WorldToPixel`，后者改为 `WorldToCamera`；
`c2f` 对应 `PixelToBearing`，`f2w` 对应 `CameraToWorld`，`pos` 对应 `CameraPositionInWorld`。
保留矩阵方向与运算正文，避免后续后端、COLMAP或3DGS接入时混用像素与三维坐标。

## 命名之外已核对的约定

- 原全局算法类型已迁入 `livo_atlas`，ROS类型继续使用ros1/ros2子命名空间。
- 公开头已移除全局 `using namespace`，以实际依赖的规范头和限定名称替代兼容大头文件。
- 单位和坐标方向进入注释与接口约定：`curvature` 当前实际保存点内时间偏移（毫秒），
  属于PCL存储兼容字段；清理时明确访问含义，不能把它误当作几何曲率或顺带改变点布局。
- IMU进入系统前与前端处理时的时间修正阶段不同；明确入口契约，不借重命名改变两次量化、
  driver fix、曝光或时间偏移顺序。
- `RotMtoEuler`、`StatesGroup`、`V3D` 等旧包装与别名已删除，调用方使用同一份规范实现。
- C++ `patch_pyrimid_level` 已更正为 `patch_pyramid_level`，外部既有参数键保留。
- 更新CMake、公开include、安装导出、现有测试和当前文档；历史验证报告保留原始名称。
- 命名迁移不改变算法公式、常量、数组/状态顺序、对象所有权与有效期，也不引入新插件层。
