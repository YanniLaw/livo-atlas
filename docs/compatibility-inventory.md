# P0 当前兼容性清单

静态来源：`launch/*.launch`、`LIVMapper.cpp`、`voxel_map.cpp`、`vio.cpp`、`IMU_Processing.cpp`、`rviz_cfg/livo_atlas.rviz`。动态LIVO基线由基线报告补充。下列“保留”指迁移初期默认外部接口，已知错误可以在独立任务中修正并记录。

## 入口与参数

- 保留包名 `livo_atlas`、可执行文件 `fastlivo_mapping` 和初期节点名 `laserMapping`。
- 保留四个launch入口及 `rviz` 参数；默认true，自动基线关闭显示。
- 算法参数位于全局命名空间，相机参数位于 `/laserMapping`；相机加载器目前依赖该命名。
- 现有默认值以 `readParameters()`、`loadVoxelConfig()` 的代码为准，实际运行值以每次 `params.yaml` 为准；YAML覆盖值不能写成C++默认值。

| 参数组 | 对外用途及迁移要求 |
|---|---|
| common | lid_topic、imu_topic、img_topic、img_en、lidar_en、ros_driver_bug_fix |
| extrin_calib | extrinsic_R/T 和 Rcl/Pcl；方向见坐标契约 |
| time_offset | img/imu/lidar偏移和exposure_time_init；加减方向及输入分支必须保留或明确变更 |
| preprocess | 雷达类型、线数、blind、滤波步长、降采样、特征与HILTI选项 |
| imu | 启用、初始化、噪声、重力与偏置估计 |
| vio | 迭代、图像噪声、patch、网格、法向、光度和曝光估计 |
| lio | 体素、层数、点数、噪声、平面阈值和迭代 |
| local_map | 滑动开关、范围和阈值 |
| publish | 稠密点、平面/有效点开关、发布扫描数、RGB盲区 |
| uav | IMU频率里程计、重力对齐 |
| evo、pcd_save、image_save | 输出开关、名称、坐标类型、保存间隔 |
| debug | plot_time、frame_cnt |

已发现：YAML中 `imu/b_acc_cov`、`imu/b_gyr_cov` 没有被当前 `readParameters()` 读取，组件初始化使用固定值0.0001；后续不能假定它们是有效可调参数。默认 `evo/seq_name` 与测试包名称不一致，验证通过独立覆盖名称解决，不改源YAML。

## 核心话题与TF

| 接口 | 类型/坐标 | 当前产生条件与时间 |
|---|---|---|
| /livox/lidar | livox_ros_driver/CustomMsg | AVIA默认输入，其他launch可不同 |
| /livox/imu | sensor_msgs/Imu | AVIA默认输入 |
| /left_camera/image | sensor_msgs/Image | AVIA默认输入 |
| /aft_mapped_to_init | nav_msgs/Odometry；camera_init / aft_mapped | LIO完成；墙钟时间 |
| /tf | camera_init → aft_mapped | 与odometry一起发布 |
| /path | nav_msgs/Path；camera_init | LIO追加；顶层header时间不随发布更新 |
| /cloud_registered | sensor_msgs/PointCloud2；camera_init | LIVO中LIO和VIO都会调用发布，LIO阶段可能是空消息；彩色点来自VIO阶段 |
| /rgb_img | sensor_msgs/Image；BGR8 | VIO显示图像；墙钟时间，未设置frame_id |
| /mavros/vision_pose/pose | geometry_msgs/PoseStamped；camera_init | LIO状态；墙钟时间 |
| /LIVO2/imu_propagate | nav_msgs/Odometry | 可选IMU传播输出，默认AVIA关闭 |
| /cloud_effected、/planes | 点云、MarkerArray | 对应开关控制 |

`/Laser_map` 等仅看到advertise不能证明实际发布，不能作为保存完整地图的接口承诺。末次 `/cloud_registered` 只是发布帧/累积窗口数据，不是全局重建地图。

RViz默认固定帧为 `camera_init`，主要读取 `/cloud_registered`、`/aft_mapped_to_init`、`/path`、`/rgb_img`。配置中还有历史调试话题，与当前发布接口并非全部一致；迁移时不能以“RViz里列出了话题”推断系统提供该功能。

## 文件与副作用

| 文件 | 格式与产生条件 |
|---|---|
| Log/mat_pre.txt、mat_out.txt | 初始化时截断；前后状态调试行，时间相对起点 |
| Log/imu.txt | IMU初始化完成后打开，可能覆盖已有内容 |
| Log/result/<seq_name>.txt | evo开启；t xyz qx qy qz qw；IMU位姿；LIO更新后写入 |
| Log/pcd/*.pcd | PCD保存开启；世界或IMU坐标取决于type |
| Log/pcd/lidar_poses.txt | 保存点云时输出IMU位姿 |
| Log/image/*.png、image_poses.txt | 图像保存开启；时间命名；pose仍为IMU位姿 |
| Log/Colmap/images、sparse/0 | COLMAP开关；相机world-to-camera与初始化点云 |

所有现有路径依赖编译期 ROOT_DIR，改变启动工作目录不能重定向它们。`scripts/colmap_output.sh` 会清理既有COLMAP输出，基线保持相关开关关闭。自动验证必须备份恢复可能被覆盖的日志，只清理本次唯一命名的输出。

## P1迁移顺序约束

先保持现有外部行为提取模块，再单独修复时间戳、统一融合结果出口、可配置输出目录。格式和名称变更需有明确旧新映射；不得同时改变数学更新造成差异难以定位。首批回归范围是实测LIVO，LIO/LO及其他数据集配置仍需补充验证。
