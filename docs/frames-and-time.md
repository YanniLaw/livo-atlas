# P0 坐标与时间契约核查

本文件描述当前实现；目标命名 `T_A_B` 表示 B→A。代码中 world 对应当前局部 `camera_init`，不是已全局优化的 map。

## 坐标与外参

| 当前量 | 实际语义 | 代码依据 |
|---|---|---|
| `_state.rot_end` / `pos_end` | `R_world_imu` / `t_world_imu` | `LIVMapper::transformLidar()`、`VIOManager::updateFrameState()` |
| `extrin_calib/extrinsic_R,T` | `T_imu_lidar` | `p_world=Rwi*(extR*p_lidar+extT)+Pwi` |
| `Rcl,Pcl` | `T_camera_lidar` | `setLidarToCameraExtrinsic()`、`initializeVIO()` |
| VIO `Rli,Pli` | `T_lidar_imu` | 对 `extrinsic_R,T` 使用转置逆变换 |
| VIO `Rci,Pci` | `T_camera_imu` | `Rci=Rcl*Rli; Pci=Rcl*Pli+Pcl` |
| `new_frame_->T_f_w_` | `T_camera_world` | `Rcw=Rci*Rwi^T; Pcw=-Rci*Rwi^T*Pwi+Pci` |
| `feats_undistort` | 传播终点时刻的 LiDAR 坐标点 | `ImuProcess::UndistortPcl()` 的反向补偿与外参逆变换 |
| PCD `type=1` | 由 LiDAR 点转换后的 IMU/body 系点 | `RGBpointBodyLidarToIMU()` |
| 轨迹 `Log/result/*.txt` | IMU 在 world 中的位姿 | `handleLIO()` 写 `_state` |
| `image_poses.txt` | 图像时刻的 IMU 位姿，不能直接作为相机位姿 | `publish_frame_world()` 写 `_state` |
| COLMAP `images.txt` | world→camera，四元数 qw qx qy qz | `dumpDataForColmap()` |

普通轨迹文件四元数为 qx qy qz qw；Eigen 构造参数与文件顺序不能混用。使用米、弧度；图像光学投影采用前方 z>0，x向右、y向下。地图帧名 `camera_init` 并不表示状态本体是 camera。

相机有效内参为 fx=646.78472、fy=646.65775、cx=313.456795、cy=261.399612，尺寸640×512。原始1280×1024内参与尺寸按scale=0.5缩放；bag已是目标尺寸，不再缩放。RGB输入转BGR；着色时按BGR分量写入RGB点。

数值核查由 `tools/check_contracts.py` 输出到 `validation/P0/contracts.json`，补充往返误差后的报告为 `contracts_v2.json`。用固定非平凡世界位姿检查变换组合和主光轴投影；只验证数学与代码语义，不是物理标定精度认证。复验命令（工作区根目录，另选新报告名）：`source /opt/ros/noetic/setup.bash`，随后执行 `/usr/bin/python3 src/livo-atlas/tools/check_contracts.py --output validation/P0/contracts_recheck.json`。

## 时间链路

1. ROS bag 记录时间用于回放调度，和 header 所在时间轴不同。
2. Livox 回调直接使用 `msg.header.stamp`；`timebase` 未参与现有处理。普通 PointCloud2 回调才加 `lidar_time_offset`，两条输入路径行为不相同。
3. IMU 回调使用 `header - imu_time_offset`；默认0。`ros_driver_bug_fix=false` 时不执行按整数秒修正的兼容逻辑。第一帧 LiDAR 前的 IMU 不入队。
4. 图像回调使用 `header + img_time_offset`，AVIA为+0.1秒。同步器再加 `exposure_time_init`，默认0。两者用途不同，不是把同一偏移加两遍。
5. Livox `offset_time` 纳秒转 `curvature` 毫秒；预处理还可能修正相邻异常偏移。不能直接将原始offset当成最终逐点时刻。
6. LIVO同步器按校正图像时刻切割当前/下一批点，点相对时间重新基于本次传播起点；IMU截取至该时刻。
7. LIO传播到 `m.lio_time`，并写 `last_lio_update_time`；下一轮VIO对同一图像时刻更新。LIO/LO模式通常以扫描尾点时刻为更新终点。

`sync_packages()` 在点分配时使用 `<` 切分，位于边界的点留给后续批次；IMU采用 `<=` 截取，且LIVO过滤不晚于前次更新时间的样本。队首数据不足时等待，序列末尾未必产生与输入图像数完全相同的输出。

## 当前输出语义和已知偏差

- `evo` 轨迹、`/aft_mapped_to_init`、`/path` 在 `handleLIO()` 中产生，属于本周期VIO之前的状态。不得将其标为最终融合关键帧。
- odometry、TF、彩色点云、显示图像和 MAVROS pose 使用 `ros::Time::now()`；基线关闭sim_time时为墙钟时间。
- path 内每个 pose 更新墙钟时间，但 path 顶层header在构造时设置，发布函数未更新该顶层时间。
- `mat_pre.txt`、`mat_out.txt` 使用相对 `_first_lidar_time` 的时间及欧拉角（度），不是标准TUM轨迹；LIO和VIO记录可共享同一时刻。
- PCD/图像文件命名使用观测时刻，保留6位小数；基线轨迹也只保留6位小数，比较时考虑量化。
- `processFrame()` 获得的是相对时间参数；持久化未来应使用明确的绝对观测时间字段。

以上为P0记录的迁移约束，不在基线阶段改写。P1时间发布修正和统一融合出口应分别验收，避免把预期行为变化误报为回归。
