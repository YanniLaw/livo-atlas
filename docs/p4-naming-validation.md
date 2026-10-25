# 命名与职责整理：迁移及验证

本轮以 `32b5892` 为参考，只修改 livo-atlas。实现范围见[设计](superpowers/specs/2026-10-25-naming-cleanup-design.md)及[计划](superpowers/plans/2026-10-25-naming-cleanup.md)。
命名与职责迁移、安装与启动验证、两端官方 bag 回放均已通过。实现提交为
`45eaaa3`、`5d76854`、`c3735a7`；最终提交与恢复状态记录在包外 `recovery.json`。

## 当前边界

`core/LivoSystem` 组合输入、同步、前端、着色和导出；共享状态与数学位于 `common/`。
ROS1/ROS2 共用唯一的 `src/app/livo_main.cc`，两端各四对接入源/头。
`LivoPublisher` 发布里程计、轨迹、点云、图像、标记及 TF；
`CloudColorizer` 累积、投影和着色；`LivoExporter` 写轨迹、诊断、PCD、图像与 COLMAP。
导出器在文件操作前创建已校验的 COLMAP 相机，使用原相机去畸变函数。

`VIOManager`、`Frame`、`Feature`、`Pose6D`、`SubSparseMap`、`VOXEL_LOCATION`
保留名称、拼写和大小写，现位于 `livo_atlas` 命名空间。
本轮保留算法公式、常量、19维状态布局、时间处理顺序和现有所有权。

## C++ 迁移表

| 旧入口 | 当前入口 |
|---|---|
| `runtime/MappingSession` | `core/livo_system.h`、`LivoSystem` |
| `runtime/SessionOptions` | `core/system_options.h`、`SystemOptions` |
| `runtime/MappingOptions` | `core/livo_config.h`、`LivoConfig` |
| `MappingAdapter` | `ros1::LivoNode` / `ros2::LivoNode` |
| `NodeParameters` / `ReadParameters` | `NodeOptions` / `ReadNodeOptions` |
| `ResultPublisher` | `LivoPublisher` |
| `ResultWriter` / `OutputOptions` | `LivoExporter` / `ExportOptions` |
| `State` / `core/state.h` | `FilterState` / `common/filter_state.h` |
| `ImuMeasurement` / `ImageMeasurement` / `PointCloudMeasurement` | `Imu` / `TimedImage` / `TimedPointCloud` |
| `Preprocess` / `ImuProcess` | `LidarPreprocessor` / `ImuProcessor` |
| `PointType` / `PointCloudXYZI` | `Point` / `PointCloud` |
| `V3D` / `M3D` / `RotMtoEuler` | `Vector3d` / `Matrix3d` / `RotationMatrixToEuler` |
| `OutputProcessor` / `PreparedOutput` | `mapping/CloudColorizer`；直接传递 `ColoredCloud::ConstPtr` |
| `.mapping.session` / `.output` / `.processor` | `.livo.system` / `.exporter` / `.colorizer` |
| `livo_atlas::runtime` | `livo_atlas::core`，完整普通系统库 |
| 原前端聚合 `livo_atlas::core` | 单独调度使用 `livo_atlas::frontend`，完整系统使用 `livo_atlas::core` |
| `livo_atlas::output` | `livo_atlas::colorizer` |
| `livo_atlas_ros1_mapper` / `livo_atlas_ros2_mapper` | `livo_atlas_ros1_node` / `livo_atlas_ros2_node` |

表中头路径均以 `livo_atlas/` 开头。其余组件与 Frame 坐标方法的迁移见[命名清单](naming-audit.md)。
旧 `internal/legacy` 宏、全局别名和转发包装已删除；调用方需要更新 C++ API 并重新编译。
`Pose6D` 在 `frontend/imu/pose6d.h`；点协方差在 `mapping/point_with_covariance.h`；
IMU传播和视觉配置校验分别归入 `frontend/imu` 与 `frontend/vision`。

点云指针为空时不发布云、不写点云/普通图像产品；非空空云保持原来的发布及累积节奏，
包括 LIVO 的 LIO 阶段默认空 PointCloud2。COLMAP按原视觉更新节奏写入。
`FrontendResult` 中图像和点云视图在下一次处理前消费，异步保存需要另行复制。
节点名 `laserMapping`、可执行名 `fastlivo_mapping`、参数键、launch、话题与 frame 保持。

## 验证结果

| 检查 | 结果 |
|---|---|
| 普通 C++ 核心 | 152 项通过，39 个普通头独立编译 |
| Noetic | 172 项单元、8 项真实 ROS 测试通过 |
| Humble | 188 项 C++ 测试通过，包含真实 DDS 节点测试 |
| shared/static SDK | 各152项测试、39个普通头；安装前缀移动后LO/LIO/连续VIO通过 |
| 无OpenMP SDK、catkin及ament消费 | 实际LO/LIO/连续VIO通过；独立SDK无ROS/vikit运行链接 |
| 启动保护 | Noetic50项、Humble16项通过；Humble独立DDS检查通过 |
| Python工具 | Noetic环境13项通过、1项版本条件跳过；Humble5项通过 |
| 官方 bag完整回放 | 两端各1351位姿，四组数组及发布内容与各自P4-convergence精确一致 |
| 官方 bag短段产品 | 两端各26位姿、57个导出文件逐字节一致 |
| 原始文件与接入接口 | Log原31文件、vikit提交/源码、config及launch字节保持 |

完整段两端数组形状相同：`trajectory.tum` 为1351×8，`mat_pre.txt` 为2707×19，
`mat_out.txt` 为2701×20，`imu.txt` 为27365×7。相同环境对照的数值差异均为零；
这不把不同编译器、Eigen/PCL/OpenCV版本之间的既有差异视为零。

实际回放绑定本轮源码及各环境的节点加13个库摘要；Humble回放使用实际colcon安装节点。
普通SDK迁移消费直接链接 `livo_atlas::core`，实际执行LO/LIO及连续视觉跟踪。
源码中旧全局类型、兼容宏和包装已消化，六个保留类型在规范头中只有实际定义。

证据统一保存在包外 `validation/P4-naming-20261025/`；旧阶段证据保持只读。
当前已有行为测试先迁移接口，再观察新入口缺失的实际编译失败，随后实现；没有为纯改名增加字段镜像测试。
着色与 COLMAP 测试按职责迁移，原数值、颜色、文件格式、计数和失败生命周期断言保留。

主要证据：`acceptance.json`、`sdk-*-checks.json`、`humble-mode2-installed.json`、
`final-startup/summary.json`、`humble-startup-summary.json`、
`{candidate,humble}-{products,full}-comparison.json`、源码与实际运行库摘要。
执行记录、独立评审及恢复记录在阶段收尾时归档至同一证据目录。

## 在其他电脑上继续

构建命令见[独立核心](core-build.md)和[Noetic/Humble](ros-builds.md)，Noetic隔离回放步骤见[验证指南](validation.md)。
需要 Eigen/PCL/OpenCV 和兼容原非模板 API 的 Sophus；不需要 vikit。
每个系统使用新的构建目录；跨电脑不复用带绝对路径的 CMake 缓存，跨平台重新编译。

本轮收尾生成工作区根目录 `livo-atlas-p4-naming-20261025.bundle`，并在空 bare 仓库验证全部引用、历史和最终提交。
实际完成状态、提交与摘要以包外 `recovery.json` 为准。迁移时同时复制 bag 和需要保留的包外证据。

```bash
git clone /path/to/livo-atlas-p4-naming-20261025.bundle livo-atlas
cd livo-atlas
git switch refactor-fast_livo2
```

后续工作仍是普通关键帧快照与存储边界，再接 GTSAM 和重建导出；本轮没有加入后端或训练框架。
