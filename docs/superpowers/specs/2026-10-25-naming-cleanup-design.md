# 命名与职责整理实施设计

用户已认可 `docs/naming-audit.md` 的集中整理方向并要求继续直接实施。
保留当前已授权的feature分支/工作区和本机验证方式，不再询问执行方式；仅修改livo-atlas。
参考代码提交为 `32b5892`（算法与上一阶段最终 `9d0a290` 相同，之后仅增加审查文档）。

## 硬约束

- `VIOManager`、`Frame`、`Feature`、`Pose6D`、`SubSparseMap`、`VOXEL_LOCATION` 的名称、拼写和大小写保持原样。
- 保留ROS节点名laserMapping、fastlivo_mapping可执行入口、launch接口、参数键、话题/frame/schema。
- Google C++17；不改变公式、常量、矩阵/状态顺序、所有权、时间切分、两次IMU量化和driver fix顺序。
- 继续单线程调用系统；ROS1 spinOnce/5000Hz空闲等待、ROS2单线程executor/1ms处理/4ms传播保持。
- 不拆VoxelMapManager的估计职责，不加入后端、关键帧、插件或训练框架。
- 新证据 `validation/P4-naming-20261025`、容器候选 `/tmp/livo_atlas_naming`；之前证据与依赖只读。
- nice/-j1串行编译；与bag回放串行，无RViz。保留原Log31与rpg_vikit源码/提交。

## 方案与范围

采用集中迁移实际调用方、删除旧兼容包装的方案。只改别名并保留旧目录会继续维护两套名称；
整体重写估计器超出本轮目标。当前方案保留现有估计实现，只整理名字、依赖与输出职责。

1. `runtime/`→`core/`；原core基础头→`common/`。
   `MappingSession`→`LivoSystem`、`SessionOptions`→`SystemOptions`、`MappingOptions`→`LivoConfig`。
   LivoConfig.system为普通系统设置；ROS NodeOptions.livo持有完整应用配置。
   `MappingAdapter`→两端LivoNode；ResultPublisher→LivoPublisher；NodeParameters→NodeOptions。
   ResultWriter/OutputOptions→LivoExporter/ExportOptions；系统字段exporter用于文件导出设置。
   同一入口文件改为app/livo_main.cc；CMake保留一个可执行目标。
2. `State`→FilterState，测量类型改为Imu/TimedImage/TimedPointCloud。
   其余采用审查表中的明确名称：LidarPreprocessor、ImuProcessor、PointWithCovariance、PatchWarp、
   VisualVoxel、VoxelOctreeNode、LidarPointAttributes、PointPlaneMatch、SyncStatus。
   MeasurementGroup→UpdateMeasurements；同步工作区成员按当前/下一点云、更新种类和游标命名。
   Frame保持类名，坐标方法使用WorldToPixel/WorldToCamera/PixelToBearing/CameraToWorld等准确名称。
3. 公共算法类型归livo_atlas命名空间。移除头文件全局using namespace与旧宏/别名包装。
   公共math_types/so3_math/filter_state保留一份实现；Pose6D移至frontend/imu/pose6d.h，名字不变。
   点协方差结构归mapping/point_with_covariance.h；终端颜色归common/terminal_colors.h。
   旧算法表达式的模板宏以等价Eigen类型替换；初始化器宏使用相同数值的原展开，保持运算顺序。
   删除无实际使用的DS_POINT及指针别名；有实际使用的FramePtr保留明确所有权。
4. 取消output目录，点云累积/投影/着色归mapping/CloudColorizer，COLMAP图像准备归io/LivoExporter。
   `CloudColorizer::Process(const FrontendResult&) -> ColoredCloud::ConstPtr`：空指针表示无有效云；
   有效结果返回非空指针，即使RGB云为空也保留旧ROS空云发布行为。
   `LivoPublisher::Publish(result, colored_cloud)`；`LivoExporter::Write(result, colored_cloud)`。
   导出器使用原相机函数准备COLMAP图像，保留相机只缩放一次及原导出格式。
   不再保留PreparedOutput。普通RGB点云别名放sensor/point_types.h，消费者不依赖通用输出容器。

## 构建与安装

删除原仅聚合的core接口目标，由完整物理库livo_atlas_core / livo_atlas::core承载LivoSystem，
显式依赖frontend/synchronization/preprocess/colorizer/io；不创建无用途的common物理库。
旧output目标替换为colorizer，runtime目标移除，两端ROS mapper目标随节点更名为ros1_node/ros2_node。
普通核心不依赖ROS或版本宏；版本宏仍仅属于唯一main。各模式安装仅提供新的规范头和目标。
算法子库仍可分别消费；测试与安装消费者使用完整core。旧API使用方通过迁移表更新。

## 文件归属

新公开核心头：core/{livo_system,system_options,livo_config,parameter_reader}.h；
基础头：common/{filter_state,math_types,so3_math,terminal_colors}.h。
ROS两端各四对源/头：livo_node/node_options/measurement_conversions/livo_publisher。
IMU传播移入frontend/imu；视觉选项校验移入frontend/vision。
ROS1节点和转换测试移到tests/ros1；其他测试按实际模块调整，保留已有数值和行为断言。
内部辅助类型留在实际使用的模块，不为每个小结构体新建文件。

## 验收

- 初始普通152项基线在新证据目录重跑；旧二进制只读，TMPDIR及XML重定向到新目录。
- 新公共入口通过迁移现有安装消费者先经历真实编译失败；纯改名不增加字段镜像测试。
- 兼容别名静态断言随别名删除，原Euler/Reset行为测试迁到公共接口，数值断言保留。
- 着色/导出已有测试先迁移接口再观察失败，覆盖空云、跨帧累积、颜色采样、COLMAP及文件保护。
- 普通shared/static/无OpenMP安装移动后实际运行LO/LIO/连续VIO，公开头独立包含。
- Noetic/Humble全部算法、转换、真实ROS/DDS、启动检查通过；源码/实际库摘要绑定。
- 每环境3秒产品段与136秒完整bag，精确对照P4-convergence；各1351位姿、四数组/发布一致，短段57产品逐字节一致。
- 一次fresh整阶段代码评审；重要问题一次RED→GREEN修复，minor明确记录。
- 更新当前文档、名称/目标迁移表，生成完整历史bundle并在空bare仓库恢复验证最终提交。
