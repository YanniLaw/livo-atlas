# P2-01：普通 C++ 同步器

本设计细化已批准的[目标架构](2026-10-11-refactor-architecture-design.md)第 6 节。用户已选择当前会话直接实施、目录由实施者安排、验证自主执行；沿用该授权，不增加阶段确认。

## 目标与边界

把 LIVMapper::sync_packages 的时间窗口、IMU 选择、点云切分和 LIO/VIO 状态转换移到可无 ROS 构建的库。ROS1 回调把 P1-03 的普通测量放入成对的队列，适配器持锁调用同步器。输出继续交给同一个现有 IMU/LIO/VIO 状态；本轮不迁移估计、发布、日志或相机。

P2 分成三个可验证步骤：P2-01 同步器；P2-02 单一 Frontend 的状态与调度；P2-03 发布和文件输出。首帧积分锚点、完整时钟回退 reset 及原特征平面参数修复必须另有数值用例，不能混入本次职责迁移。

只修改 livo-atlas，vikit 保持 `6c886c8e5d83997806e00294826d528cea3581dd`。C++17、Google C++ 风格；新 API 不含 ROS、vikit、线程、时钟和文件输出。

## 方案选择

1. 推荐：普通 SensorBuffers 持有成对测量队列，SensorSynchronizer 操作队列和同步输出状态。与现有直接调用接近，单一执行者和离线读取都能使用。
2. 仅搬成一个访问五条平行队列的函数：改动少，但仍有点云/图像与时间队列错配风险。
3. 同时重写 Frontend、异步队列和发布：单次改变过多，难以区分同步和估计偏差，留到后续步骤。

不增加消息总线、插件接口、调度线程或预设后端抽象。

## 数据和接口

新增 `include/livo_atlas/frontend/synchronized_measurements.h`：

- `FrontendMode`：kLidarOnly=0、kLidarInertial=1、kLidarInertialVisual=2。
- `UpdateKind`：kWaiting=0、kVisual=1、kLidarInertial=2、kLidarOnly=3。
- `MeasurementGroup` 和 `SynchronizedMeasurements` 移入命名空间，保留现有 MeasureGroup/LidarMeasureGroup 的成员、类型、初值和共享点云语义。common_lib.h 只提供原名称和枚举常量的兼容别名，不复制状态。
- 输出中的 last_lio_update_time 仍由 IMU 处理更新；同步器只按原逻辑设置首次扫描时间，不擅自推进它。

新增 `include/livo_atlas/sensor/synchronizer.h`、`src/sensor/synchronizer.cc`：

```cpp
struct SensorBuffers {
  std::deque<PointCloudMeasurement> point_clouds;
  std::deque<ImuMeasurement> imu;
  std::deque<ImageMeasurement> images;
  double latest_imu_time = -1.0;
};

enum class SynchronizationResult {
  kWaiting, kReady, kImageDropped, kInvalidCloud
};

class SensorSynchronizer {
 public:
  explicit SensorSynchronizer(
      FrontendMode mode = FrontendMode::kLidarInertialVisual,
      double exposure_time = 0.0);
  SynchronizationResult TrySynchronize(
      SensorBuffers* buffers, SynchronizedMeasurements* measurements) const;
};
```

两个指针必须非空。队列和输出由调用者独占修改；ROS1 适配器使用同一个 mutex 保护入队和整个同步操作。图像复制保留 cv::Mat 引用计数，像素已在转换层独立拥有；点云仍使用实际 PCL Ptr。同步器不向输入点写入修正，切分输出按原顺序复制点。

latest_imu_time 表示接收端用于 ONLY_LIO 覆盖判断的时间。ROS1 必须保留原回调最后一次 fromSec 之前的 double 值，不能换成最终纳秒舍入后的测量时间；非 ROS 输入可使用最后样本时间。LIVO 仍以队尾测量时间判断覆盖。

## 必须保留的行为

- 时间为传感器 double 秒，curvature 为 ms；曝光时间只加在图像窗口上。
- LIVO 从 WAIT/VIO 产生 LIO，再对同一图像产生 VIO。同步器不调用估计器。
- 点云头时间不大于图像时间时参与切分；点偏移严格小于浮点 ms 截止值进入当前云，等于截止值进入下个窗口。保留两条 float/double 加法表达式、点顺序和剩余云的复制/清空方式。此前剩余云按原样复制到下一轮当前云，不对它二次切分；这也是原实现特征测试固定的行为。
- IMU 消耗到窗口终点（含等号）；LIVO 仅输出大于上次更新时刻的样本，之后的样本留在队列。ImuProcess 保留的跨帧样本不在本轮重写。
- 图像时间小于 last_lio_update_time+0.00001 时丢弃一帧，并由返回值让 ROS1 记录原日志。覆盖不足时等待，不提前消费输入。
- 保留原 LIVO 在所有状态下要求三种输入队列非空的就绪条件。本轮不增加 EOF 排空语义；否则完整 bag 可能多一次末尾 VIO，破坏本阶段数值参考。
- ONLY_LIO 仍等接收覆盖时间达到扫描尾部，ONLY_LO 不要求 IMU/图像。扫描尾部仍取最后一个点，不改成最大点时间或重新排序。
- 不再保留冗余 lidar_pushed：等待时每次从同一队首重建相同扫描视图。输入只能由入队/同步路径修改，因此已等待扫描不会被后续扫描替换。

## 输入边界

普通输入也可能来自离线读取：队首或队尾的空/空指针点云应移出并返回 kInvalidCloud，不能解引用 back()。切分遇到内部无效云同样跳过。ONLY_LIO 少于两个点的队首帧移出后返回 kInvalidCloud，避免原先永久堵塞队列。其他模式允许单点云；这些边界不改有效 bag 的计算。

回调继续在转换/预处理成功后才处理 LiDAR 时间回退，清理一条成对点云队列。IMU/图像的原回退拒绝与驱动修正不迁移。同步器不声称提供完整估计器 reset。

## 验证与可迁移证据

参考为 P1-03 结束提交 `1bca9a3d706b2c25afd50a6d197db51e4474d567`。先在独立候选副本上为原 sync_packages 添加真实 ROS 特征测试，记录严格切分、交替、覆盖舍入和 ONLY_LO 行为；迁移后保留断言。

新核心先通过可编译空实现得到行为 RED，再实现并验证 GREEN。`tests/frontend/CMakeLists.txt` 只查找 Eigen/PCL common/OpenCV core/GTest，独立构建实际同步源及两个公共头，不提供 ROS/vikit 路径。

复核 P1-03 最终候选源码、程序、全部本地动态库、bag 和编译环境后，复用它的两份已采集短段/完整参考记录，不重新运行或更改该工作区。新候选独立构建，依次运行短段 30 s、完整 136 s、默认并行 45 s；预期轨迹 296/1351/446，控制配对的时间、输入/更新计数、TUM、mat_pre/mat_out 导出值零差异。若参考校验不通过，就从固定提交新建参考并重新采集。

构建使用 nice -n 10、-j1，不与回放并行，不启用 RViz。P2-01 证据在包外 `validation/P2-01-noetic-20261018/`，不覆盖 P1-03。最后执行一次独立审阅，重要问题以 RED→GREEN 闭环，并保存文档、执行记录和离线 bundle。
