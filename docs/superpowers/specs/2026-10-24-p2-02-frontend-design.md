# P2-02：单一前端状态和处理流程

承接已批准的[架构](2026-10-11-refactor-architecture-design.md)和已验收的 P2-01。用户授权当前会话直接实施、目录和常规取舍由实施者安排、全部本机验证自主运行；继续当前 refactor-fast_livo2 分支，不增加设计/执行确认。

## 目标和选择

从 LIVMapper 提取真实前端：一个 Frontend 拥有共享滤波状态、传播状态、IMU/VIO/局部地图组件及算法点云；按顺序执行首次时间标记、IMU、可选重力对齐、LIO/VIO、地图更新。ROS 接入消费结果，不能通过发布清理算法下次需要的数据。保留数值公式及有效输入的导出结果。

实施者选择具体 Frontend + 私有实现：公开头仅有普通配置、状态、测量及 PCL/OpenCV 数据，私有实现调用现有算法。另两种选择为只聚合旧字段（无法迁移处理职责），或一次搬走全部 ROS/文件/相机依赖（与 P2-03/P3 混合，难以定位数值变化）。不使用虚拟算法接口、事件总线或阶段回调。

旧算法内部仍有 ROS 日志、地图发布和 vikit；本轮 Frontend 完整实现仍在 Noetic 构建。只对新的公共头做无 ROS/vikit 独立编译，不宣称完整核心已无 ROS。两个明确的过渡借用访问口供旧发布代码使用，P2-03/P3 负责移除。

## 文件和公开契约

新增：

- `include/livo_atlas/mapping/voxel_map_options.h`：原 VoxelMapConfig 的唯一普通实现，旧头用别名。默认值逐项采用 loadVoxelConfig 原默认值，成员保留旧拼写供算法使用。
- `include/livo_atlas/frontend/frontend_options.h`：ImuOptions、VisualOptions、FrontendOptions。包含 Calibration、VoxelMapOptions、原 IMU/VIO 参数、表面体素大小、重力对齐、稠密点云选项和调试绘制选项。值来自原 ROS 参数，不新增 YAML 配置。
- `include/livo_atlas/frontend/frontend.h` 和 `src/frontend/frontend.cc`：具体 Frontend 和公开 FrontendResult。私有实现隔离旧算法头，持有固定地址的两份状态（主状态及其传播先验）；禁止复制/移动，避免 VIO 的非拥有状态指针悬空。

接口：

```cpp
struct FrontendResult {
  UpdateKind kind = UpdateKind::kWaiting;
  bool first_frame = false;
  bool updated = false;
  double timestamp = 0.0;
  double relative_time = 0.0;
  State prediction;
  State estimate;
  PointCloud::ConstPtr world_cloud;
  PointCloud::ConstPtr undistorted_cloud;
};
class Frontend {
 public:
  explicit Frontend(const FrontendOptions& options);
  ~Frontend();
  Frontend(const Frontend&) = delete;
  Frontend& operator=(const Frontend&) = delete;
  FrontendResult Process(SynchronizedMeasurements* measurements);
  const State& state() const;
  bool imu_needs_initialization() const;
  double imu_mean_acceleration_norm() const;
  VIOManager& visual();
  VoxelMapManager& local_map();
};
```

VIOManager/VoxelMapManager 仅在公开头前置声明，不包含 ROS/vikit 头。visual/local_map 返回本实例拥有的旧组件借用引用，仅供顺序 ROS 发布；非后端或可异步持有的通用接口。FrontendResult 的状态是值快照，点云是只读视图，使用期限至下一次 Process；需要长期关键帧时后续另行复制。Process 由单一执行者调用，输入必须是成功同步的、非空 measurement group；等待状态/空组返回 updated=false 且不推进首次标记。其他输入仍遵循 P2-01 的有序、有限时间契约。

## 执行与所有权

1. P2-01 缓冲、同步器、同步输出暂留 Mapper，继续同 mutex 转换/入队/同步。Frontend::Process 接受同一同步输出的非空指针；IMU 仍原地推进 last_lio_update_time。没有第二份同步或滤波算法。
2. 首次成功同步时记录原 last_lio_update_time，先给 IMU first_lidar_time，再 Process2。保留 ONLY_LO 原初值及首帧公式，不借重构修正锚点。
3. 可选重力对齐仍在 IMU 之后；将主状态复制为传播先验及地图状态。结果 prediction 在此后、LIO/VIO 更新前捕获，供原 mat_pre 写入；空点云也保留该条日志。
4. LIO：同 PCL 滤波参数和顺序；先变换下采样云、首次 BuildVoxelMap、StateEstimation，再按原表达式更新世界点/协方差、UpdateVoxelMap、可选滑窗及显示点云。状态和公式不改。地图建图标记由 Frontend 自己拥有。
5. VIO：先检查上一 LIO 世界云是否为空；沿用绘图判断和 processFrame 参数，复用同一个主状态及传播先验。成功时返回该世界云的只读视图，并在 Frontend 内换成空云，使下一轮是否可处理不再依赖发布函数。
6. Mapper 在 Process 返回后写入原预测日志；updated=false 时不写更新后日志或发布。成功 LIO 保存轨迹、发布里程计/点云/地图/path/mavros；成功 VIO 发布彩色云/图像；最后写原 mat_out。发布调用从地图更新前移到完整处理后；地图更新不改滤波状态，状态日志的值和行数必须通过零容差回放确认。
7. 高频 IMU 路径仍在 ROS 层，仅读取前端的初始化状态/均值及 FrontendResult 的值快照。沿用已有传播公式及锁，不新增异步执行器。

VIO 相机在私有实现中创建并先于 VIO 析构；继续使用已验证的 vikit 分配/析构工厂。Mapper 先验证所有现有参数并准备原输出目录，再构造前端，避免 VIO 的 COLMAP 初始化先于目录创建。完整输出迁移留给 P2-03。

LIO 计时由 Mapper 在外侧测量完整 Process 调用，保留每次成功更新的一处 `LIO Mapping Time` 和 Current Total Time 标记供现有验证计数；其测量范围包含 IMU，不能与旧分阶段时延直接比较。旧算法内部 VIO 计时继续保留，本轮不新增核心时钟。

## 验证与边界

参考为 `f77ae4871b9f72a469918ea6a5617db6067566d8`。先在独立源码/构建副本记录当前实际 Mapper 流程：ONLY_LO 首次标记/输出点和共享状态指针、LIO 初始化空点云不会触发地图估计，并补 P2-01 延期的真实 imu_cbk 舍入/回退用例。新测试留同一 ROS 测试编译单元。

新 Frontend 测试先用可编译空 Process 得到行为 RED，再调用真实 IMU、PCL 和地图组件得到 GREEN；不使用算法 mock。覆盖 LO 成功处理、状态值快照、等待/空组、LIO 初始化空输出、LIVO 空视觉数据、两个实例状态独立、非单位外参及地图初始化连续调用。三个新公共头无 ROS/vikit 包含检查。

主工程及独立候选完整 C++/Python/ROS/40 启动检查；参考复用 P2-01 已采集记录前逐项校验源码、程序、8 个本地动态库、bag、工具链和六个估计源参数。候选新跑 30 s/136 s/默认并行 45 s，目标 296/1351/446 点；配对传感器时间、输入/更新计数、轨迹与状态导出零差异，不对齐、不裁剪。

只修改 livo-atlas，vikit 保持 6c886c8e5d83997806e00294826d528cea3581dd。C++17/Google 风格，无 native/fast-math。所有构建 nice -n 10、-j1，不与回放并行，无 RViz；ROS 构造/测试/回放使用独立副本，输出 `validation/P2-02-noetic-20261024/`。保留执行账本和源码 bundle 以便断电后恢复。

不更改首帧积分、EOF、时钟回退 reset、特征 disB、驱动单位或 missing-ring 决策。非特征 Livox bag 不能证明其他设备/特征分支、ARM/Humble、异步安全、实时性能或真值精度。阶段结束一次独立全量审阅；重要问题一次 RED→GREEN 修复并全套验证。
