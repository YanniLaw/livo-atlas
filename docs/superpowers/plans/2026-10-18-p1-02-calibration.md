# P1-02 配置与标定边界 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将外参与相机标定从 ROS 参数读取中拆出，保留现有有效 YAML 的数值行为，让错误配置在处理传感器数据前明确失败。

**Architecture:** 普通 C++ 标定类型和校验仅依赖标准库、Eigen。ROS1 适配器负责参数名称、必填项和默认值；临时相机工厂从普通配置构建现有 vikit 对象。LIVMapper 只连接这些入口，前端公式与调度不变。

**Tech Stack:** C++17、Eigen 3、OpenCV 4、vikit、ROS1 Noetic、CMake、GoogleTest、Python 3。

**Spec:** [已认可的整体设计](../specs/2026-10-11-refactor-architecture-design.md)，第 5、7、10 节。

## Global Constraints

- 只修改 `livo-atlas`；`rpg_vikit` 为只读依赖。
- 新代码遵循 [Google C++ 编码约定](../../cpp-style.md)，使用 C++17；不引入通用配置框架、额外状态库或插件系统。
- `T_A_B` 表示 `p_A = R_A_B * p_B + t_A_B`；`extrinsic_R/T` 是 `T_I_L`，`Rcl/Pcl` 是 `T_C_L`；平移单位为米。
- 数组矩阵按原来行优先填入的语义解析，不能直接使用默认列优先 `Eigen::Map`。
- 外参旋转检查容忍现有标定文件的舍入误差，不能擅自正交化并改变有效标定。
- 参数名称及合法配置的默认值保持原样。相机参数来自节点私有命名空间，默认节点名仍为 `laserMapping`。
- 支持包内 YAML 实际使用的 `Pinhole`、`EquidistantCamera`；其他模型启动时报告不支持。此次不扩展模型，也不改写投影算法。
- 核心校验返回错误字符串，空字符串表示成功；ROS 边界将失败转为带参数上下文的启动错误。输出参数仅在成功时更新。
- 构建使用 `nice -n 10` 和 `-j1`，编译与 bag 回放依次运行；验证产物在包外 `validation/P1-02-noetic-20261018/`。
- 数值参考为阶段开始的 `2b515ae471451e073dd26fd8add5bf6a9830801e`，参考和候选均包含 P1-01 的三项 VIO 修复。

## Review Focus

1. 缺失、错误类型或长度不正确的外参必须在任何数组索引前失败，不能退回空向量继续启动；Task 2 的真实节点检查覆盖。
2. 非单位旋转、带平移的两组外参不能转置或反向；Task 1 的行优先测试与 Task 2 的真实 VIO 初始化测试覆盖。
3. 非整数缩放后的尺寸沿用 vikit 的截断语义，内参只缩放一次；零尺寸、非有限值和整数溢出必须被拒绝；两项任务分别覆盖校验和真实相机行为。
4. 可选参数缺失可使用原默认值，但提供错误类型必须报错；重命名节点应读取自己的相机标定；Task 2 的真实节点检查覆盖。
5. 相机尺寸与 VIO 网格不兼容、窗口/金字塔参数造成除零或整数溢出、鱼眼相机启用仅支持针孔的 COLMAP 导出，应在文件输出及估计初始化前失败；Task 2 的真实节点检查覆盖。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| `include/livo_atlas/sensor/extrinsics.h`, `src/sensor/extrinsics.cc` | `RigidTransform`、旋转/平移校验、行优先数组解析。 |
| `include/livo_atlas/camera/camera_config.h`, `src/camera/camera_config.cc` | 相机标定值与数值/尺寸校验，无 ROS 或 vikit。 |
| `include/livo_atlas/frontend/calibration.h` | 两组传感器外参与相机标定的简单组合。 |
| `include/livo_atlas/camera/vikit_camera_factory.h`, `src/camera/vikit_camera_factory.cc` | 从普通配置创建 vikit 对象，保留原构造参数和顺序。 |
| `include/livo_atlas/ros1/calibration_reader.h`, `src/ros1/calibration_reader.cc` | ROS1 参数名称、类型、默认值和错误上下文。 |
| `include/LIVMapper.h`, `src/LIVMapper.cpp`, `src/main.cpp` | 连接新入口、管理相机生命周期、启动错误处理及图像处理参数检查。 |
| `include/vio.h`, `src/vio.cpp` | 用已解析的外参替代原始向量 setter，保留原矩阵计算。 |
| `CMakeLists.txt`, `tests/core/CMakeLists.txt` | 独立标定库、catkin 接线、独立包含编译和测试入口。 |
| `tests/core/calibration_test.cc`, `tests/camera_config_test.cc`, `tests/vio_sampling_test.cc` | 真实校验、相机与 VIO 外参行为测试。 |
| `tools/check_calibration_startup.py` | 独立 ROS master 中加载真实 YAML、构造坏配置、检查真实节点启动及输出副作用。 |
| `docs/p1-02-validation.md`, `docs/README.md` | 本阶段结果、复现步骤及后续边界。 |

### Task 1: 普通标定类型与独立校验

**Files:** 上表的 extrinsics、camera_config、calibration 头和实现；`tests/core/calibration_test.cc`；两个 CMake 入口。

**Interfaces:**
- Consumes: Eigen 的 `Matrix3d`、`Vector3d`，标准库 `std::array`、`std::vector`、`std::string`。
- Produces: `RigidTransform { Eigen::Matrix3d rotation; Eigen::Vector3d translation; }`，默认单位旋转及零平移。
- Produces: `CameraConfig { std::string model; int width, height; double scale, fx, fy, cx, cy; std::array<double, 4> distortion; }`，尺寸与内参默认零，scale 默认 1，畸变默认零。
- Produces: `Calibration { RigidTransform imu_from_lidar; RigidTransform camera_from_lidar; CameraConfig camera; }`。
- Produces: `std::string ValidateExtrinsic(const RigidTransform&)`；`std::string ParseExtrinsic(const std::vector<double>& rotation, const std::vector<double>& translation, RigidTransform* result)`；`std::string ValidateCameraConfig(const CameraConfig&)`。

- [x] **Step 1: 先写行为测试与最小可编译接口**

  测试有效的带平移 Z 轴 90° 旋转，行优先数组 `{0,-1,0,1,0,0,0,0,1}` 将点 `(2,0,0)` 加平移 `(1,2,3)` 后得到 `(1,4,3)`。覆盖长度、NaN/Inf、反射、非正交旋转及失败不修改输出。
  原 `avia.yaml` 舍入矩阵校验成功且逐元素不变；旋转的正交残差与行列式误差阈值均为 `1e-4`。
  相机用例覆盖两种模型；非法模型、尺寸、焦距、缩放、畸变与主点的非有限值；缩放后尺寸不足 1、超出 int、像素索引乘积溢出和内参乘积非有限。允许有限的负主点，不擅自修正标定。

- [x] **Step 2: 运行 RED**

  在 `validation/P1-02-noetic-20261018/core-build` 独立配置、单线程构建。先给接口最小占位实现以取得行为失败；缺头或链接错误不算校验逻辑的 RED 证据。保存失败日志。

- [x] **Step 3: 实现最小校验及解析**

  在索引前检查 9/3 长度；逐行填入 Eigen；成功才提交结果。校验不正交化。相机尺寸在浮点乘法、有限性和范围检查通过后才转换为整数；保持 vikit 的正数截断规则。

- [x] **Step 4: 独立验证 GREEN**

  Run: `cmake --build validation/P1-02-noetic-20261018/core-build -- -j1`，然后在该目录运行 `ctest --output-on-failure`。
  Expected: 原数学/状态与新标定行为测试均通过；新公共头独立包含可编译，编译命令中没有 ROS、PCL、vikit 路径。

- [x] **Step 5: 提交**

  提交普通类型、实现、测试与构建接线：`refactor: define and validate plain sensor calibration`。

### Task 2: ROS1 配置入口、相机接线与数值回归

**Files:** 上表的 ROS1 reader、vikit factory、mapper/main/VIO、相机测试、启动工具和验证文档。

**Interfaces:**
- Consumes: Task 1 的三个类型及三项校验/解析函数，签名保持一致。
- Produces: `std::string ros1::ReadCalibration(const ros::NodeHandle& node, const ros::NodeHandle& camera_node, Calibration* result)`，失败不修改输出；外参从 node 读取，相机从 camera_node 读取。
- Produces: `std::unique_ptr<vk::AbstractCamera> CreateVikitCamera(const CameraConfig&)`；无效配置返回空指针，有效参数按原顺序传给相同的 vikit 构造函数。
- Produces: `VIOManager::SetLidarToCameraExtrinsic(const livo_atlas::RigidTransform&)`；Rcl/Pcl 为直接拷贝，Rli/Pli、Rci/Pci 的表达式保持原样。
- Produces: `tools/check_calibration_startup.py --workspace <隔离构建副本> --output <新目录> --port <独立端口>`；全部案例通过才返回 0，保存每项参数、退出码和日志及源/二进制摘要。

- [ ] **Step 1: 建立真实边界的 RED 证据**

  工具先对阶段开始的独立构建运行：缺失/短外参应明确失败；可选项错误类型应失败；重命名节点仍能加载私有相机参数。原版不能满足的案例记录为 RED，不接受段错误作为成功拒绝。
  为工厂建立可编译的空指针占位实现，真实相机测试先失败；该测试随后对针孔/鱼眼的缩放、投影、反投影与原 vikit 对象逐值对照。
  VIO 测试使用两组不同的 90° 旋转和非零平移，手算 Rci/Pci 的期望，避免单位旋转掩盖方向错误。

- [ ] **Step 2: 实现读取与接线**

  保留四组外参数组的参数名；必填相机参数为 model/width/height/fx/fy/cx/cy；scale 默认 1，四个畸变系数默认 0。
  支持 ROS 数值参数的整数与浮点表示，拒绝布尔值、字符串及错误数组元素。提供的可选参数类型不对时不得默认。
  在 mapper 创建文件/初始化估计器前完成全部标定、网格、窗口与金字塔检查；只检查实际使用的图像处理参数，不引入完整前端配置框架。
  相机由 mapper 的 `unique_ptr` 拥有，VIO 继续使用生命周期明确的非拥有指针。替换原向量 setter，更新现有采样测试调用。main 捕获初始化错误，输出原因并返回非零状态。

- [ ] **Step 3: 构建、测试与启动 GREEN**

  构建独立候选副本，不复用其他路径的 CMake 缓存；运行全部 C++ 与 Python 测试、启动检查。
  启动检查覆盖包内四套 YAML 配对、可选默认值、私有命名空间、错误数组/类型/非有限值/旋转/尺寸/焦距/缩放/模型、图像网格和窗口/金字塔溢出，以及鱼眼 COLMAP 约束。
  Expected: 正确案例完成订阅注册并正常退出；错误案例有相关参数诊断、非零正常退出、没有注册估计器订阅，也未修改 mapper 输出文件。保存完整证据。

- [ ] **Step 4: 短段及完整 bag 配对回归**

  以阶段开始提交创建独立参考副本，依次构建参考和候选。统一 `OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE`，使用现有 `tools/run_baseline.py`，30 秒、0.5 倍先各跑一次；通过后各回放 136 秒覆盖完整 bag。另跑候选默认并行 45 秒检查正常运行。
  Expected: 短段 296、完整 1351 个轨迹点；输入计数、传感器时间、更新序列一致；同时间戳位置/姿态及 `mat_pre.txt`、`mat_out.txt` 在导出精度内零差异；所有运行正常退出。不对齐轨迹、不裁剪时间、不放宽阈值。

- [ ] **Step 5: 更新文档并提交**

  更新本计划的完成状态、文档入口和 `docs/p1-02-validation.md`，记录真实命令、测试数量、摘要、数值差异与限制。提交：`refactor: isolate ROS1 calibration loading and camera construction`。

## 阶段结束

按执行技能进行一次完整变更的独立只读审阅；对重要问题先复现再修复并运行相关完整套件。保存审阅与执行记录，不推送、不合并。独立标定库可无 ROS 构建并不意味着完整前端已经无 ROS；后续继续 P1-03。
