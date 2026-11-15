# P6-03a：在线高斯建图的观测准备

2026-11-15，基于 `5e66852`。本阶段将已有关键帧转换为拥有数据的图像与稀疏深度视图，供
后续在线 Gaussian mapper 使用。它是 P6-03 的第一段，不包含高斯参数、训练、渲染或在线调度，
不能单独称为在线 3DGS。用户已允许在当前分支自主实施与验证；正式修改产品代码前先通知用户。

## 背景和已核实条件

P6-02 的在线子地图、几何回环、GTSAM 位姿版本和 ROS 全局输出已实现。`OnlineSubmapUpdate`
携带拥有数据的关键帧、成员修订和完整位姿快照，但现有 COLMAP 视图计算在 `src/io/` 内部，
没有供在线消费者直接使用的成对图像、相机和稀疏深度接口。`LivoSystem` 的在线回调仍只有一个，
现有入口负责向回环、轨迹和几何消费者分发；本阶段不改变这一机制。

本机 Noetic 完整 Retail_Street 回放产生 172 个视觉关键帧。基于其版本 0 和已有 COLMAP 相同的
中心化针孔投影，图像为 640×512，有效去畸变区域为 324,038 像素；每帧最近点深度覆盖中位数约
13,419 像素，即有效区域的 4.14%（范围约 3.56%–4.27%）。前三帧的投影像素与现有 COLMAP
导出完全一致、去畸变图像逐像素一致，比较深度的最大差小于 1.5e-14 米。此盘点用的是保存的
前端位姿和单帧已去畸变点云，不是深度真值。脚本、逐帧统计、输入 SHA-256 和对照产物保存在
工作区包外 `validation/P6-03-design-20261115/`，迁移电脑时需另行复制。

本机为 RTX 4060 Laptop，显存 8,188 MiB，驱动 535.183.01；调查时未在当前 Noetic Python
环境发现 PyTorch、gsplat 或 CUDA 编译器，已有 Humble 容器未分配 GPU。NVIDIA 驱动和 CUDA
版本兼容规则只能作为环境筛选，不能代替实际内核运行验证。Python/OpenCV 在 CPU 0 上构建本
数据分辨率的去畸变映射和掩码，中位数约 1.46 ms；这不代表 C++ 完整观测转换或在线建图吞吐。

近期方法为选择提供参考：

- [Gaussian-LIC2 官方仓库](https://github.com/APRIL-ZJU/Gaussian-LIC) 将 LIV 定位与高斯建图
  结合，其公开构建使用 CUDA、LibTorch、CUDA OpenCV 和 TensorRT；这些不应成为本工程核心必需依赖。
- [GS-LIVO 官方仓库](https://github.com/HKUST-Aerial-Robotics/GS-LIVO) 描述 LiDAR/图像
  初始化和 CPU/GPU 活动窗口；其公开实验硬件与本机不同，不能直接套用性能预算。
- [gsplat 官方仓库](https://github.com/nerfstudio-project/gsplat) 提供 Python/PyTorch CUDA
  栅格化；官方安装路径需要 PyTorch，并可能在首次运行编译 CUDA。候选后端和准确版本须在实际
  GPU 环境验证后确定，不在本阶段选择或安装。
- [LoopSplat](https://github.com/GradientSpaces/LoopSplat) 将高斯子地图和闭环联系起来；
  它的 RGB-D 输入条件与本数据集的稀疏 LiDAR 不等同。

## 方案与边界

| 方案 | 优点 | 代价 |
| --- | --- | --- |
| 每次先导出 COLMAP，再读取文本和图像 | 重用已验证出口 | 需要中间文件，难以直接消费在线快照 |
| **直接复用现有 COLMAP 投影，返回普通 C++ 观测值** | 与现有几何、像素和去畸变约定一致，无 GPU 依赖 | 当前每次构造投影器和生成可见点样本，尚无在线吞吐保证 |
| 现在重写通用投影框架并替换 COLMAP 出口 | 将来可能统一更多用途 | 改动已有稳定出口，扩大回归范围；当前无必要 |

采用第二种。只扩展现有私有 `ColmapCamera` 的有效图像区域只读访问，并在 `livo_atlas::io`
提供一个直接返回结果的函数。不引入 Pimpl、插件框架、后台线程、新 ROS 话题或持久化 schema。
调用方可以在自己的 worker 中对保留的关键帧和位姿快照调用该函数；本阶段不声称已经接入
真实在线训练队列。现有 COLMAP 出口的文件、算法和输出语义保持不变。

## 输入与输出契约

拟新增 `livo_atlas/io/reconstruction_view.h`：

```cpp
struct ReconstructionViewLimits {
  std::size_t max_pixels = 0;
  std::size_t max_cloud_points = 0;
};

struct ReconstructionView {
  std::uint64_t keyframe_id = 0;
  std::uint64_t calibration_id = 0;
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  PoseFrame parent_frame = PoseFrame::kOdom;
  double state_timestamp = 0;
  double cloud_timestamp = 0;
  double image_timestamp = 0;
  double inverse_exposure_time = 1;
  CameraConfig camera;
  RigidTransform camera_from_parent;
  cv::Mat image_bgr;
  cv::Mat image_mask;
  cv::Mat depth_m;
  cv::Mat depth_mask;
  std::size_t valid_depth_pixels = 0;
};

ReconstructionView MakeReconstructionView(
    const CameraConfig& camera, const Keyframe& keyframe,
    const PoseSnapshot& poses, ReconstructionViewLimits limits);
```

- 调用方提供与 `keyframe.calibration_id` 对应的相机标定；当前存储格式只有标定 ID 0，函数不
  从外部文件猜测或替换标定。拒绝无图像关键帧、未覆盖的 ID、不合法标定/位姿、图像尺寸不符及
  非正资源上限。版本 0 还须检查该帧快照位姿与 `odom_from_body` 相符：沿用既有 pose 比较
  的 1e-9 米平移和 1e-9 弧度旋转阈值。
- 位姿和输出都在同一 `PoseSnapshot` 的父坐标系：版本 0 为 odom，正版本为 map。保留版本、
  前沿及已优化前缀，使消费者能判断位姿来源；构造视图本身不拒绝未来变成过期的结果，异步提交
  时仍由未来消费者检查版本；同版本新增尾帧不必然使旧观测位姿过期。不同子地图的成员修订与
  高斯内容修订不借此字段冒充。
  ID 和版本仅在单个运行或数据集内有意义；跨会话消费者须自行区分来源，本阶段不生成全局会话 ID。
- 两种上限由调用方显式给出。先计算受校验标定影响的像素数并检查上限，再分配去畸变映射；先
  检查输入点数上限，再复制或投影。该边界限制单次输入规模，不是整进程、图像来源或未来 GPU
  队列的字节上限。
- `camera` 是已经缩放一次且去畸变的中心针孔相机，`scale=1`、畸变系数为零。公开的
  `CameraConfig` 沿用工程相机类使用的 OpenCV 整数像素中心约定：第 `(x,y)` 个像素中心为
  `(x,y)`，中心主点为 `((width-1)/2,(height-1)/2)`。内部 COLMAP 投影使用像素角点约定；
  转换为 COLMAP 内参时对主点各加 0.5。相机、图像、有效区域和深度必须对应同一映射。
- `image_bgr` 为 `CV_8UC3`；`image_mask` 和 `depth_mask` 为 `CV_8UC1`，0 表示无效、1 表示
  有效。前者描述去畸变采样可用区域，后者描述该区域中有 LiDAR 观测的像素，两个掩码不能混用。
  `depth_m` 为 `CV_32FC1`、相机光轴正向 z、单位米。无观测处存 0，但必须由 `depth_mask`
  排除，不能作为零深度监督；非正、不可表示或非有限的有效深度不得悄悄进入结果。
- 点投影复用 `ColmapCamera::MakeView` 的去畸变和栅格规则：前方且处于有效图像区域的点参与，
  同一像素只保留相机 z 最近的点；可见像素不做训练点数下采样。重建视图直接使用帧局部坐标投影，
  避免全局位姿修正的浮点往返改变像素归属；原 COLMAP 导出保留其父坐标投影路径，恰在像素边界
  的点可能存在数值差异。允许没有可见深度的合法图像，掩码全 0；后续训练器可以根据策略跳过
  该观测或仅用于已有高斯的外观更新。
- 图像、掩码和深度均为函数新分配的存储，不别名输入关键帧；OpenCV 的结果副本仍可能共享底层
  内存，消费者应按只读约定使用。函数不保存对输入的引用或指针。
- 三个传感器时间原样保留。`inverse_exposure_time` 是前端估计量，不是硬件曝光时长。点云已由
  前端按自身方式处理，快照不含完整逐点时间或连续轨迹；当前投影不能被描述为重新做了连续时间
  运动补偿。

坐标变换继续使用 `T_A_B` 表示 B 到 A。已有出口对合法但略有舍入误差的旋转作 SO(3) 投影：

```text
T_C_B = rigid(T_C_odom(raw)) * rigid(T_odom_B(raw))
T_C_parent = T_C_B * inverse(rigid(T_parent_B(version)))
p_C = T_C_B * p_B
```

最终相机旋转按已有 COLMAP 路径归一化。比较误差时不能把旧的近似旋转当作严格正交矩阵。
局部点投影不依赖父坐标变换；整体位姿改变时，同一观测的像素和相机深度保持一致，地图系相机
位姿随版本变化。更复杂的跨帧融合、扫描时序和图内重优化另行设计。

## 验收及下一步

1. 测试像素中心、去畸变、深度 z 与遮挡、不同相机模型、空投影、非有限/异常数据、资源上限和
   结果所有权；用非平凡旋转/平移及两个位姿版本验证坐标与深度不变性。
2. 通过独立参考核对已有 COLMAP 输出；对真实 Retail_Street 关键帧检查深度覆盖、像素及掩码，
   记录实际运行资源。测试不能把没有深度的像素当真值。
3. 普通 C++ 核心、Noetic 和 Humble 的适用测试通过，验证新增 SDK 头和安装消费不依赖 ROS、
   vikit、CUDA 或 PyTorch。没有修改既有 ROS/前端路径时，按受影响范围执行回归，不重复无关矩阵。
4. 文档记录坐标、像素、内存和跨电脑构建复现。独立审阅重要问题闭环。

P6-03 的后续阶段才处理选定渲染器、隔离 GPU 环境、训练队列、子地图归属、位姿修正和过期
任务提交。必须实际测量 GPU 显存、吞吐和闭环更新；实时渲染不等于实时训练，短 bag 和无真值
也不能证明长期一致性或绝对精度。

## 设计检视

独立构建者检查围绕标定、像素坐标、版本、时间、所有权和资源上限阅读了本方案及现有代码，
没有提出阻碍制定实施计划的重要问题。该检查不是实现审阅或运行验收。
