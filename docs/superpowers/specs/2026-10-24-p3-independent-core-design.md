# P3：解除 vikit 依赖与独立核心

状态：用户已授权当前会话直接实施、安排目录并主动本机验证；本阶段按该授权推进，不重复询问执行许可。
基线：`752a3be256b4560ce537a09077f2b419b0b584b1`。

## 目标与约束

只修改 livo-atlas，使同一套真实 Frontend、同步、预处理、相机、输出处理与文件写入可在不查找、不包含、不链接 ROS/vikit 的标准 CMake 工程中构建和运行；Noetic 节点继续使用这些库。rpg_vikit 源码保持提交 `6c886c8e5d83997806e00294826d528cea3581dd`，只允许读取或构建固定版本用于参考。

Google C++17，普通数据和具体组件；不引入插件框架、服务定位器、异步调度或另一套估计算法。保持 IMU/LIO/VIO 数学、时间单位、标定方向、同步门控、首帧锚点及已有输出契约。Humble 接入、后端、关键帧存储和 3DGS 训练分别留给后续阶段。

## 选择与范围

选择小型相机接口和两个实际实现，而不是复制整个 vikit，或给全部算法套虚拟接口。相机确有 Pinhole / Equidistant 两个实现；其余组件仍直接调用。只迁移实际使用的投影、反投影、帧范围检查、Pinhole 图像去畸变、灰度插值和 Shi–Tomasi 评分。

`frame_utils::createImgPyramid` / `ImgPyr` 在包内没有调用，删除该遗留入口；不复制未使用的 SSE/NEON 金字塔路径。删除未使用的 vikit 数学/鲁棒核/计时头，Frame / VisualPoint 用 C++ 删除拷贝代替 boost::noncopyable。旧 IMU 头内非 inline 的比较器收回实现文件，确保静态库也能链接。

不把数值修正藏在相机替换中：原模型仅以第一个畸变系数的绝对值是否大于 1e-7 决定启用畸变；Equidistant 在 r<1e-8 时直接返回归一化坐标，反投影光轴处会产生非有限值。这些兼容行为明确记录并单独测试，后续模型修正另立数值变更。Pinhole 使用 float OpenCV K/D、五项畸变（第五项固定 0）、原 map 类型/插值、原缩放截断；Equidistant 仍用五次定点迭代及原多项式顺序。

参考 vikit 带 native / 数学优化编译标志，本包不引入这些标志。先按严格输出零容差对照；若出现差异，追到具体算子/编译约定，记录原因，不能静默放宽门槛或改变估计公式。

## 文件与接口

- `include/livo_atlas/camera/camera.h`：Camera 小接口，非虚元数据访问 width/height/scale/fx/fy/cx/cy；虚 `Project(const Eigen::Vector2d&)`、`Unproject(const Eigen::Vector2d&)`；普通 `Project(Vector3d)` 做原 x/z、y/z，`Unproject(u,v)` 转发，`IsInFrame(pixel,border,level=0)` 保留整数边界。析构虚；不持 ROS/vikit 对象。
- `pinhole_camera.h/.cc`、`equidistant_camera.h/.cc`：各自原模型；仅 Pinhole 暴露 `UndistortImage(raw, cv::Mat*)`，模型实例拥有自己的映射缓存。元数据与缩放只应用一次。
- `camera_factory.h/.cc`：`std::unique_ptr<Camera> CreateCamera(const CameraConfig&)`，无效配置返回 nullptr；直接模型构造也在浮点转整数或分配前校验。移除旧 vikit factory 和 OwnedVikitCamera 过渡类。
- `image/image_ops.h/.cc`：`float InterpolateGray8(const cv::Mat&, float u, float v)` 与 `float ShiTomasiScore(const cv::Mat&, int u, int v)`；保留原 float 权重/累加顺序和边界规则，支持行步长。插值前提为灰度 8 位且四邻域有效，非法输入报告错误而不越界。
- Frontend、OutputProcessor、Frame、VIO 及现有测试改用上述接口。Pinhole COLMAP 保持原图/pose/点输出；无额外归一化或重采样。
- 核心旧时钟采集、仅用于耗时的计数及终端表格移除；ROS1 Mapper 用已有 steady_clock 对完整 Frontend 调用分别输出 LIO/VIO 耗时标签，保持基线工具可解析的更新次数。OpenMP 仅保留实际并行所需调用，不作为测量时钟。核心错误走普通返回值/异常，不打印 ROS/终端报告。

## 构建与安装

根 CMake 最低 3.16（Noetic 本机版本），缓存字符串 `LIVO_ATLAS_ROS_VERSION` 默认 `1`，支持 `0`（纯 C++）和 `1`（Noetic）；`2` 目前明确报不支持，P4 再实现。`BUILD_TESTING` / `CATKIN_ENABLE_TESTING` 在对应模式控制测试，`BUILD_SHARED_LIBS` 支持 shared/static 核心；默认延续 shared。编译保持默认 ISA，不加 fast-math/native；构建 nice -n 10/-j1，与回放串行。

`cmake/core.cmake` 定义有 livo_atlas 前缀的内部目标和普通公共库；`livo_atlas::core` 聚合前端/同步/预处理，另提供 camera、output、io 命名空间目标。`cmake/ros1.cmake` 独占 catkin、ROS 依赖、转换、参数入口、发布器和原 `fastlivo_mapping` 可执行文件。`cmake/tests.cmake` 共享普通测试清单，ROS 专用测试只在模式 1 注册。不得依赖 catkin 的全局 include/link 路径。

普通依赖为 Eigen、PCL（common/filters/io）、OpenCV（core/imgproc/calib3d/imgcodecs）、当前兼容的 Sophus 和 Threads；OpenMP 可选。尊重 Sophus 配置提供的路径/库，包装成可导出的依赖目标，不再硬写 libSophus.so。不在 vikit 内改 CMake 或数学代码。package.xml 删除 vikit 依赖，ROS1 话题、节点名、launch 和参数保持可用。

模式 0 支持 install/export 与 `find_package(livo_atlas CONFIG REQUIRED)`；Config 查找实际依赖，不保存构建树路径。单独消费工程链接安装后的公开目标，真实推进 LO/LIO/VIO，并检查 ldd / 编译路径无 ROS/vikit。移动安装前缀后再消费，不借用源树或 devel。

## 补齐已登记项

1. 文件流写入/关闭失败明确报告具体文件名，保持原成功输出字节、失败后禁止 Write/Finish 报错。用带路径的私有文本流数据和显式流状态检查即可，不设计 I/O 插件。Linux 用受控临时路径及 /dev/full 或已关闭的本文件描述符重现真实失败；不增加测试专用生产 setter。
2. 两个 writer 使用不同目录和 PCD/图像间隔，交错 LIO/VIO，检查实际文件及位姿时间；保留原尾批行为。
3. ROS1 高频传播原函数静态时间游标改为 Mapper 实例字段，公式和单实例行为不变；真实初始化后的两个 Mapper 相同输入应产生相同传播状态。并发 writer 共用目录仍不支持。

## 验证与交付

新证据 `validation/P3-noetic-20261024/`；P2-03 证据/程序只读复用，先核验源码、运行库、bag、参数和环境摘要。相机及图像工具先从固定版 vikit 采集可迁移的精确/高精度 golden，记录配置、输入、特殊分支与生成器；产品和最终测试不查找 vikit。

测试先行：投影/反投影两模型、开关阈值、缩放截断、边界/坏配置、图像去畸变、非连续灰度插值、评分与边缘；异常文件名；交错 writer；双 Mapper 游标。纯 C++ 的真实 Frontend 要覆盖不经发布的连续 LIO/VIO 和生命周期，不能只编头。

模式 0 shared/static、安装/迁移前缀后独立消费、无 ROS/vikit 的编译和运行依赖检查；模式 1 全量普通测试/实际 ROS/启动验证。运行 3 秒真实残差 gdb、生命周期定向泄漏诊断；控制线程 30 秒与完整 136 秒 Retail_Street 对照，默认线程 45 秒，3 秒 PCD/图像/COLMAP 导出。先使用与 P2-03 相同的数组/消息/产物零容差门槛，数值例外必须有原因、差值和单独裁定。

一次新上下文整阶段评审，按实际影响分级；Critical/Important 一批修复并验证，Minor 明确留档。保存设计、计划、恢复步骤、原始结果和完整历史源码 bundle；保留当前开发分支，不合并或推送。
