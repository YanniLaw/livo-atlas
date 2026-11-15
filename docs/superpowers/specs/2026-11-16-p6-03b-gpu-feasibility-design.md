# P6-03b：离线 GPU 高斯建图可行性验证

2026-11-16，基于 `398da90`。本阶段先验证已有重建观测能否在本机驱动一条可复现的
GPU 高斯渲染、反向传播和小规模训练链路，为后续在线 mapper 选择后端与资源预算提供证据。
它是离线实验，不是在线 3DGS SLAM，也不把实验结果接入前端、ROS 或闭环。

用户认可先写方案与实施计划，正式编码前通知用户以便切换模型。仅修改 livo-atlas；C++ 使用
Google 风格、函数实现之间空一行，保留现有命名，不引入 Pimpl 或通用插件框架。

## 已有能力与本机事实

- `MakeReconstructionView` 输出独立的去畸变 BGR 图像、0/1 图像掩码、稀疏相机 z 深度、
  0/1 深度掩码、OpenCV 整数像素中心内参和 `camera_from_parent`；位姿修正不会改变同一帧
  的局部深度。详见[观测说明](../../reconstruction-view.md)。
- 上一阶段对 Retail_Street 保存的 172 帧做过独立对照：640×512，图像有效区域中稀疏深度
  覆盖率中位数约 4.14%。此前验证及输入清单位于工作区包外；搬到其他电脑要另行复制，
  或在新机重新生成数据集和清单。数据不是绝对深度或轨迹真值。
- 当前本机 NVIDIA RTX 4060 Laptop，显存 8,188 MiB，计算能力 8.9，驱动 535.183.01；
  检查时桌面占用约 547 MiB。Noetic 系统 Python 为 3.8，默认路径没有 `nvcc`，系统
  Python 未发现 PyTorch 和 gsplat。此检查不代表其他 Python 环境中不存在它们。
- Docker 安装有 `nvidia-container-cli`，但当前 Docker 运行时列表没有 `nvidia`，
  既有 Humble 容器未分配 GPU。是否能用新容器访问 GPU 尚未测试，不能假定容器可用。

### 官方预编译包的实际 ABI 检查

候选 [gsplat v1.5.3 发布页](https://github.com/nerfstudio-project/gsplat/releases/tag/v1.5.3)
列出了 Python 3.10、PyTorch 2.3、CUDA 12.1 的 Linux wheel。只读下载并核对官方 SHA-256：

```text
gsplat-1.5.3+pt23cu121-cp310-cp310-linux_x86_64.whl
bb90590f8276b039a6f249afabfdc4b87edda56412d96e19da43ff204f4e6a41
```

对其中 `gsplat/csrc.so` 执行 `readelf --version-info`，得到要求 `GLIBC_2.32` 和
`GLIBCXX_3.4.29`；本机系统分别为 glibc 2.31 和最高 `GLIBCXX_3.4.28`。因此不能把
这个 wheel 视为可直接在本机系统 ABI 下运行的方案。包与检查输出暂存在工作区包外
`validation/P6-03b-design-20261116/`；检查没有加载或运行该二进制。

NVIDIA 的 [CUDA 12.1 发布说明](https://docs.nvidia.com/cuda/archive/12.1.0/cuda-toolkit-release-notes/index.html)
列出 12.1 minor-version compatibility 的最低 Linux 驱动；本机驱动高于该最低值。
这只是筛选条件，不证明具体扩展、PTX、GPU 架构或训练能够运行；
[兼容性说明](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)
也列出了功能和 PTX 限制。最终以实际 CUDA 内核前向、反向和小规模训练为准。

## 方案选择

| 方案 | 收益 | 代价和决定 |
| --- | --- | --- |
| **主机用户目录隔离环境，按固定源码在 Ubuntu 20.04 构建** | 不要求改系统 Python、驱动、ROS 或 Docker 守护进程；可直接访问现有 GPU | 需要匹配的 CUDA 开发工具及本机编译，耗时和内存未知；作为第一候选 |
| 独立 Ubuntu 22.04 GPU 容器及官方预编译 wheel | 可匹配 wheel 的较新系统 ABI | 当前 GPU 容器未验证；仅在已有可用 GPU 容器条件下作为备选，不为本阶段自动改 Docker 配置 |
| 直接并入现有 Gaussian-LIC2 或 GS-LIVO | 能参考完整方法 | 会同时引入前端、训练器及较多环境依赖，不适合先验证当前接口；保留为方法对照 |

第一候选是 [gsplat v1.5.3](https://github.com/nerfstudio-project/gsplat/tree/v1.5.3)
源码提交 `937e29912570c372bed6747a5c9bf85fed877bae`，在用户目录的隔离 Python 3.10 环境中
尝试 PyTorch 2.3.0 CUDA 12.1 构建与 CUDA Toolkit 12.1.1；版本组合参考其
[构建矩阵](https://github.com/nerfstudio-project/gsplat/blob/v1.5.3/.github/workflows/building.yml)
和 [PyTorch 官方旧版本说明](https://pytorch.org/get-started/previous-versions/)。
该组合是**待试验的候选**，不是已经验证的兼容矩阵或长期后端承诺。
当前上游开发分支已提高依赖要求，不能在不复核驱动和 ABI 的情况下直接改用其移动的 `main`。

环境包、第三方源码、编译缓存及大文件放在包外独立实验目录；只安装到用户目录。
普通 C++ 核心、公共头、ROS1/ROS2 和包的默认构建不依赖 Python、CUDA、PyTorch 或 gsplat。
如本机方案失败，记录完整原因并评估已有可用容器或重新选版本，不自动升级系统驱动、
替换系统库或更改 Docker 守护进程。

## 实验数据流

```text
停止写入的关键帧数据集
        ↓  单独构建的普通 C++ 准备工具（livo_atlas::io）
12 个拥有数据的观测文件 + 实验清单
        ↓  独立 Python/GPU 进程
输入检查 → 合成几何冒烟 → 固定预算离线训练 → 渲染与指标
```

准备工具链接安装后的 `livo_atlas::io`，调用 `MakeReconstructionView`，不在算法库
中嵌入 Python，也不依赖 ROS。它只读取停止写入的单个数据集和版本 0 原始 odom 位姿。
路径、视图 ID、资源上限由命令行给定；源文件不修改。输出必须是数据集目录之外的新目录，
检查输入、写入和关闭错误，完整清单最后写入；失败不得被当作完整结果。

这是**实验内部文件格式**，不是新的稳定公共数据协议：

- 清单为 JSON，记录 schema 1、数据来源摘要、每个关键帧 ID、train/validation 分组、
  图像尺寸、`fx/fy/cx/cy`、行优先 `camera_from_parent` 4×4 矩阵、父系和位姿版本。
  ID/版本用十进制字符串保存；状态、点云、图像时间和前端逆曝光估计采用有限 double，
  写入和读取禁止静默更改单位或用逆曝光估计直接修正颜色。
- `image.png` 由 BGR `cv::Mat` 写出为彩色 PNG；Python 使用 Pillow 按 RGB 读取并转换到
  `float32 / 255`，不要再次按 BGR 翻转。两张灰度 PNG 掩码值保持 0 或 1。
- `depth.f32le` 为逐行连续、小端 IEEE-754 float32，单位米；像素数量必须与宽高相符。
  读取方拒绝缺文件、错误大小、非法掩码、非有限值、无效矩阵及不受支持的 schema。
- 像素和点数上限只约束 `MakeReconstructionView` 的单次输入；关键帧读取、整个进程、
  第三方构建和未来队列不受该上限保证。本实验仅处理已经保存、可信的测试数据。

先从当前 172 帧数据集固定选取 ID 0–11：训练 ID 为 0、1、2、4、5、6、8、9、10、11，
留出 ID 为 3、7；不得使用留出帧初始化或训练。只从训练帧有效深度像素反投影，
每帧按固定栅格顺序均匀取至多 1,000 个，总高斯数至多 10,000。反投影使用像素中心，
只近似原连续 LiDAR 点的横向位置；不把这一近似当作真实表面或稠密深度。
先前的逐帧统计显示这 12 帧各有 12,834–13,663 个有效深度像素；真实训练和 alpha 覆盖
仍需实际测量。新机器使用不同数据时应重新记录对应资产和统计。
这里的“留出”只针对高斯初始化及训练：已有位姿由连续视觉惯性前端产生，留出图像可能已影响
其后训练帧的状态与位姿。留出帧指标用于观察固定前端结果下的渲染表现，不是端到端独立测试；
若要评估完全独立的位姿和图像数据，需要另行设计数据准备与前端运行。

`inverse_exposure_time` 是前端估计量，不是硬件曝光时间；第一版不自动乘到颜色上。
真实数据必须记录输入资源哈希、源码提交、参数、训练/留出 ID 和实际所用软件版本。

## 相机、深度和损失

P6-03a 的像素中心为整数。固定版本的 gsplat 栅格内核在第 `(x,y)` 个像素的
`(x+0.5,y+0.5)` 处采样；其投影读取传入内参的主点。因此适配到该版本时：

```text
K_gsplat.fx = camera.fx             K_gsplat.fy = camera.fy
K_gsplat.cx = camera.cx + 0.5       K_gsplat.cy = camera.cy + 0.5
viewmat = camera_from_parent        # 父系到相机，行优先 4×4
```

这一转换依据 [v1.5.3 投影内核](https://raw.githubusercontent.com/nerfstudio-project/gsplat/v1.5.3/gsplat/cuda/csrc/ProjectionEWA3DGSFused.cu)
和[像素采样内核](https://raw.githubusercontent.com/nerfstudio-project/gsplat/v1.5.3/gsplat/cuda/csrc/RasterizeToPixels3DGSFwd.cu)；
不同版本需重新验证。合成测试必须使用已知像素、非单位平移/旋转和不同 RGB 值，
检查投影中心、图像行列、颜色顺序及变换方向，而不是只比较可视化图像。

高斯四元数按 `(w,x,y,z)`；第一版采用单位四元数、固定数量和简单正尺度初始化。
按照 [v1.5.3 渲染接口](https://docs.gsplat.studio/versions/1.5.3/apis/rasterization.html)，
`RGB+ED` 中的 `ED` 是按透明度权重归一化的高斯**期望 z**，不是 LiDAR 最近回波；
它可作为有偏差的稀疏约束，但不能称为深度真值。

- RGB L1 只在 `image_mask == 1` 的像素上计算；空有效区域应拒绝，不能除以零。
- 深度 L1 仅在 `image_mask == 1`、`depth_mask == 1` 且渲染 alpha 至少 0.05 的像素上计算；
  alpha 仅用于选择，不能把无 LiDAR 观测的 0 当作监督。权重先设为 0.05，单位和偏差在报告中明确。
- 如果某帧没有满足条件的深度，跳过该帧深度项并记录数量；若所有训练帧都没有有效深度约束，
  视为本实验未通过。另报原始 LiDAR 有效像素数、参与监督的像素数和 alpha 覆盖比例，防止
  训练通过降低透明度掩盖深度误差。
- 输入、损失、渲染结果和梯度必须有限；颜色和透明度通过有界参数化，尺度保持正值。
  不同深度、非单位位姿、空深度和零 alpha 都有明确测试。

## 实验配置和验收

1. **环境冒烟：** 记录显卡、驱动、Python、CUDA toolkit、PyTorch、gsplat 提交及构建方式；
   `torch.cuda.is_available()`、真实 `rasterization` 前向、非零有限梯度和一次参数更新全部
   成功才算 GPU 路径通过。仅 import 成功或只渲染不能代替反向测试。
2. **合成几何与文件往返：** 用可人工核算的 8×6 图像、已知颜色/深度和非单位 SE(3)，验证
   公开 OpenCV 内参与 gsplat 主点加 0.5 后得到同一像素；检查深度单位、掩码和 RGB。
   分别覆盖零深度/零 alpha、缺文件、错误长度、无效值和输出已存在。
3. **真实短段训练：** 用上述 10 个训练视图与 2 个留出视图，固定随机种子，最多 100 次
   单视图更新，不进行 densification、剪枝、地图扩展或多 GPU。固定初始和最终评估相同
   训练视图的掩码 RGB L1；作为本机可行性门槛，最终值必须比初始下降至少 1%，并有有限
   参数更新及至少一个有效深度监督像素。留出帧记录初始/最终渲染、RGB 指标、覆盖率和
   误差，不预设它一定改善，也不声称地图精度。
4. **成本与边界：** 分别记录准备时间、首次加载/可能的编译、预热后每步 CPU 墙钟及
   CUDA 同步计时、实验进程 PyTorch allocated/reserved 峰值与系统 GPU 使用量；说明其他
   进程也占用显存。以保守单进程显存分配上限、有限步数和单线程构建执行；资源不足、
   内核不可用或指标失败时保留日志并明确失败，不改写成功报告。
5. **回归：** CPU 工具独立链接安装 SDK，核心测试通过；默认构建和 ROS 不新增 GPU 依赖。
   本阶段不改变 ROS 路径，不以离线训练替代在线 bag 验收。

初始资源预算：单次 GPU 训练最多 100 步及 300 秒训练墙钟时间；每次只使用一个 GPU，
PyTorch 分配上限为设备总显存的 40%（该设置不约束所有 CUDA/驱动分配），起跑前检查
至少 4 GiB 可用显存；构建 `MAX_JOBS=1`、`OMP_NUM_THREADS=1`。
剩余磁盘空间低于 10 GiB 时不安装或编译，已存在的实验输出不得覆盖。上述值是安全起点，
不是吞吐或显存保证；实际资源不足时报告失败并记录实测值。

## 阶段边界与后续

实验成功只证明在指定软硬件和这段数据上能够准备观测、渲染和优化固定规模高斯。
它不证明实时性、长距离重访、在线训练、闭环修正、子地图持久化、长期地图一致性或定位精度。
后续单独设计在线有界队列、子地图归属与活动集、可靠位姿修正、版本/成员修订与过期结果拒绝，
再进行 ROS1/ROS2 在线回放和 GPU 资源测量。更换后端、升级驱动或处理其他平台需重新做兼容测试。

## 设计检查

独立实施者检查了相机约定、训练/留出数据、SDK 交接和资源边界。检查指出：留出图像可能已
影响此前运行的视觉惯性前端位姿，因此文档明确将留出结论限制在固定前端结果下的高斯训练。
检查者未提出其他阻碍编写实施计划的重要问题；这不是 GPU 运行验收。
