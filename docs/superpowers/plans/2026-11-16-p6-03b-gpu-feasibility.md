# P6-03b GPU Feasibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in the current session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变默认核心与 ROS 依赖的前提下，用保存的关键帧验证真实 GPU 高斯渲染、反向传播及小规模离线训练。

**Architecture:** 独立 C++ 小工具使用已安装的 `livo_atlas::io` 导出实验观测；独立 Python 进程在用户目录环境中使用固定版本 gsplat 消费这些观测。两者只通过一次性文件交接，不引入进程通信框架或在线 mapper。

**Tech Stack:** C++17、CMake、Eigen、PCL、OpenCV、Python 3.10、NumPy、Pillow、PyTorch 2.3.0 CUDA 12.1、gsplat v1.5.3、CUDA Toolkit 12.1.1。

**Spec:** [P6-03b 离线 GPU 高斯建图可行性设计](../specs/2026-11-16-p6-03b-gpu-feasibility-design.md)。

## Global Constraints

- 本计划当前仅供审阅；用户要求正式编码前先通知，以便切换模型。实际执行由用户继续指示后开始。
- 只修改 livo-atlas；Google C++17，函数实现之间空一行，不使用 Pimpl，现有 ROS1/ROS2 入口与默认构建不依赖 GPU。
- 数据集只读，输出必须是数据集之外的新目录；实验包、第三方源码、环境、缓存和大产物放包外，不覆盖既有证据。
- 固定训练 ID `0,1,2,4,5,6,8,9,10,11`，留出 `3,7`；留出仅适用于高斯初始化与训练，不是前端端到端独立测试。
- 输入图像为 640×512；只用训练帧有效深度反投影，每帧最多 1,000 点，总计最多 10,000；图像/深度掩码均为 0/1。
- `MakeReconstructionView` 内参使用整数像素中心；调用固定版本 gsplat 前 `cx/cy` 各加 0.5；`viewmat` 为 `camera_from_parent`。
- RGB L1 使用图像掩码；深度使用 `RGB+ED` 和 `image_mask && depth_mask && alpha >= 0.05`，权重 0.05，无深度时不除零。
- 单进程、单 GPU，训练最多 100 步、300 秒；PyTorch 分配比例 40%，起跑可用显存至少 4 GiB；编译 `MAX_JOBS=1`、`OMP_NUM_THREADS=1`、`nice -n 10`。
- 开始安装或编译前检查实验磁盘至少剩余 10 GiB；保留系统驱动、系统库、系统 Python、Docker 配置和原始数据。
- 数据准备仅为可信的离线数据；单帧限制不是全进程字节上限；实验文件不是稳定公共协议。
- 不引入 densification、在线队列、闭环修正、ROS 话题或训练结果持久化格式；失败必须记录为失败。

## File Structure

- `tools/3dgs_probe/rasterizer.py`：固定版本的内参适配和一次 gsplat 前向调用。
- `tools/3dgs_probe/prepare_views.cc`：SDK 消费工具，读取版本 0 快照并生成有限观测文件。
- `tools/3dgs_probe/CMakeLists.txt`：独立构建 `livo_atlas_prepare_3dgs_views`，不改主 CMake。
- `tools/3dgs_probe/bundle.py`：读取实验清单、PNG、掩码及小端深度，校验数据。
- `tools/3dgs_probe/train.py`：固定数量高斯初始化、掩码损失、受限训练和报告。
- `tests/3dgs_probe/test_rasterizer.py`：GPU 合成几何、颜色、深度及梯度。
- `tests/3dgs_probe/make_fixture.cc`、`tests/3dgs_probe/test_bundle.py`：小型真实 SDK 数据集与文件往返。
- `tests/3dgs_probe/test_train.py`：掩码损失、留出隔离和确定性种子。
- `docs/p6-03b-gpu-probe.md`、`docs/p6-03b-validation.md`、`docs/README.md`：使用方法、版本、复现、结果与范围。
- 实验环境和输出：新的 `$LIVO_PHASE` 目录；不纳入版本库。

## Review Focus

- gsplat 采样中心、行列顺序及父系到相机方向：Task 1 用非单位 SE(3) 的指定像素测试。
- PNG 解码后的 RGB 与源 BGR：Task 1 测 RGB 比例，Task 2 从 SDK 文件往返测具体颜色。
- 深度空掩码、零 alpha、无观测的 0：Task 3 测零损失和有限梯度，真实实验报告有效数量。
- 留出帧影响初始化或训练：Task 3 改动留出图像后比较种子和训练调度，真实评估固定留出 ID。
- 文件损坏、输出覆盖、资源不足或无 GPU：Task 1/2/3 要求明确失败，验收不把 skipped 测试当通过。

## Shared Experiment Contract

调用前设置绝对路径：`LIVO_REPO` 指向包根，`LIVO_PHASE` 指向新建的包外实验目录，
`LIVO_DATASET` 指向停止写入的 keyframes 目录，`LIVO_SDK` 指向独立 SDK 安装前缀；
`LIVO_CORE_BUILD` 是该 SDK 的构建目录。新机器按 `docs/core-build.md` 新建 SDK。
先确认 `$LIVO_PHASE` 不存在并创建该目录，进入 `$LIVO_REPO`，再运行 `nvidia-smi`、
`df -h "$LIVO_PHASE"`，并确认输入 ID 0–11 存在。
在 Task 1 创建后令 `LIVO_ENV="$LIVO_PHASE/env"`、`LIVO_PYTHON="$LIVO_ENV/bin/python"`；
其余命令均显式使用该解释器，不依赖 shell 激活或系统 Python。

观测目录包含 `bundle.json` 和 `views/<id>/{image.png,image_mask.png,depth_mask.png,depth.f32le}`。
JSON 顶层 `schema=1`、`source_manifest_sha256` 和 `views` 数组；每个视图记录：

- `keyframe_id`、`calibration_id`、`pose_version`、`frame_count`、`optimized_frame_count`：
  十进制字符串；`parent_frame="odom"`、`split="train"|"validation"`。
- `state_timestamp`、`cloud_timestamp`、`image_timestamp`：有限秒数；
  `inverse_exposure_time` 为有限前端估计，不用于颜色修正；`width`、`height` 为正整数；
  `fx`、`fy`、`cx`、`cy` 为有限 double，焦距为正，主点使用 OpenCV 约定。
- `camera_from_parent` 为 16 个行优先数值；四个相对文件名分别为 `image`、`image_mask`、
  `depth_mask`、`depth_m`。深度为 `width*height*4` 字节小端 float32，PNG 图像用 Pillow RGB 读入。

bundle 中 ID 唯一，train 与 validation 均非空且不重叠；拒绝越界相对路径、非有限值、
错误矩阵、错误掩码、尺寸或长度。深度掩码必须是图像掩码的子集，掩码为 1 时深度有限且
为正，为 0 时深度为 0。先校验读取内容再转 GPU；输出目录已存在时失败，
文件全部成功后才发布 `bundle.json`。
`source_manifest_sha256` 标识本次所列资产清单，不是跨会话全局 ID；不同数据集的关键帧 ID
不能仅凭数值合并。

---

### Task 1: 隔离环境与合成 GPU 几何

**Files:** Create `tools/3dgs_probe/rasterizer.py`、`tests/3dgs_probe/test_rasterizer.py`；环境和日志放 `$LIVO_PHASE`。

**Interfaces:** `to_gsplat_camera(K: numpy.ndarray, camera_from_parent: numpy.ndarray) -> tuple[numpy.ndarray, numpy.ndarray]`
接收 3×3 OpenCV `K` 与 4×4 父系到相机变换，返回带 0.5 偏移的 3×3 `K` 和相同方向的 4×4
变换；`render(means, quats, scales, opacities, colors, viewmat, K_gsplat, width, height)` 调用
`gsplat.rasterization(..., render_mode="RGB+ED")`，返回 RGB、alpha 和相机 z 深度 Tensor。

- [ ] **Step 1: 新建环境。** 在新 `$LIVO_PHASE` 使用官方发布的 micromamba `2.9.0-0`
  Linux x86_64 二进制，下载后核对 SHA-256
  `366cd9cd8be14df1ab8ed50352a82111082a36686b2d389fdb79a92c3fafb3e3`。
  按 NVIDIA `nvidia/label/cuda-12.1.1` 创建仅用户目录可写的 Python 3.10、CUDA Toolkit
  12.1.1 环境；用 PyTorch 官方 `cu121` 索引安装 `torch==2.3.0`，另安装 NumPy 1.26.4、
  Pillow 10.4.0、ninja 和 gsplat v1.5.3 的普通依赖。把完整 conda 显式清单、pip freeze、
  下载来源和哈希写到实验目录，不改 shell 启动文件。

  ```bash
  mkdir -p "$LIVO_PHASE/bin"
  curl --fail --location --output "$LIVO_PHASE/bin/micromamba" \
    https://github.com/mamba-org/micromamba-releases/releases/download/2.9.0-0/micromamba-linux-64
  printf '%s  %s\n' 366cd9cd8be14df1ab8ed50352a82111082a36686b2d389fdb79a92c3fafb3e3 \
    "$LIVO_PHASE/bin/micromamba" | sha256sum --check -
  chmod +x "$LIVO_PHASE/bin/micromamba"
  export MAMBA_ROOT_PREFIX="$LIVO_PHASE/mamba-root"
  "$LIVO_PHASE/bin/micromamba" create -y -p "$LIVO_ENV" -c conda-forge python=3.10 pip
  "$LIVO_PHASE/bin/micromamba" install -y -p "$LIVO_ENV" \
    -c nvidia/label/cuda-12.1.1 -c conda-forge cuda-toolkit=12.1.1
  "$LIVO_PYTHON" -m pip install --index-url https://download.pytorch.org/whl/cu121 torch==2.3.0
  "$LIVO_PYTHON" -m pip install numpy==1.26.4 Pillow==10.4.0 ninja jaxtyping rich wheel
  ```

  Expected: 每一步成功，`$LIVO_ENV/bin/nvcc --version` 显示 12.1；实际解析到的全部版本
  后续通过 `micromamba list --explicit` 和 `pip freeze --all` 固化。创建环境和安装前均复查磁盘。

- [ ] **Step 2: 本机编译固定源码。** 克隆 gsplat tag `v1.5.3` 和其递归 submodule，核对
  提交 `937e29912570c372bed6747a5c9bf85fed877bae`。设置 `CUDA_HOME` 指向隔离 toolkit、
  `TORCH_CUDA_ARCH_LIST=8.9`，单任务编译并安装到该环境。记录编译日志、submodule SHA、
  `nvcc --version`、gcc、Python、torch 和 CUDA 版本。若下载、ABI、CUDA 或磁盘检查失败，
  保存完整错误并停止本阶段；不要更改系统驱动或静默改用 CPU。

  ```bash
  git clone --depth 1 --branch v1.5.3 --recurse-submodules \
    https://github.com/nerfstudio-project/gsplat.git "$LIVO_PHASE/gsplat-src"
  test "$(git -C "$LIVO_PHASE/gsplat-src" rev-parse HEAD)" = \
    937e29912570c372bed6747a5c9bf85fed877bae
  git -C "$LIVO_PHASE/gsplat-src" submodule status --recursive > "$LIVO_PHASE/gsplat-submodules.txt"
  env CUDA_HOME="$LIVO_ENV" PATH="$LIVO_ENV/bin:$PATH" TORCH_CUDA_ARCH_LIST=8.9 \
    MAX_JOBS=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 10 \
    "$LIVO_PYTHON" -m pip install --no-build-isolation --no-deps -v \
    "$LIVO_PHASE/gsplat-src" > "$LIVO_PHASE/gsplat-build.log" 2>&1
  "$LIVO_PYTHON" -c 'import torch, gsplat; print(torch.__version__, torch.version.cuda); assert torch.cuda.is_available(), "CUDA unavailable"'
  "$LIVO_PHASE/bin/micromamba" list -p "$LIVO_ENV" --explicit > "$LIVO_PHASE/conda-explicit.txt"
  "$LIVO_PYTHON" -m pip freeze --all > "$LIVO_PHASE/pip-freeze.txt"
  ```

  Expected: 构建退出码为 0，CUDA 可用断言通过；实际内核是否可执行由 Step 5 判定。
  `8.9` 对应本机计算能力，其他 GPU 要先实测其架构并使用该 toolkit 支持的值。

- [ ] **Step 3: 写 RED 测试。** 合成图像 8×6、`fx=fy=4`、OpenCV `cx=3.5,cy=2.5`；
  `T_C_parent` 使用 z 轴 90° 旋转与平移 `(1,2,3)`，父系点 `(-1.5,0.5,1)` 对应相机
  点 `(0.5,0.5,4)` 和整数像素 `(4,3)`。一个单位四元数、正尺度、非灰色 RGB 高斯应在
  该像素有非零 alpha 且严格高于左侧和上方邻点，`ED≈4 m`；除以 alpha 后 RGB 顺序正确。
  对有意不同的目标计算损失并反传，至少一个颜色参数梯度有限非零且一次优化改变参数。
  在运行任何 GPU 测试前检查 4 GiB 可用显存并设置 PyTorch 40% 分配比例。
  `LIVO_REQUIRE_GPU=1` 时没有 CUDA 或 gsplat 内核必须报错，不能 skip 为成功。

- [ ] **Step 4: 运行 RED。** 使用隔离环境 Python 运行
  `LIVO_REQUIRE_GPU=1 PYTHONPATH="$LIVO_REPO/tools/3dgs_probe" "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_rasterizer.py -v`。
  Expected: 因适配实现尚不存在而失败，保留完整日志；环境加载失败要先诊断，不算预期 RED。

- [ ] **Step 5: 实现并运行 GREEN。** 实现 Interfaces 中的两个简单函数，保持图像宽高和
  颜色原样，主点仅在 gsplat 边界加 0.5；`render` 返回前验证形状和有限值。运行 Step 4。
  Expected: 实际 CUDA 前向/反向及像素、颜色、米制深度断言全部通过，记录 GPU 显存和耗时。

- [ ] **Step 6: 提交。** 仅提交本任务的源文件、测试与必要的依赖说明；环境和日志留在
  `$LIVO_PHASE`。提交示例：`test: verify isolated Gaussian rasterization`。

### Task 2: SDK 观测数据交接

**Files:** Create `tools/3dgs_probe/prepare_views.cc`、`tools/3dgs_probe/CMakeLists.txt`、
`tools/3dgs_probe/bundle.py`、`tests/3dgs_probe/make_fixture.cc`、`tests/3dgs_probe/test_bundle.py`。

**Interfaces:** 独立可执行文件
`livo_atlas_prepare_3dgs_views --dataset DIR --output NEW_DIR --train-ids CSV --validation-ids CSV --source-sha256 HEX --max-pixels N --max-cloud-points N`；
`bundle.load_bundle(path: pathlib.Path) -> list[dict]` 读取 Shared Experiment Contract 并在错误时抛
`ValueError` 或 `OSError`；返回项包含 RGB、两张掩码、米制深度、内参、`camera_from_parent` 和 split。

- [ ] **Step 1: 写 RED 测试。** fixture 使用 `KeyframeStore` 写两个 8×6 BGR 关键帧，
  相机 `fx=fy=4,cx=3.5,cy=2.5`；原始 body 位姿为单位矩阵，`camera_from_odom` 为
  Task 1 的非单位变换，点 `(-1.5,0.5,1)`，BGR 像素 `(10,20,30)`。黑盒运行准备工具
  （train 0、validation 1），通过 Python 验证 `RGB=(30,20,10)`、`depth[3,4]=4`、
  对应深度掩码为 1、OpenCV 内参及 4×4 变换精确保持。验证两组 ID 不重叠。
  对已有输出、输出在数据集内（含符号链接别名）、缺文件、错误字节长度、非法掩码及非有限矩阵分别要求失败。
  先创建独立 CMake 的测试脚手架，仅构建 `p603b_make_fixture`；`prepare_views.cc`、
  `bundle.py` 和正式工具目标在 Step 3 添加。

- [ ] **Step 2: 运行 RED。** 单独配置 `cmake -S tools/3dgs_probe -B "$LIVO_PHASE/prepare-build" -DCMAKE_PREFIX_PATH="$LIVO_SDK" -DBUILD_TESTING=ON`，
  单独构建 `p603b_make_fixture`。设置 `LIVO_FIXTURE_BIN` 为该 fixture 路径、
  `LIVO_PREPARE_BIN` 为未来的工具路径并导出这两个变量，然后以隔离解释器运行：
  `PYTHONPATH="$LIVO_REPO/tools/3dgs_probe" "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_bundle.py -v`。
  Expected: 因 `bundle` 接口尚不存在而失败；fixture 使用临时目录，不访问真实 bag。
  如果 SDK 启用了 GTSAM，为该 CMake 配置另传实际 `GTSAM_DIR`。

- [ ] **Step 3: 实现。** CMake 只在独立工程中 `find_package(livo_atlas CONFIG REQUIRED)`，
  添加正式工具目标并链接 `livo_atlas::io`，按源数据集的完整原始位姿前沿构造版本 0 `PoseSnapshot`。
  先对所选帧调用 `MakeReconstructionView` 再写图片、掩码和显式小端 IEEE-754 float；
  不直接假设宿主字节序，不支持的 float 表示明确报错。JSON 固定键、
  classic locale、17 位有效数字，64 位 ID 使用字符串。只在全部文件成功后发布清单；
  `bundle.py` 用 Pillow、NumPy 和标准库检查 Shared Experiment Contract。

- [ ] **Step 4: 运行 GREEN。** 重建 fixture 与工具，运行 `test_bundle.py`；通过 `ldd` 检查
  该 SDK 消费工具没有 ROS、vikit、CUDA、torch 或缺失库。Expected: 文件往返、错误边界及
  拒绝覆盖全部通过。实际读取器本身和整个进程不声称受视图点数上限约束。

- [ ] **Step 5: 生成真实输入。** 对 `dataset.yml` 和帧 0–11 的 `frame.yml`、`cloud.pcd`、
  `image.png` 写 SHA-256 清单并验证，清单自身的 SHA-256 传给 `--source-sha256`。
  用 `--max-pixels 1000000 --max-cloud-points 100000` 生成新的 12 帧 bundle。
  Expected: 10 个 train、2 个 validation，所有图像和深度数据有效；再次验证源文件哈希未改变。

  ```bash
  (cd "$LIVO_DATASET" && sha256sum dataset.yml frames/{0..11}/{frame.yml,cloud.pcd,image.png}) \
    > "$LIVO_PHASE/source.sha256"
  (cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
  LIVO_SOURCE_SHA=$(sha256sum "$LIVO_PHASE/source.sha256" | cut -c1-64)
  "$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views" \
    --dataset "$LIVO_DATASET" --output "$LIVO_PHASE/bundle" \
    --train-ids 0,1,2,4,5,6,8,9,10,11 --validation-ids 3,7 \
    --source-sha256 "$LIVO_SOURCE_SHA" --max-pixels 1000000 --max-cloud-points 100000
  (cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
  ```

- [ ] **Step 6: 提交。** 提交本任务工具、测试和最少量使用说明；不提交数据或实验 bundle。
  提交示例：`feat: prepare isolated Gaussian probe views`。

### Task 3: 固定规模离线训练及验收记录

**Files:** Create `tools/3dgs_probe/train.py`、`tests/3dgs_probe/test_train.py`、
`docs/p6-03b-gpu-probe.md`、`docs/p6-03b-validation.md`；modify `docs/README.md`。

**Interfaces:** `initialize_gaussians(train_views: list[dict], max_per_view: int) -> dict` 返回父坐标
高斯中心、单位 `(w,x,y,z)`、正尺度、颜色和透明度；`masked_losses(rgb, depth, alpha, view)`
返回 RGB L1、带 alpha 选择的深度 L1 与有效深度数量；CLI
`train.py --bundle DIR --output NEW_DIR --steps 100 --seed 0` 使用 Task 1 的 `render`。

- [ ] **Step 1: 写 RED 测试。** 断言只从 train 帧生成种子；把 validation 图像完全改色后，
  初始化点/颜色及训练采样顺序不变。检查有效深度位置反投影后的父系坐标，
  小于 1,000 个和超过 1,000 个时采样结果确定且不超过上限。无深度或 alpha 全 0 时深度
  数量为 0、损失有限且 RGB 仍可反传；纯黑/纯白输入初始化必须有限，无图像有效像素必须报错。

- [ ] **Step 2: 运行 RED。** `PYTHONPATH="$LIVO_REPO/tools/3dgs_probe" "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_train.py -v`。
  Expected: 初始化/损失接口尚不存在而失败；不得将缺依赖误作预期失败。

- [ ] **Step 3: 实现并 GREEN。** 有效像素按行优先索引，取 `N=min(M,1000)`、
  `index_i=floor(i*M/N)`；用 OpenCV `K` 与 `inverse(camera_from_parent)` 反投影。
  每次渲染前通过 Task 1 的 `to_gsplat_camera` 将清单里的 OpenCV `K` 转成 `K_gsplat`，
  不修改或重复偏移原始内参。
  初始尺度 0.03 m、透明度 0.5、颜色取训练像素 RGB；四元数固定单位旋转。采用 sigmoid
  颜色/透明度和指数尺度，颜色转 logit 前夹到 `[1e-4,1-1e-4]`，优化前后检查有限值；
  Adam 分别对均值、log 尺度、颜色和透明度
  使用 `5e-4, 2e-3, 1e-2, 1e-2` 学习率，尺度限制在 `[0.001,0.2]` m。
  将 train ID 升序排列并以 `step % train_count` 循环取样，100 步上限；
  `masked_losses` 的 RGB 项为有效像素三个通道的绝对误差和除以 `3*有效像素数`；深度项为
  选中像素的米制绝对误差均值，空集合返回 0 和计数 0；合计为 `RGB + 0.05*depth`。
  运行 Step 2
  和 Task 1 的 GPU 合成测试，Expected: 全部通过。

- [ ] **Step 4: 真实离线实验。** 源清单校验通过、`torch.cuda.mem_get_info()` 可用显存至少
  4 GiB 后运行 CLI；设置 `torch.cuda.set_per_process_memory_fraction(0.4)`、单线程和 300 秒
  训练墙钟上限。保存软件/驱动版本、源码与输入哈希、固定配置、实际步数、完整日志、
  训练前后每视图 RGB L1、2 个留出帧训练前后 RGB 渲染、alpha 覆盖及深度监督数量。
  单独记录首次调用时间、预热后 `torch.cuda.synchronize()` 的墙钟、CUDA Event 时间、
  allocated/reserved 峰值及系统 GPU 使用量。Expected: 所有结果和梯度有限，至少一个有效
  深度监督像素、至少一个参数真实更新；同一批 train 视图按相同掩码逐帧平均的 RGB L1 比初始降低至少
  1%。未达到时原样报告失败，留出帧结果只用于观察，不作为必然改善的验收门槛。

- [ ] **Step 5: 回归和文档。** 重新核验源 SHA-256；运行两个 Python 测试集和 SDK
  `cmake -E chdir "$LIVO_CORE_BUILD" ctest --output-on-failure --timeout 30 -j1`，确认实际
  测试数非零且全部通过；检查产品库及准备工具未链接 GPU/vikit。记录两种操作系统依赖
  边界、固定版本、环境恢复、试验图像/指标和失败限制；不把离线结果称为实时 3DGS SLAM。

- [ ] **Step 6: 提交与整阶段审阅。** 提交本任务源码、测试和文档；完成任务检查后按
  `executing-plans` 对整个 P6-03b 范围做一次独立审阅，修复影响正确性的发现并记录证据。
  提交示例：`feat: validate offline Gaussian optimization`。

## Self-review

任务 1 独立确定真实 GPU 和坐标约定；任务 2 在安装 SDK 与 Python 之间定义且验证唯一文件契约；
任务 3 消费相同契约并记录可重复的成功或失败。在线消息分发、队列、子地图更新和 SLAM 反馈不在本计划中。
