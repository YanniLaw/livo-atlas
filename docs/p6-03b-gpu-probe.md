# P6-03b 离线 GPU 高斯实验

这个实验验证已保存的关键帧观测能否驱动真实 GPU 高斯渲染、反向传播和固定规模优化。
工具独立于默认构建与 ROS 节点；C++ 准备程序只消费已安装的 `livo_atlas::io`，Python
和 GPU 依赖只属于单独运行的实验。它不是在线 3DGS SLAM，也没有接入闭环或在线地图。
本机结果见[验证记录](p6-03b-validation.md)。

## 环境边界

已在 Ubuntu 20.04、ROS1 Noetic 所在主机的 Linux x86_64 环境验证；工具运行时不需要
启动 ROS。Ubuntu 22.04 / ROS2 Humble 上，普通 SDK 的构建和消费方式仍适用，但本次没有
在该环境验证 CUDA、gsplat 编译或训练，不能直接视为已支持。其他 GPU、驱动、操作系统或
不同的 gsplat 版本必须重新验证。本机 CUDA 12.1、PyTorch 2.3.0+cu121 和 gsplat 1.5.3
的实际组合与驱动版本见验证记录。

需事先准备独立 C++ SDK，见[核心构建](core-build.md)；若该 SDK 启用了 GTSAM，配置
准备工具时还须提供对应的 `GTSAM_DIR`，必要时提供 `Sophus_DIR`。另需支持 C++17 的编译器、
CMake、Git、NVIDIA 驱动及可访问 GPU。以下环境清单只针对 **Linux x86_64**：

- [conda 清单](../tools/3dgs_probe/conda-linux-64-explicit.txt)固定本机 29 个 conda 包的
  URL 与 conda 元数据中的 MD5，用于恢复该层，不是安全签名。
- [pip 清单](../tools/3dgs_probe/requirements-cu121.txt)固定本机 Python 包版本；
  gsplat 必须单独按下述源码提交构建。可用软件源和驱动随电脑变化，固定版本不保证其他机器兼容。
- 本机留下的原始 `pip freeze` 包含本地 wheel 路径；最初的 freeze 还继承了 Noetic 的
  `PYTHONPATH`。它们是运行证据，不能直接作为跨电脑安装清单。

## 在另一台机器准备环境

在包根目录执行，并将下面三个路径替换为实际值：SDK 安装目录、已经停止写入的数据集、
包外尚不存在的实验目录。下面各段依次在 Bash 脚本中运行，出错时停止。实验会下载和编译
第三方软件；每次安装或编译前确认所在分区至少剩余 10 GiB。不要将环境、数据或实验结果
放进源码仓库。

```bash
set -euo pipefail
export LIVO_REPO="$PWD"
export LIVO_SDK=/path/to/sdk-install
export LIVO_DATASET=/path/to/keyframes
export LIVO_PHASE=/path/to/new-experiment-directory
test ! -e "$LIVO_PHASE" && test ! -L "$LIVO_PHASE" || exit 1
mkdir -p "$LIVO_PHASE/bin"
df -h "$LIVO_PHASE"
test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
nvidia-smi
export LIVO_ENV="$LIVO_PHASE/env"
export LIVO_PYTHON="$LIVO_ENV/bin/python"
export MAMBA_ROOT_PREFIX="$LIVO_PHASE/mamba-root"

curl --fail --location --output "$LIVO_PHASE/bin/micromamba" \
  https://github.com/mamba-org/micromamba-releases/releases/download/2.9.0-0/micromamba-linux-64
printf '%s  %s\n' 366cd9cd8be14df1ab8ed50352a82111082a36686b2d389fdb79a92c3fafb3e3 \
  "$LIVO_PHASE/bin/micromamba" | sha256sum --check -
chmod +x "$LIVO_PHASE/bin/micromamba"
"$LIVO_PHASE/bin/micromamba" create --no-rc -y -p "$LIVO_ENV" \
  --file "$LIVO_REPO/tools/3dgs_probe/conda-linux-64-explicit.txt"
df -h "$LIVO_PHASE"
test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
env -u PYTHONPATH PYTHONNOUSERSITE=1 PIP_CONFIG_FILE=/dev/null \
  "$LIVO_PYTHON" -m pip install --no-cache-dir \
  --index-url https://pypi.org/simple --extra-index-url https://download.pytorch.org/whl/cu121 \
  -r "$LIVO_REPO/tools/3dgs_probe/requirements-cu121.txt"
```

上述 micromamba 文件是 Linux x86_64 的 2.9.0-0 发布文件；更换二进制不能沿用该哈希。
本机缓存的 PyTorch wheel SHA-256 为
`0a12aa9aa6bc442dff8823ac8b48d991fd0771562eaa38593f9c8196d65f7007`，
Triton 2.3.0 Linux wheel 为
`5ce4b8ff70c48e47274c66f269cce8861cf1dc347ceeb7a67414ca151b1822d8`；
它们仅对应本机具体 wheel 文件，不适用于其他平台或文件名。

再次检查磁盘后，构建固定 gsplat 源码。`8.9` 是本机显卡的计算能力；其他设备先确认该
toolkit 支持的实际架构，再自行替换并重新验证。以下只在实验环境安装，不改系统驱动。

```bash
df -h "$LIVO_PHASE"
test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
git clone --depth 1 --branch v1.5.3 --recurse-submodules \
  https://github.com/nerfstudio-project/gsplat.git "$LIVO_PHASE/gsplat-src"
test "$(git -C "$LIVO_PHASE/gsplat-src" rev-parse HEAD)" = \
  937e29912570c372bed6747a5c9bf85fed877bae
git -C "$LIVO_PHASE/gsplat-src" submodule status --recursive
test "$(git -C "$LIVO_PHASE/gsplat-src/gsplat/cuda/csrc/third_party/glm" rev-parse HEAD)" = \
  33b4a621a697a305bc3a7610d290677b96beb181
env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_HOME="$LIVO_ENV" \
  PATH="$LIVO_ENV/bin:$PATH" TORCH_CUDA_ARCH_LIST=8.9 MAX_JOBS=1 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 10 \
  "$LIVO_PYTHON" -m pip install --no-build-isolation --no-deps -v \
  "$LIVO_PHASE/gsplat-src" > "$LIVO_PHASE/gsplat-build.log" 2>&1
env -u PYTHONPATH PYTHONNOUSERSITE=1 "$LIVO_PYTHON" -m pip check
```

源码、子模块、wheel、编译输出和环境版本均应留作证据。GPU 算法可能有微小数值波动；
相同种子及输入选择不保证不同运行或设备逐字节一致。

## 生成观测并训练

从包根目录单独配置准备工具。下面命令适用于能从 SDK 自动找到依赖的情况；若提示找不到
GTSAM 或 Sophus，按安装位置给 CMake 增加相应的 `-D..._DIR=...`。

```bash
df -h "$LIVO_PHASE"
test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
cmake -S tools/3dgs_probe -B "$LIVO_PHASE/prepare-build" \
  -DCMAKE_PREFIX_PATH="$LIVO_SDK" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 10 \
  cmake --build "$LIVO_PHASE/prepare-build" -- -j1

(cd "$LIVO_DATASET" && sha256sum dataset.yml frames/{0..11}/{frame.yml,cloud.pcd,image.png}) \
  > "$LIVO_PHASE/source.sha256"
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
LIVO_SOURCE_SHA=$(sha256sum "$LIVO_PHASE/source.sha256" | cut -c1-64)
"$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views" \
  --dataset "$LIVO_DATASET" --output "$LIVO_PHASE/bundle" \
  --train-ids 0,1,2,4,5,6,8,9,10,11 --validation-ids 3,7 \
  --source-sha256 "$LIVO_SOURCE_SHA" --max-pixels 1000000 --max-cloud-points 100000
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")

env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  timeout --signal=TERM --kill-after=10 330s nice -n 10 \
  "$LIVO_PYTHON" tools/3dgs_probe/train.py \
  --bundle "$LIVO_PHASE/bundle" --output "$LIVO_PHASE/run" --steps 100 --seed 0
(cd "$LIVO_DATASET" && sha256sum --check "$LIVO_PHASE/source.sha256")
cat "$LIVO_PHASE/run/report.json"
```

准备工具和训练器都要求输出目录不存在且与输入目录互不包含。观测包是实验内部临时格式，
不是稳定地图协议：清单最后发布，图像为 RGB PNG，深度为米制小端 float32，掩码值为 0/1，
位姿为 `camera_from_parent`。`source.sha256` 标识所列源文件，准备前后都应核验；准备
工具本身不重新计算该哈希。单帧像素和点数限制只约束观测构造，不能保证整个读取进程的内存上界。

训练器固定 10 个训练帧、2 个留出帧；每训练帧最多 1,000 个初始高斯，总共最多 10,000 个。
它要求起跑时至少 4 GiB 可用显存，将 PyTorch 分配比例限制在总显存的 40%，最多训练 100 步。
该比例不限制驱动或其他进程的全部显存分配。训练前后逐步检查 300 秒时限；单个卡住的 CUDA
调用不能由 Python 安全地即时中断，所以示例用外层 330 秒进程超时兜底，强制终止时可能没有
完整报告。若无 GPU、数值异常或验收不通过，进程返回非零；已建立输出目录后正常捕获的失败
会记录到 `report.json`，不能将不完整输出当成成功。

`report.json` 包含输入清单摘要、源码摘要、软件信息、参数、每视图 RGB L1、PSNR、
alpha 覆盖率、深度监督数、耗时和显存。两个留出帧各保存训练前、训练后与目标 RGB 图像。
深度项只在有效图像/深度且 alpha ≥ 0.05 的像素计算；渲染深度是透明度加权的期望相机 z，
并非 LiDAR 最近回波或真值。留出图像未参与高斯初始化与训练，但前端位姿来自连续运行，
因此留出结果不能作为整个 SLAM 系统的独立测试。

## 测试

```bash
export LIVO_FIXTURE_BIN="$LIVO_PHASE/prepare-build/p603b_make_fixture"
export LIVO_PREPARE_BIN="$LIVO_PHASE/prepare-build/livo_atlas_prepare_3dgs_views"
export PYTHONPATH="$LIVO_REPO/tools/3dgs_probe"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
CUDA_VISIBLE_DEVICES=0 LIVO_REQUIRE_GPU=1 nice -n 10 \
  "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -v
```

`LIVO_REQUIRE_GPU=1` 使缺 GPU 时测试失败而非跳过。建议记录测试日志，并按
[核心构建](core-build.md)运行独立 SDK 的 CTest。实际硬件与数据上的测试结果不能外推为实时性能。
