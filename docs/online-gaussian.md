# P6-03c1 在线 Gaussian 预览

本阶段让独立 GPU 进程读取当前在线子地图的关键帧观测，增量训练一个有界 Gaussian 模型，
并输出带位姿版本的 RGB 预览。前端、回环后端、轨迹和几何输出仍通过同一个 `LivoSystem`
回调工作。Gaussian 不向前端或后端反馈测量，也不保存跨子地图的 Gaussian 地图；它是在线
实验接口，还不是完整的在线 3DGS SLAM。实测环境与结果见[验证记录](p6-03c1-validation.md)。

默认构建和运行均关闭此功能。启用时需要 Linux、已开启的关键帧和 `online` 子地图、一个
单独启动的 worker，以及此前不存在的预览输出目录。ROS1 Noetic 和 ROS2 Humble 使用同一个
`src/app/livo_main.cc` 入口，参数分别使用 `/` 和 `.` 分隔。当前 GPU 训练只在文档记录的
Noetic 主机验证；Humble 已验证构建、配置、共用入口和本机 socket 假 worker，尚未验证真实 GPU。

## 环境与构建

先按[核心构建](core-build.md)准备编译依赖，再按[离线 Gaussian 实验](p6-03b-gpu-probe.md)
创建独立 Python/CUDA 环境并预编译 `gsplat.csrc`。worker 启动时要求 CUDA 可用、至少 4 GiB
空闲显存，并将 PyTorch 的进程分配比例限制为显存总量的 40%；缺少预编译扩展时不会自动
触发 JIT 编译。该限制不能约束驱动、其他进程或所有非 PyTorch 分配。

以下示例在包外放运行数据，并假设当前目录是工作空间根目录。`LIVO_PYTHON`、`Sophus_DIR`
等路径需要按实际电脑替换；如果启用已有回环后端，还须同时开启 GTSAM 和回环编译选项，
并提供对应的 `GTSAM_DIR`。构建前请检查所在分区空间。

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j1 -DLIVO_ATLAS_ROS_VERSION=1 \
  -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON \
  -DSophus_DIR=/path/to/Sophus/build
source devel/setup.bash
```

ROS2 Humble 使用现有工作空间和 `livox_ros_driver2` 环境，例如：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select livo_atlas --parallel-workers 1 --cmake-args \
  -DLIVO_ATLAS_ROS_VERSION=2 -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON \
  -DSophus_DIR=/path/to/Sophus/build
source install/setup.bash
```

两种构建的 `LIVO_ATLAS_WITH_ONLINE_GAUSSIAN` 默认均为 `OFF`。关闭编译选项时，运行时启用
会在创建关键帧和预览目录之前失败。开启编译选项但运行时保持默认关闭，不会启动 worker。

## 启动 worker

先在单独终端启动一个 worker；它只接收一个会话。socket 的父目录必须由当前用户拥有且
只对该用户开放，socket 路径不得已存在。下面的预览目录也必须不存在，且不得包含关键帧
数据集或位于其中；其父目录必须已经存在。Linux Unix socket 的绝对路径最多 107 字节，
路径太长时可把运行目录放在较短的位置。

```bash
export LIVO_REPO="$PWD/src/livo-atlas"
export LIVO_PYTHON=/path/to/3dgs-env/bin/python
export LIVO_RUN=/path/to/new-run-directory
mkdir -m 700 "$LIVO_RUN"
export LIVO_GAUSSIAN_SOCKET="$LIVO_RUN/worker.sock"
export LIVO_GAUSSIAN_OUTPUT="$LIVO_RUN/previews"

env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  "$LIVO_PYTHON" "$LIVO_REPO/tools/3dgs_probe/online_worker.py" \
  --socket "$LIVO_GAUSSIAN_SOCKET"
```

将上述三个路径也设置到节点终端。正常停止节点会结束 worker；socket、GPU、协议或 worker
错误会使 Gaussian 停止，前端继续运行。当前没有自动重连；要重新运行请启动新 worker，
并选择新的预览目录。worker 的错误与节点最终的 `livo_atlas gaussian:` 统计应一起检查。

## ROS1 Noetic

在已启动的 ROS master 上、启动节点之前设置参数。下面以仓库提供的 Avia 标定为例；更换
设备时应使用其实际标定。`LIVO_DATASET_OUTPUT` 为关键帧及其他节点输出所在位置，须与预览
目录分开。

```bash
export LIVO_DATASET_OUTPUT="$LIVO_RUN/node-output"
rosparam load "$LIVO_REPO/config/avia.yaml"
rosparam load "$LIVO_REPO/config/camera_pinhole.yaml" /laserMapping
rosparam set /output/directory "$LIVO_DATASET_OUTPUT"
rosparam set /keyframes/enabled true
rosparam set /submaps/enabled true
rosparam set /submaps/mode online
rosparam set /gaussian/enabled true
rosparam set /gaussian/socket_path "$LIVO_GAUSSIAN_SOCKET"
rosparam set /gaussian/output_directory "$LIVO_GAUSSIAN_OUTPUT"
rosrun livo_atlas fastlivo_mapping
```

使用 bag 时，在启动节点前设置 `/use_sim_time`，再在另一个终端使用 `rosbag play --clock`。
程序在仿真时间暂停时仍使用墙钟轮询 worker。节点结束时先完成后端及其最后的位姿通知，
再停止 Gaussian。

## ROS2 Humble

节点参数必须在启动前提供。以下从现有两份标定文件生成一个 ROS2 参数文件；Python 需要
PyYAML。命令在工作空间根目录运行，使用与 worker 相同的路径变量。

```bash
export LIVO_DATASET_OUTPUT="$LIVO_RUN/node-output"
python3 - <<'PY'
import os
from pathlib import Path
import sys
import yaml

repo = Path(os.environ['LIVO_REPO'])
sys.path.insert(0, str(repo / 'tools'))
from run_ros2_baseline import load_config

parameters = load_config(repo / 'config/avia.yaml')
parameters.update(load_config(repo / 'config/camera_pinhole.yaml'))
parameters.update({
    'output.directory': os.environ['LIVO_DATASET_OUTPUT'],
    'keyframes.enabled': True,
    'submaps.enabled': True,
    'submaps.mode': 'online',
    'gaussian.enabled': True,
    'gaussian.socket_path': os.environ['LIVO_GAUSSIAN_SOCKET'],
    'gaussian.output_directory': os.environ['LIVO_GAUSSIAN_OUTPUT'],
})
Path(os.environ['LIVO_RUN'], 'params.yaml').write_text(
    yaml.safe_dump({'laserMapping': {'ros__parameters': parameters}}))
PY
ros2 run livo_atlas fastlivo_mapping --ros-args --params-file "$LIVO_RUN/params.yaml"
```

对仿真时间输入，在生成参数时加入 `'use_sim_time': True`；ROS2 的处理定时器使用墙钟。
两端的可选回环、全局轨迹和几何输出仍按各自已有参数启用，无需增加第二个节点入口。

## 使用 ROS1 bag 做成对检查

`tools/run_baseline.py` 要求一个与开发目录分开的、已构建的 catkin 源码副本，避免节点运行
产物进入开发源码。下面假定该副本已启用 Gaussian、GTSAM 和回环构建选项，且包根目录是
当前目录；bag 和各输出路径按实际机器修改。比较工具使用 NumPy、PyYAML 和 Pillow。

```bash
export LIVO_WS_COPY=/path/to/isolated-catkin-workspace
export LIVO_BAG=/path/to/Retail_Street.bag
export LIVO_CHECK=/path/to/existing-validation-directory
source /opt/ros/noetic/setup.bash
source "$LIVO_WS_COPY/devel/setup.bash"

OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 \
python3 tools/run_baseline.py --workspace "$LIVO_WS_COPY" --bag "$LIVO_BAG" \
  --output "$LIVO_CHECK/off" --port 11341 --cpu-affinity 0 \
  --submap-mode online --loop-closure --semantic-output --image-queue-size 32 \
  --rate 1 --duration 10

# 此时按前文启动 worker；socket 和预览目录均使用此前不存在的路径。
OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 \
python3 tools/run_baseline.py --workspace "$LIVO_WS_COPY" --bag "$LIVO_BAG" \
  --output "$LIVO_CHECK/on" --port 11342 --cpu-affinity 0 \
  --submap-mode online --loop-closure --semantic-output --image-queue-size 32 \
  --rate 1 --duration 10 --gaussian-socket "$LIVO_GAUSSIAN_SOCKET" \
  --gaussian-output "$LIVO_GAUSSIAN_OUTPUT"

python3 tools/compare_gaussian_replays.py --off "$LIVO_CHECK/off" \
  --on "$LIVO_CHECK/on" --previews "$LIVO_GAUSSIAN_OUTPUT"
```

`--cpu-affinity 0` 只是示例，两次应选用本机允许的相同 CPU。比较工具要求输入、共享参数、
二进制、四组前端数组、发布图像/点云语义及原始关键帧文件完全一致，并检查回环最终结果、
有界统计和播放期间真实训练确认的完整预览；差异会原样报告，不自动调整数值容差。确认短段
后可使用新的输出路径按相同方式回放完整 bag。结果不包含真值评价，也不能替代长距离或
长期连续运行测试。

## 结果与约束

- 每次只训练最近一个主子地图；其封存本身不会废弃模型，切换主子地图或收到新的位姿版本
  会使当前内存预览立即失效，并独立重置 worker 的模型、窗口和 Adam。位姿未变时新增关键帧
  不会使在途候选失效。只有候选确认后、再次核对身份成功的结果才可用。
- 单视图最多 1,000,000 像素和 100,000 云点；位姿前沿最多 512 帧。保留最多 10 个主关键帧，
  每帧最多初始化 1,000 个高斯、共最多 10,000 个；每个候选训练 20 步。输入超限、无图像或
  无有效深度会分别计入统计。固定上限不等于对整机内存的完整约束。
- 预览目录保留最近 8 对 `preview-<job>.png` 和同名 JSON；JSON 最后原子发布。元数据记录
  会话、子地图、位姿版本、实际覆盖帧、损失、训练步数、确认延迟及采样内存峰值。RGB 图像
  以 PNG 保存；内容是该时刻的渲染预览，不是可恢复训练状态的 Gaussian 地图。
- `status.json` 是后台写入的异步采样，可能暂时落后，异常退出后也可能过期；应核对其中的
  时间、会话与版本。节点内存中的就绪状态才与当前 generation 同步。
- 节点退出时输出 `livo_atlas gaussian:` 计数和固定延迟分桶；`latency_seconds` 从观测
  进入桥接到候选确认，包含队列、准备和进程通信，`worker_seconds` 包含视图准备与训练等待。
  这些数值不是单独的 GPU kernel 时间，也不能据此判断地图质量。

一般 socket 等待有超时，停止时会关闭 socket 唤醒通信线程；底层文件系统或 GPU 调用若被
内核长期阻塞，线程/进程无法保证严格的硬实时退出。运行中不要删除或替换输出目录中的文件。
