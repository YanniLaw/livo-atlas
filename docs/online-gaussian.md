# 在线 Gaussian 子地图

独立 GPU 进程接收拥有数据的重建观测，训练有界 Gaussian 工作集，并输出版本化预览。
默认 `gaussian.persist_submaps=false` 保留 [C1 单活动模型](p6-03c1-validation.md)；设为
`true` 后启用 C2b 的历史资产、刚性重用、组内重建和持久目录，见
[阶段验证记录](p6-03c2b-validation.md)。普通 C++ 核心保持与 ROS、torch、CUDA 分离。

两个模式均使用同一个 `src/app/livo_main.cc` 和同一个 Python worker 入口。ROS1 Noetic
与 ROS2 Humble 的区别仍在接入和参数表示；分别使用 `/` 和 `.` 分隔参数。实际 GPU
覆盖在 Noetic 主机，Humble 验证构建、配置、安装消费及真实入口与受控 socket peer。

这里保存的是每块最近至多 10 个视图的稀疏工作集。Gaussian 不向前端或后端反馈定位测量，
当前阶段也不做 densification、跨块联合渲染或整套 SLAM session 恢复。

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
export LIVO_GAUSSIAN_OUTPUT="$LIVO_RUN/gaussian"

env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  "$LIVO_PYTHON" "$LIVO_REPO/tools/3dgs_probe/online_worker.py" \
  --socket "$LIVO_GAUSSIAN_SOCKET"
```

将上述三个路径也设置到节点终端。正常停止节点会结束 worker；socket、GPU、协议或 worker
错误会使 Gaussian 停止并关闭连接，前端继续运行。当前没有自动重连；要重新运行请启动新 worker，
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
rosparam set /gaussian/persist_submaps true
rosparam set /gaussian/shutdown_timeout_ms 30000
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
    'gaussian.persist_submaps': True,
    'gaussian.shutdown_timeout_ms': 30000,
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
  --gaussian-output "$LIVO_GAUSSIAN_OUTPUT" --gaussian-persist-submaps

python3 tools/compare_gaussian_replays.py --off "$LIVO_CHECK/off" \
  --on "$LIVO_CHECK/on" --previews "$LIVO_GAUSSIAN_OUTPUT"
```

`--cpu-affinity 0` 只是示例，两次应选用本机允许的相同 CPU。比较工具要求输入、共享参数、
二进制、四组前端数组、发布图像/点云语义及原始关键帧文件完全一致，并检查回环最终结果、
有界统计和播放期间真实训练确认的完整预览。关闭组和开启组必须使用不同目录，预览目录、
会话及图像尺寸须与开启组配置和元数据一致；差异会原样报告，不自动调整数值容差。确认短段
后可使用新的输出路径按相同方式回放完整 bag。结果不包含真值评价，也不能替代长距离或
长期连续运行测试。

## 模式、目录和停止

`gaussian.persist_submaps` 是严格 boolean，默认 `false`，必须与 `gaussian.enabled=true`
一起使用。`gaussian.shutdown_timeout_ms` 是正整数，默认 3000；上述持久化示例和 runner
显式设置 30000。省略 `--gaussian-persist-submaps` 就沿用 C1；节点切换参数后需启动新会话。

| 行为 | C1 默认模式 | C2b 持久模式 |
| --- | --- | --- |
| IPC | schema 1 | schema 2，worker 从 hello 选择模式 |
| 切换主分组 | 清空原模型 | 先保存并提交引用，再释放、加载/初始化 |
| 纯刚性修正 | 失效并重置 | 外部绑定更新，reuse 0 步，参数/资产不改写 |
| 任一来源 primary 局部位姿改变 | 重置 | 立即 needs_rebuild，保留视图各训练 20 步，最多 200 步 |
| 候选拒绝 | 清空模型 | 释放候选 GPU 张量，恢复原驻留确认；按最新绝对位姿判定可用性 |
| 新增同版本前沿 | 允许旧覆盖继续使用 | 允许旧覆盖继续使用，实际覆盖单独记录 |

持久模式输出示例：

```text
gaussian/
├── map.json                   # 带会话、序号、目标版本和覆盖的异步目录
├── status.json                # 当前预览的异步采样
├── preview-<job>.png/.json     # 最近 8 对确认预览
└── states/
    └── <asset_id>/
        ├── manifest.json
        ├── gaussians.npz
        └── observations.npz
```

内容身份 `session_id/submap_id/source_generation/content_job_id` 只在 view/rebuild 确认后改变；
`binding_job_id` 也记录接受的 reuse。目录分别描述最近确认内容和旧持久资产，`volatile=1`
表示更新仍仅在内存。逻辑 primary、来源完整 primary、最近保留的训练帧是三个不同范围。
被 coalesce 跳过或容量不足的分组明确为 missing，文件不会声称补齐未训练帧。

所有者用完整绝对位姿比较原 canonical primary；不会只检查最近窗口或累积回环增量。
整块刚性变换只修改父坐标绑定。即使变化成员已从窗口淘汰，也触发重建；重建只用保留视图。
CPU 确认快照在候选原地修改期间保持，跨块拒绝恢复原驻留块；恢复成功本身不保证它仍 current。

保存顺序为：三文件资产同步成功、C++ 持久提交 map.json、worker 确认完整持久引用集合，
再删除被替换的旧资产。第一次保存也必须确认引用集合。rename 后同步不确定会停止 Gaussian
并保留旧、新资产，前端继续。目录和 status 都是带时间、会话、版本的异步快照；在线消费者
使用所有者线程上的 `GetMapSnapshot()` 读取最新内存状态。

正常 Finish 优先拒绝未决候选、保存 dirty 确认、提交目录并清理替换资产。超时或失败明确
输出 `incomplete=1` 和未保存身份；中断可能丢失最近 volatile 更新。已有历史文件可独立
按[资产说明](gaussian-state.md)检查/恢复，在线 worker 不自动接管旧会话，也不恢复前端和位姿图。

## 资源和统计

- 位姿前沿、逻辑目录最多 512 帧/项；拥有确认内容的分组最多 32 块。达到容量后新组保持
  missing/resource_limit，已有块继续维护。
- 一个 GPU 模型、一个稳定 CPU 确认；跨块决策期间最多额外一个有界 CPU 加载状态。
  单块最多 10 视图、每视图 1M 像素和 100,000 输入云点、每来源 1,000 个高斯、总 10,000 个。
- map.json ≤512 KiB；每资产清单/参数/观测分别 ≤512 KiB/1 MiB/192 MiB。
  最多 32 个引用资产和 1 个替换中资产，逻辑文件总量 ≤33×193.5 MiB，写入前保留至少
  1 GiB 加一个最大资产空间。实际 RSS、文件系统块数和第三方 GPU 分配另行测量。
- pending/in-flight 各 1；最多两次前台尝试后尝试一个历史 repair，循环选择历史块。
  激活前置操作即使过期也消耗尝试预算；成功激活及其后续 view 只计一次。
  有工作时两次任务之间最多一次目录/状态发布，目标更新合并到最新绝对状态。
- PNG 是该确认时刻的 RGB 渲染。元数据区分 view 20 步、reuse 0 步、rebuild 20×视图数。
  比较工具不把 0 步重绑计作训练；最近 PNG 全为重用时，还需校验持久资产和此前的
  `effective_training` 累计计数。该计数只包含已确认、有深度监督、非零梯度及参数变化的训练，
  不以 accepted 减 rebinds 推断有效训练。
- 退出输出 checkpoint/load/rebind/rebuild/prune、map_sequence、missing/needs_rebuild/dirty、
  逻辑资产字节及 incomplete，原延迟分桶保持。view latency 从观测入桥计时，独立 reuse/rebuild
  从调度选中计时，均到确认并包含任务前置保存、准备和通信；worker 时间仅覆盖该命令的准备
  与 GPU 等待，不能等同 kernel 时间或画质。

文件系统/GPU 长期阻塞时无法保证硬实时退出。CPU/GPU 测试、短 bag 和重复输入不能替代
长距离数据、长期耐久或真值精度评价；当前资源上限也不代表完整覆盖或高质量稠密重建。

## 可选的最终位姿收敛

`gaussian.finalize_on_shutdown` 默认 `false`，保留原先的在线调度和普通 `Finish()`。
需要最终交付时将它设为 `true`，并同时开启 Gaussian 与 `persist_submaps`。
共用 main 先完成回环后端和 `LivoSystem`，再修复已有确认内容、保存最后一个驻留模型、
提交目录及停止 worker。整个步骤使用 steady clock，不依赖 ROS 时钟继续前进。

ROS1 在节点启动前增加：

```bash
rosparam set /gaussian/finalize_on_shutdown true
rosparam set /gaussian/finalize_timeout_ms 60000
```

ROS2 参数文件使用 `gaussian.finalize_on_shutdown: true` 和
`gaussian.finalize_timeout_ms: 60000`。两种接入都要求真正的 bool 和整数，修复预算范围
为 1..600000 ms。正式持久运行仍建议显式设置原有 `gaussian.shutdown_timeout_ms: 30000`。

SDK 使用 `GaussianFinalizationResult OnlineGaussianMapper::Finalize(milliseconds) noexcept`。
调用者先成功完成自己的 backend/system，`LivoSystem::IsFinished()` 才能放行；
单独 `SealInput()` 不够。前置条件失败不改变 mapper，可修正顺序后重试；一次被接纳的
调用就是终端操作，随后重复调用返回缓存结果，普通 `Finish()` 和析构仍然幂等。

最终位姿包含后端优化前缀和投影尾部。排空之前已发出的 accept 回执后，固定至多 32 块
已有确认内容为 E，只修复 E 的历史几何状态。未确认候选被拒绝，missing 组不会补训练。
纯刚性修正重用数组；组内变化仍只使用该块保存的至多 10 个视图，每视图 20 步重建。
因此 succeeded 表示已有内容与最终位姿一致，不表示所有关键帧已训练或稠密地图完整。

`finalize_timeout_ms` 覆盖排空、修复及中途切块保存。到期停止派发/接纳修复，再执行原有
shutdown 预算内的持久收尾。正常修复完成后的最后一次保存、目录确认和 stop 单独计时，
可跨过修复截止时间；晚到修复结果不能将 timed_out 改为 succeeded。
在途 GPU、文件同步和 join 无法硬抢占，实际退出耗时可能更长。

显式请求才新增至多 16 KiB 的 `finalization.json`，不改变 map.json 或资产 schema。
报告记录目标、冻结的 E、引用资产、current/needs_rebuild/missing/dirty、durable、
worker_stopped、阶段耗时及 before/after 计数。文件的 elapsed_seconds 到报告发布前为止；
节点日志/返回 DTO 的 total_seconds 还包含报告发布。before/after 区分 Finalize 入口前与终端阶段计数；入口前也可能含 EOF 排空，
不能单独当作纯回放吞吐。原有 Gaussian 总计数和延迟桶累计到停止为止。

报告文件描述地图事实，不能自证它本身的 fsync。节点日志另有 report_persisted，只有
status=succeeded 且 report_persisted=1 才返回成功；报告发布失败仍完成轨迹/几何消费者的
收尾。rename 后目录同步失败保留可见报告并返回 commit_uncertain，不再次写盘或回滚。

回放工具增加 `--gaussian-finalize-on-shutdown` 与
`--gaussian-finalize-timeout-ms 60000`，要求 `--gaussian-persist-submaps` 和 worker 路径。
启用后 SIGINT 等待为修复预算加 90 秒，另写 shutdown-resources.csv，summary 记录实际
退出时间和 forced_termination；原 resources.csv 仍只描述在线回放。
严格比较工具会逐资产核验 SHA、数组及完整 canonical primary，再核对最终报告、节点日志
和正常退出。单独查看一个可见 JSON 不能证明原进程完成了持久发布。

## 最终快照联合渲染

`tools/3dgs_probe/render_gaussian_map.py` 只读消费已停止且 Finalize 成功的目录，
将显式选中的一至两块 Gaussian 放在同一次光栅化中处理遮挡。查询相机来自某个
已经保留的训练观测，使用完整最终位姿，包括优化前缀之后的投影尾部。
查询所属块可以不在选择集合内；它仅用于额外的单块参考图，不会自动加入联合输入。

在包根目录运行，路径和 ID 换成实际快照中的值：

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=0 \
  "$LIVO_PYTHON" tools/3dgs_probe/render_gaussian_map.py \
  --map-directory /path/to/final-gaussian \
  --keyframes /path/to/keyframes \
  --keyframe-id 17 --submap-ids 0 1 \
  --output /path/to/new-render
```

使用[已固定的 GPU 环境](p6-03b-gpu-probe.md)，另需 PyYAML（本机验证为 6.0.2）；
原 GPU pip 清单不包含 PyYAML，可在对应独立环境中安装 `PyYAML==6.0.2`。
本机验证复用已有隔离前缀，没有升级环境。无需启动 ROS，也无需加载训练模型或优化器。
CUDA0 至少有 4 GiB 可用，沿用 PyTorch 40% 分配比例及预编译 gsplat 1.5.3。

输入必须在整个调用中保持不变。工具完整检查目录、最终报告、分组、原始/优化位姿和
逐资产 SHA/数组；重读及发布前再次检查来源。已有内容必须 current、durable，且 E 已冻结、
worker 已停止。未选中的逻辑 missing 明确记录，选中 missing、未保留查询帧或坏引用会失败。
`frames/` 的规范 ID 集合及 `online_submaps/poses/` 最新版本必须与 map 前沿一致；
`online_submaps/records/` 及每个分组必须是真实目录，不能省略或通过目录符号链接替代。
ID 使用规范非负 uint64 十进制；重复、三块以上或超预算请求会拒绝，不自动减块或降分辨率。
输出父目录须存在，输出本身须为新目录，且不能与两个输入目录互相包含。

成功目录包含：

| 文件 | 含义 |
| --- | --- |
| `joint.png` | 选中块的联合 RGB |
| `reference.png` | 保存的去畸变训练观测 |
| `single-reference.png` | 查询所属块在同一最终相机下的渲染 |
| `render.npz` | float32 `rgb/alpha/depth_m` 和 uint8 `valid_mask`（alpha ≥ 0.05） |
| `report.json` | 来源、实际训练覆盖、相机、误差、阶段耗时及资源；最后发布的完成标记 |

深度是 alpha 归一化的相机 z，须结合 mask 使用。RGB L1/MSE/PSNR 在观测的有效图像
mask 内计算；深度误差只使用同时满足图像 mask、稀疏深度 mask 和 alpha 阈值的像素。
零深度样本的误差为 null；完美 MSE 的 PSNR 也为 null，并用 `rgb_perfect` 区分。
报告标注 `training_view=true`、选中/省略/missing 块、实际 retained IDs、capture 与最新绑定，
不能将局部选择解释为完整场景或新视角评价。变化像素只证明数值变化，不保证画质提升。
`blocks` 只列有内容的块，所有无内容分组完整保留在 `missing_ids`，避免合法稀疏目录撑大报告。
小文件集合摘要按带来源前缀的相对路径流式计算，同一份快照换挂载路径后摘要保持一致。

每次至多 20,000 Gaussian、一个至多 1M 像素的相机；CPU 一次只读取一个完整资产，
GPU 只接收精简推理参数。相机相对变换先在 float64 中完成，中心和各向异性形状使用同一旋转。
报告区分全目录读取/验证、CUDA 初始化、首帧、单块/联合渲染和传输；总时间到报告发布前，
GPU 峰值仅指 PyTorch allocator，RSS 为进程历史峰值。离线启动耗时不能当作在线刷新速度。

OOM、设备/扩展或文件错误返回非零；创建输出后的失败尽量留下至多 4 KiB 的 `failure.json`，
不会发布成功报告或覆盖旧输出。完成标记不新增 fsync 承诺，也不能由文件自证原 Finalize 的
持久提交。来源变化检测不提供并发事务隔离，不能传入仍在更新的在线目录。
实际结果与复现见 [C3b 验证记录](p6-03c3b-validation.md)。自动选块、在线显示、跨块去重、
自由相机与外观修正留待后续设计。
