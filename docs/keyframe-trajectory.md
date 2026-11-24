# 版本化关键帧轨迹（C3b-01）

同一份普通 C++ 轨迹可由 SDK、Noetic 和 Humble 消费。每个关键帧采用该版本自己的位姿，
优化提交后历史轨迹会重发。使用 [C3b 设计](superpowers/specs/2026-11-15-p6-02c3b-history-design.md)
和[验收记录](p6-02c3b1-validation.md)了解范围和证据。几何地图显示留给 C3b-02。

## 配置与构建

沿用[双 ROS 构建入口](ros-builds.md)，重新构建并加载所选安装前缀的 setup。
两端使用同一份 `msg/*.msg` 和 `src/app/livo_main.cc`；普通 SDK 不依赖 ROS 消息生成器。
将下面的配置并入原 Avia YAML，保留传感器、标定和输出目录设置：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
global_output:
  enabled: true
  map_frame: map
  trajectory:
    enabled: true
    max_frequency_hz: 1
    max_keyframes: 20000
```

轨迹默认关闭。开启时频率必须有限且在 `(0,10]` Hz，帧数预算在 `[1,100000]`。
ROS1 参数如 `/global_output/trajectory/max_keyframes`；Humble 如
`global_output.trajectory.max_keyframes`，沿用启动时读取、重启后生效的方式。
它需要全局输出、关键帧存储和 online 子地图；回环、GTSAM、几何体素构建各自独立。
频率限制按上次消费者完成的 steady clock 计时；初始空状态和停止排空可绕过正常限频。

Humble 在只用 CMake 安装的前缀中，执行消费者或回放工具前也要激活：

```bash
source /opt/ros/humble/setup.bash
source /absolute/livox_install/setup.bash
source /absolute/livo_install/share/livo_atlas/local_setup.bash
```

这会设置生成的 Python 消息与运行时类型支持搜索路径。静态算法构建仍保留 ROS2 标准生成的
消息共享模块；普通静态 SDK 无这些依赖。不要固定为某个 RMW 来绕过环境激活。

## 两个话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/keyframe_trajectory` | `livo_atlas/KeyframeTrajectory` | 有明确 producer、revision、位姿版本和前沿的完整结果 |
| `/keyframe_path` | `nav_msgs/Path` | RViz 等标准工具显示同一组关键帧 |

Noetic 为 latch、队列 1；Humble 为 reliable、transient_local、depth 1。
两个消息先构造完成再分别发送，跨话题不保证原子交付，也不承诺停止时最后一条必达。
晚加入订阅者在发布器仍运行时可取得保留的最新状态。

| 字段 | 含义 |
| --- | --- |
| `producer_id` | 输出实例随机 128 bit，32 个小写十六进制字符；重启后更换，不能充当 dataset UUID |
| `revision` | 同一实例递增的派生输出编号；合并请求时允许跳号 |
| `pose_version` | 已正式提交的位姿版本 v |
| `frame_count` | 本次固定读取前沿 M |
| `optimized_frame_count` | 正式优化前沿 N |
| `status`、`error` | 状态和最多 1024 字节的错误信息 |
| `keyframes` | Ready 时 ID 为 0…M−1 的完整数组，含原始 double 秒时间和 body 位姿 |

状态 `READY=0`：数组恰有 M 项；M=0 是合法初始空状态。
`RESOURCE_LIMITED=1`：M 超预算，保留真实元数据，数组与 Path 清空；不静默截断。
`ERROR=2`：读取、来源校验或转换失败，数组与 Path 清空，该派生输出停止继续工作。
消费者自身抛异常时记录一次失败，不递归调用同一个消费者来发送错误。
失败不主动写入核心后端失败状态；共享的权威文件若已损坏，原后端仍可能独立报告错误。

Path 本身没有 producer 或位姿版本字段，不能用于可靠的数据版本同步。
它是选定关键帧的历史路径；现有 bag 约为 172 个关键帧，原始连续路径约为 1351 个位姿，
两者采样含义不同。原始 path、odometry、云和已有 TF 继续按原有逻辑发布。

## 位姿与时间

v=0 时，输出在 `camera_init`，每帧使用其原始 odom 旋转的 SO(3) 投影，平移保持原值。
v>0 时，输出在配置的 `map_frame`，优化前缀各帧采用 `P_v(i)`；
未优化尾部使用最后优化锚点和原始位姿按既有 projected-tail 规则物化。
不能用一次当前 `map_from_odom` 修正替代整个优化历史。
在线正版本要求 projected-tail；legacy 正版本仍可按既有离线接口读取，在线轨迹会报 Error。

body 是系统的 IMU/body 坐标，图像相机外参仍需从数据集标定读取。
每行保留采集时的 double 秒数；ROS Path 的每行时间由该秒数四舍五入到纳秒。
两端共同支持 `[0,2^31)` 秒，越界或非有限时间转为 Error。
Header 使用最后一个关键帧的采集时间；空状态和非 Ready 为 0。
重复或回退的有效采集时间不会阻止新的历史 revision。
同一采集时间出现新的优化版本时，历史仍会更新；不会为此补造当前 TF 或 `/global_pose`。

## SDK 所有权与调度

[BuildTrajectorySnapshot](../include/livo_atlas/mapping/trajectory_snapshot.h) 接受
`PoseSnapshot` 和按 ID 排序的完整时间数组，返回普通拥有数据的结果。
调用者负责为纯构建结果指定 producer/revision；无效来源抛 `invalid_argument`。

[TrajectoryOutput](../include/livo_atlas/core/trajectory_output.h) 连接 `LivoSystem`。
共享 `main` 在输入第一帧之前用 `SetOnlineSubmapDeltaCallback` 注册
`ObserveDelta`。使用完整快照的 SDK 调用方可继续调用 `Observe`。
系统所有者负责通知、`Poll` 和 `Finish`。工作线程读取固定的
`KeyframeReadView`、缓存时间并调用消费者。消费者在调用期间借用 `const TrajectorySnapshot&`，
需要保留时自行复制，回调只处理已交付的结果。

最多一个执行任务和一个合并待处理状态。当前完整结果交付后，再处理最新待处理状态。
初始化、追加与收尾先读位姿前沿，正常任务派发时才捕获对应的完整位姿和固定读取视图。
超预算时只交付头部和状态。预算覆盖本组件的任务数量和有效帧数。
注册完整快照回调的消费者仍会在通知时收到位姿历史。在线 3DGS 使用这一入口，局部几何和
全局几何地图的常规追加与封存使用增量前沿。

正常停止先停止节点处理，再 `LoopBackend::Finish()` 或 `LivoSystem::Finish()`，
最后 `TrajectoryOutput::Finish()` 排空最终状态并 join。Finish 幂等。
析构只停止并 join，不再次访问系统。发布句柄和消费者捕获的资产必须活到 join 结束。
ROS context 关闭后发布器跳过转换和发送；SDK 完成消费仍可记录为 delivered，
因此统计 `delivered` 表示消费者正常返回，不能解释为订阅者已收到。

## bag 验证入口

两个工具新增 `--keyframe-trajectory`，自动开启其所需全局输出和在线存储。
原来的回环和几何开关仍需显式指定；输出目录必须不存在。

```bash
OMP_THREAD_LIMIT=1 nice -n 10 python3 tools/run_baseline.py \
  --workspace /absolute/snapshot_ws --bag /absolute/Retail_Street.bag \
  --output /absolute/new_noetic_run --duration 136 --rate 1 \
  --cpu-affinity 0 --image-queue-size 32 --semantic-output --keyframe-trajectory
```

```bash
OMP_THREAD_LIMIT=1 nice -n 10 python3 tools/run_ros2_baseline.py \
  --bag /absolute/Retail_Street.ros2 \
  --executable /absolute/install/lib/livo_atlas/fastlivo_mapping \
  --output /absolute/new_humble_run --duration 136 --rate 1 \
  --cpu-affinity 0 --image-queue-size 32 --keyframe-trajectory
```

新增记录在 `keyframe-trajectory.json`，原有数组和 semantic 记录保留原义。
验证工具会在内存中保存每次收到的完整数组，适用于当前短 bag 的验收；
长期训练数据采集需要另行设计流式写入，不能把此工具的内存增长归因于 C++ worker 队列。
最终权威结果仍可从原始关键帧和正式 pose 版本离线恢复，不依赖最后一次 ROS 交付。
