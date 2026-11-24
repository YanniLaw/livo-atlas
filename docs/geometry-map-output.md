# 版本化几何地图显示

几何地图显示使用在线子地图的 **primary 关键帧**，在独立工作线程中重建局部体素质心，
并发布带来源的完整显示快照。它不改变 FAST-LIVO2 前端用于匹配的地图，也不自动开启
前端几何缓存、回环、历史轨迹或 GTSAM。默认关闭；原始关键帧、图像、标定、成员修订和
位姿文件仍是可重建的权威数据。

本功能为后续完整 SLAM 和 3DGS 提供可追溯的几何输入，不包含 Gaussian 优化、在线渲染、
动态物体过滤或全局表面去重。体素的点分布协方差也不是可以直接使用的 Gaussian 参数。

## 启用

在启动前配置关键帧和在线子地图，示例数值适用于 ROS1 和 ROS2 共同配置：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
  max_keyframes: 20
  geometry:
    enabled: false             # 前端几何缓存可独立保持关闭
    voxel_size: 0.2
global_output:
  enabled: true
  map_frame: map
  geometry:
    enabled: true
    max_frequency_hz: 0.2
    max_keyframes: 20000
    max_points: 100000
    max_cached_points: 300000
    max_message_bytes: 8388608
```

ROS1 使用参数路径，例如 `/global_output/geometry/enabled`；ROS2 使用点分参数名，例如
`global_output.geometry.enabled`。具体构建和启动方式见 [ROS 构建说明](ros-builds.md)。
必须启用关键帧与 `online` 子地图，且每个 primary 子地图最多 64 帧。仅启用本显示时，
`submaps.geometry.enabled` 仍可为 `false`；表示参数仍从同一组 `submaps.geometry.*` 读取并验证。
`map_frame` 必须满足全局输出的帧名规则，启用地图时最长 256 个 ASCII 字节。
配置错误在创建输出文件或工作线程前报告。

| `global_output.geometry.*` | 默认 | 启用时范围 | 作用 |
| --- | ---: | --- | --- |
| `enabled` | `false` | 布尔值 | 启用地图工作线程和发布器 |
| `max_frequency_hz` | `0.2` | 有限数，`(0, 2]` | 消费者返回后的发布频率上限 |
| `max_keyframes` | `20000` | `[1, 100000]` | 本组件可处理的关键帧前沿 |
| `max_points` | `100000` | `[1, 1000000]` | 一条显示云最多点数 |
| `max_cached_points` | `300000` | `[0, 3000000]` | 局部显示片段缓存点数；0 表示不缓存 |
| `max_message_bytes` | `8388608` | `[4096, 67108864]` | 单条版本化消息的保守字节预算 |

消息预算使用 `4096 + 128 × 子地图数 + 16 × 点数`，有效点数还受 `max_points` 限制。
非空前沿无法容纳一个点或超过关键帧上限时输出 `RESOURCE_LIMITED`。该状态会保持到实例重启；
要恢复显示，请按数据规模调整配置并重新启动。预算只限制本组件的缓存、采样堆和消息，
不包含原始点云读取、完整几何缓存、DDS、图像或整个进程的 RSS。

## 话题和坐标

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/geometry_map` | `livo_atlas/GeometryMap` | 版本、参数、来源区间及内嵌点云 |
| `/global_map` | `sensor_msgs/PointCloud2` | 同次结果的标准显示云 |

Noetic 使用 latch 和队列 1；Humble 使用 reliable、transient-local、深度 1。两条话题不是
跨话题原子事务：需要点和来源严格配对时，应读取 `/geometry_map` 内的 `cloud`。
点云固定为小端 `FLOAT32` 的 `x/y/z/intensity`，偏移 `0/4/8/12`，`point_step=16`。
这是一种显示精度，不会回写原始观测。

位姿版本 0 使用原始里程计坐标，Header 帧为 `camera_init`；正版本使用配置的 `map_frame`。
每个块按**当前快照中自己的锚点位姿**变换，不能用一个当前变换替代全部历史锚点。
消息时间是本次前沿最后一个关键帧的传感器时间；空数据或非 Ready 状态为 0。
相同采集时间上的位姿更新或子地图封存仍会产生新 revision；显示输出不会伪造当前 TF 或位姿。
ROS1 的 `Header.seq` 是传输序号，不是地图 revision。

RViz 可订阅 `/global_map` 并选择有可用 TF 的 Fixed Frame。建议 `Decay Time` 设为 0。
非 Ready 状态发送字段完整、宽度为 0 的空云，但 RViz 是否清除旧显示仍受 TF、时钟和
显示器行为影响；收到空消息本身不等于所有 RViz 场景都已清屏。

## 版本、来源和采样

`producer_id` 标识一次输出实例，`revision` 在该实例内递增；它们不是持久化数据集 UUID。
根消息同时给出 `pose_version`、`frame_count`、`optimized_frame_count`、`submap_count`、
表示参数、预算和 `source_voxel_count`。`sampled=true` 表示候选体素多于本条消息选出的点。
每个来源记录包含 primary 连续帧区间、成员 revision、active/sealed 状态、局部几何
`representation_revision`、实际 `built_pose_version`、候选/输入计数和所选点的连续区间。
即使某块没有被采样到点，它的来源记录仍存在。

每帧只融合到一个 primary 块；support 帧仅表示邻接关系，不再次融合。不同块仍可能覆盖
相同物理表面。对于固定候选集合，`geometry_rank_v1` 按子地图 ID 和有符号体素键的固定哈希
排序，保留最小的 K 项，最后按子地图和体素键排序。它避免简单截取前几块，但不保证
空间均匀覆盖。

局部片段在实际构建时的相对位姿与当前相对位姿满足既有 `1e-9` 容差时可复用；整体刚体
修正因此通常只需更新块的外部变换，组内变化则重建。极靠近体素边界时，容差内复用旧片段
和重新体素化可能产生不同候选。应结合 `producer_id`、`representation_revision` 和
`built_pose_version` 解释内容，不能只凭根位姿版本断言两份云逐字节相同。

状态 `READY` 表示完整遍历了该前沿，允许合法空地图；`RESOURCE_LIMITED` 表示配置预算不足；
`ERROR` 表示读取、索引、重建或转换失败。非 Ready 消息保留真实前沿，来源和点数组为空，
`source_voxel_count=0` 表示该计数未知。派生显示失败不会使前端或回环后端失败。

## 独立 C++ 使用和生命周期

`GeometryMapBuilder` 只接受普通 C++ 的成员、位姿和只读点云 reader。`GeometryMapOutput`
绑定一个新建、尚未接收首帧的在线 `LivoSystem`。共享 `main` 通过
`SetOnlineSubmapDeltaCallback` 将成员修订和位姿前沿交给 `ObserveDelta`。使用完整快照的
SDK 调用方可以继续调用 `Observe`，同一位姿版本的已提交前缀仍逐项比较。
系统所有者在处理返回后调用 `Poll`，并在系统或后端 `Finish` 后调用输出的 `Finish`。
有轨迹输出时，先结束后端/系统，再结束轨迹，最后结束几何地图。发布器及 consumer 捕获的
对象必须存活到工作线程 join 之后；consumer 不得重入活系统。

`Poll` 派发正常任务时捕获与当前前沿匹配的完整位姿和固定关键帧读取视图。工作线程持有这批
位姿、成员和读取视图。超预算时任务只携带前沿和状态。最多一个任务执行、一个最新请求待处理，
已完成的旧任务仍会交付，随后请求合并到最新状态。
停止时在帧读取、块遍历和采样批次之间检查取消；已经开始的一次文件读取、几何块构建或
外部 consumer 调用不承诺被强制中断。`delivered` 只表示 SDK consumer 返回，ROS context
已经关闭时不能据此推断网络已收到最后一条。

## 回放记录

两端的验证工具均支持 `--geometry-map`，自动启用所需在线存储，不隐式开启前端几何、回环或
轨迹。例如在各自环境中运行：

```bash
python3 tools/run_baseline.py --workspace /验证用独立catkin工作空间 \
  --bag /数据/Retail_Street.bag --output /新的输出目录 --geometry-map

python3 tools/run_ros2_baseline.py --bag /数据/Retail_Street.ros2 \
  --executable /安装前缀/lib/livo_atlas/fastlivo_mapping \
  --output /新的输出目录 --geometry-map
```

工具将消息元数据、来源和 XYZI 保存到 `geometry-map.json`。这是验证用途的完整记录，
长时间运行或高点数会占用较多观察进程内存和磁盘；它不是在线地图数据库。
Humble 工具分别保存节点的 `node.log` 和 `node-stderr.log`，glog 的计时和诊断记录默认在
`node-stderr.log`。工具读取两份日志来汇总计时并检查退出状态。
最终精确结果仍可从权威观测和位姿使用既有离线导出重建。
