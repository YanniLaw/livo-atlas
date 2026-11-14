# 全局 TF 与当前位姿（C3a）

状态：实现、正式构建矩阵和 bag 主验收完成，整阶段独立审阅待执行。
设计和验收依据见 [C3a 设计](superpowers/specs/2026-11-14-p6-02c3a-global-pose-design.md)、
[实施计划](superpowers/plans/2026-11-14-p6-02c3a-global-pose.md)及[验证记录](p6-02c3a-validation.md)。

## 配置

将下面的配置并入启动时加载的 Avia YAML，保留原有传感器、标定和输出目录设置：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
global_output:
  enabled: true
  publish_tf: true
  map_frame: map
```

ROS1 使用对应的 `/global_output/enabled` 等参数；Humble 的共享参数名是
`global_output.enabled`、`global_output.publish_tf`、`global_output.map_frame`。
现有 Humble launch 可通过 `config:=/absolute/avia.yaml` 读取同一嵌套 YAML。
参数在启动时读取，Humble 以只读参数声明；更换配置后重新启动。

默认 `enabled=false`，`publish_tf=true`，`map_frame=map`。
开启要求关键帧存储和 online 子地图；没有 GTSAM 或没有开启回环时也可使用，
尚未提交优化版本时提供单位修正。几何构建和回环分别由其原有开关控制。
纯全局输出不受回环配准的 64 成员上限约束。

帧名只允许 ASCII 字母、数字、下划线和单斜杠分段；禁止空名称、首尾斜杠、空段，
以及 `camera_init`、`aft_mapped`、`world` 三个现有名称。可用 `map` 或 `atlas/map_2`。
关闭功能时忽略其帧名和在线运行前提；开启时在输出文件 I/O 前检查配置。

## 输出与坐标

```text
map_frame --map_from_odom--> camera_init --原始 body 位姿--> aft_mapped
```

| 输出 | 类型与坐标 | 发布时机 |
| --- | --- | --- |
| `map_frame -> camera_init` | 动态 TF | 有效 LiDAR 位姿发布，且 publish_tf=true |
| `/global_pose` | PoseStamped，frame=map_frame | 同一次有效 LiDAR 位姿发布 |
| `camera_init -> aft_mapped` | 原动态 TF | 保留原有发布分支、表示和数值 |

不增加 body 的第二条 TF 父边。`publish_tf=false` 时只保留原始 TF，并继续发送 `/global_pose`。
全局位姿队列为 1；Noetic 不 latch，Humble 为 reliable/volatile。
消息没有新增 twist、协方差或精确后端版本字段。

原始点云、path、odometry 和 IMU propagation 继续使用原有坐标。
RViz 可将 Fixed Frame 设为配置的 map_frame，按新 TF 显示当前输出；
历史关键帧轨迹和几何地图的逐帧优化重建留给 C3b。

## 修正值与普通 C++ 接口

[MapCorrection](../include/livo_atlas/mapping/map_correction.h) 是普通值：
`pose_version`、当前 `frame_count`、`optimized_frame_count` 和 `map_from_odom`。
[LivoSystem](../include/livo_atlas/core/livo_system.h) 的 `GetMapCorrection() const`
在原所有者线程读取已提交状态，时间和额外空间均为 O(1)，不复制完整轨迹、不读文件或点云。
节点只在普通结果回调的已更新、非视觉分支读取它，后端 worker 不访问系统。

版本 0 为单位修正。正版本 v 取最后一个优化锚点 k：

```text
k = optimized_frame_count - 1
map_from_odom = P_v(k) * inverse(ProjectPose(O_raw(k)))
map_from_body_live(t) = map_from_odom * O_published(t)
```

`ProjectPose` 在临时副本将已验证的近似旋转投影到 SO(3)，保持平移。
这与既有 projected tail 物化规则一致；正版本的 legacy tail 不支持修正 getter，
原有 legacy 数据读取和离线物化仍保留。

当前 ROS 位姿使用**实际原始发布四元数**组成 `O_published`。
旧发布先将滤波旋转转成 Euler 再生成四元数，近似非正交输入下可能与后端投影有差别。
当前全局位姿保证符合当次 TF 链，历史优化关键帧仍以 `PoseSnapshot` 中各帧的 `P_v(i)` 为准。
一个尾部修正不能代替整段优化历史。

SDK 调用 `ApplyMapCorrection(correction, published_odom_from_body)` 可得到同样的刚体组合。
输入必须是调用者实际使用的严格刚体表示；该函数验证版本头、旋转、有限值及组合溢出，
不会修改输入。非在线系统的 getter 抛 logic_error。
已提交通知失败后或正常 Finish 后允许只读诊断，后续处理仍遵循原有失败和停止约束。

## 时间与停止语义

新 TF 和 global_pose 复用该次原始 odometry 的整数 ROS 时间。
两个 TF 边通过同一个 broadcaster 的一次向量调用发送；TF 与 pose 跨话题传输不保证原子性。

只有严格正、且大于上次全局输出时间的样本才新增输出。
零时间、重复时间和时钟回退时保留原始发布，跳过新增 map TF 与 global_pose；
Waiting 期间的后端 Poll 和原有调度继续运行。
时钟恢复到大于最后全局输出时间后，下一次有效样本采用最新已提交修正恢复发布。
本轮验证范围为 ROS 支持的非负时间；本机 Humble 时间源处理负 `/clock` 会在进入发布器前异常中止。
不使用 wall clock 替代 ROS 时间，也不清空外部 TF buffer。

Visual、Waiting 和单独 IMU propagation 不产生新增全局位姿。
Poll 在原始结果发布之后接纳的版本，从下一次合法原始发布起可见；空闲提交不补造样本。
停止时先离开节点回调循环，再排空所有者。最后一次原始发布之后提交的版本可能只存在于
正式 pose 文件和 SDK 状态中，不承诺在 ROS 话题补发或保证最后一条送达。

启用后，直接 publisher 调用在非视觉 updated 分支必须提供 MapCorrection；
缺失或非法修正会在发送本次消息前报错。发送阶段的异常不能回滚已发 TF 或已提交后端文件。

## 官方 bag 验证入口

两个现有工具均支持 `--global-output`，自动启用 online 存储，回环保持独立。
输出目录必须新建；Noetic 仍要求独立 catkin 源码副本。

```bash
OMP_THREAD_LIMIT=1 nice -n 10 python3 tools/run_baseline.py \
  --workspace /absolute/snapshot_ws --bag /absolute/Retail_Street.bag \
  --output /absolute/new_noetic_run --duration 136 --rate 1 \
  --cpu-affinity 0 --image-queue-size 32 --semantic-output --global-output
```

```bash
OMP_THREAD_LIMIT=1 nice -n 10 python3 tools/run_ros2_baseline.py \
  --bag /absolute/Retail_Street.ros2 \
  --executable /absolute/install/lib/livo_atlas/fastlivo_mapping \
  --output /absolute/new_humble_run --duration 136 --rate 1 \
  --cpu-affinity 0 --image-queue-size 32 --global-output
```

需要回环时另加 `--loop-closure`；`--global-output --submap-mode keyframe_groups`
在任何输出 I/O 前拒绝。`global-output.json` 单独记录实际参数、原始/全局位姿及 TF 批次的整数时间，
旧四数组、语义摘要和原有统计键继续保留。

## 后续消费者

`PoseStamped` 和 TF 没有 pose_version，不能通过 ROS1 Header.seq 携带版本。
同一个修正值可能对应多个已提交版本；离线校验只报告可解释版本集合。
需要精确版本的 SLAM/3DGS 消费者继续使用 `MapCorrection`、`PoseSnapshot` 与原始观测。
C3b 将另行设计优化历史轨迹、版本化几何地图和跨 ROS 来源协议；本轮没有加入 Gaussian 训练或 iSAM2。
