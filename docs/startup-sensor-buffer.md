# 首帧传感器暂存

激光、IMU 和图像来自不同的话题，启动时它们的回调顺序可能不同。`LivoSystem` 默认暂存首个
有效激光扫描前到达的 IMU 和图像，在建立激光时间锚点后交给已有同步器处理。ROS1、ROS2 和
直接使用 C++ 核心的程序使用同一套规则。

## 配置

默认启用。需要复现原有的首帧前拒绝行为时，在传感器配置文件的 `common` 中设置：

```yaml
common:
  buffer_before_first_lidar: false
```

ROS1 和 ROS2 使用项目提供的同一份传感器配置时均可这样设置；如果直接传入 ROS2 的参数文件，
对应的参数名为 `common.buffer_before_first_lidar`。SDK 调用方可设置
`SystemOptions::buffer_before_first_lidar`。修改默认值可能改变初始化使用的样本以及之后的
轨迹，因此比较历史结果时应记录该参数。

## 容量和统计

首个有效激光扫描的时戳非负，点云经预处理后达到当前模式所需的点数。首帧前最多保留 2,000 条
IMU 和 8 张通过原有采样与时间检查的图像。`sensor_buffer.max_imu_samples` 或
`sensor_buffer.max_images` 更小时采用配置值。首扫到达后按校正时戳保留符合 `max_span_s` 的最新
连续样本。输入关闭时会释放未交接的启动缓存。 [运行期队列参数](ros-builds.md#传感器模式与输入缓存)

两种 ROS 节点报告输入计数时还会输出：

```text
livo_atlas startup: imu_buffered=... image_buffered=... imu_evicted=... image_evicted=... imu_rejected=...
```

`buffered` 是进入启动缓存的累计数量，`evicted` 是条数或时间跨度超过预算的淘汰数量，`imu_rejected` 是
释放暂存时因时间回退或校正后时间越界被拒绝的数量。SDK 可在拥有 `LivoSystem` 的同一线程上
调用 `GetStartupBufferStats()`，额外查看尚未释放的启动 IMU 数量和首帧前图像队列数量。
建立首帧锚点后，`pending_images` 返回 0；这不代表队列里的图像已经被同步器消费。接收函数
返回 `true` 表示数据已接受，后续仍可能因启动淘汰、过期或时间校验而不参与估计。同步队列在运行期间
超过预算会报告异常并停止会话。

## 时间与适用范围

IMU 在暂存前应用配置的时间偏移并检查时间范围；需要 `ros_driver_bug_fix` 时，校正依赖首帧
激光时间，因此在释放暂存时进行。图像仍执行已有的 Hilti 采样、去重和时间顺序检查；过期图像
由同步器按原有规则处理。

当各话题自身有序、消息没有丢失、启动缓存没有溢出，并且相关输入在首次同步处理之前到达时，
首帧激光与 IMU/图像回调的相对先后不会改变初始化输入。这个机制不负责一般的跨话题重排、
首帧后迟到的旧消息、时钟回退，或驱动时间校正锚点在后续扫描时变化；负激光时间仍沿用原有的
“尚无激光”哨兵约定。完整回放没有真值，不能据此判断绝对轨迹精度。

设计及测试范围见[首帧暂存设计](superpowers/specs/2026-11-15-startup-sensor-buffer-design.md)，
实测及复现步骤见[验证记录](startup-sensor-buffer-validation.md)。
