# LivoSystem 后端与 ROS 执行器

`LivoSystem` 管理前端、在线子地图以及可选的回环后端。`SystemOptions` 注入普通 C++ 配置，ROS1 和 ROS2 共用同一个 `main`。Cartographer 的 `MapBuilder` 同时持有位姿图和后台线程池，本工程采用同样的系统所有权关系。[Cartographer MapBuilder](https://github.com/cartographer-project/cartographer/blob/2.0.0/cartographer/mapping/map_builder.cc)

## 生命周期

`LivoSystem` 从已经提交的关键帧和子地图修订向后端交付快照。它在 `ProcessNext()` 中收取结果，包含等待输入的周期。回环任务使用已有的 Cartographer 线程池，位姿由系统串行提交。`Finish()` 先封存输入并排空回环任务，再结束输出和写完回环摘要。外部在线子地图观察器继续接收提交后的快照，可以单独注册和清除。

保留 `LoopBackend` 的现有 SDK 装配方式以兼容调用方。新的系统配置启用内部后端后，同一会话只允许系统拥有的一套位姿提交者。`livo_atlas::core` 提供完整系统，`livo_atlas::online_backend` 继续作为兼容链接目标。

## ROS 回调

Noetic 使用 `AsyncSpinner`，主线程推进系统。Humble 使用 `MultiThreadedExecutor`，每个传感器的回调组保持订阅内串行，处理与传播共用一个互斥组。消息转换可以并行，两个接入层都在调用系统时串行化。停止时等待执行器退出，再完成系统及发布器的收尾工作。

## 启动配置

两端 launch 读取 `config/backend.yaml` 和 `config/gaussian.yaml`，之后加载传感器配置和标定。`backend.yaml` 包含执行器线程数、回环搜索、配准、几何、接纳与位姿优化参数。旧配置可继续使用 `loop_closure.options_file`，它作为迁移入口。Gaussian 的 ROS 桥接与 Python worker 从同一配置读取各自的设置，共用 socket 地址和传输超时。

## 验证

检查 SDK 不经 ROS 的后端初始化、等待周期轮询、外部通知与关闭。Noetic 和 Humble 分别验证节点回调、启动参数与 `Retail_Street` 回放。使用已有回环档案检查器复核封存输出。
