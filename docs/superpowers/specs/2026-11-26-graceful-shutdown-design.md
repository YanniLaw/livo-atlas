# 双 ROS 正常退出与预览终止通知

根因：共用入口当前使用两端默认信号处理。SIGINT 先使 ROS 上下文失效，随后
`GaussianMapping::FinishPublication()` 发布终止帧时返回 unavailable。完整 GPU 回放的
两端均未收到 STOPPED，虽然封口和进程退出成功。

## 范围与顺序

共用入口拥有正常 SIGINT/SIGTERM 停止请求。信号处理器只设置无锁标记，不做日志、
分配、ROS 调用或线程 join；安装/恢复处理器使用一个局部生命周期对象。保留两端
现有多线程 spinner/executor，共用一个 main，不引入后台调度线程。

正常停止次序：停止执行输入回调并等待在途回调结束；在有效 ROS 上下文中撤下预览并
发布 STOPPED；排空已有输入、完成系统收尾和 Gaussian Finalize；完成其余输出后再
销毁发布器和关闭 ROS。保留“先撤下预览，再开始可能较慢的封口”的现有顺序。

ROS2 使用 wall timer 观察停止请求并取消多线程 executor；ROS1 的现有 wall 循环观察
同一标记。停止与排空不依赖仿真时钟。上下文已由外部/异常关闭时不复活上下文，也不
将本地 publish 返回等同于订阅端实际收到。

先用真实子进程、受控空会话 worker 和订阅端复现/验证，观察快速退出是否还会丢终止
消息。如需额外传输等待，只增加有界、基于 wall/steady clock 的最小等待，并记录 ROS1
与 ROS2 的能力差异；不引入确认话题、重复发布或新的线程池。

## 验收

- 实际节点 SIGINT/SIGTERM 正常退出，匹配订阅者收到一次清空图像的 STOPPED；正常
  ROS 发布统计无 unavailable。已关闭上下文的已有测试仍不得声称发送成功。
- `/use_sim_time=true` 且不发布 `/clock` 时也能完成退出。
- 慢 worker、重复停止信号、worker 超时均不使进程卡在 ROS 时间等待；保留失败退出码
  和 incomplete/timeout 事实，不能把超时改成成功。
- 两端短段真实 GPU + Finalize 回放验证实际链路；SDK 与相关双 ROS 测试通过。
- 不改变 SLAM 算法、消息格式、线程预算、位姿图约束或 Gaussian 训练。

磁盘只复用当前构建并生成小规模验证；输入 bag、既有参考和上轮归档保持不变。
