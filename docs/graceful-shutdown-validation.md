# 双 ROS 正常退出验证

本轮接续 [GPU 全链路验证](gpu-slam-validation.md) 中未收到 STOPPED 的问题。
共用入口保持多线程 ROS 接入，正常停止请求先停止执行并 join 输入回调，在有效 ROS
上下文中发布清空图像的 STOPPED，然后排空输入、完成系统/高斯/其他输出收尾，最后
关闭 ROS。没有改动 SLAM 算法、消息格式、线程预算或 Gaussian 训练。

## 修复与边界

默认信号处理会先关闭 ROS，随后 `FinishPublication()` 已无法发送。现在用 app 私有的
`ShutdownSignal` 接管 SIGINT/SIGTERM，处理器只执行无锁 atomic store；ROS1 的 wall
循环与 ROS2 的 wall timer 负责退出现有多线程执行器，不使用仿真时间。

只在 node 创建后接管信号。Noetic 第一份 NodeHandle 等待不可达 master 时仍保留原生
SIGINT 处理，避免初始化重试无法取消。信号对象的生命周期覆盖异常报告和最后的 ROS
shutdown，防止收尾末段重复 SIGTERM 把预定的0/1退出码改为信号终止。

上下文已被外部或异常关闭时，原有发布器仍返回 unavailable，不声称送达。正常退出的
接收证明来自实际订阅者。修正时序后，快速空会话和真实 GPU 回放均能收到终止帧，
因此没有额外增加固定休眠、确认话题或传输参数。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| 原始两端进程回归 | 正常退出0，但未收到 STOPPED，修复前稳定失败 |
| Noetic 退出与信号测试 | 最终6/6；含不可达 master 的启动取消 |
| Humble 退出及相关节点/发布器测试 | 最终9/9；含正常/失败退出的末段重复信号 |
| 最终退出测试重复 | Noetic 6项各2次，Humble 7项各2次，均通过 |
| SDK 完整 CTest | 79/79 |
| ROS1 发布器及已关闭上下文检查 | 两个独立测试程序通过 |

真实进程测试不依赖 bag 或 CUDA，使用实际节点、实际 ROS 订阅者和受控空会话 worker。
`use_sim_time=true` 且不发布 `/clock`，覆盖 SIGINT、SIGTERM、慢停止、重复信号和配置
截止时间。受控超时仍退出1，记录 `incomplete_shutdown`、`worker_stopped=0`，同时收到
STOPPED；正常会话退出0并持久封口。STOPPED 后没有复活的 READY，图像载荷为空。

独立复核的两项重要发现均有失败/通过证明：不可达 master 时，首版节点在 SIGINT 后
超过3秒仍不退出，修复后约0.43秒结束；测试专用拦截库在真实 `rcl_shutdown` 入口暂停，
此时发送第二次 SIGTERM，首版两个用例退出-15，修复后分别保留0与1。拦截库只用于
测试，不链接或安装到生产节点。

| 最终真实 GPU 短段 | Noetic | Humble |
| --- | ---: | ---: |
| 请求回放时长（秒） | 10 | 10 |
| 轨迹行数 / 关键帧数 | 98 / 5 | 98 / 5 |
| 已封口子地图 | 1 | 1 |
| 输入截止前收到 READY | 4 | 5 |
| 收到 STOPPED | 是 | 是 |
| 发布 unavailable | 0 | 0 |
| Finalize（秒） | 0.400 | 0.404 |
| 节点 / worker 退出码 | 0 / 0 | 0 / 0 |

GPU 环境沿用 RTX 4060 Laptop、Torch2.3.0+cu121、gsplat1.5.3，worker 未修改。Humble
仍在宿主机运行同一个 worker，通过同绝对路径的共享目录交换 socket 和资产。本轮没有
重跑135秒全段，也不将10秒短段当作真实回环或长距离验证。

另做了一次真实 Noetic 的1ms Finalize 预算试验。已有内容已收敛，报告仍为 succeeded，
总耗时约0.37秒；静止/修复阶段预算不包含另有预算的持久化和停止尾段。因此该运行没有
计作超时负例。负例来自受控 worker 截止时间；SDK 原有 Finalize 超时测试也通过。

低优先级待加强：进程测试可进一步直接断言 `repeated`，以及超时 reason 和
`worker_stopped`；本轮记录和汇总核对已保存这些事实。该建议未作为额外改动加入。

## 复现与证据

设计与实施步骤见[设计](superpowers/specs/2026-11-26-graceful-shutdown-design.md)及
[计划](superpowers/plans/2026-11-26-graceful-shutdown.md)。在已配置对应 ROS 的构建目录运行：

```bash
ctest -R 'shutdown_' --output-on-failure
```

Noetic 进程测试自行启动私有 master；Humble 使用本地 DDS 域。构建需包含实际节点、
生成消息及测试目标。受控单项也可显式运行：

```bash
python3 tests/ros_common/shutdown_process.py --ros-version 1 \
  --executable /path/to/fastlivo_mapping --output /new/private/test-output
```

本机证据位于 `validation/graceful-shutdown-20261126/`：两端原始失败、复核失败、最终
CTest、进程 result.json、真实 GPU 参数/接收记录、worker 命令和日志分别保留。
`noetic-final-publication.json`、`humble-final-publication.json` 保存最终接收证明。
只复用既有构建并生成短段数据；原 bag、此前完整回放和参考数据保持不变。
