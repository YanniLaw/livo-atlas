# P4：双 ROS 接入验证记录

状态：实现与验收已通过，整阶段独立评审待关闭。

基线为 P3 `973a5cce533da34952bfa33574e2a28de4ead579`。设计见
[P4 spec](superpowers/specs/2026-10-24-p4-humble-design.md)，执行步骤见
[P4 plan](superpowers/plans/2026-10-24-p4-humble.md)，跨电脑操作见
[ros-builds.md](ros-builds.md)。代码保留在 `refactor-fast_livo2` 分支。

## 实现边界

- mode0 普通核心、mode1 Noetic、mode2 Humble；format3 manifest 按 ROS_VERSION 选择依赖。
- ROS2 转换、只读参数、单线程节点、发布器和私有普通 session；共用原 Frontend、
  Synchronizer、Preprocess、OutputProcessor 与 ResultWriter。
- ROS1 委托共享的图像约束校验和 IMU 单步传播函数。估计、地图、相机、预处理、
  输出处理和 writer 的原算法源码相对 P3 没有修改。
- ROS2 复用原 YAML，相机缩放只应用一次；launch 可选可靠 QoS 用于 bag。
- 迁移/回放工具独立于核心运行依赖；没有增加 GTSAM、关键帧存储或 3DGS 框架。

## 环境与固定输入

| 环境 | 编译器/数学依赖 | ROS |
|---|---|---|
| Ubuntu20.04 | GCC9、Eigen3.3、PCL1.10、OpenCV4.2、兼容旧 Sophus | Noetic |
| Ubuntu22.04.5 容器 | GCC11、Eigen3.4、PCL1.12、OpenCV4.5.4、同源旧 Sophus | Humble |

Humble 使用独立临时目录和 localhost DDS domain；实际 RMW 为 `rmw_fastrtps_cpp`。
构建串行 `nice -n10/-j1`，回放与构建分开，无 RViz。

Retail_Street.bag：1,961,140,106 字节，135.470252 秒，30,157 条消息，SHA256：

```text
1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427
```

其中 IMU 27,447、Livox 1,355、图像 1,355。ROS1→rosbag2 转换固定 rosbags0.10.9，
对每条消息往返检查所有载荷字节及 bag 记录时间，移除 Header.seq，并映射 Livox 类型名。
ROS2 消息定义取官方提交 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`。
Humble 原生 rosbag2 Reader 与 `ros2 bag play` 都实际读取了转换结果。

## 构建和测试

| 检查 | 实测结果 |
|---|---|
| Noetic C++ 单元 | 158 项通过 |
| Noetic 真实 ROS / 启动 | 12 / 50 项通过 |
| Noetic Python | 13 项通过；1 个 ROS2 模块因 Python3.8 显式跳过 |
| Humble C++ | 171 项通过，包括转换、session、参数和真实 DDS/schema |
| Humble Python 迁移工具 | 4 项通过，Python3.10 / rosbags0.10.9 |
| Humble 实际启动/退出 | 9 个进程案例通过，坏配置在输出目录创建前失败 |
| mode0 shared / static | 各138项、各25个公共头检查通过 |
| mode0 shared / static / 无OpenMP | 三种移动安装前缀后的独立消费者均推进 LO/LIO/连续VIO |
| Humble mode0 / ament 安装 | 独立消费者推进 LO/LIO/连续VIO；核心无 ROS 动态依赖 |
| 真实 colcon | 单任务构建安装通过；原生 launch、参数服务、bag CLI 与退出通过 |

测试先运行可编译空实现观察失败，再实现行为并验证通过。扩展的 session/发布契约也
分别用空实现反证过。运行库审计绑定四次 Humble 回放的实际可执行文件与14个库/程序摘要。

## bag 回放

| 场景 | LIO / VIO | 位姿 | 结果 |
|---|---:|---:|---|
| Noetic 3s 产品 | 26 / 25 | 26 | 对 P3 严格一致，57个产品逐字节一致 |
| Noetic 30s | 296 / 295 | 296 | 对 P3 严格一致 |
| Noetic 完整136s窗口 | 1351 / 1350 | 1351 | 对 P3 严格一致 |
| Humble 3s 产品 | 26 / 25 | 26 | 全输入、有限数值、正常退出 |
| Humble 30s | 296 / 295 | 296 | 全输入、有限数值、正常退出 |
| Humble 完整136s窗口 | 1351 / 1350 | 1351 | 全30157条输入、有限数值、正常退出 |
| Humble 默认线程45s | 446 / 446 | 446 | 全输入、有限数值、正常退出 |

Humble 同时核验外部观察者的完整序列化输入字节与节点自身三个回调接收计数。
完整场景峰值采样 RSS 为778.51MiB。末尾 VIO 数量沿用三队列就绪条件，没有强制补帧；
45s窗口原 Noetic 参考同样为446/446。

原生 bag CLI 使用默认 best_effort 的一次冒烟出现图像丢包；增加 launch 的可靠 QoS
选择后，经真实参数服务确认生效，8秒墙钟测试收到40帧云、40帧图像，两个进程退出码均0。
可靠回放配置已写入使用说明。

## 跨环境差异及归因

Noetic 与 Humble 使用不同编译器和数学依赖，不能宣称二者逐位一致。
先严格比较，再建立包外离线控制程序：

1. 同一份普通 session 在 Noetic 直接按 bag 顺序处理，3s/完整窗口的四组记录
   精确等于 P3；3s的57个产品也逐字节一致。
2. Humble 同样直接处理 rosbag2，3s/完整窗口的四组记录精确等于实际 DDS 回放；
   3s的57个产品也逐字节一致。

这两组控制把接入/调度与环境计算差异分开，未调整估计数学或放宽同环境阈值。
完整四组记录形状为1351×8、2707×19、2701×20、27365×7。跨环境传感器时间和
全部 IMU 记录精确一致。未对齐坐标，直接比较同时间位姿：

| 窗口 | 最大位置差 | 位置差RMSE | 终点位置差 |
|---|---:|---:|---:|
| 3s | 0.00000316m | 0.000000981m | 0.00000141m |
| 30s | 0.003515m | 0.000620m | 0.000614m |
| 完整 | 0.025269m | 0.003782m | 0.000453m |

完整最大旋转差为0.00117746rad。这是两套环境结果的差值，不是真值精度指标；
记录不足以推断两套系统的真值精度相同。没有验证物理 Livox 驱动、其他CPU或所有传感器。

## 证据与恢复

本机证据位于工作区 `validation/P4-humble-20261024/`，P3 证据只读。主要入口：

- `task4-acceptance.json`、`conversion-summary.json`：完整输入和验收。
- `environment-controls.json`、`cross-environment-metrics.json`：同环境精确控制与跨环境差值。
- `products/short/full-comparison.json`：Noetic 严格比较；对应实际文件名为
  `products-comparison.json`、`short-comparison.json`、`full-comparison.json`。
- `humble_*/summary.json`、`native-cli-summary.json`：真实回放与命令行。
- `sdk-*-checks.json`、`source-runtime-audit.json`、`humble-runtime-artifacts.json`：SDK与源码/运行库。

31个原源树 Log 文件和旁边的 rpg_vikit 均未改变。验证脚本、bag、编译目录和迁移工具的
隔离 Python 环境不进入源码仓库；迁移机器时单独复制需要的证据和输入。
最终评审和完整历史 bundle 恢复结果在本阶段关闭时补入。
