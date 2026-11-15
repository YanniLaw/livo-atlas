# 首帧传感器暂存验证

本轮使用 2026-11-15 的源码 `082645b` 构建节点；后续文档提交不改变节点代码。验证产物位于
工作区包外 `validation/startup-sync-20261115/`，包括原始回放日志、输入/源码 SHA-256、对照
脚本和结果。移动电脑时需要单独复制这些产物与 bag；它们不随源码仓库提交。
原始 `Retail_Street.bag` 的 SHA-256 为
`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
旧 C3b-02 对照文件和本轮验证工具的单独哈希在 Noetic 回放和初次比较之后才固定；已有原始
运行记录和工具仍保留，但这些哈希不能反证固定前文件从未被修改。

## 受控输入与修复前证据

从官方 Retail_Street 的前约 3.2 秒生成两份短包，三条话题的消息内容及各话题内部顺序相同：

| 话题 | 消息数 | 序列 SHA-256 |
| --- | ---: | --- |
| `/livox/imu` | 647 | `0a7f7427017b2fe76cb078eced54d113ed53209faccba63d3869d2e6e20bb225` |
| `/livox/lidar` | 32 | `18493d544f32116e0dc98c7da981d09fd5546134cd0cccdec4012423e1915a92` |
| `/left_camera/image` | 32 | `14368255bfc45a883d2f5dbca8f536334766135bdbf8ebee73c43b82653dcc74` |

两份包仅改变首条激光消息的播放时间：分别在原始起点前 0.1 秒、起点后 0.08 秒；首张图像在
两种场景均移至起点后 0.095 秒，以隔离 IMU 的启动顺序。消息的传感器时间和内容均未修改。
这是人为构造的顺序实验，不是新的实测数据集。早包和晚包的文件 SHA-256 分别为
`cc86778f17cdc7dcd89c7b02460a9464fb7e47f314ae945e7cc7c1f536157cec`、
`e42d5c6a970c07613645f96bd5942ec5d44fed49445956ce8428b3e4ac92c703`。

修复前的 Noetic 节点实际接收到相同数量的消息，但首条进入初始化的 IMU 时间分别为
`946685437.510941982` 和 `946685437.620972037`；LIO/VIO 更新分别为 29/29 和 28/28，
四组导出数组均不相同。逐条接收和初始化跟踪、比较结果保存在 `before-*.trace` 和
`before-reproduction.json`。上一阶段另一次异常没有逐条接收记录，不能反推其确切回调顺序。

## 验证方法

Noetic 在 Ubuntu 20.04.6、GCC 9.4 的主机运行；Humble 在 Ubuntu 22.04.5、GCC 11.4 的
容器运行。独立源码副本分别编译两端节点；核心与可选 GTSAM/回环库一起构建。所有回放串行
执行，以 1 倍速运行，映射进程固定逻辑 CPU 0，`OMP_THREAD_LIMIT=1`、图像发布队列深度为 32。
`runtime-artifacts.json` 记录实际节点和加载库的哈希，`runtime-conditions.json` 记录进程资源
与调度条件；不要仅凭脚本命令推断加载了哪个版本。

短包比较同平台四组数值数组、发布里程计的数值、点云/图像语义摘要和保存产品。Noetic 发布消息
的时间戳使用模拟接收时间，因此里程计比较只去掉该时间列；`trajectory.tum` 的传感器时间仍参与
精确比较。完整包兼容模式还比较同平台旧基线的原始关键帧和在线子地图成员文件。新开关在旧基线
中不存在，配置比较只排除该一项；不排除其他前端参数。

独立核心的 60 项 CTest、Noetic 的 28 项节点/回调测试和 Humble 的 63 项 CTest 均通过；
两端实际节点的动态依赖检查未发现 vikit 或未解析的库。Humble 首次编译期间主机关机，保留了
中断日志；重启后重新校验源码并完成增量构建、安装和整套测试。

每次检查观察者和节点各自的三个话题计数、正常退出、有限值、启动溢出和释放拒绝。观察者计数
只能说明消息到达观察者；节点回调计数用于检查消息是否到达节点，二者均不等同于所有消息都参与
滤波。实验没有真值，仅验证输入处理与数值行为，不能证明绝对定位精度。

## 实际结果

Noetic 受控短包开启缓冲时，早到和晚到场景均为 29 次 LIO、29 次 VIO；四组数组、发布里程计、
点云/图像语义及 65 个导出文件完全一致。晚到时暂存 22 条 IMU，早到时暂存 0 条，均无溢出或
释放拒绝。关闭缓冲后两种场景分别为 29/29 和 28/28 次，四组数组分别与修复前对应场景一致。
激光早到时开启和关闭缓冲也严格一致。

Humble 短包也得到同样的 29/29、29/29 次开启结果和 29/29、28/28 次关闭结果；开启
时晚到暂存 22 条 IMU，早到暂存 0 条，均无溢出/释放拒绝。开启的早/晚四数组、发布里程计、
点云/图像语义和 65 个产品逐项一致；激光早到时开启与关闭也严格一致。转换工具逐条核对了
ROS1 到 ROS2 的载荷与播放时间，转换后的两份短包还单独核验了各话题 CDR 消息序列一致。

Noetic 完整原始 bag 两次均收到 27,447 条 IMU、1,355 帧激光和 1,355 张图像，并正常退出。
关闭缓冲时 1,351/1,350 次 LIO/VIO 与旧基线四数组、发布语义、里程计、517 个原始关键帧
文件和 186 个在线子地图索引文件完全一致。开启时暂存 20 条 IMU，无溢出/拒绝，得到
1,352/1,351 次更新；以相同传感器时间对应的 1,351 个位置计算，与关闭缓冲的中位、最大和末尾
位置差分别约 0.047、0.167、0.012 米。此差异反映初始化输入变化，并非真值误差。

Humble 完整包的输入数量、缓冲统计和更新次数与上面的 Noetic 结果相同。关闭缓冲时，四数组、
发布语义和里程计与同平台旧基线一致，517 个原始关键帧文件与 187 个在线索引文件逐字节一致。
开启时同时间对应位置的中位、最大和末尾差约 0.047、0.161、0.013 米，关键帧从 172 帧变为
169 帧；这些是新旧处理结果的差异，不是绝对误差。四次完整包节点峰值采样 RSS 约为
789–825 MiB，不构成对其他场景的内存上界。

## 跨电脑复现

先按[双 ROS 构建说明](ros-builds.md)分别在 Noetic 和 Humble 上构建；构建所用源码版本、GTSAM
开关、编译器与依赖版本均会影响严格数值比较。准备不与运行脚本所在源码相同的 Noetic 验证
工作区副本 `LIVO_TEST_WS`；可参考[源码副本步骤](validation.md#2-创建源码副本)，其中历史
vikit 链接对当前工程不是必需的。在副本的 `config/avia.yaml` 的 `common` 中分别设置
`buffer_before_first_lidar: true` 和 `false`。保留配置副本及其哈希，不要覆盖已有结果。
`LIVO_PACKAGE` 是源代码包目录，`LIVO_BAG` 为原始 bag 路径，`LIVO_OUTPUT` 为尚不存在的
输出目录。以下为完整包 Noetic 示例：

```bash
source /opt/ros/noetic/setup.bash
export OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
nice -n 10 /usr/bin/python3 "$LIVO_PACKAGE/tools/run_baseline.py" \
  --workspace "$LIVO_TEST_WS" --bag "$LIVO_BAG" --output "$LIVO_OUTPUT" \
  --duration 136 --rate 1 --cpu-affinity 0 --image-queue-size 32 \
  --port 11719 --semantic-output --geometry-submaps
```

如 CPU 0 不在当前进程允许的 CPU 集合中，应改为实际可用的 CPU，并记录该差异。运行受控短包时
将 `--bag` 改为相应短包，使用 `--duration 5 --save-products` 并去掉 `--geometry-submaps`。
两种配置及两个顺序须使用四个不同输出目录。生成短包的
`validation/startup-sync-20261115/make_startup_bags.py` 需要 ROS1 的 bag 和消息环境；复制该
脚本到新的证据目录，修改其中的原始 bag 路径后执行，并核对生成的每话题摘要。脚本拒绝覆盖
已有短包。

Humble 环境需要官方 Livox ROS2 消息定义。可用仓库的 `tools/convert_ros1_bag.py` 将原始包和
短包分别转换为 rosbag2；该工具要求隔离的 `rosbags==0.10.9`，并逐条核对转换后的载荷和时间。
source Humble、Livox 消息及 livo-atlas 安装环境后，完整包回放示例为：

```bash
export OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 ROS_LOCALHOST_ONLY=1
nice -n 10 /usr/bin/python3 "$LIVO_PACKAGE/tools/run_ros2_baseline.py" \
  --bag "$LIVO_ROS2_BAG" --executable "$LIVO_INSTALL/lib/livo_atlas/fastlivo_mapping" \
  --config "$LIVO_CONFIG" --camera "$LIVO_PACKAGE/config/camera_pinhole.yaml" \
  --output "$LIVO_OUTPUT" --domain 91 --duration 136 --rate 1 \
  --cpu-affinity 0 --image-queue-size 32 --geometry-submaps
```

短包同样切换为 `--duration 5 --save-products`、去掉几何子地图选项。选取未占用的 DDS domain。
保留每次回放的配置、输入哈希和实际二进制哈希；在另一台机器上应先比较同一机器内的早/晚顺序，
不要把跨 CPU、编译器或 DDS 实现的逐位一致性当作本轮已经证明的结论。
