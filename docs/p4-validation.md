# P4：双 ROS 接入验证记录

状态：实现、独立评审修复及最终构建的数据重核均已通过。

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
| Humble C++ | 176 项通过，包括转换、session、参数和真实 DDS/schema |
| Humble Python 迁移工具 | 5 项通过，Python3.10 / rosbags0.10.9 |
| Humble 实际启动/退出 | 16 个进程案例通过，坏配置在输出目录创建前失败 |
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
最终完整场景峰值采样 RSS 为781.56MiB。末尾 VIO 数量沿用三队列就绪条件，没有强制补帧；
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
- `products-comparison.json`、`short-comparison.json`、`full-comparison.json`：Noetic 严格比较。
- `humble_*/summary.json`、`native-cli-summary.json`：真实回放与命令行。
- `sdk-*-checks.json`、`source-runtime-audit.json`、`humble-runtime-artifacts.json`：SDK与源码/运行库。

31个原源树 Log 文件和旁边的 rpg_vikit 均未改变。验证脚本、bag、编译目录和迁移工具的
隔离 Python 环境不进入源码仓库；迁移机器时单独复制需要的证据和输入。
修复前四次 Humble 原始结果保存在 `pre-review/`，最终运行逐场记录14个程序/库的
前后摘要。四个最终场景的数值、发布摘要和短段57个产品均精确等于修复前。
完整历史交付文件为工作区 `livo-atlas-p4-20261024.bundle`；恢复检查、摘要和最终提交
记录在 `recovery.json` 与 `completion.json`。迁移时先校验摘要，再克隆 bundle：

```bash
git clone /path/to/livo-atlas-p4-20261024.bundle livo-atlas
git -C livo-atlas switch refactor-fast_livo2
```

## 独立评审闭环

一次 fresh-context 整阶段评审覆盖 `973a5cc..7e55b70`。无 Critical；4个 Important
和1个由执行者升为 Important 的验收工具问题已在 `fc63f65` 一次集中修复，没有二次评审。

- 无效话题：调用 Humble 自身的展开/remap解析，在构建 session 前校验；新目录不创建，
  旧诊断/COLMAP哨兵文件保留；合法私有名称与 remap 正向测试通过。
- 协方差：图像观测协方差必须正，IMU/曝光过程协方差不得负，非法值在文件副作用前拒绝。
- 非有限 IMU：转换层与普通 session 均在入队前拒绝；插入坏包后的有效序列精确等于干净参考。
- 时间修正补测：直接测试 session 的非零偏移、整数秒驱动修正、大纪元、纳秒进位和反向拒绝。
  移除偏移/驱动修正的两个候选变体均被测试识别；54个显式参考时间经独立 Noetic Time 核验。
- 验收假阳性：发布数量、最终 path 长度、发布位姿有限性与四元数归一化纳入成功条件，
  关系按实际 LIO/VIO 更新数量推导，不写死该 bag 的发布数量。

各修复都有 RED→GREEN 或反证→恢复记录，最终 C++176/176、Python5/5。
没有延期 Minor。评审报告及裁定保存在包外验证目录。

## 支持边界

物理驱动、真值精度、单个依赖的数值归因和未测平台/RMW仍需各自验证。
session 由单一执行者调用；保留原前端的负时间哨兵约定，不声明负纪元流的端到端支持。
保留原缓存策略，长期缺流时队列可能增长；自动丢弃/恢复策略需单独设计。
GTSAM、持久关键帧与3DGS仍未实现。

## 实施裁定与代价

1. PCL1.12 新增坐标构造器使原测试的整数颜色构造歧义，显式使用 uint8_t 选择原颜色重载 — 只修复跨版本测试夹具，算法不变 — 若误判原意，会改变夹具颜色，现有颜色断言会检测。
2. 节点 Finish 输出三个输入接收计数，用于 DDS 回放核验 — 外部观察者计数不能证明数据到达节点本身，计数只在 ROS2 接入层 — 若日志格式后续改变，验收 runner 需同步调整。
3. M1 从 Minor 重分级为 Important 并修复 — 无发布时验收工具仍可能返回成功，会误导跨机验证，既有回放有实际输出不抵消工具缺陷 — 若新检查过严，合法的发布抑制场景需调整关系式，按实际 LIO/VIO 更新数制定检查。
4. 物理 Livox 驱动/SDK/设备仍未认证 — 已验证官方消息和实际 DDS，实机联调超出本阶段 — 真实设备仍需独立验证。
5. 轨迹差值不作为真值精度结论 — 没有提供真值，现有实验衡量实现/环境差异 — 不能保证两平台真值精度等价。
6. 数值差异归因于环境组合，不指定某一个依赖 — 同环境完整离线控制均精确一致，未逐个替换编译器/数学库 — 更换单个依赖仍需重做对照。
7. 支持范围限于已测平台/RMW和单执行者 — 其他 CPU/OS/ROS2 发行版/RMW/并发 session 不在本阶段证据内 — 其他环境或并发调用可能失效。
8. 保留负时间哨兵契约 — 协议转换支持有符号时间，但估计调度沿用原前端起始约定 — 负纪元流可能不推进，不声明端到端支持。
9. 保留原缓存队列，长期缺流策略另行设计 — 当前设计与验收使用有限输入，自动丢弃会改变恢复语义 — 持续缺流时内存可能增长。
10. GTSAM/持久关键帧/3DGS继续后续实施 — 用户本阶段目标为前端隔离和双 ROS 接入准备 — 当前交付尚非完整闭环 SLAM 或 3DGS 系统。
11. 本计划 scratch 完成后移至包外执行记录归档 — 多次断电续做与跨机实施需要保留可恢复记录，计划要求归档 — 若判断不合适，会多保留少量本地文件。
