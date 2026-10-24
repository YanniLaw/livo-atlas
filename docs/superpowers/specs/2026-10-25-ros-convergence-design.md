# ROS1 / ROS2 接入收敛设计

## 意图与授权

用户要求继续参考 OpenVINS 对两种 ROS 系统的实现收敛。上一轮只统一实际 main 和
旧算法目录：ROS1 LIVMapper 仍独立编排，ROS2 MappingSession 则另行组合同一批组件。
本轮把组合、输入队列/预处理/同步驱动、估计结果消费与传播状态收敛为一份普通 C++。
延续已授权的当前会话直接实施、目录自行安排、本机验证；先文档后编码，Google C++17，
仅修改 livo-atlas，不修改 rpg_vikit，不合并/推送。

参考 OpenVINS 的关键关系：同一 main 创建 VioManager / 配置，再将 manager 交给
ROS1Visualizer 或 ROS2Visualizer；ROS API 留在相应接入实现。借鉴职责关系，保留本工程
独立核心和 mode0 构建，不复制其源码或将 ROS 条件扩散到普通算法。

## 选择与边界（实现选择由代理决定）

采用普通 MappingSession + 两套具体 ROS 适配器。一个入口创建 Session，适配器只接收
引用，不能再自己创建 Frontend、同步器或 writer。相较于将整套运行流程模板化/按 ROS
版本条件编译，这保持 mode0 真正无 ROS，也无需增加通用节点基类、工厂或插件机制。

参数采用一个小的普通 C++ ParameterReader 接口，只有实际需要的标量/数组读取和名称描述；
两个 ROS 读取器各自实现，公共 schema 只写一份参数清单、默认值、标定装配和校验。
不增加 YAML 依赖或新配置格式。保留原 YAML、Noetic 参数路径和 Humble 点号名称。

不修改 IMU/LIO/VIO、地图、相机、同步器、预处理和输出/writer 的估计实现。
不加入 GTSAM、持久关键帧或 3DGS；本轮让它们之后能接到同一个普通运行层。

## 结构与接口

```text
include/livo_atlas/runtime/
  mapping_session.h             # 普通输入/处理/完成接口
  options.h                     # SessionOptions、时间范围、派生配置
  parameter_reader.h            # 两个真实配置源所需的小接口
  mapping_options.h             # 公共参数 schema / 普通应用配置
src/runtime/
  mapping_session.cc
  options.cc
  mapping_options.cc
include/livo_atlas/ros{1,2}/
  mapping_adapter.h
  parameters.h
  measurement_conversions.h
  result_publisher.h
src/ros{1,2}/
  mapping_adapter.cc
  parameters.cc
  measurement_conversions.cc
  result_publisher.cc
src/app/mapping_main.cc
```

同一 `livo_atlas::MappingSession` 拥有 Preprocess、SensorBuffers、SensorSynchronizer、
Frontend、OutputProcessor、ResultWriter 和 IMU 传播状态。复用已通过 P4 的 ROS2 session
内部流程，移除 ROS2 命名空间与目录归属。增加单步入口，以保留 Noetic 调度节奏：

- `MappingSession(SessionOptions)`：先完成普通校验，再产生文件副作用。
- `bool AddLidar(const LidarScan&)`、`bool AddImu(ImuMeasurement)`、
  `bool AddImage(ImageMeasurement)`：普通测量入队，保持既有拒绝/回退/抽帧规则。
- `SynchronizationResult ProcessNext(const ResultCallback&)`：同步一次；就绪时执行
  同一份估计/输出/写入并同步调用结果回调；非就绪返回原同步状态。
- `void ProcessAvailable(const ResultCallback&)`：通过 ProcessNext 处理直到 waiting，
  保留跳过坏帧后继续处理的行为，用于离线/批量消费。
- `std::optional<PropagationResult> PropagateLatest()`、`void Finish()`。

所有调用由同一个执行者串行进行。回调中的点云/图像视图消费期限仍为下一次处理之前；
不提供测试专用缓冲区访问或可修改内部状态的 getter。

`ParameterReader` 的必要方法：Boolean、Integer、Number、String、NumberArray、
IntegerArray、Name。默认值来自普通 options；Number 接受整数/浮点，数组保留数值校验。
`LoadMappingOptions(ParameterReader&) -> MappingOptions` 读取公共字段并装配普通配置。
`PrepareSessionOptions(SessionOptions) -> SessionOptions` 校验并填充 mode/calibration 等派生值，
供公共配置与直接 Session 消费者复用。ROS2 只读声明、QoS、话题展开/remap留在ROS2读取器；
Noetic全局/私有相机命名空间、XmlRpc类型处理和话题校验留在ROS1读取器。

两套 `NodeParameters` 都持有 `MappingOptions mapping`；ROS2再持有自身QoS选项。
`MappingAdapter(Node&, const NodeParameters&, MappingSession&)` 不拥有 Session。
两端的 `ProcessNext()` 调用相同 Session 单步，再发布同一普通快照；`ReportInputs()`
记录节点实际接收的三类输入计数。ROS1 ImageTransport 由适配器持有。

main 中参数读取、Session构造、适配器构造和正常 Finish/输入计数报告共用代码；
ROS版本条件仅选择节点类型/初始化、接入命名空间、执行器和错误报告。
Noetic 保持 spinOnce / 5000Hz 空闲等待的一次处理，ROS2 保持单线程 executor、1ms
wall timer drain 和4ms传播；线程/队列/QoS策略不顺便重写。原节点名/话题/frame/schema保持。

## 时间与行为兼容

保留两次 fromSec/toSec 量化及 driver fix 的运算顺序，特别是 coverage 使用第二次
量化之前的时间、测量使用量化后的时间。范围用普通数值 TimestampRange 传入：
Noetic `[0, 2^32)` 秒，Humble `[-2^31, 2^31)` 秒；普通层不判断 ROS 版本。
保留负时间哨兵约定，不扩大负纪元端到端估计支持。

普通校验统一采用现有 Humble 的有限数值/正图像协方差/非负过程协方差、有效预处理和地图边界。
这会使 Noetic 也拒绝过去可能进入算法的非法配置和非有限 IMU；合法输入必须精确保持。
所有话题/类型/普通选项在 Session 文件写入之前验证；错误配置不能破坏已有输出。
标定缩放只应用一次。空云/坏布局不得先清掉有效回退队列；Hilti计数/传播游标属于实例。

## 构建与迁移

新增普通目标 `livo_atlas::runtime`，依赖现有 core/output/io；core 不反向依赖 runtime。
三模式安装导出同一 runtime，mode0 不查找 ROS。两个接入各自只留 conversions 和 mapper
库，mapper 编译 adapter/parameters/publisher，保持同一节点目标和节点私有版本宏。
删除 ROS2 私有 session 目标与 ROS1 独立 output 目标；不保留重复实现或转发旧类。
移除旧 LIVMapper、ROS1 独立 calibration/voxel readers；其职责分别进入公共 schema 和
ROS1 typed reader。ROS2 私有头迁到规范 include 目录，全部主动引用同步更新。
普通 Frontend 和已有算法库 API不变；原接入内部类/头的使用方按新表迁移。

## 验证与交付

参考提交 `dae47ef4d4c330c8c5ff0c619cfa4a51882eb3d8`。新证据
`validation/P4-convergence-20261025`，新容器候选 `/tmp/livo_atlas_convergence`，P4与layout证据只读。
沿用本机Noetic、已有Humble容器和固定P4依赖/bag。编译nice -n10/-j1，回放与编译串行，无RViz。

1. 以真实普通消费者先证实新 runtime API 缺失，再实现；抽离 session 的普通边界测试
   在mode0/Noetic/Humble都运行。保留ROS消息转换配对测试，拆掉旧ROS1测试对公开内部状态的依赖。
2. 原11项ROS1组合测试的语义逐项映射到同步/前端已有测试、普通runtime新测试和实际ROS1
   输入/发布测试；不为保留数量添加字段测试。覆盖回退前校验、coverage舍入、LO/LIO/连续VIO、
   非有限IMU、跨实例传播/Hilti、无发布结果消费和输出目录；测试矩阵入报告。
3. 两端实际参数读取比较同一配置默认值/覆盖值，整数浮点标定、Noetic相机私有命名空间、
   Humble只读参数/QoS/remap；非法配置与话题对新目录/旧哨兵输出均无副作用。
4. 保留全部原估计/转换/发布测试；新真实ROS适配器通过消息传输推进并发布有限状态。
   两个构建编译同一main，入口只构造一次Session；普通库无ROS宏/includes/runtime依赖。
5. mode0 shared/static/关闭OpenMP安装消费，以及Humble ament安装，实际调用普通runtime推进
   LO/LIO/VIO；全部公开普通头独立编译。算法源/头正文对照参考不变。
6. 官方bag每环境先3s产品后完整136s，与各自layout冻结候选严格比较四数组、发布语义和
   短段57个产品；原始输入/节点接收计数与实际程序/库前后摘要绑定，拒绝以容差掩盖差异。
7. 一次整阶段独立评审，重要问题一次RED→GREEN修复。更新当前文档、迁移表、完整历史bundle
   和空仓库恢复证明；执行记录归档，保留当前分支和工作区，旧Log与vikit保持原样。

## 参考

- [OpenVINS同一入口与共享manager](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/run_subscribe_msckf.cpp)
- [ROS1接入](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS1Visualizer.h)
- [ROS2接入](https://github.com/rpng/open_vins/blob/master/ov_msckf/src/ros/ROS2Visualizer.h)
