# P6-03c3c2b：有版本的双 ROS Gaussian 预览

2026-11-19，基于 `69c0e0e`，运行算法基线 `e20385a`。
用户本轮明确选择“这次直接设计并实现”，沿现有边界自主实施，不再停在模型切换点。
下述消息、参数和队列细节是执行者的工程选择，未冒充用户逐字段确认。

## 1. 目标、路线与范围

把 owner 已接纳、拥有 RGB 的 GaussianRender 发布到 ROS1 Noetic/ROS2 Humble。
图像、来源、相机和失效状态必须可以按版本一起解释；慢消费者不能卡住前端或 GPU 通道。
继续同一 main、普通 C++ 核心、共享消息转换和两个薄 ROS publisher，无 Pimpl。

选择：自定义 GaussianRenderFrame（含 Image 和状态）作为权威输出，同时镜像标准 Image。
仅 Image+独立状态会遇到跨话题乱序及重复采集时间，无法原子对应版本；专用 RViz 插件扩大范围。
标准镜像方便现有 RViz Image 查看，但只有组合消息能声明某一版本是否 READY。
本阶段不读 PNG/map、不触发额外渲染，不发 CameraInfo/TF，不改变选块/训练/存储 schema。
不增加深度/误差图、自由相机、增密、跨块训练或通用输出框架。

## 2. 来源与生命周期

新增普通 C++ GaussianRenderSnapshot 与 GaussianRenderStatus：
READY=0、WAITING=1、INVALIDATED=2、DISABLED=3、ERROR=4、STOPPING=5、STOPPED=6。
快照含 producer_id/revision（输出控制器赋值）、session_id、generation、map_sequence、
pose_version、frame_count、optimized_frame_count、parent_frame、policy、reason，
以及仅 READY 存在的 shared_ptr<const GaussianRender>。

OnlineGaussianMapper::GetRenderSnapshot() 只能在 owner 线程调用。它读取当前标量、poses
头和已拥有结果；GaussianMapIndex 增加 O(1) sequence()，不为这个接口复制全图或遍历主帧。
READY 要求结果身份与当前头一致；Observe/激活/失败/停止会及时撤回旧结果。
无确认时 WAITING；曾有结果而当前无结果为 INVALIDATED；OOM/写失败为 DISABLED；
Gaussian 会话失败为 ERROR；finalization/finish 期间 STOPPING；join 后 STOPPED。
终端优先 STOPPED，reason 保留失败/禁用原因类别；不把旧 wait_reason 当作当前生命周期。
reason 是闭集 ASCII 代码（最多64字节），READY 为空；原始错误仍在已有统计/日志中。

共享输入结果在全部消费者结束前不可修改。输出只持有内存，不延长资产文件租约。

## 3. 有界 CPU 输出

GaussianRenderOutput(options, Consumer) 提供 Observe(snapshot)、Finish()、GetStats()。
Consumer 是 bool(const GaussianRenderSnapshot&)；true只表示两个 native publish 调用完成，
不是远端收到。false 表示 ROS context 已不可用。用户回调必须最终返回，Finish 会 join。

- 一个 CPU worker、一个执行槽、一个最新 pending；最多持有两个不同 RGB 对象。
- owner 的 Observe 仅检查有界标量/尺寸、比较身份和复制 shared_ptr；不复制 RGB、不调用 ROS。
- 输出实例生成独立 producer_id；每次不同观察分配递增 revision，允许合并造成序号缺口。
- 常规更新按 joint_render_interval 节流，采用 steady_clock；初始状态、READY→非READY、
  ERROR/DISABLED/STOPPING/STOPPED 立即唤醒，待发的失效紧急标记不能被同类更新抹掉。
- worker 转换前检查是否已被新观察覆盖；已执行的 native publish 不能强行撤销。
  所有回调串行，所以旧 revision 不会在较新 revision 之后由本输出调用。
- 回调及完整 DTO 校验在锁外执行，慢回调不持有 owner 所需的锁。
- 非法输入或转换/发布异常只使这个派生输出失败；清 pending/RGB，至多尝试一次无图 ERROR。
  不改变 GaussianMapper、前端、GPU、轨迹或几何输出。
- Finish 撤回普通 pending，安排一个无图 STOPPED，唤醒并 join；失败后的 ERROR 不被伪装成功。
  无任何来源时不构造虚假的 session。析构调用同一收尾，无 detached thread。

统计包括 observed/submitted/coalesced/published/unavailable/failed、当前及峰值 pending/in_flight、
last_revision、worker时间、finished/error；join 后当前计数为0且不持有 RGB。
缓冲上限只约束本进程新增持有对象与显式转换；RMW/连接端缓存另受各自 QoS/连接数影响。

## 4. ROS 消息、坐标与接收合同

新增 msg/GaussianRenderSource.msg 与 msg/GaussianRenderFrame.msg，避开现有 C++ 类名。
Source：submap_id、binding_generation、source_generation、content_job_id、membership_revision、
resident、asset_id、manifest_sha256、frame_ids、gaussian_count。READY有1..2源，每源最多10帧。

Frame：七个状态常量、Header、producer_id/revision、session_id、status/reason、上述版本头、
parent_frame、policy、job_id、query_keyframe_id/query_submap_id、neighbor_reason、sources、
calibration_id、capture_time、k[9]、parent_from_camera(Pose)、changed_pixels、
candidate_count/candidate_pixels/projection_uncertain，以及 Image。

READY：RGB8、is_bigendian=false、step=3*width，最多1M像素/3M字节；共享现有 render DTO 验证。
Header 与 image.header 完全一致，stamp 是查询 capture_time，frame_id 是独立虚拟光学帧。
复用 RosNanoseconds 的跨 ROS 范围 [0,2^31)秒及舍入边界；不以 now() 替换历史时间。
k 为原始整像素中心 K，镜像不再加0.5。parent_frame：odom映射 camera_init，map映射配置的 map_frame。
parent_from_camera 为 camera_from_parent 的逆，四元数归一化作显示；不承诺矩阵逐位无损。
此光学帧不发布 TF、不复用原 live camera TF；标准镜像仅用于 Image 显示。

非READY：job/query、来源、K、采集时间、指标清空；Pose identity；Image 宽高/step/data为0，
Header stamp=0、frame_id仍为虚拟光学帧。时间/DTO错误不能保留上一份 READY 像素。

同一份 src/ros/gaussian_render_messages.h 构造两个生成类型；ROS差异仅 time构造和publisher调用。
话题使用相对名 gaussian_render（组合）、gaussian_render/image（标准镜像），支持 namespace/remap。
Noetic 两者 queue1/latch；Humble 两者 reliable/transient_local/keep_last1。
后加入的 Humble reader 需要请求 transient_local 才保证取得缓存；volatile reader 等新消息。

接收者只对组合消息按 (producer_id,revision) 排序，拒绝倒退/重复，非READY立即撤回图像。
单话题只配置一个本逻辑发布者；不能合并多个发布者的 session/revision。换发布实例须明确重绑定。
跨话题不保证顺序；不以 stamp、ROS1 seq 或镜像 Image 判断版本。镜像失效时发空 Image 替换缓存，
但 RViz 可能保留最后纹理，不承诺自动清屏。

每个 READY 都是 as-of 快照，不承诺传输期间地图从未前进。ROS关闭/链路中断不能保证送达终端帧；
接收者须监控发布者/连接，并在未知存活时停止声称“当前有效”。不改信号处理，不发假黑图或 now 时间。
对活着的 ROS context 测试显式 Finish 的 STOPPED；SIGINT 后只报告实际尝试/不可用计数。

## 5. 参数、接线和兼容

新增 gaussian.publish_preview=false、gaussian.preview_frame_id=gaussian_camera_optical。
共享 LivoConfig 末尾追加 GaussianRenderOutputOptions，保持 OnlineGaussianOptions 聚合布局。
开启要求 Gaussian+persist_submaps+joint_render_enabled 且编译 Gaussian ON；在创建输出前检查。
光学帧名采用1..128字节ASCII非空slash分段，父地图帧同样校验并限256字节，不得与 camera_init/aft_mapped/world/map_frame 重名。
节流直接复用 joint_render_interval（100..60000ms），不增加同义频率配置。

同一 main 创建 publisher 后创建 output；after_process 中先 mapper.Poll，再 Observe(GetRenderSnapshot)。
退出时先停止派生发布，再进入 Gaussian 终端流程；发布对象必须活到 output join 后。
初始及空闲/暂停仿真时钟也照常观察，使用已有 wall-clock调度，不改前端ROS话题。
ROS库只在 Gaussian ON 时链接新增发布实现；OFF继续不依赖 Gaussian/Torch/CUDA/vikit。
纯 C++ DTO/输出类属于可选 online_gaussian 库，ROS生成类型不进入公开算法头。

## 6. 验证与资源

新证据 validation/P6-03c3c2b-20261119；原 bag及全部前序证据/SDK/环境只读。
OMP1在配置前固定；单 compiler/GPU workload，-j1；不 push/merge/tag；仅改 livo-atlas。

CPU用例：生命周期、同序号不同相机、失效优先、阻塞消费者下合并/版本顺序、回调异常、
非法大尺寸/时间/源、Finish join/释放、稳态时钟、旧聚合消费及禁用零工作。
ROS1/2真实收发：RGB/stamp/K/pose/来源一致；后加入订阅者在READY和失效后看到正确缓存；
重复/新旧revision与同capture_time不同job；暂停sim时钟；context关闭不假称已发布；坏DTO清图。

Noetic短段+完整官方bag成对：空间joint渲染与持久训练/Finalize均开，仅publish_preview OFF/ON不同。
原始前端数组/观测/原有话题语义严格相同，最终13块current/durable；收集实际收到的组合消息与镜像。
至少一份输入结束前的READY与owner接纳/PNG的RGB、来源和相机对应；状态revision单调、
非READY无图、晚到旧版本不升级状态，新增发布当前/峰值槽数≤1/1。
Humble验证CPU/消息/入口/安装及实际ROS传输；CUDA证据在主机，明确不声称Humble CUDA回放。
完整组件/工具回归、shared/static OFF/ON SDK和双ROS矩阵；一次新上下文最终审阅，必要修复RED/GREEN。
归档ledger/原始审阅/SHA后，只清理本阶段重复构建/数据，保留目标≤1GiB。

## 7. 依据

[Image](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/Image.msg)
规定采集时间和光学坐标；[Humble QoS](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)
定义缓存/兼容；[rcl publish](https://github.com/ros2/rcl/blob/humble/rcl/include/rcl/publisher.h)
说明可靠发布可能阻塞及对象生命周期要求。单工作线程和版本消息合同是本工程选择。
