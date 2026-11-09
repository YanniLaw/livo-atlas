# P5-01：关键帧快照与持久化存储

## 范围与授权

用户要求按已讨论的P5第一步继续：关键帧选择、独立快照、保存/重读、前端原始位姿与优化位姿版本分离。
先文档、后实现；沿用当前会话直接实施及全部本机验证授权。以下实现细节为执行者选择。
基线为用户创建的干净`refactor2`分支`c84368f`，源码与已验证`62f628e`一致；保留master及v0.1.0。
仅修改livo-atlas，保留六个指定类型名，Google C++17，单任务nice/-j1，编译与bag回放串行。
本阶段交付数据基础，不实现GTSAM、自动回环、子地图融合或3DGS训练，不新增线程/数据库/插件。

## 现状及方案选择

FrontendResult的状态为值，图像和点云是短期视图；world_cloud是已有局部里程计坐标中的几何。
LIVO的LIO与VIO分别更新，图像有效时间包含现有时间偏移和曝光修正；不能把云时间与图像时间默认为同一个字段。
现有Frame/SubSparseMap属于视觉跟踪，不承担持久关键帧职责。

比较过仅在内存积累全部数据与目录式存储：前者不能直接跨进程复用且随运行增长；选择后者。
复用OpenCV FileStorage/PNG和PCL binary PCD，便于直接检查与跨电脑读取；不引入新的序列化依赖。
内存仅持有当前选中快照和ID索引。目录含schema版本，旧前端默认不创建数据集。

## 数据与坐标约定

新增mapping/Keyframe普通结构：id、calibration_id、timestamp、cloud_timestamp、可选image_timestamp、
odom_from_body、可选camera_from_odom、inverse_exposure_time、独立BGR图像和PointCloud::ConstPtr。
所有时间单位秒；timestamp为前端状态时刻，cloud_timestamp为生成该云的LIO时刻，
image_timestamp为现有同步器给出的有效图像时刻，均为已应用现有修正的时间，不声明原始ROS Header时间。

body指IMU坐标，odom指原前端局部坐标（现有ROS camera_init），T_A_B将B坐标映射到A。
CaptureKeyframe(id,result)仅在选中帧调用：clone干净的color_image，逐点把world_cloud转换到
该快照body坐标：p_body=R_odom_body.transpose()*(p_odom-t_odom_body)，保留强度。
该云表示已去畸变的几何，不把curvature/normal解释为额外传感器测量。
camera_from_odom直接保存现有视觉结果中的相机位姿，用于保持原图像位姿关联；原内参及scale保存一次。
图像是现有前端缩放后的带畸变BGR图像，不使用绘制跟踪标记的tracking_image，不重复去畸变。
inverse_exposure_time为前端估计值，保留其有限值而不冒充硬件曝光时间。

CaptureKeyframe对图像和云完全脱离源缓冲区；消费者按只读契约使用快照。
未使用的普通结构复制不提供跨线程同步保证，本阶段所有调用串行。
TransformKeyframeCloud(keyframe,parent_from_body)按给定位姿重建几何；不修改快照和前端地图。

FrontendResult新增cloud_timestamp和optional image_timestamp；Frontend::Impl仅记录LIO云生成时刻，
视觉成功更新时记录当前visual_update_time。只增加输出元数据，不改变估计/同步/量化顺序。
云与图像可以不同时间，按上述几何与各自时间明确关联。

## 关键帧选择及系统集成

KeyframeSelector(mode,options).Select(result)只接收成功且有非空world_cloud的候选。
LIVO只选择Visual并要求BGR图像与image_timestamp；LO/LIO只选择对应LiDAR更新。
首个候选选中；以后相对上次选中帧的平移、SO(3)旋转角、时间间隔任一达到阈值时选中。
默认阈值0.5米、10度、2秒；允许零，拒绝负数和非有限值。
同一时间不重复选择；时间回退选择新起点，ID保持递增，不借此重置或修正前端估计。
非法状态/云/图像不推进选择状态；工厂与存储边界各自验证其所需数据。

SystemOptions.keyframes新增enabled=false和三个阈值。公共参数清单增加keyframes.enabled、
keyframes.translation_threshold、keyframes.rotation_threshold_deg、keyframes.time_threshold。
两端节点仍由同一LivoSystem驱动，无需新增ROS私有处理流程或main。
启用时数据集固定为exporter.directory/keyframes，避免两套输出根目录。
PrepareSystemOptions在组件/I/O构造前校验；KeyframeStore先于现有LivoExporter构造，
数据集已存在时拒绝启动，保护已有数据及原诊断文件。
ProcessNext按原路径完成结果消费，并在下一次估计前选择、快照和同步保存；保留原回调签名。
只有Append成功才递增ID；失败向上报告，主循环按现有异常退出策略停止，不自动吞掉保存失败。
默认关闭时不做数据复制、选择或文件操作。

## 存储接口及格式

io/KeyframeStore(directory,calibration)创建新数据集，目录必须不存在；
KeyframeStore(directory)打开现有数据集，原关键帧只读，可追加新的优化位姿版本，不支持继续Append原帧。
接口：Append(const Keyframe&)、Read(uint64_t id)、ids()、calibration()、
WritePoses(uint64_t version,const vector<KeyframePose>&)、ReadPoses(version)。
无常驻文件流，无需另加Finish。Append/WritePoses返回时已关闭文件并提交；不声称具备断电fsync事务保证。
KeyframePose记录id和map_from_body；version从1开始，0保留给原始里程计位姿。

```text
keyframes/
├── dataset.yml                  # schema_version=1、标定ID=0、相机与两组外参
├── frames/
│   ├── 0/{frame.yml,cloud.pcd,image.png}
│   └── 1/{frame.yml,cloud.pcd}   # 无图像的LO/LIO帧
└── poses/
    └── 1.yml                   # 一个完整的全局位姿版本
```

每个数据集仅使用当前一份固定标定，calibration_id=0；本阶段不设计动态标定注册表。
ID用十进制字符串保存uint64，浮点元数据用max_digits10字符串保存并严格解析，避免YAML实现损失时间/位姿精度。
尺寸、schema使用整数。元数据中采用固定资产名，读取时不接受任意路径。
Append要求从0开始的连续ID，临时目录frames/.pending-ID中写齐资产与元数据，最后rename提交。
打开时忽略.pending-目录，拒绝已提交帧的ID空洞或缺失元数据；资产按Read实际校验。
优化版本同样先写临时文件再rename，拒绝已有版本，覆盖写入时全部关键帧ID且不重复。
版本中记录frame_count，冻结该时刻的完整帧前缀；后续追加新关键帧不能使既有版本失效。
读取者按该版本返回的ID集合重建，不能将尚未覆盖的新帧假装属于旧优化版本。
新版本仅增加poses文件，原帧/标定不覆盖。模拟修正重建云时原始数据及前端状态应保持。
单数据集单写者、串行调用；不承诺多进程并发写入或动态刷新读取索引。

创建/Append/版本写入先验证输入，不合法时不产生输出副作用。
写入中I/O失败保留已提交帧，当前写实例进入失败状态并拒绝后续写入；重开可读已提交数据。
读取拒绝未知schema、非法数值/变换、错误标定、缺失/损坏资产、图像尺寸与标定不符、点云数量/XYZI字段不符。
PCD先检查有界文本头、点数/XYZI字段及实际二进制长度，再调用PCL readHeader及加载；
PCL1.10的readHeader会调整data内存，不能单独作为分配前保护。PNG先检查IHDR尺寸/8位RGB，再解码并检查BGR/尺寸。
所有错误包含数据集/帧/资产路径。不会为了恢复自动删除用户目录或既有版本。

## 文件及构建边界

mapping/keyframe.h/.cc：快照、验证与点云重投影；mapping/keyframe_selector.h/.cc：选择策略。
io/keyframe_store.h/.cc：目录格式、资产I/O及优化位姿版本；无ROS、GTSAM、CUDA类型。
新增livo_atlas::keyframes物理库承载前两项；io依赖它，完整core依赖io及keyframes，防止循环。
catkin/ament及独立SDK均导出实际依赖，ROS库与入口不加入算法实现。
新增公开头与已有头一起独立编译；安装示例实际捕获/保存/重读关键帧并使用另一个位姿版本。

## 验收与后续边界

先重跑稳定基线152项；每个新行为先看到真实RED，再实现并运行全套测试。
测试覆盖选择边界/回退/重复时间，非连续图像stride与源缓冲区修改，非平凡旋转下坐标字面值，
不同云/图像时间，两个模式，精确浮点元数据往返、损坏资产、重复ID/版本、写失败保护、模拟位姿修正。
真实LivoSystem对照开启/关闭的状态与发布序列，默认关无目录，开启得到可重读、独立的关键帧。
Noetic/Humble全套单元/真实ROS/DDS、SDK shared/static/无OMP及ament迁移消费。
官方bag每环境短段及完整段：开启P5并验证每个关键帧可读，同时旧四数组/发布与P4-naming严格一致；
短段旧57产品逐字节一致。另验证默认关接口。编译与回放串行，无RViz。
新证据validation/P5-01-20261026，容器/tmp/livo_atlas_p5_01；旧证据与原Log只读；vikit若存在则只读，当前缺失则保持缺失。
一次fresh整阶段评审，重要问题一次RED→GREEN修复，minor记录；保留refactor2，不合并/推送/改tag。
阶段结束更新数据格式/跨电脑复现文档和完整历史bundle，空bare恢复检查全部引用与最终提交。
后续再加入真实子地图管理、后端因子图与回环、基于选定位姿版本的COLMAP/3DGS导出。
