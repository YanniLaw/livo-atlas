# P5-04 图像保存与回放可靠性实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成用户批准的两项可靠性收尾：历史图像关闭错误、回放异常定位与重复/1倍速/耐久验证。
**Architecture:** 一个现有io库内的私有写图函数；临时离线重放与发布探针分层归因；证实缺陷后才补字面回归并最小修正。
**Tech Stack:** Google C++17、原有Eigen/PCL/OpenCV/Sophus、Noetic/Humble；诊断工具留在新验证目录。
**Spec:** docs/superpowers/specs/2026-10-31-p5-reliability-design.md

## Global Constraints

- 基线304ca13739c674b57eb3ffe68132be4491f1bdfa，正常refactor2工作区，仅livo-atlas产品改动。
- Google C++17，六个指定名称与共享ROS main保持；无GTSAM/CUDA/训练、无新物理库/依赖/算法线程。
- 不做猜测性滤波公式修改；不删漏样、改传感器时间、轨迹对齐或放宽比较。
- 原P5-03及更早证据、Log31、bag、其他引用只读；vikit缺失保持，不合并/推送/改标签。
- 新证据validation/P5-04-20261031，容器/tmp/livo_atlas_p5_04；nice/-j1、OMP_THREAD_LIMIT=1，构建/回放串行、无RViz。
- 用户确认无更长连续bag；30分钟合成传感器时间与真实墙钟耗时分开报告。

## Review Focus

1. PNG尾部写入失败时，KeyframeStore不得提交帧；LivoExporter不得伪造成功位姿且必须保持失败状态。
2. 共享写图不能把JPEG等扩展名强制成PNG，灰度/彩色/非连续图像及正常字节行为保持。
3. 启动IMU的接纳与初始化受不同消息到达顺序影响时，必须有样本级因果证据，不能以事后日志相同断言输入相同。
4. 发布成功与观察接收必须分别核对；不同QoS/发布速率不得被混写成同条件的1倍速成功。
5. 重复会话与单进程长时间处理分别验收，合成静止场景不冒充长距离真实定位精度。

### Task 1: 共享写图与历史失败路径

**Files:** 新src/io/image_writer.h/.cc；修改src/io/keyframe_store.cc、src/io/livo_exporter.cc、src/io/colmap_exporter.cc、cmake/core.cmake；测试tests/io/keyframe_store_test.cc、tests/io/livo_exporter_test.cc。
**Interfaces:** 私有livo_atlas::WriteImageFile(const std::filesystem::path&,const cv::Mat&)->void；失败runtime_error包含目标路径。三类调用者保持提交/终止失败策略。

- [x] 写KeyframeStoreTest.RejectsPngCloseFailureWithoutCommittingTheFrame：24×24固定噪声，元数据/云小于1500，正常PNG大于1500；子进程RLIMIT_FSIZE1500且忽略SIGXFSZ，要求异常、无已提交目标帧、旧前缀可读、后续Append/WritePoses拒绝。
- [x] 写LivoExporterTest.ReportsPngCloseFailureAndKeepsTheWriterFailed：实际Write视觉帧和PNG超限，要求异常含PNG路径、后续Write/Finish失败、没有该图像成功位姿行；正对照正常可解码。两项在旧实现应FAIL，保留storage-red.log。
- [x] 写JPEG扩展名与灰度正常文件消费测试，原PNG/调试/目录错误测试保持；JPEG检查真实文件魔数、解码尺寸/类型，不要求有损像素逐字相同。
- [x] 实现私有函数：按扩展名imencode，检查二进制write/close并附路径；不创建父目录、不改变覆盖/重试/fsync规则。三个调用者移除重复实现；加入已有io目标，私有头不安装。
- [x] Google格式，普通全套/45公开头通过；正常PNG与P5-03源图/导出字节保持。提交并运行task gate storage。

Expected：真实PNG关闭错误被两个历史调用者报告；原有成功输出与格式选择保持。
Commit：fix: share checked image writes across all exporters。

### Task 2: Noetic启动与数值差异归因

**Files:** 新阶段diagnostics/native_replay.cc、parameter_reader.h、CMakeLists.txt、run_native.py、compare_trace.py，仅包外诊断；必要产品修正在证实后追加本任务明细。
**Interfaces:** native_replay BAG PARAMETERS OUTPUT DURATION SKIP_STARTUP_IMU；使用现有LivoSystem、真实ROS1转换与冻结配置；N=0正常，N>0仅跳过首个LiDAR之后N个初始IMU接纳，所有原始输入另行计数。

- [x] 复制并适配P4只读离线控制思路，使用公开LoadLivoConfig/ParameterReader从冻结parameters.yaml读取，设置Noetic时间范围；新目录、同一数值配置，无ROS master。
- [x] 记录首200个IMU的原始/修正时间和接纳bool、首扫描边界、结果完整精度gravity/状态与源事件序号；N=0先与冻结8秒前缀对照，确保诊断入口可信。
- [x] N=1/2等单变量控制与P5-03历史失败四数组前缀逐值对照；若匹配，补消息跨主题到达顺序复现，形成causal-startup.json；不把仅相似的曲线当相同根因。
- [x] 如未匹配，按输入接纳→同步组→预处理/初始化→更新首偏差扩展临时源副本追踪，每个假设独立留档。无证据不修改算法。
- [x] 原因确认后，先将精确文件/接口/字面测量/预期行为追加本任务，再写真实RED→GREEN，普通全套保持。最终正常参考的保持或有意纠错差异明确记录。

Expected：证明或排除启动接纳原因，最终给出可重复的数值差异因果与覆盖测试；不能仅靠成功重跑关闭本任务。
Commit：在因果明确后按实际最小修复命名；诊断证据不进入产品依赖。

**已证实的实施补充：** CPU16完整四数组逐值复现P5-03失败，CPU0复现正常；Eigen缓存参数交叉控制反转结果。
不是启动IMU或算法调度修复。修改tools/run_baseline.py、tools/run_ros2_baseline.py；新增tools/cpu_affinity.py、tests/test_cpu_affinity.py。
接口with_cpu_affinity(command,cpu)->list，None保持启动命令；否则校验os.sched_getaffinity并用taskset --cpu-list CPU在exec前绑定。
两端新增--cpu-affinity整数可选参数；summary保存mapping_cpu_affinity（实际允许集）及requested_cpu_affinity。
字面回归：实际子进程第一次执行即报告单CPU、默认继承允许集、负数/不允许CPU在执行前拒绝；旧CLI不支持该选项为RED控制。
最终回放显式CPU0，诊断缓存覆盖不进入产品。提交fix: control CPU affinity for reproducible bag validation。

### Task 3: Humble应用发布与观察损失归因

**Files:** 新阶段diagnostics/publication_probe.cc和诊断构建/运行脚本；复用原output_digest与实际ROS2基线工具，必要产品修正在证实后追加本任务明细。
**Interfaces:** LD_PRELOAD仅注入映射子进程，拦截实际rcl_publish，记录目标topic、调用序号、返回码及图像尺寸/编码/数据摘要；观察端保留原语义摘要，并另存相同数据摘要。

- [x] 小型真实发布/订阅控制校验探针计数/摘要；未注入时不影响原工具和正常二进制。记录工具源码和实际库摘要。
- [x] 在0.5/1倍速真实输入中逐样本比较应用提交与观察接收；必要时只暂停自己启动的观察进程作可控拥塞，定位丢失边界，保存首个差异和QoS/中间件信息。
- [x] 若证实为发布历史/观察处理限制，记录具体配置与受控反例后追加最小修正设计；保持低延迟默认策略与明确的完整记录配置，不能声称有限队列可保证任意阻塞下无损。
- [x] 相关修正先真实RED→GREEN，保留未调整和调整的同条件控制，最终1倍速观察1350图像/2701云及内容必须完整；不从参考文件补缺样。

Expected：区分未提交与提交后观察损失，给出可控因果及有效修正；降速不作为完成依据。

**已证实的实施补充：** 真实1倍速成功提交1350张、接收1334张且四数组精确；受控暂停20次成功提交仅接收最后1张（发布1、接收100）。
修改include/livo_atlas/core/livo_config.h、src/core/livo_config.cc：int image_queue_size=1，读取publish.image_queue_size并在输出前拒绝<=0。
修改两端LivoPublisher头/实现：构造函数新增int image_queue_size=1，正数检查并传给原生图像发布；两端LivoNode传入config.livo.image_queue_size。
新增tests/ros2/image_history_test.cc到现有ros2_node_test，实际节点DDS查询默认1与参数32深度，旧实现32分支应FAIL；
补两端node_options_test默认/覆盖/0和-1/wrong-type路径。共享配置属于接入选项，不放进SystemOptions算法配置。
修改两个回放工具：可选--image-queue-size，参数设置和summary记录；Humble观察输出深度max(20,有效配置)，输入QoS不变。
新阶段相同20张暂停实验使用32验证全部恢复，保持默认1丢失对照；最终各端3次完整1倍速显式CPU0、image queue32。
先记录字面真实RED，再实现、普通/两端测试和同条件控制GREEN；文档明确有界队列的内存/阻塞限制。
提交fix: configure bounded image publication history for recording。

### Task 4: 最终构建、重复与耐久验收

**Files:** 新阶段最终串行运行/验收脚本、tests外的独立soak消费者；新增docs/p5-04-validation.md及使用说明，更新docs/README.md。
**Interfaces:** 读取最终普通配置与公开LivoSystem/Frontend；复用同一快照和安装目标，不依赖内部私有对象。

**合成夹具细节：** 独立安装消费者调用公开Frontend；4Hz、7201次扫描覆盖1800秒，固定三面正交墙共192点，512×512固定纹理针孔图像。
惯性模式使用200Hz静止IMU（零角速度、加速度[0,0,-9.81]）、初始化阈值1；首组25条，后续每扫描50条，累计360025条。
LO预期7201次更新，LIO7200次；LIVO额外7201次图像调用、7200次有效视觉更新并检查实际跟踪覆盖。
current_cloud曲率相对上一状态时间，lidar_cloud曲率相对扫描起点，避免把时间间隙当运动；不重复回退真实bag时间戳。
120秒传感器时间暖机后每30秒采RSS，固定场景最大增量上限8MiB，另有1GiB失败保护；记录墙钟和具体RSS，不称为30分钟墙钟或ROS节点长路线验收。

- [ ] shared/static/无OpenMP SDK、Noetic/Humble单元/真实ROS/DDS与启动检查，迁移后实际三模式和图像/关键帧/子地图/导出消费。
- [ ] 两端短段原57产品/原始资产及六组COLMAP原始/优化实际读取与正常字节对照。
- [ ] 两端各至少3次独立完整1倍速回放，记录全部输入/更新/发布与接收、四数组、实际源码/15运行产物、参数、速率、时序和峰值内存，逐次核对；已证实的有意纠错单独比较原参考。
- [ ] 单进程自洽静止合成LO/LIO/LIVO各至少1800秒传感器时间，状态有限、跟踪有效、处理数量正确；暖机120秒后记录内存，静态工作集增量不应持续随帧数增长；设置仅用于测试的失败保护避免内存耗尽。实际墙钟耗时和资源界限如实记录。
- [ ] 最终完整验收绑定HEAD，记录诊断证据、全部决策和限制；一次独立整阶段评审，重要问题一轮真实RED→GREEN+相关全套，小项留档；完整历史bundle/空bare全引用恢复。

Expected：事项1、2均有实际修复/因果与最终证据；没有连续真实长bag和真值的范围如实说明。
Commit：docs: validate storage failures and repeatable real-time playback。
