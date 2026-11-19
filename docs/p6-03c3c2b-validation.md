# P6-03c3c2b：有版本双 ROS 预览验证

最终运行源码：`e0a26e9`，基于 `69c0e0e`，分支 `framework_completion`。
实现、使用方法见[预览说明](p6-03c3c2b-ros-preview.md)，边界见
[设计稿](superpowers/specs/2026-11-19-p6-03c3c2b-ros-preview-design.md)与
[实施计划](superpowers/plans/2026-11-19-p6-03c3c2b-ros-preview.md)。
最终结果采用独立审阅修复后的重新构建、回放和接收记录。

## 行为与构建检查

- 原基线72 CTest通过；最终普通核心74 CTest、196 Python/CUDA、124工具测试通过。
  Python/CUDA使用实际GPU，指定`LIVO_REQUIRE_GPU=1`，没有以跳过代替GPU验收。
- 发布控制器16个用例及独立 admission 测试通过：阻塞回调下100次替换、单 pending、
  完整校验期间失效/Finish、同采集时间不同job、状态转换优先和同状态节流、
  非法时间/未来来源/逆位姿溢出、单次终端状态、不可用context、join与像素释放。
- Noetic44个原生用例、Humble47个原生用例通过。包括RGB/K/时间/逆相机位姿/来源、
  uint64大ID、晚加入READY与失效缓存、12种非法DTO清图、活context停止和关闭context。
  原生用例与CTest存在包含关系，不把它们相加成独立覆盖量。
- shared/static × Gaussian OFF/ON四种独立SDK配置和旧C++17消费端通过；
  OFF导出不含Gaussian目标。普通SDK与关闭功能的节点不链接Torch/CUDA/vikit。
- Humble全量78 CTest通过；两端同一main的暂停仿真时钟入口、发布开启与编译关闭
  准入检查通过。编译关闭时在创建输出前报错，两端均恢复ON后复验。
- 新增文件、共用main和修改的参数测试共21个C++文件通过固定clang-format18.1.8
  Google格式检查；`git diff --check`通过。

## 官方 Retail_Street.bag 成对回放

原bag：135.47秒，SHA256：
`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
两侧都打开Gaussian、持久化、空间联合渲染、Finalize；只切换ROS发布。
OMP/BLAS线程为1，节点CPU affinity为0，回放1倍，图像发布队列32。

| 项目 | 10秒短段 | 完整bag |
| --- | ---: | ---: |
| 严格成对比较 | 通过 | 通过 |
| 原始文件数（逐字节一致） | 16 | 517 |
| 实际收到组合消息 | 12 | 216 |
| 输入截止前实际READY | 4 | 68 |
| 截止前与保留PNG完成相机/RGB核对 | 3 | 2 |
| 标准镜像与组合图像匹配（总数） | 5 | 70 |
| native调用完成 | 12 | 216 |
| pending / in-flight峰值 | 1 / 1 | 1 / 1 |
| 不可用context尝试 | 1 | 1 |

前端 `trajectory.tum/mat_pre.txt/mat_out.txt/imu.txt` 数组、原始文件和原有话题语义
严格相等。完整两侧均172帧、13个回环查询、2次接受、pose_version2；最终13块current且durable。
比较器独立核验两侧Finalize，再比较eligible子地图ID集合及current/durable覆盖；
会话、资产ID和异步调度计数允许不同。原始文件数不是关键帧数量。

SIGINT后ROS context已关闭，实际未收到STOPPED，报告明确记为false；没有把native完成计数
当作远端送达。完整校验之后、回调接纳之前的旧revision会被丢弃；已接纳的native调用
可以完成一个旧as-of版本，串行revision和接收端拒旧约束保持顺序。
标准Image跨话题不具版本权威，也不承诺RViz自动清屏。

保留PNG为有限历史，因此仅列明确匹配数量，其余READY验证原owner接纳身份及版本/来源。
接收器拒绝旧job复活、上下文倒退、异producer/session、非法载荷和不同形状的镜像；
启用发布但任一话题零接收会判失败，WAITING-only接收不冒充READY证明。
Humble验证为CPU、真实DDS和入口，不声称做过Humble CUDA bag回放。

## 独立审阅与执行决定

一次全范围独立审阅覆盖`69c0e0e..869581d`，候选运行源码为`01203e2`。
审阅发现0项Critical、11项Important和2项Minor；没有Declined to judge。
11项Important均在一次修复中先复现失败、再通过定向测试，并以`e0a26e9`重新执行
全套、双ROS矩阵及短段/完整bag对照。没有以候选结果替代最终结果。
原始候选表`paired-results.json`保留17/478条接收，最终表为`review-paired-results.json`。

首次交付延期的两项 Minor 已完成[后续补验](#接收证据补验2026-11-19)：

1. 镜像准入错误提示仍含“before cutoff”，实际检查/表格是镜像总匹配数。
2. 全量空间READY可进一步逐条比对原请求中的相机元数据；当前相机/RGB完整核对
   限明确保留的PNG子集，不宣称所有READY像素均经PNG比对。

执行中的6项决定及代价全部保存在`execution-ledger.md`和`execution-decisions.json`：
沿用当前checkout（失去独立工作区隔离）；地图帧限256字节（超长名称需改名）；
所有状态转换优先（频繁切换增加native调用）；STOPPED仅表示预览关闭（需另看Finalize）；
冻结源码期间审阅与剩余矩阵并行（审阅者不背书未完成矩阵，由执行者验收）；
11项修复共享一次全套门禁（组合问题依靠该门禁及矩阵/回放发现）。

## 证据位置与复现

本机独立阶段：`validation/P6-03c3c2b-20261119`。
`review-{short,full}-comparison.json`、`review-paired-results.json`保存最终实际比较；
`product-source.tar.gz`及SHA、`native-libraries.sha256`、`bag.sha256`固定代码/二进制/输入。
`review-final-matrix.log`及`humble-evidence/`保存矩阵；`execution-records.tar.gz`
保存本计划原始ledger、任务门禁输出、审阅和全部决定。

`run-publication-replay.sh short|full off|on`复现实验；运行前改用新阶段路径，禁止覆盖旧结果。
`run-all.sh`执行普通C++、旧SDK及Python工具/GPU套件；编译或GPU任务按workload.lock串行。
清理后的阶段不保留完整构建树；先从源码归档恢复到新目录，再配置本机依赖与构建路径。
保留最终`review-full-on`原始观测和`g-review-full-on`完整13块资产，可继续只读检查。
删除的重复观测、NPZ和构建文件记录在`removed-artifacts.sha256`；
保留文件由`retained-artifacts.sha256`校验，完成状态和大小记录在`completion.json`。
原始bag和上一阶段2681个保留文件全部通过原manifest复核，未改动。

归档整理完成，阶段保留约1022 MiB（小于1 GiB）；主机释放约3.42 GiB，
本阶段Docker目录释放216.45 MiB。历史回放文本、旧静态消费端和Humble运行文件
经逐文件哈希核对后无损压缩，索引见`compressed-artifacts.json`；
最终地图/原始观测保持原目录，可用`verify-retained.py`检查。
本计划scratch已在逐文件核对归档后移除，其他阶段目录保留。

固定归档SHA256：

- 最终运行源码：`63cd40560b7fdce62eef44699a6dd98a3e27ce37a093a2a15f0a018af9995ec6`。
- 执行记录：`b8154b693d6537e2a59e9e92a8726357d7fe326cda738e7972550ca151d511f0`。
- 原始独立审阅：`4a182d097f9287c56ccd58c1a9f5f1a6965861941ff886b926dc625e79214d91`。

运行源码归档固定在`e0a26e9`；后续提交只整理本阶段说明、结果和计划状态。
保持`framework_completion`本地分支，不推送、合并或建立tag。

<a id="接收证据补验2026-11-19"></a>

## 接收证据补验（2026-11-19）

空间 READY 的采集时间、标定 ID、K 和图像尺寸逐条比对已接纳请求中的查询相机。
历史 PNG 淘汰后仍可核对这些字段。全局相机位姿及 RGB 核对依赖保留的 PNG 和元数据。
镜像总匹配数单独判定，诊断分别指出镜像不匹配或截止前缺少 READY / PNG 佐证。

定向用例先复现原工具漏检，修复后通过。全部 125 项 Python 工具测试通过。
用新校验器读原完整回放归档，216 条组合消息中有 70 条空间 READY 通过相机字段核对。
输入截止前的 68 条 READY 中有 2 条还能核对保留的 PNG、RGB 与位姿。标准镜像总匹配数为 70。
原归档的 2082 条文件哈希均通过核对。复验报告在
`validation/P6-03c3c2b-followup-20261119/retained-preview-report.json`。
