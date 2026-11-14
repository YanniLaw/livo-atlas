# P6-02c2b 验证记录

2026-11-14。主验收产品冻结于 `a7df648e52ca91d7807672ece8d8f15891c2a2d1`；
独立审阅后的检查器修复为 `65c69453fd3c655e461f737ec024bf4f725cdf8c`。
前一阶段 C2a 产品为 `8d4bcf6`，C2b 实施前文档提交为 `d120433`。
状态：本阶段已完成。主验收14组构建/安装消费、8次官方回放及故障数据通过；
整分支独立审阅的两项 Important 已修复，修复后的7组受影响构建/安装消费及19项数据检查通过。

范围为拥有的读取前沿、输入封存、有界 worker、持久接纳/只读检查、唯一所有者协调及共用 ROS 入口。
没有修改 C2a 算法与实数据默认门限，没有向原始前端状态回写优化结果。
实现与接口见[使用说明](online-loop-backend.md)、[设计](superpowers/specs/2026-11-14-p6-02c2b-online-backend-design.md)
和[实施计划](superpowers/plans/2026-11-14-p6-02c2b-online-backend.md)。

## 源码与执行条件

证据根目录为工作区 `validation/P6-02c2b-20261114/`。
`product-source.tar` 来自主验收产品提交，499 个 archive entries、427 个普通文件；
Noetic 和 Humble 冻结副本均逐文件比对通过。最终文档另行提交。
`review-fix-source.tar` 单独保存 `65c6945`，501 个 entries、429 个普通文件、无符号链接；
比主归档多出已提交的使用说明与验证报告，两端副本再次逐文件比对。不覆盖原归档与验收日志。
构建、测试、回放分开串行执行；重型构建 nice10/-j1，回放 CPU0、OMP_THREAD_LIMIT=1、1倍速、图像队列32，无 RViz。

依赖沿用本机 Noetic、现有 ros2 容器中的 Humble 与 GTSAM 4.2.2。
SDK 仍可在没有 ROS 的配置下构建，未修改工作区的 vikit 包。
各回放的 runtime-artifacts.json 保存实际执行文件、19 个 livo-atlas 共享库及 GTSAM 的 SHA256，
runtime-conditions.json 保存 CPU、OMP、nice、进程 RSS/线程峰值和总壁钟。
本轮八次回放使用同一双开构建切换运行参数；运行关闭时没有 loop_closure 目录和后台会话。
全关构建的 GTSAM 缺席由构建/安装消费矩阵另外检查。

## 官方数据回归

使用当前唯一的 Retail_Street.bag，完整回放请求136秒，短段3秒。
原 bag SHA256 为 `1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
所有八组都满足正常退出、输入计数无缺失，四组前端数组逐项相等，发布摘要相等，原始资产逐字节相等。
每组均对照同平台 C2a 的全关和双开基线；没有使用跨平台数值相等作为前提。
新增 online_submaps/poses 与 loop_closure 记录由检查器单独核对。

| 平台 | 运行后台 | 段长 | LIO / VIO | raw / 成员索引文件 | 旧导出文件 | 实测 RSS峰值 MiB | 总壁钟秒 |
| --- | --- | ---: | --- | --- | ---: | ---: | ---: |
| Noetic | OFF | 136 | 1351 / 1350 | 517 / 186 | — | 793.63 | 143.34 |
| Noetic | ON | 136 | 1351 / 1350 | 517 / 186 | — | 812.70 | 143.63 |
| Noetic | OFF | 3 | 26 / 25 | 7 / 4 | 57 | 196.61 | 10.31 |
| Noetic | ON | 3 | 26 / 25 | 7 / 4 | 57 | 204.79 | 10.58 |
| Humble | OFF | 136 | 1351 / 1350 | 517 / 187 | — | 829.39 | 143.68 |
| Humble | ON | 136 | 1351 / 1350 | 517 / 187 | — | 851.49 | 143.45 |
| Humble | OFF | 3 | 26 / 25 | 7 / 4 | 57 | 218.75 | 7.98 |
| Humble | ON | 3 | 26 / 25 | 7 / 4 | 57 | 223.65 | 7.97 |

RSS 是本次采样观测值，壁钟包括启动与收尾，不能解释为求解硬截止或一般地图内存界。
源数据、计数、数组形状、摘要及执行条件保存在各 replay 目录。

## 在线接纳与真实尾帧增长

| 平台 / 段长 | 帧数 | 提交/完成任务 | 接纳/拒绝 | stale / skipped / resource | 最终位姿版本 | 载荷槽/待查峰值 |
| --- | ---: | --- | --- | --- | ---: | --- |
| Noetic / 136 | 172 | 13 / 13 | 2 / 11 | 0 / 0 / 0 | 2 | 1 / 1 |
| Humble / 136 | 172 | 14 / 14 | 2 / 12 | 0 / 0 / 0 | 2 | 1 / 1 |
| 两端各自 / 3 | 2 | 1 / 1 | 0 / 1 | 0 / 0 / 0 | 0 | 1 / 1 |

两端 ON 检查均返回 consistent/0，summary.complete=1，prepared 与正式 pose 连续配对。
Noetic 两版的 N→M 为 141→142、169→169；Humble 为 135→135、168→169。
因此官方 bag 实际覆盖了在途任务期间新增尾帧的接纳，原始前端四数组仍严格保持一致。
任务数和接纳数属于此次调度观测，未来回放不以固定数量作为成功门槛；确定性 fixture 另要求真实 C2a 正向提交。

## 确定性边界与启动检查

测试覆盖固定读取前缀、writer 销毁后的视图、PCD预算/非有限值、封存后提交、最后关闭、共享容量1、
慢任务 join、bad_alloc/未知异常、同版本增长前沿、纯时效检查和 projected tail。
所有者测试经真实 Store/Manager 追加三维数据并运行实际 C2a，覆盖最新查询合并、完整元信息前缀、
跨帧/回环预算、等待时接纳、独占位姿写入、观察器失败、析构丢弃以及发布前/后的失败。

持久记录用例包括 prepared-only、pose配对、版本/来源/噪声篡改、不可变记录、摘要替换失败、
未封存/未索引尾部、不读点云检查及 CLI 0/3/1/2。
补上 C2a 延期的实际 CLI「预览拒绝→后续查询成功」，只在合成 fixture 中设置区分阈值，产品默认值保持原样。

本轮失败复现与修复均保留日志：成员首ID越界访问、约束与原验证矛盾、pending目录误解析、
错误 runner 噪声、缺观察回调、查询诊断缺失/前沿矛盾，以及 /clock 暂停导致主循环停止处理。
最后一项在真实 main 上从0个位姿复现，修复为 runtime ON 的壁钟等待后，同样的2帧数据在没有 /clock 消息时输出2个位姿。
启动检查确认选项错误/缺失与不合法存储在输出 I/O 前退出；关闭后台时忽略不存在的选项文件。

## 证据与复查

SDK 六组合已全部通过，每组外部安装消费者另通过 1 项测试：
下列两张构建表及八次回放均对应主验收 `a7df648`，审阅修复的增量复验另列在后文。

| SDK 模式 | CTest / XML程序 | XML根用例 |
| --- | ---: | ---: |
| 双开 shared | 52 | 432 |
| 全关 shared | 37 | 320 |
| 仅 GTSAM | 39 | 333 |
| 仅几何回环 | 42 | 353 |
| 双开 static | 52 | 432 |
| 双开、无 OpenMP | 52 | 432 |

七个可保存数据集的 CLI 退出码已实测，输入文件 SHA256 和目录清单前后相同：

| 数据集 | CLI退出码 | 已提交版本 |
| --- | ---: | ---: |
| empty-complete | 0 | 0 |
| normal-proposal | 0 | 1 |
| prepared-only | 3 | 0 |
| published-notification-failure | 3 | 1 |
| unknown-worker | 3 | 0 |
| unsealed-exit | 3 | 0 |
| missing-prepared | 1 | 0 |

故障副本位于 fault-datasets/，报告位于 fault-inspections/，字节与目录核对记录为 fault-cli-checks.json。
双 ROS 的 shared/static、全关/双开矩阵均已通过：

| 平台/模式 | 程序数 | XML根用例 | 另计实际ROS1用例 | 安装消费者 |
| --- | ---: | ---: | ---: | ---: |
| Noetic shared 双开 | 53 | 452 | 12 | 1 |
| Noetic shared 全关 | 38 | 340 | 12 | 1 |
| Noetic static 双开 | 53 | 452 | 12 | 1 |
| Noetic static 全关 | 38 | 340 | 12 | 1 |
| Humble shared 双开 | 55 | 473 | — | 1 |
| Humble shared 全关 | 40 | 361 | — | 1 |
| Humble static 双开 | 55 | 473 | — | 1 |
| Humble static 全关 | 40 | 361 | — | 1 |

61 个普通头独立编译通过。C++ XML 无失败和跳过；另跑 Python共20项，19项通过、1项按环境条件跳过。
每个模式都检查 online_backend 的存在/缺席；静态消费没有加载 livo-atlas 共享库，
ROS全关节点没有加载GTSAM。汇总与原始根计数见 final-evidence.json。

Noetic ON 最终通信测试最初继承了沙箱网络限制；原失败日志保留，成功重跑使用 finalnetwork，
XML来源在 matrix-noetic-on-final-resume.json 和汇总中明确记录。
其余沙箱/Docker权限失败、验证脚本的工作目录/日志缓冲/检查器路径修正也保留原始记录，
未把这些失败当作产品成功，也未以改门限或放宽回归条件消除问题。
SDK/ROS 外部消费者实际创建空会话、Finish 并检查 consistent，静态消费覆盖私有 I/O 源码的传递链接。
只统计各 XML 根的 tests/errors/failures，实际 ROS1 rosunit 单列，避免重复累加 catkin 包装层。

## 独立审阅与修复复验

仅进行一次新的整分支独立审阅，范围为 `d120433..d113d8d`，原报告保存在证据根的 final-review.md。
没有 Critical、Minor 或 Declined-to-judge；两项 Important 均先在当前代码上观察测试失败，再修复：

1. 截断 prepared.diagnostics，或把候选/图试算/几何复查的嵌套记录改为标量，旧检查器仍可能返回 consistent。
   修复增加 prepared/query 共用的阶段相关结构校验，检查有限数值、来源修订、图残差和原验证。
   这验证审计记录的结构及上下文，不重新证明几何匹配的正确性。
2. complete=0 时，已提交任务的现有 query 可以写成 stale，旧检查器只报告 incomplete。
   修复始终检查现有处置与提交权威一致；缺少最终 query 的合法发布后中断仍保留已提交前缀。

新增四个用例，包含合法的 insufficient_points 早期拒绝，确保尚未计算的可选诊断不被强制要求。
review-red-test.log 记录三个新用例的实际失败；review-diagnostics-green-test.log 记录第一项修复后的通过。
第一次完整回归暴露了 OpenCV FileNode::empty 与零长度序列含义不同、以及新增合成选项组合不合法的问题；
保留原日志后，改用序列 size 并修正 fixture 的 min_points/max_points，最终 review-fix2-suite.log 为52/52、436根用例通过。
没有改算法默认值、序列化格式、所有者接纳路径或调度策略。

修正后的检查器复查八份回放数据（四份 ON 实际 CLI，四份 OFF 核对无会话）、七份故障数据和四份审阅探针，
共19项通过，其中15次实际 CLI；所有文件 SHA256 与目录清单前后相同。
原七份故障退出码保持；审阅的截断诊断/损坏嵌套诊断/矛盾处置均改为1，矛盾处置仍报告 committed_version=1。
控制样本仍返回0。完整记录为 review-fix-inspections/results.json。

受影响的双开 SDK（shared/static/无OpenMP）、Noetic/Humble（shared/static）共七组重新构建、测试并安装消费。
每组均通过1项外部安装消费者，静态消费者不加载 livo-atlas 共享库，所有消费者能解析 GTSAM 依赖：

| 修复复验模式 | 程序数 | XML根用例 | 另计实际ROS1用例 | 安装消费者 |
| --- | ---: | ---: | ---: | ---: |
| SDK shared 双开 | 52 | 436 | — | 1 |
| SDK static 双开 | 52 | 436 | — | 1 |
| SDK shared 双开、无OpenMP | 52 | 436 | — | 1 |
| Noetic shared 双开 | 53 | 456 | 12 | 1 |
| Noetic static 双开 | 53 | 456 | 12 | 1 |
| Humble shared 双开 | 55 | 477 | — | 1 |
| Humble static 双开 | 55 | 477 | — | 1 |

复验结果与 XML 根计数记录在 review-fix-evidence.json；单开/全关不会编译本次修改的私有 I/O 源码，沿用主验收结果。
本次修复只改变停止后的数据集检查器，八次真实前端回放不重复执行；以上八次结果明确属于主验收源码。
全部46个本阶段 C++ 文件的 Google 格式与函数间空行检查通过，最终文档本地链接和 git diff --check 通过。
文档索引已删除重复的 C2b 条目，总体 C2 设计的“尚未编码”状态已同步更新。

两项执行取舍保留在工作日志中：为可重复 ON/OFF 验收给既有回放工具增加 --loop-closure，
代价是新增 CLI 依赖/默认行为回归风险，以参数测试及八次实际回放覆盖；
将唯一整分支审阅安排在 Task7 内、最终收尾提交前，代价是审阅后的报告编辑由作者核对证据和链接，没有第二次独立审阅。
没有延期的 Minor，也没有隐去未判断事项。

迁移电脑时需另行复制包外证据和相应 C2a 参考数据，修改构建脚本中的本机依赖前缀，并在目标平台重建。
本轮约135秒数据不能证明长距离完整 SLAM 鲁棒性或在线 3DGS 效果；仍不包含恢复续写和 fsync 掉电事务保证。
