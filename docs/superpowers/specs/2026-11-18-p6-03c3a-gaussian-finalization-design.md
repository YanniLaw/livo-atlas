# P6-03c3a：Gaussian 历史修复收敛与最终版本交付

2026-11-18，设计基于 `edd7b66`，产品基线为 `de4f303`。
用户选择先完成历史修复收敛，再研究跨子地图联合渲染。本轮交付设计；
具体接口、默认值和预算属于执行者提案，尚不代表用户逐项批准或代码已经实现。
方案确认后另写实施计划，正式编码前按用户要求通知切换模型。

## 为什么先做这一项

[C2b 最终验收](../../p6-03c2b-validation.md)已经证明单 GPU 历史工作集、拒绝回退、
刚性重用、非刚性重建和持久引用安全。官方 bag 的最终目录有 13 块已确认内容，
4 块 current、9 块 needs_rebuild；确认内容均已落盘。这符合原 Finish 契约，
但还不能给后续联合渲染提供全部已有内容都与最终位姿一致的交付。

本阶段的“收敛”是**已有确认工作集相对于指定最终位姿的状态收敛**，不是优化损失达到
极小值，也不是稠密地图完整、所有关键帧都训练过或定位精度提高。

## 目标、边界与不可接受的结果

- 后端完成后固定完整最终位姿和逻辑分组，在有明确调度时限的终端步骤中修复历史内容。
- 成功时，每个纳入收敛的确认内容都 current、拥有匹配持久资产，最终目录已提交并获确认。
- 分别给出几何 current、持久提交、缺失分组和实际工作集覆盖；不能合并成一个“地图完成”标志。
- 超时和失败保留可检查的确认身份与旧资产，输出真实未完成状态。
- 普通 C++ 核心与 ROS 继续隔离；Noetic/Humble 共用 main、配置及 Gaussian 实现。
- 默认仍关闭 Gaussian 和本阶段步骤；普通 Finish、C1、IPC schema 1、资产 schema 1、
  map.json schema 1、在线前端算法与输出数值契约保持。
- 不增加在线服务、暂停/恢复协议、第二个 worker、多个常驻 GPU 模型或 Pimpl。
- 不做新分组补训练、恢复已淘汰图像、增密、跨块渲染、跨 session 接管或整套 SLAM 恢复。

不可接受：只清 needs_rebuild 标志、把读取成功当 current、固定旧位姿冒充后端最终版本、
收敛前停止 worker、把 missing 算作已经重建、重新训练所有历史图像、让默认退出隐式等数分钟。

## 方案比较与选择

| 路线 | 收益 | 代价与选择 |
| --- | --- | --- |
| 让现有 Finish 总是重建所有历史内容 | 调用点少 | 改变默认退出和析构语义，修复耗时无法独立报告；不选 |
| 显式、可选的终端 Finalize，然后执行既有持久收尾 | 版本边界、时限和结果清楚，复用现有确认/保存链 | 新增小型结果类型和生命周期分支；本阶段采用 |
| 新进程接管旧目录离线修复 | 与在线退出分离 | 需要新 session 授权、输出接管及错误恢复契约，已超出本阶段；后续再设计 |

用户已认可“优先修复收敛”的方向。显式 Finalize 是执行者根据默认兼容、单 owner 和
避免提前做跨 session 恢复的约束选择的实现路线，不写成用户逐项批准。

## 源码事实与调用顺序

当前 `LivoSystem::SealInput()` 只关闭传感器输入，仍允许后端更新位姿；
`LoopBackend::Finish()` 排空回环任务后调用 `LivoSystem::Finish()`，后者才结束在线位姿更新。
共享 main 已先完成 backend/system，再执行 Gaussian Finish。此顺序必须保留。

新增普通只读 `LivoSystem::IsFinished() const noexcept`，仅在整个 Finish 成功后为真，
不能用 input_sealed 代替；Finish 失败、只调用 SealInput 时均为假。
若后端随后通过现有 FailLoopBackend 记录终端失败，该查询也必须为假，不能只看历史完成位。
它不含 Gaussian 类型，也不让 Gaussian 反向接管后端。默认核心因此增加一个通用状态查询，
代价是本次需要重建普通 SDK/消费者。

启用后的共享入口顺序为：停止 ROS 处理；backend Finish（未启用后端则 system Finish）；
Gaussian Finalize；其余轨迹/几何消费者 Finish。后端异常不得继续宣告 Gaussian 最终版本成功。
ROS 已关闭或仿真时钟暂停时，Finalize 使用 steady_clock/Poll，不再依赖 ROS 事件或模拟时间。

## API 与配置提案

在现有类增加：

```cpp
GaussianFinalizationResult Finalize(std::chrono::milliseconds timeout) noexcept;
```

这是 owner 线程上的**终端、可阻塞**调用，不可在在线通知回调内重入。SDK 调用者先完成
自己的后端和 system Finish；方法检查 `IsFinished()`，并核对 mapper 已观察到的最终位姿
前沿、完整绝对位姿及分组封口状态。未完成输入/位姿生产或前沿不一致时返回失败，
不替调用者关闭后端，也不开始收敛。

| 参数 | 默认 | 规则 |
| --- | --- | --- |
| gaussian.finalize_on_shutdown | false | 为 true 必须同时 enabled、persist_submaps |
| gaussian.finalize_timeout_ms | 60000 | 启用 Gaussian 时校验为整数，范围 1..600000；SDK 参数同界限 |
| gaussian.shutdown_timeout_ms | 原 3000 | 继续只负责后续持久收尾/停止，正式持久验收显式 30000 |

60000/600000 是本阶段实验预算选择，不是 GPU 性能保证。两个新参数由普通配置层读取，
双 ROS 使用同一类型和约束；错误在创建数据集/输出之前报出。编译未包含 Gaussian 时仍
按现有规则拒绝启用请求。标志为 false 时不调用新 API、不生成最终收敛报告。

结果为普通 C++ DTO，放在现有 Gaussian 数据头中；至少包含：

- status：succeeded / timed_out / failed；reason 为有界诊断；target_valid 表示是否完成前置检查。
- session_id、目标 pose_version/frame_count/optimized_frame_count/parent_frame。
- eligible_submap_ids（最多 32）、eligible/current/needs_rebuild/missing 数量。
- eligible_set_frozen：旧任务已排空、E 已固定；排空尚未完成就超时不能把空集合当成功。
- durable、尚未持久化数量、最终已确认 map_sequence、被引用资产 ID 集合（最多 32）。
- worker_stopped、elapsed_seconds、repair_attempts，以及进入/退出时的训练、重建、保存计数，
  便于把在线阶段和结束后修复分别统计。
- report_persisted 与 report_error：报告发布的结果，属于调用返回/节点日志，不在文件中自证 fsync。

不把完整图像、点云、Gaussian 数组、全图参数缓存放进 DTO。实际窗口 ID、参数数目及来源
仍由 map.json 逐块描述，不能由 eligible 数量推断图像或场景覆盖。

前置检查不通过时不改变运行中的 mapper；调用者修正顺序后可重试。一次通过前置检查的
Finalize 就进入终端流程，之后重复调用返回缓存结果，不再次训练/写盘或重置时限。
先普通 Finish 再 Finalize 无法重启 worker，返回 already_finished；Finalize 后 Finish/析构幂等。

## 固定目标与待修复集合

1. 从成功完成的 system 读取完整最终 PoseSnapshot，核对 mapper 的所有绝对位姿与前沿；
   比较数值和身份，不能比较 shared_ptr 地址。GetPoseSnapshot 当前会返回新实例。
2. 逻辑分组必须是完整连续前缀，所有已封口；保存其 ID、成员修订和完整成员身份。
   冻结点包括优化后的尾部投影，而不只是 optimized_frame_count 个前缀。
3. 从此不接纳新前台 view，不再追逐新的版本；合法生产者已经结束。检测到不匹配的通知或
   目标变化时失败，不静默刷新固定目标，也不从头延长时限。
4. 先排空进入时的单个在途命令：尚未发出 accept 的候选一律 discard；已经发出 accept
   的回执无法撤回，必须按确切身份记入确认历史。checkpoint、map_committed、prune 保留原次序。
5. 排空这些旧命令后，取所有有 content 的逻辑块为集合 E（最多 32），再开始历史修复。
   E 包括进入前已发 accept 而此时才确认的内容，不能在入口过早固定而漏掉它。

pending 的未训练前台数据按终端收尾规则放弃；未确认候选不会因为进入 Finalize 被提升。
E 只允许已有内容被重建/替换，不新增分组训练。missing、容量不足和实际窗口覆盖仍完整保留。
E 为空时可以成功完成空集合/目录收尾，但报告 eligible=0；不能称作得到可用重建，真实 GPU
验收正例要求非空。若全部逻辑组 missing，同样不能仅凭 succeeded 解释为地图有覆盖。

## 生命周期与调度

| 阶段 | 允许的工作 | 离开条件 |
| --- | --- | --- |
| 普通运行 | 保留现有前台/历史公平调度 | 显式开始 Finalize |
| 排空旧任务 | 处理旧回执、拒绝未决候选、必要的提交确认 | 没有旧候选、在途前台命令或其接纳回执 |
| 固定版本修复 | 只从 E 选择 needs_rebuild，沿用轮转、单 mailbox 和实际重建协议 | E 全部 current，或时限/失败 |
| 持久收尾 | 保存最后一个 dirty resident、提交完整目录、map_committed、清理替换资产、停止回执 | 原 Finish 链完成或明确 incomplete |
| 已结束 | 返回缓存结果 | 不再接纳新工作 |

实现可选简单 enum 或清晰成员，不能继续让一个 finishing_ 同时表示“拒绝所有候选”和
“允许最终修复”。不增加调度线程、通用任务框架或第三份工作队列。

重建候选仅在目标与固定版本完全匹配、E 内身份正确、时限未到时接纳；commit 回执仍要
完整校验。已发送 accept 的迟到确认继续记账，但不能绕过超时规则获得 succeeded。

沿用原 source canonical 的完整 primary 比较（包括窗口外成员）；纯刚性变化只更新绑定，
不重写数组。组内变化使用保存的至多 10 个视图、每视图 20 步重建。一次成功后该块相对于
固定目标应为 current；仍不一致属于失败，不反复重训到超时或提高位姿门限。
修复前离开 dirty 驻留块仍先 checkpoint；集合修复完后必须主动进入持久收尾，以保存
最后一个 dirty 模型，不能依赖下一次切块触发保存。

保持资产提交 → map.json 文件/目录/父目录 fsync → 完整 map_committed 引用确认 → prune。
第一次 checkpoint 没有旧资产也必须得到引用确认。同步失败或发布不确定时，不继续修复、
第二个 staging 或清理；前端/关键帧/后端结果已经完成，不被 Gaussian 失败改写。

## 完成判据、时限与失败

succeeded 必须同时满足：

1. 固定目标仍有效，逻辑目录覆盖完整最终前沿；E 没有增减。
   eligible_set_frozen 必须为 true；不能跳过排空旧接纳回执这一步。
2. E 每个内容都相对于完整最终 primary 为 current，所有 binding 属于同一最终前沿；
   不要求来源数组的 capture pose_version 被伪装为新版本。
3. 每个确认都有匹配的持久 asset，dirty=0，asset_current=true，最终 map_sequence 已获确认；
   没有未决候选、在途工作、未保存预览、未完成提交或待清理替换资产。
4. 正常 stop 回执匹配，incomplete_shutdown=0；整个修复与准备提交的工作在其预算内完成。

missing 不属于 E，单独报告，绝不自动转成 current。保留视图数小于逻辑成员数不妨碍上述
几何判据，但禁止据此声称全图或全部帧已训练。正式 bag 正例另要求 missing=0。

finalize_timeout 从通过前置检查后开始，用 steady_clock 覆盖旧任务排空、修复和为切块
发生的保存/目录提交，不在每块重置。到期后不再派发新修复、不接受新的候选，转为既有
shutdown_timeout 的持久收尾；即使迟到结果最终碰巧全 current，结果仍为 timed_out。
实现必须在候选接纳、开始下一命令和进入成功收尾时检查截止时间。

正常进入持久收尾的边界是 E 已全部 current、没有待处理的修复接纳回执且尚未到期。
最后一个 dirty 模型落盘、最终目录确认和 stop 属于随后独立的 shutdown_timeout，
可以在修复截止时间之后完成；它们失败则不能 succeeded。不能把“修复做完”提前当作
整个请求成功，也不能因正常持久收尾跨过修复截止时间而反过来误报修复超时。

这是调度/接纳时限，不是硬实时抢占。已运行的 GPU、文件系统调用、既有 job_timeout 和
join 仍可能使墙钟超出两项预算之和；不得通过直接删线程、丢掉已确认身份或删除未知
资产实现“按时退出”。记录实际总耗时，并把普通持久完整性与收敛超时分别输出：
timed_out 且 durable=true 是允许的，不能把它改写为收敛成功。

连接失败、损坏资产、worker 错误、目标不符或提交不确定时立即停止新修复，保留资产，
使用 failed 和具体 reason。已有 Gaussian 失败的对象只做安全终端收尾，不自动重连。
无正常停止回执、持久链未完成时继续设置现有 incomplete_shutdown。

## 最终报告及检查语义

显式请求才新增 `finalization.json`，schema 1、kind=`livo_atlas.gaussian_finalization`，
上限 16 KiB。它引用同 session 的最终 map_sequence、目标前沿、E 和引用资产 ID，包含
上述结果与计数；不复制整份地图，不改 map.json/资产 schema，不新增 C++ 哈希依赖。

所有 worker 工作结束并 join 后，由终端调用通过 io 层写这个小报告；在线 Observe/Poll
仍不写文件。使用独占、拒绝符号链接的 finalization.tmp.json，写入/fsync/关闭、rename、
同步输出及父目录。可复用现有 Gaussian map writer 的持久发布辅助逻辑；仅编入可选库。

报告内容描述已经发生的修复/地图提交/停止事实，不能自证报告文件本身的 fsync 已成功。
若发布前失败，不产生成功报告；rename 后同步失败保留可见文件并返回 report_persisted=false、
report_error=commit_uncertain，不回滚或再次写盘。文件内可能仍描述已成功收敛的地图，
这与调用是否成功持久发布报告是两个结果，不能混淆。

节点显式请求时，只有 status=succeeded 且 report_persisted=true 才以该请求成功退出；
其余结果设置非零退出码，并继续完成其他消费者的必要收尾。普通模式的退出语义保持。
底层报告无法写出时，返回 DTO 和有界节点日志仍包含确切会话、目标、确认/资产身份及失败原因。

检查器不能只信报告 status：复核 session/map_sequence/前沿/E/资产集合、现有 map.json 全部
逻辑分组和 canonical currentness、逐资产 SHA/数组/覆盖及停止统计。最终回放验收同时要求
report_persisted=true、正常进程退出和现有严格前端比较。离线单独检查一个可见报告，只能
验证当前文件及地图事实，不能从文件存在反推原进程曾成功 fsync 或正常退出。

## 接入、统计和磁盘约束

runner 新增 `--gaussian-finalize-on-shutdown` 与正整数 `--gaussian-finalize-timeout-ms`
（默认 60000，同上界限），前者必须同时有持久化和 socket/output 参数。
启用后写入同名节点配置，持久收尾仍显式 30000 ms。

当前 runner 的 SIGINT 等待只有 10 秒。请求收敛时改为至少 finalize 预算加 90 秒墙钟等待，
给原持久收尾、在途 IPC 和后端留余量；原模式仍用原等待。
这不是对永久阻塞的保证；若最终触发 SIGTERM/SIGKILL，记录强制终止，不能判作成功。
ROS smoke/其他复用停止辅助函数同样需要覆盖新模式，不能被外层较短 timeout 杀掉。

保留原在线资源采样格式，另采样停止等待期间的 RSS/CPU，并单独报告在线时段、修复及
总退出耗时；训练/重建计数保留进入和结束的差值。不得用结束后训练结果冒充在线吞吐或画质。

所有原预算继续适用：512 前沿/目录、32 有内容块、单 GPU、单稳定 CPU 确认、跨块最多
额外一个临时 CPU 加载状态、32 引用加 1 staging、单块 10 视图/10,000 高斯。
不复制另一套全图参数或为最终交付永久保留第二套资产；最终输出目录关闭后不再被本进程
修改，后续渲染以只读方式消费它。实时修复过程的中间目录不算已封口快照。

新验证用独立阶段目录，保持原 C2b 正式数据和环境；不要重新生成所有旧阶段。
预期新增产物控制在约 6 GiB 内，保留摘要、最终一组资产和必要对照；中间失败大文件在
保留报告后清理，不再积累多套全量回放。继续使用现有 Python/GTSAM/Sophus，不升级依赖。

## 验收矩阵与主要负例

普通 shared/static、Gaussian OFF/ON，Noetic/Humble OFF/ON、公开头及安装消费者；
新完成状态也要验证普通核心。遵循 Google C++17、函数间空行、直接成员与必要注释。

- 生命周期：仅 SealInput 不足、system Finish 失败不放行、后端最后修正后才固定目标；
  零帧、全 missing、32 内容加 missing、重复 Finalize、普通 Finish 后调用及默认模式。
- 旧任务竞争：view/rebuild 候选未接纳、accept 已发送而回执迟到、空激活、checkpoint/
  map_committed 途中进入；E 不漏晚到确认，不提升未确认内容，最后 dirty 必须保存。
- 几何：纯刚性不重写数组；窗口外成员改变、跳版本、舍入标定/投影尾部仍按原门限；
  非法目标变化不能重置预算或成功。
- 提交：第一资产无 prune 仍确认；最后一块目录提交延迟不能提前成功；map/report 的
  文件同步、rename 前/后目录同步、stop 身份或回执失败；不删旧/新不确定资产。
- 时限：排空阶段到期、训练中到期、候选边界到期、迟到 commit、收尾成功但修复超时；
  保留 timed_out 与 durable/incomplete 的独立含义，不做全局 sleep 假时序断言。
- 比较器拒绝：伪造 succeeded、错前沿/会话/资产集合、remaining 非零、dirty、缺失报告、
  report_persisted=false、错误 stop、无有效训练和强制终止；保留明确 missing/部分覆盖的语义。

正式回放使用同最终源码/二进制/CPU/配置，短段再完整 Retail_Street：OFF、原持久模式、
新收敛模式成组比较，C1 另做短段。四数组逐值、原始关键帧逐字节、发布语义和后端最终
接纳一致；Gaussian 结束后计数及资产允许按声明变化。默认模式不要求历史修复清零。
新模式预期保留 13 个有内容块并全部 current、dirty=0、missing=0、无未引用资产，
再由新 GPU 实例逐块冷恢复；数量变化必须解释真实调度/覆盖，不能降低检查掩盖遗漏。

真实 GPU 另覆盖固定目标多块修复、拒绝恢复和故障；控制故障只作用于本阶段新副本。
Humble 无实际 GPU 时只报真实入口/受控 IPC，不能把 peer 当作 CUDA 证据。
当前短 bag 不能替代长距离精度、稠密质量或长期耐久评价。

## 实施拆分建议与交给构建者的决定

后续计划按“完成状态及结果/配置 → 终端生命周期 → 持久报告 → 双 ROS/runner/检查器 →
真实 GPU 与成对回放”拆分，每步先写能失败的行为测试；最终一次整范围审阅。
这只是设计中的职责顺序，不是已经批准或执行的编码计划。

内部小函数/文件拆分、状态成员的具体表示和日志字段排列可由构建者决定；不能为测试
新增产品注入开关。冻结时点、晚到接纳、完成判据、报告发布不确定性和预算必须按本文落实。

## 可实施性检查

2026-11-18 完成一次独立只读 builder check，结果为“nothing important left”。检查只针对
构建者是否仍需猜测重要行为，不运行构建、GPU 或 bag，不代替后续代码审阅和真实验证。
执行者进一步明确：已记录终端失败的系统不能只凭历史完成位放行，旧任务尚未排空时也
不能以 E 为空宣告成功。这些是前述完成判据的落实，未扩大本阶段功能。

本轮执行者作出的主要选择为显式终端 API、普通完成状态查询、排空后固定 E、分离修复与
持久收尾时限、报告发布单独记结果。它们的代价分别是增加一次可选调用、重建 SDK、终端
排空时间计入预算、实际退出可能超过调度预算、文件检查需与退出/发布结果共同判断。
设计检查记录在包外 `validation/P6-03c3a-design-20261118/`。
