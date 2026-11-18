# P6-03c3c1：在线一致快照与联合预览设计

2026-11-18，基于 `d25ccdf`。本文件是下一阶段的设计建议，尚未实现在线产品代码。
用户已选择“先最终快照联合渲染，再接在线”；本轮在完成该前置阶段后展开在线方案。
同 worker、首版文件预览、查询/选块策略和下面的具体预算均为执行者建议，待确认。
正式编码前仍按用户要求通知切换模型，确认设计后再写逐步执行计划。

## 目标、现状与路线

目标是在真实输入仍然到达、Gaussian 仍然训练时，从同一份已确认状态渲染至多两块，
证明在线结果的来源、跨块遮挡、过期拒绝和资源边界。这里的在线是接入现有串行训练会话，
不是不断读取正在变化的最终输出目录，也不承诺固定刷新频率或当前视频帧的新视角重建。

已核对的当前实现：

- `OnlineGaussianMapper` 的 owner 线程持有 PoseSnapshot、GaussianMapIndex 和接纳状态，
  一个 C++ I/O 线程通过单 mailbox 与一个 Python GPU worker 通信。
- `GetMapSnapshot()` 仅复制元数据，不拥有资产租约；map/status 文件是异步输出。
  活动确认可能仍为 dirty/volatile，历史资产会在新 map 确认后 prune。
- Python `GaussianSubmapWorker` 保留一个训练模型和一个 CPU confirmed_state；候选训练
  会原地修改 GPU 参数，直到 decision 才成为确认。handle 中普通异常会使会话失败。
- 当前 `preview-*.png/.json` 是单块训练/重用/重建结果；ROS1/ROS2 尚无 Gaussian 图像发布器。
- C3b 的数值组合已验证，但 `load_snapshot()` 明确要求最终收敛、完整且固定的输入，
  不能直接拿来轮询在线目录。只复用相对几何和联合光栅化。
- 现有 `render_blocks()` 重置 allocator 峰值并 empty_cache；在线直接调用会污染训练累计
  资源统计和缓存行为。需要抽取小型共享推理函数，保留离线 wrapper 的原有生命周期。

两条实际路线如下：

| 路线 | 收益 | 代价与建议 |
| --- | --- | --- |
| 在现有 GPU worker 中串行增加只读 render 命令 | 确认边界明确，资产读与 prune 串行，只有一个 CUDA 训练会话 | 会占用训练时间；推荐首版采用，明确调度配额与降级条件 |
| 独立渲染进程消费专门交付的快照 | 渲染故障/刷新可以独立管理 | 需跨进程资产租约、volatile 参数交付、双 CUDA 上下文和全局显存准入；目前单卡基础上工作量更大，留待确有需要时设计 |

建议分两步：本阶段 C3c1 完成在线一致性、有限选块、串行推理和文件预览；后续 C3c2 再加入
空间可见性选块及双 ROS 图像/状态显示。首版仍在两端同一个 main 和同一个 worker 中运行。

## 范围与查询方式

新增默认关闭的 `gaussian.joint_render_enabled`，要求 Gaussian 和 persist_submaps 同时开启。
建议 `joint_render_interval_ms=1000`，范围 100..60000；这是最大派发频率，不是保底刷新率。
建议 `joint_render_timeout_ms=5000`，范围 1..60000；只覆盖已派发命令的执行/返回，不硬抢占 CUDA。
逻辑与路径验证在创建输出前完成；关闭时保持旧握手、调度及输出行为。

查询不是任意新视频帧，而是会话所有确认内容中 ID 最大的 retained 训练帧：

1. 先在所有有确认内容的 entry 中找最大的 frame_ids.back()，包括暂时 needs_rebuild 的内容。
   这是本次期望查询；若其 owner 尚不可用，则等待并记录原因，不退回更早相机冒充最新。
2. 该 owner 必须 current，且要么是 resident 的 CPU 确认，要么有与确认 content 完全相同、
   asset_current=true 的持久资产。非 resident 的 dirty/不匹配资产不得替代较新的确认。
3. 选择集合必须包含查询 owner。额外只尝试它紧邻的前一个逻辑分组；该组满足上述可用
   条件时联合两块，否则明确输出单块并记录省略原因。不跳到更远的历史块。
4. 查询图像、K、camera_from_body、mask、时间和曝光来自该份确认/资产保存的观测。
   最终本次 parent_from_body 来自 owner 捕获的 PoseSnapshot，包括当前版本的投影尾部。

因此历史 repair 临时占据 GPU resident 时，查询不会自动跳回那块的旧相机。
resident 可以是未被选中的第三块，渲染不得将其训练参数偷偷并入联合输入。
本策略是确定性的时间邻接候选集，不称作空间可见集合；几何共视不足可以得到单块或没有
可见贡献，不能悄悄改选块、补训或扩大到整图。

## 一次请求如何固定来源

owner 在安全派发点，从同一份 map 和 poses 构造拥有数据的只读请求，包含：

- session、render job ID、map_sequence、pose_version、frame_count、optimized_frame_count、parent_frame；
- query keyframe/owner ID、对应 parent_from_body；
- 每个选中块的 content ID、完整当前 target（含 canonical primary 局部位姿）、实际 retained IDs 和参数数目；
- 来源类型 resident 或 asset；asset 必须包含确切 asset_id/manifest SHA/字节数及其 content ID。

pending 只表达“有最新渲染需求”，不提前固定可能被 prune 的资产。真正入队时重新选择并
构造请求，要求 map 已由 worker 确认、没有未完成 checkpoint/map_committed/prune 链。
同一 mailbox 一次仅有一项命令；从 render 入队至返回期间，不派发 activate/view/decision/
checkpoint/map_committed/prune。主线程仍可 Observe 新位姿和分组，但它们不会修改已交付数据。
这构成会话内执行期间的资产保护，不把 GetMapSnapshot() 本身称作租约，不支持外部进程
修改该会话资产。无需首版再实现独立资产租约服务。

Python 端使用自身已知的确认与引用表：

1. 拒绝 pending candidate 时的 render；请求来源身份、map_sequence、resident fence、
   已知引用和 SHA 必须一致。只读命令不调用 activate、decide、capture_state 或优化器。
2. resident 使用 CPU confirmed_state，不能读取可能被候选原地修改的 model.parameters。
   asset 仍用完整 read_state 校验，每次只加载一个完整状态。
3. 对所有来源 primary 比较当前 target 的局部位姿，使用既有 canonical 容差；capture
   版本可以较旧，刚性 current 仍合法。任何组内非刚性变化必须先完成既有 rebuild。
4. 从确切查询来源捕获一份观测；本次 C→P→S 相对矩阵、均值和协方差处理沿用 C3b。
   校验 query parent_from_body 与 query owner target 的相应 primary 关系。
5. 先渲染 query owner 单块基准，再单次联合光栅化选中块；两个推理缓冲顺序释放。
   返回联合 RGB8、两路指标、变化像素与来源/资源。alpha/z 在 worker 内计算指标，不逐帧
   传完整浮点图或写 NPZ。深度仍是 alpha 归一化相机 z，训练视图标签保留。

readonly render 可以推进独立的消息 job ID，但不改变训练 generation、确认 content、
frame_ids、checkpoint、known_submaps、canonical 基线或有效训练计数；也不推进训练的
max_version/max_frontier 检查状态。模型、Adam、梯度及 CPU 确认在命令前后逐值保持。

## 结果接纳和过期

GPU 返回的是候选显示结果，没有训练 decision。owner 的 Poll 在发布前再次检查：

- session/请求 ID 匹配，且没有进入 Finish/Finalize/failed；
- 当前 map_sequence、位姿版本/前沿/parent 与请求完全一致；
- 选中 content、来源引用、分组 revision 和绑定一致，query 仍为期望 retained 帧。

任一变化就丢弃结果并计 stale，不重贴新版本标签。首版采用严格 map_sequence 匹配，
即使只是无关分组追加也可能过期；代价是刷新较少，先保证来源解释简单。
没有强行“追上所有帧”的承诺。延迟和过期率进入实测，若没有足够在线正例必须报告不足。

GetLatestRender() 返回一份只读的已接纳 RGB/元数据快照；相关 owner 状态变化时立即清空。
不在高频 getter 内做 I/O、扫描全目录或重建整个 poses。map/index 的相关变更路径均须触发
失效；结果不能在闭环后继续作为当前结果返回。

文件是带固定版本的历史诊断记录：接纳时固定 accepted_at 与来源，之后通过现有 I/O 线程
写盘。写盘时 map 可以再次前进，文件仍标明原 as_of 身份，不声称“此刻当前”。
不以一个持久 PNG 文件充当实时有效性信号。双 ROS 显示的失效通知在下一阶段定义。

## 调度与资源

保持一个 GPU worker、一个训练模型、现有 single in-flight mailbox。只新增一个可覆盖的
render 需求和一个待保存结果，不建立第三套后台执行器。

安全边界要求没有未决 decision 和正在进行的 activate→view/rebuild→decision 工作链。
接纳/拒绝回执、checkpoint、map 确认、prune、停止控制以及已有必要的保存具有优先权。

在这些前置操作完成后：

- 如果有训练/repair 工作，至多两次完整内容工作尝试后允许一个已到期 render。
  过期、拒绝和空激活结束的尝试也计入这个额度，避免异常输入长期饿死渲染。
- 一次 render 后，有内容工作时把下一机会还给原前台/repair 选择器；render 不清零或改变
  既有 foreground_attempts，保留原历史修复公平性。繁忙时不连续派发 render。
- 无内容工作时，只在查询、内容或绑定改变且间隔到期时渲染；不重复生产完全相同的图。
  需求覆盖计 coalesced，禁用/无来源/未同步/资源原因分别计 skipped。
- Finish/Finalize 停止产生需求，清掉 pending，处理或丢弃已派发返回，再走原持久收尾；
  预览不成为最终收敛的修复对象，也不能凭空增加 accepted/effective_training。

| 项目 | 首版建议上限 |
| --- | --- |
| 来源 | 沿用 512 帧/组、32 确认内容、单块 10 视图/10,000 Gaussian |
| 渲染 | 1..2 块、总 20,000 Gaussian、1 个 ≤1M 像素相机，不裁剪 |
| CPU | 原 confirmed_state + 一次完整加载 + 两块精简参数 + 一份查询；结果数组按相机大小有界 |
| GPU | 原训练模型保持；额外仅一份临时联合推理缓冲，无第二训练模型、Adam 或梯度 |
| 频率/队列 | 默认最大 1Hz，pending render/待保存结果各 1，in-flight 总量仍 1 |
| 文件 | 最近 4 对 joint-render-<job>.png/.json；PNG ≤4MiB、JSON ≤64KiB |

共享数值函数必须 no_grad，只释放自身临时张量。在线不重置全进程 allocator 峰值、不在
每帧 empty_cache；报告全 worker 累计峰值、当前 allocated/reserved，不能伪称区间独占峰值。
离线 render_blocks wrapper 的重置/释放行为保持。CUDA 仍使用现有 4GiB free 准入和
40% 分配比例；训练常驻和推理临时量都计入，参数数目不是硬显存或实时保证。

## 协议、错误和外围接线

沿用现有 schema2 framing（header ≤64KiB、body ≤16MiB）。render 开启时才在 hello 请求
附加 `joint_render_v1` 能力请求，新增 worker 只对请求了该能力的握手回应能力字段；关闭
时旧 schema1/2 字节结构保持，不能让旧调用者因多出 hello_ok 字段而失败。
旧 worker 不支持所需能力时明确拒绝 Gaussian 会话，不能静默假装渲染已开启。

建议新增 joint_render / joint_render_ok / joint_render_error；header 描述至多两个 target 和
固定 body 分段，body 仅含 float64 primary poses 与 query body pose。请求 body 保守上限
131,200 字节（两块各 512×128，加 128），不得因 uint64 ID 分配跨度数组；响应 RGB8 长度
严格为 3×width×height。严格解析、独立 ID/尺寸/变换/身份校验与旧协议测试同时保留。
具体字段排布、内部小 helper 和文件拆分由实施者按现有 validator 风格确定。

| 故障 | 策略 |
| --- | --- |
| 正常缺少邻块/无深度监督/无可见贡献 | 显式单块或零样本指标，非错误，不补训/换相机 |
| 只读推理 OOM | 释放推理局部量，返回有界 render_error；禁用本次会话的联合预览，训练继续；必须用真实后续训练测试证明状态可继续 |
| PNG/JSON 写入失败 | 禁用联合预览并记录失败；保留原训练/持久链，不留下成功 JSON |
| 坏协议、坏来源、资产 SHA/canonical 关系错误 | 沿用会话失败策略，不能把未知内容画出来 |
| CUDA 设备错误/worker 退出/超时 | 无法安全保证上下文或流同步，沿用关闭 Gaussian 会话；前端按现有故障策略继续，不自动重连或恢复 |

可恢复渲染异常必须在 handle 的“任意异常使 session failed”外层之前被识别；禁止无差别
catch 所有 CUDA/协议错误后继续训练。超时只限制等待，不能硬中断已在 GPU 执行的 kernel。

C++ 建议只增加普通 GaussianRender/每块来源 DTO、渲染统计以及 mapper 的少量命令/调度
分支，放在现有 gaussian 目录；内部实现可按 render_commands/render_scheduler 拆分。
无 Pimpl、通用插件/服务框架或新的 ROS main。参数由已有共享清单同步到 Noetic/Humble。
Python 共享纯指标/数值放到推理模块，worker 不导入 CLI 的 main/文件发布流程。
首版输出独立 joint-render 文件名，不改写原单块 preview 或原 map/资产 schema。

## 验证与实施切分建议

1. **普通选择/接纳/调度核心。** 覆盖最新确认查询不回退、resident 与 query owner 不同、
   current/dirty/missing/旧 capture、邻块退化、全版本绑定、pending 覆盖、严格 stale、
   最多两次内容尝试后的渲染公平性、原 repair 配额和 Finish/Finalize 排空。
2. **协议与真实 worker。** 对称字段、边界、坏消息、老握手兼容；实际 CUDA 验证当前驻留
   模型不在选中集合时仍只画选中块、基准/联合遮挡一致。比较模型/Adam/梯度/confirmed/
   references 的前后值，渲染后继续训练必须仍有监督、非零梯度和参数更新。
3. **外围最小接线。** 默认关闭、逻辑/路径/能力检查、两端同 main、文件发布/保留上限、
   状态失效和统计。恢复 C++/普通 SDK、共享/静态、Noetic/Humble、可选编译 OFF/ON 的
   相应构建/安装/真实入口矩阵；不能沿用 C3b 的 Python-only 豁免。
4. **真实在线 bag 配对。** 使用现有 Retail_Street：持久模式 render OFF/ON，短段与全量；
   固定线程/速率/输入/标定，不与构建或其他 GPU 测试并行。前端轨迹/矩阵/原始关键帧
   按同平台严格比较；异步确认数与模型参数可以因调度改变，分开报告。
5. **在线正例标准。** 全量至少六次接纳的双块结果，覆盖至少三个分组，且在输入 EOF 前
   完成接纳；至少有实际非参考块可见变化。以捕获的 steady 运行事件与 bag 结束事件核对，
   不能只看 JSON 时间或在结束后重跑最终资产冒充在线。短段初始单块退化也须实测。
   CPU/worker RSS、进程总显存、allocator、实际刷新/延迟、stale/coalesced、训练/repair
   吞吐及退出单列；达不到正例数量就如实报告，不降低定义。
6. **异常与最终交付。** 受控真实 worker 的渲染 OOM/写盘失败/超时/闭环在途变化、
   最新查询 needs_rebuild、剪枝和 dirty 切块；复验 Finalize current/durable 和正常 stop，
   渲染不得改变冻结 E 或原 accepted/effective_training 语义。独立整范围审阅后固定源码
   重跑关键实际用例，保存完整证据，阶段新产物目标 ≤1GiB。

完整正式实施计划在设计确认、模型切换后再编写。当前不启动新构建、不安装依赖、不运行
新的全量 bag；现有 `d25ccdf` 的唯一产品变更是已验证的 Unicode 诊断修复。

## 范围外与后续

C3c2 研究块空间范围/相机视锥的可见性选择及双 ROS 图像与失效状态，避免把时间邻接误称
几何可见。自由相机、真正 held-out 评价、跨块去重/外观优化、在线增密和高斯约束反哺定位
需要各自的目标/预算；本阶段不据此宣称完整在线 3DGS SLAM。

本设计的工程依据为当前 [mapper](../../../include/livo_atlas/gaussian/online_gaussian_mapper.h)、
[调度](../../../src/gaussian/submap_scheduler.cc)、[worker](../../../tools/3dgs_probe/submap_worker.py)、
[协议](../../../tools/3dgs_probe/submap_protocol.py)、[既有联合数值](../../../tools/3dgs_probe/joint_renderer.py)
和 [C3b 实测](../../p6-03c3b-validation.md)。本轮不升级 gsplat/训练依赖，不附带新框架性能结论。

## 可实施性检查

2026-11-18，一次 fresh-context、只读 builder check 返回“nothing important left”。
检查读取设计和必要现有源码，没有改动产品、运行构建/GPU/bag 或再委派。检查不代替
实施后的代码审阅和实际在线验收。原始结果与设计 SHA 保存在
`validation/P6-03c3c1-design-20261118/`。本设计仍待用户确认及编码前模型切换。
