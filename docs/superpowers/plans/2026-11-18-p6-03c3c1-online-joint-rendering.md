# P6-03c3c1 Online Joint Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [ ]`). 用户已确认设计并要求当前会话直接实施，沿用原地 native 执行和一次最终独立整范围审阅。

**Goal:** 在现有串行 Gaussian 会话中只读联合渲染至多两块已确认内容，交付有版本、可过期、可验证的在线文件预览。

**Architecture:** owner 捕获同一 map/poses 的请求；原 GPU worker 串行读取 CPU 确认或已知资产，独立推理缓冲不触碰训练模型。owner 严格核对返回版本后接纳，原 I/O 线程保存至多四对独立文件；关闭时保持旧路径。

**Tech Stack:** Google C++17、现有 OpenCV/AF_UNIX、Python3.10/NumPy1.26.4/PyTorch2.3.0+cu121/预编译 gsplat1.5.3；Noetic、Humble 共用 main。

**Spec:** [已确认设计](../specs/2026-11-18-p6-03c3c1-online-joint-rendering-design.md)，用户在文档/切换模型通知后回复“继续”。

实施进度：Task1–4及Task5最终实测已通过；完成检查和归档收尾见[验证记录](../../p6-03c3c1-validation.md)。

## Global Constraints

- 基线838d327，保留 framework_completion 原地开发，不改 vikit、无 Pimpl、不合并/推送/tag。
- 一个 GPU worker/训练模型/mailbox，总 in-flight1，pending render/待保存结果各1。
- 最大 retained 确认帧为期望查询；owner 不 current 时等待，不能退回旧相机；owner+紧邻前组，邻组不可用明确单块。
- 只用 CPU confirmed_state 或与确认 content 相同的 current 资产，验证完整 canonical primary；旧 capture 刚性 current 合法。
- resident 可以是未选中的第三块；不复制/修改训练 GPU 参数、Adam、梯度或训练前沿。
- 快照包含同一 session/map_sequence/poses 前沿/绑定/content；入队重建资产引用，render 到返回不并发 prune/训练命令。
- owner 完全匹配当前 map_sequence 才接纳；任何变化/终端阶段丢弃且立即使 GetLatestRender 失效。
- float64 相对变换、proper SO3、wxyz、K+0.5一次；1..2块/20,000 Gaussian/1M像素；无裁剪和隐藏回退。
- 默认 disabled；interval1000ms，100..60000；timeout5000ms，1..60000；enabled要求 Gaussian+persistence。
- mandatory decision/持久链/stop 优先；两次完整内容尝试后给到期 render 一次，render 后有内容工作必须还给原选择器；不改原 repair 配额。
- 文件 joint-render-<job>.png/.json 至多4对，PNG≤4MiB、JSON≤64KiB，最后 JSON rename 才完成；固定 as_of 身份。
- OOM/写文件失败禁用本会话预览并保留训练；坏协议/来源/设备/超时沿用 Gaussian 会话失败，前端继续。
- schema1/2关闭握手原字段保持；开启才请求/回应 capabilities=[joint_render_v1]；原资产/map schema不变。
- header≤64KiB/body≤16MiB；render request body≤131200bytes；RGB8响应精确3WH；uint64规范字符串。
- 不重置 worker 累计 allocator 峰值，不每帧 empty_cache；离线 wrapper原生命周期保持；沿用4GiB free/40%预算。
- 正式GPU/回放不与构建或其他GPU任务并行；至少6次EOF前双块接纳、跨3组且有非参考贡献，否则如实失败。
- 新证据只写 validation/P6-03c3c1-20261118；保留所有旧资产/SDK/env/bag。最终保留目标≤1GiB，临时新构建和重复回放可在核验/摘要归档后仅清理本阶段自产冗余；记录临时峰值。

## Review Focus

1. repair 驻留历史第三块且最新查询非驻留：身份/梯度/Adam不变，只画选中块，查询不回退。
2. pose更新/新帧/asset替换发生在入队和返回之间：严格过期、剪枝串行，磁盘文件仍只描述原as_of。
3. 持续前台、失败/空激活、repair和render争用：原修复公平性、渲染额度、终端持久链均可前进。
4. 零image/depth样本、完美RGB、长Unicode错误：wire无null且字段受限，文件null语义正确，诊断不盖原错。
5. 真实OOM/写盘/握手不支持/超时：只在可恢复错误时继续原训练；关闭模式协议、计数、文件和双ROS入口不回归。

---

## 文件和共享接口

新增 public `include/livo_atlas/gaussian/gaussian_render.h`，实现 `src/gaussian/gaussian_render.cc`：
- GaussianRenderBlock：GaussianTarget target、GaussianContentId content、optional<GaussianAssetReference> asset、frame_ids、gaussian_count、resident。
- GaussianRenderRequest：session_id、job_id、generation（resident fence）、map_sequence、pose_version、frame_count、optimized_frame_count、parent_frame、query_keyframe_id、query_submap_id、parent_from_body、blocks。
- GaussianRenderMetrics：image/covered/depth_available/depth_used计数、rgb_perfect，optional<double> rgb_l1/rgb_mse/rgb_psnr_db/alpha_coverage/depth_l1_m/depth_rmse_m。
- GaussianRender：request、width/height、rgb拥有字节、single/joint指标、changed_pixels、capture/accepted时间、latency/worker秒数、GPU累计峰值/当前分配/当前reserved、RSS。
- `optional<GaussianRenderRequest> SelectGaussianRender(const GaussianMapSnapshot&, const PoseSnapshot&, uint64_t generation, string* reason)`；job_id由mailbox入队指定。
- `bool IsCurrentGaussianRender(const GaussianRenderRequest&, const GaussianMapSnapshot&, const PoseSnapshot&)`；`void ValidateGaussianRenderRequest(const GaussianRenderRequest&)`。

Task2新增 `tools/3dgs_probe/joint_render_protocol.py` 与 `online_render.py`：
- `decode_render_request(header:dict, body:bytes)->dict`，返回拥有的query pose与blocks targets；
- `validate_render_message(header,body)->None`，`encode_render_reply(request,result)->tuple[dict,bytes]`；
- `render_confirmed(actor, request:dict)->dict`：拥有CPU结果/metadata，不改变actor确认/训练状态。
- `joint_renderer.infer_blocks(blocks,K,width,height,device)->dict`：与原render_blocks同输出，保留allocator；原render_blocks仍作离线生命周期wrapper。
- `joint_renderer.image_metrics(result,query)->dict` 从CLI移动，CLI导入/重导出保持现有调用。

Task3新增内部 `src/gaussian/render_protocol.h/.cc`、`render_scheduler.cc`：
- `GaussianMessage EncodeGaussianRenderRequest(const GaussianRenderRequest&)`；
- `GaussianRender DecodeGaussianRenderReply(const GaussianMessage&,const GaussianRenderRequest&)`；
- mapper新增 `GetLatestRender()->shared_ptr<const GaussianRender>`、`GetRenderStats()->GaussianRenderStats` 和私有render需求/额度/待保存/计数。

Task4新增 `include/livo_atlas/io/gaussian_render_writer.h`、`src/io/gaussian_render_writer.cc`：
- `void WriteGaussianRender(const filesystem::path&, const GaussianRender&)` 保存新文件对；保留队列属于原I/O线程。
- options追加bool joint_render_enabled、chrono::milliseconds joint_render_interval{1000}/joint_render_timeout{5000}。
- 参数沿用src/core/livo_config.cc和共享ParameterReader，两端实际测试；tools/run_baseline.py/run_ros2_baseline.py追加对应三个CLI参数及执行证据。

## 协议具体约定

hello/hello_ok仅显式开启时增加capabilities数组，必须精确包含joint_render_v1；缺失/未知拒绝。
请求type=joint_render，公共schema2/session/job/generation/pose_version，加map_sequence/frame_count/
optimized_frame_count/parent_frame/query_keyframe_id/query_submap_id和blocks数组。
每block包含target（现有target字段加generation）、content（3个uint64身份）、resident(0/1)、
frame_ids、gaussian_count；非resident加asset引用（现有描述字段），resident不带asset。
body为query parent_from_body 16个<f8，之后按block ID升序拼每块primary poses（各count×16个<f8）；
根据counts算固定offset，不接收任意offset。blocks唯一/有序，含query owner且最多邻前组。

reply type=joint_render_ok：回显request的身份/前沿/query/选块source身份，query相机/K/时间、
width/height、single/joint指标、changed_pixels、资源和timings；body仅RGB8。
OpenCV4.2 FileStorage实测不能解析JSON null，因此wire指标用validity flags+有限数值0占位：
image_valid=image_pixels>0、depth_valid=depth_used>0、psnr_valid=image_valid且mse>0；不适用的数值必须0。
rgb_perfect必须image_valid且mse==0；计数子集关系和coverage比值要校验。
C++ Decode转optional；对外文件按optional写null，不能用占位0冒充误差。
reply type=joint_render_error：回显请求身份，code=oom、error≤4096bytes，无body；只禁用预览。
其他异常仍原error/session失败。PNG保存失败在C++区分显示I/O错误，不发训练decision。

## 验证入口

phase=/home/yanni/livo-atlas_ws/validation/P6-03c3c1-20261118；repo为当前包。
phase/run-core.sh构建新phase/core（ROS0、GTSAM/LOOP/GAUSSIAN ON、Release、-j1），ctest全部；
phase/run-python.sh复用C3b固定环境/YAML前缀、prepare binaries，LIVO_WIRE_PEER指向本阶段新core。
phase/run-all.sh依次run-core、149基线及新增Python/CUDA、root工具；LIVO_REQUIRE_GPU=1，禁止skip算通过。
每个task-done跑run-all.sh；Task5最终另核对实际回放、产物SHA/版本/在线时序。

### Task 1: 普通只读请求与选择规则

**Files:** gaussian_render.h/.cc、cmake/online_gaussian.cmake、cmake/tests.cmake、新tests/gaussian/gaussian_render_test.cc。
**Interfaces:** 产生上述DTO/SelectGaussianRender/IsCurrentGaussianRender/ValidateGaussianRenderRequest，供Task2 wire和Task3 owner。

- [ ] **Step 1: RED。** 用真实GaussianMapIndex/poses构造三组：最新query5在组2，resident组0；组2持久current，组1邻前current，预期blocks[1,2]、query5，不含0。最新组needs_rebuild时返回空，不能选组1旧query。邻组missing/dirty/mismatch时只含owner；resident dirty确认合法；asset旧capture刚性current合法；pose/sequence/content/revision/asset/query变更均stale；输入拷贝不别名。
- [ ] **Step 2: Run RED。** 新目标只因缺头/符号编译失败；基础GaussianMapIndex/旧67核心测试先通过。
- [ ] **Step 3: 实现。** 最新query先在所有content中选，再检查可用性；从同一poses构造target，保留完整primary；exact source身份及有限范围检查。函数无ROS/I/O，GetLatestRender暂无接线。
- [ ] **Step 4: GREEN。** 新选择器测试和全部core+Python/CUDA/root工具通过；输出手算query pose和canonical binding。
- [ ] **Step 5: Commit/task-done。** `feat: define confirmed Gaussian render requests`；gate phase/run-all.sh。

### Task 2: 只读 Python 推理与协议能力

**Files:** 新joint_render_protocol.py、online_render.py；joint_renderer.py、render_gaussian_map.py、submap_protocol.py、submap_worker.py、online_worker.py；新test_online_render.py/test_joint_render_protocol.py。
**Interfaces:** 消费Task1上述wire布局；产生infer_blocks/image_metrics、decode/encode、render_confirmed；actor构造追加keyword joint_render_enabled=False。

- [ ] **Step 1: RED。** 真实CUDA训练三组资产（现有fixture），最新query owner为非resident资产；选中owner+邻组，验证第三resident未入渲染。deep snapshot比较模型参数/Adam/grad、confirmed、content/refs/训练前沿前后；成功渲染后继续真实20步训练有监督/非零梯度。独立旧camera render/交错遮挡回归保持；旧capture/投影query/零样本/完美RGB保护。
- [ ] **Step 2: Run RED。** 新API缺失/未协商render被拒绝；不是坏fixture或跳过GPU。wire边界坏ID/重复/超尺寸/错误source/长度/NaN/反向pose/错误validity失败；off握手字节保持。
- [ ] **Step 3: 实现推理。** 共享无allocator副作用函数，CLI重导出image_metrics；来源每次只一个完整read_state，提取拥有参数/一份query；来源完整兼容检查，单块基准后联合，不创建训练模型/优化器。报告当前及全worker累计allocator，不重置峰值。
- [ ] **Step 4: 实现协议/错误。** 新模块严格解析，schema2仅cap opt-in；actor读取自己的confirmed/refs，readonly不推进训练max_frontier。只在推理局部捕获torch.cuda.OutOfMemoryError返回oom；后续训练验证可继续，坏来源/协议仍failed。真实allocator受限OOM另单独测试，不以mock数值代替真实正常渲染。
- [ ] **Step 5: GREEN/Commit/task-done。** 新protocol/actualGPU及旧全部通过，`feat: render confirmed Gaussian blocks without training mutations`；gate run-all.sh。

### Task 3: C++ 协议、owner 接纳与公平调度

**Files:** render_protocol.h/.cc、render_scheduler.cc、online_gaussian_mapper.h/.cc、submap_scheduler.cc、submap_commands.cc、finalization.cc；cmake；tests/gaussian/新增online_gaussian_render_test.cc、submap_peer.h、wire_peer.cc和Python跨语言fixture。
**Interfaces:** 消费Task1 DTO和Task2wire；产生GetLatestRender/GetRenderStats、kRender/kRenderSaved等私有命令，以及render请求/接纳事件。

- [ ] **Step 1: RED。** 真实socket peer记录命令：decision/持久确认/prune优先、链中不render；两次完成/过期/空尝试后到期render一次，下一次content/repair；pending覆盖无预先asset租约。EOF前接纳后查询getter返回同一只读对象，新frame/pose/content/checkpoint/释放/failed/Finish立即失效。请求在途更新后返回丢弃、无重新贴版本。
- [ ] **Step 2: Run RED。** 新GetLatestRender/codec缺失，旧peer核心通过。Python生成消息由C++读、C++输出由Python读，uint64>2^53不丢失；缺能力/invalidity/null/长度/错误echo拒绝。
- [ ] **Step 3: 实现codec。** 新codec使用现有GaussianMessage/CV JSON，指标validity转optional；只读request显式resident_generation fence、map_sequence和body严格校验；OOM专门reply不触发mapper Fail，fatal/timeout按旧Fail。
- [ ] **Step 4: 实现owner/调度。** map修改统一清理latest_render；pending只是dirty flag，真正dispatch取新map/poses及已ack refs；原忙/候选/持久链优先。新增最多2的render尝试额度，不改foreground_attempts。记录render专用submitted/accepted/stale/coalesced/skipped/failed，不加原训练accepted等。终端清需求、等待/丢弃inflight并继续原收尾。
- [ ] **Step 5: GREEN/Commit/task-done。** 独立peer/socket、旧deadline/finalize回归、真实C++owner到新Pythonworker接线通过；`feat: schedule and validate online joint Gaussian renders`；gate run-all.sh，外控GDB/真实GPU单列执行。

### Task 4: 有界文件预览、参数和运行工具

**Files:** gaussian_render_writer.h/.cc、online_gaussian_options.h/.cc、src/core/livo_config.cc、src/app/livo_main.cc、cmake、config默认文件；tools/run_baseline.py、run_ros2_baseline.py、compare_gaussian_replays.py；tests/io/gaussian_render_writer_test.cc、tests/core/global_output_options_test.cc、tests/ros1/node_options_test.cc、tests/ros2/node_options_test.cc、tests/test_baseline_runner.py及比较器测试。
**Interfaces:** 消费已接纳GaussianRender；新增3参数、bounded写盘；回放summary包含joint stats和训练/EOF steady事件，用于Task5。

- [ ] **Step 1: RED。** 默认关闭/真bool/范围/依赖/无GPU编译时明确拒绝，发生在output I/O前。真实PNG和JSON最后标记、null零样本、旧文件不碰、最多4对；保存故障只禁用渲染；接纳后map变化文件仍as_of，无假当前标签。
- [ ] **Step 2: Run RED。** 新参数/CLI/写者缺失。Noetic/Humble参数用真实节点，工具非法输入在子进程/输出前失败；比较器不能把EOF后文件算在线正例。
- [ ] **Step 3: 实现。** writer用小型有类型JSON输出（classic locale、precision17、optional->null；所有quoted身份/文件名严格受控），不引入JSON框架；先清本线程已知最老pair再写下一对，PNG关闭且≤4MiB后临时JSON rename。仅I/O线程保存，pending结果1，不改变原preview/map格式。
- [ ] **Step 4: 外围接线。** options字段只在启用时影响；两端共享ParameterReader读3字段，main报告joint独立stats。runner3flags/资源/steady事件，提供检查器验证source/PNG计数/as_of和accepted_before_EOF；启用和关闭比对前端数值，接受异步训练量不同。
- [ ] **Step 5: GREEN/Commit/task-done。** 普通core/完整Python/root及双ROS参数入口通过；`feat: publish bounded online Gaussian joint previews`；gate run-all.sh+本Task实际ROS参数检查。

### Task 5: 平台矩阵、真实在线验收与归档

**Files:** docs/online-gaussian.md、docs/README.md、新docs/p6-03c3c1-validation.md、设计/计划状态；全部产物phase。
**Interfaces:** 前四项真实节点/worker/检查器，输入同一Retail_Street.bag和保留env/dependencies。

- [ ] **Step 1: 候选矩阵。** 固定源码、peer/node/lib和bag SHA；普通shared/static各Gaussian OFF/ON，Noetic/Humble ON/OFF、installed消费者/公开头及真实入口。Humble至少真实参数+协议入口，GPU实测在Noetic主机，不宣称Humble CUDA通过。
- [ ] **Step 2: 候选实际回放。** Noetic rate1固定cpu0/OMP1/queue32，短/完整persist+Finalize的render OFF/ON两对，C1短回归；独立单GPU无构建并行。严格前端/原始观测/最终current durable，完整ON至少6双块接纳跨3组且在输入EOF前；受控真实观测/worker验证OOM后训练、写盘失败、超时和修复驻留第三块。
- [ ] **Step 3: 一次整范围审阅。** 候选证据/文档提交后，fresh most-capable reviewer读取全部实现/ReviewFocus/ledger；按效果评级，Important/Critical统一一次RED→GREEN修复并绿全套，所有declined/Ruling/Minor记录。无任务级review/无复审。
- [ ] **Step 4: 最终重跑。** 固定最终源码后重建受影响矩阵，至少同短/完整OFF/ON及关键故障/第三块用例；冻结source+二进制+日志/产物hash，核对在线时序，不能沿用修复前GPU/回放结果。统计GPU总显存/RSS、真实训练与render吞吐/延迟/过期，Finalize独立时间。
- [ ] **Step 5: 完成/Commit/task-done。** `docs: record verified online joint rendering`；完整gate+实际结果/身份核对。归档ledger/rawreview字节与SHA后仅清本计划scratch；保留分支。摘要归档后仅移除本阶段重复大产物/临时构建，以最终≤1GiB为目标；不删旧来源/环境，剩余限制明确报告。

## Self-Review

Spec查询/快照由Task1/3覆盖；数值/无训练副作用/能力/错误由Task2/3；参数/文件/双ROS由Task4；
真实在线而非最终离线、矩阵/资源/收尾由Task5。Task1->2/3字段和wire布局明确，Task2->3格式一致，
Task3->4只读结果与单待存储契约明确。ReviewFocus1..5分别落在Task1/2、3/4、3、2/4、2/3/4。
当前CV null解析的真实CPU试验已说明wire选择，不把协议占位值暴露成零误差。
用户本轮“继续”确认设计/模型切换，已多次选择当前会话自主执行和自动验证；写完本计划后沿用该授权实施，
不再增加一轮相同范围的计划权限询问。关键偏离仍按ledger裁决，外部发布/破坏未知数据不在授权内。
