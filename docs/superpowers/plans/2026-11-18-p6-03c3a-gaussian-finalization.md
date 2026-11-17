# P6-03c3a Gaussian 最终收敛实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 用户已选择当前会话原地直接实施；一次最终独立整范围审阅，不增加逐任务代理。

**Goal:** 后端成功完成后，以可选终端步骤修复已有 Gaussian 内容，输出与最终位姿一致且有明确持久/退出结果的交付。

**Architecture:** 普通核心提供成功完成状态；既有 Gaussian owner/单 worker 排空旧回执后固定 E，在单独预算内仅修复历史内容，再沿原持久收尾停止。小型最终报告在 worker join 后由 io 层原子发布，双 ROS 共用入口与参数。

**Tech Stack:** Google C++17、Eigen、OpenCV、现有 Linux socket/fsync、Python/NumPy/PyTorch/gsplat 固定环境、Noetic/Humble。

**Spec:** [最终收敛设计](../specs/2026-11-18-p6-03c3a-gaussian-finalization-design.md)，设计与编码前通知后用户回复“好啊继续”。

## Global Constraints

- 实施基线 d0f7339；仅 livo-atlas，保留 framework_completion 原地工作，不改 vikit、不合并/推送/tag。
- 原普通 Finish、C1、IPC schema 1/2、资产/map.json schema 1 继续兼容。
- finalize_on_shutdown 默认 false；为 true 要求 enabled 和 persist_submaps。
- finalize_timeout 默认 60000ms，范围 1..600000；原 shutdown_timeout 默认 3000ms，正式持久验收 30000ms。
- 全部 sensor/backend 结束后固定最终完整绝对位姿（含投影尾部），不是 SealInput 或 shared_ptr 地址。
- 先排空旧 accept 回执，再固定 E；不接纳未决候选、不给 missing 补训练。
- 512 前沿/目录、32 内容块、10 视图/10000 参数、单 GPU、一个稳定 CPU 确认及跨块临时状态不变。
- 单 mailbox/in-flight/pending；新生命周期不新增线程、服务、Pimpl 或通用框架。
- 截止时间约束排空、修复与中途切块保存；最后持久收尾独立，超时不伪装成功。
- 正常资产→目录 fsync→map_committed→prune；报告单独原子发布，16KiB，无新 C++ hash 依赖。
- 请求成功要求工作 status=succeeded 且 report_persisted=true；其他消费者仍需收尾。
- 假 peer 仅证明控制语义，GPU 另实测；前端四数组/原始文件严格比较无容差。
- 新验证目录 validation/P6-03c3a-20261118；目标新增≤6GiB，保护已验收 C2b 数据和现有环境。
- 按既有用户授权持续完成全部任务，不在已批准范围内追加逐阶段确认。

## Review Focus

1. 系统刚完成但后端随后写入终端失败，或仅封输入；完成查询不能把失败当冻结许可。
2. accept 已发、回执延后至 Finalize；E 必须包含确认内容，不能早冻结漏块或提升未决候选。
3. 全部已 current 时最后 dirty/目录确认仍在途，或 stop/report 同步失败；不得提前成功。
4. 截止时间恰在候选/commit/转入收尾边界；不重置预算、不让正常收尾被误算成修复超时。
5. 整数预算合法但外层 runner 等待短、报告可见但 fsync 不确定；不能被误杀或只信 JSON succeeded。

---

## 文件职责与固定接口

| 文件 | 职责 |
| --- | --- |
| include/livo_atlas/core/livo_system.h、src/core/livo_system.cc | IsFinished 成功状态，不含 Gaussian 类型 |
| include/livo_atlas/core/online_gaussian_options.h、src/core/online_gaussian_options.cc、src/core/livo_config.cc | 新开关与正整数预算 |
| include/livo_atlas/gaussian/gaussian_map.h | GaussianFinalizationStatus/Counters/Result 普通 DTO |
| include/livo_atlas/io/gaussian_map_writer.h、src/io/gaussian_map_writer.cc | 复用持久发布逻辑写 finalization.json |
| src/gaussian/finalization.cc、现有 mapper/scheduler/header | 终端前置检查、排空、固定 E、预算、结果 |
| tests/gaussian/submap_peer.h | 从旧测试提取受控 socket peer，测试工具而非产品注入 |
| tests/gaussian/gaussian_finalization_test.cc | 使用真实 system 生产/封口的终端集成测试 |
| src/app/livo_main.cc、双 ROS 参数测试 | 共享调用点、结果及退出状态 |
| tools/run_baseline.py、tools/compare_gaussian_replays.py、tools/3dgs_probe/gaussian_map.py | 等待/资源/验证完整结果 |
| docs/online-gaussian.md、docs/p6-03c3a-validation.md、docs/README.md | 用法、迁移、最终证据 |

DTO 固定为 status enum（kSucceeded/kTimedOut/kFailed），reason、target_valid、session_id、
pose_version/frame_count/optimized_frame_count/parent_frame；eligible_set_frozen、eligible_submap_ids、
current/needs_rebuild/missing/dirty、durable、map_sequence、asset_ids；worker_stopped、
elapsed_seconds/repair_seconds/shutdown_seconds、repair_attempts、before/after 四项计数
（accepted/effective_training/rebuilds/checkpoints）、report_persisted/report_error。
报告记录 elapsed_seconds 至发布准备点；返回/日志额外 total_seconds 计入报告发布，避免文件自证未来耗时。
所有 ID 以十进制字符串持久化，列表≤32，有界诊断≤1024字节。

## 验证命令约定

当前 shell：LIVO_REPO=/home/yanni/livo-atlas_ws/src/livo-atlas，
LIVO_PHASE=/home/yanni/livo-atlas_ws/validation/P6-03c3a-20261118。
core 配置 ROS_VERSION=0、BUILD_TESTING=ON、Gaussian/GTSAM/LOOP_CLOSURE=ON、-j1，
Sophus 使用 /home/yanni/3rd-party/Sophus/build；GTSAM 使用保留的
validation/P6-02b-20261111/gtsam-install-noetic/lib/cmake/GTSAM。
新 phase/run-python.sh 使用既有 P6-03b env 和有界准备工具；不升级依赖。

全核心 gate：`cmake -E chdir "$LIVO_PHASE/core" ctest --output-on-failure -j1`。
Python gate：`"$LIVO_PHASE/run-python.sh" -m unittest discover -s tests/3dgs_probe -v`。
长输出保存包外，读取尾部和退出码；GPU/正式回放不并行运行。

### Task 1: 完成屏障、配置与结果类型

**Files:** 修改 core/livo_system、core/online_gaussian_options、core/livo_config、gaussian/gaussian_map 对应头/源；测试 tests/core/input_seal_test.cc、global_output_options_test.cc、tests/core/loop_backend_test.cc。

**Interfaces:** 提供 `bool LivoSystem::IsFinished() const noexcept`；OnlineGaussianOptions 新增 bool finalize_on_shutdown=false、milliseconds finalize_timeout{60000}；提供上述 DTO。

- [x] **Step 1: 写 RED。** 在真实 InputSealTest 中新增/扩充以下断言：

```cpp
EXPECT_FALSE(system.IsFinished());
system.SealInput();
EXPECT_FALSE(system.IsFinished());
system.Finish();
EXPECT_TRUE(system.IsFinished());
```

  封口/写入异常时查询为假；后端失败后查询为假。配置测试用真实共享 loader 拒绝
  finalize=true 而 enabled/persist 为假，预算0/-1/600001，接受1/60000/600000，输出尚未创建。
- [x] **Step 2: 运行 RED。** 新构建的 input_seal_test/global_output_options_test 缺少 API/字段而编译失败；先观察缺接口错误，不先改产品。
- [x] **Step 3: 实现。** 成功 Finish 后置完成位，查询同时排除终端 submap_error；不改变 SealInput/算法。补普通配置/DTO，接口不引入 ROS/GPU。
- [x] **Step 4: GREEN。** 两目标及相关 loop_backend_test 通过，完整核心 CTest 通过，公开头检查通过。Expected: 新状态边界和旧核心行为均通过。
- [x] **Step 5: 提交并 task-done。** `feat: expose completed source and Gaussian finalization options`；任务 gate 为全核心 CTest。

### Task 2: 最终报告的有界持久发布

**Files:** 修改 io/gaussian_map_writer.h/.cc；测试 tests/io/gaussian_map_writer_test.cc。

**Interfaces:** 消费 Task1 DTO；提供 `void WriteGaussianFinalizationResult(const std::filesystem::path&, const GaussianFinalizationResult&)`，异常沿用 GaussianMapCommitUncertainError。

- [x] **Step 1: 写 RED。** 真实文件测试读取报告断言 schema=1、kind/session/map_sequence/目标/E/状态/计数匹配，未序列化 report_persisted、report_error、total_seconds；缺函数编译失败。
  覆盖≤32 IDs、16KiB、重复/非法 ID/时间/状态拒绝，FIFO/符号链接临时路径不触碰；资产文件不变。
- [x] **Step 2: 运行 RED。** 构建 gaussian_map_writer_test，Expected: 缺少新写入函数。
- [x] **Step 3: 实现。** 复用私有原子发布函数（map路径保持），finalization.tmp.json→finalization.json，独占创建0600、fsync/close/rename、目录/父目录 fsync；不引入通用存储框架。
- [x] **Step 4: GREEN。** 定向故障注入覆盖文件 fsync/close/rename 前失败不发布、rename 后两级目录同步失败保留可见内容并抛 uncertain；原10个 map writer 用例继续通过，完整核心 CTest 通过。
- [x] **Step 5: 提交并 task-done。** `feat: persist bounded Gaussian finalization reports`；gate 全核心 CTest。

### Task 3: 固定版本终端生命周期

**Files:** 新 src/gaussian/finalization.cc、tests/gaussian/gaussian_finalization_test.cc、tests/gaussian/submap_peer.h；修改 mapper.h/.cc、submap_scheduler.cc、旧 persistent 测试与 cmake/online_gaussian.cmake、tests.cmake。

**Interfaces:** 消费 Task1/2；提供 `GaussianFinalizationResult OnlineGaussianMapper::Finalize(std::chrono::milliseconds) noexcept`。仅前置失败可重试，接纳后终端/幂等；原 Finish 不自动修复。

- [x] **Step 1: 写 RED。** 抽测试 peer 不改行为，新 fixture 从真实 LivoSystem LO 生产关键帧并转发拥有数据的受控 RGB/深度到 mapper；使用 system 的正式快照、更新和封口，不伪造 IsFinished。初始新 API 缺失编译失败。
  必须实现下列行为测试，各断言 status、E、目录/资产、stop、文件及原始数据，而非私有成员：
  RequiresCompletedSourceAndValidBudget；FinalPoseIncludesBackendTail；RepairsAllConfirmedBlocksAndFlushesLastDirty；
  EmptyAndMissingStayExplicit；CapacityKeepsMissing；RepeatedFinalizeDoesNotRetrain；FastFinishStaysFast；
  PendingCandidateIsNotPromoted；AlreadyAcceptedCommitJoinsEligibleSet；EntryDuringCheckpointAndAck；
  FrozenTargetMismatchCannotSucceed；RigidContentDoesNotRewriteArrays；
  QuiescingTimeoutDoesNotFreezeEmptySet；CandidateAfterDeadlineIsRejected；LateAcceptedCommitStaysTimedOut；
  CleanupMayOutliveRepairBudget；WrongStopAndUncertainMapCannotSucceed；ReportFailureKeepsActualConvergence。
- [x] **Step 2: 运行 RED。** 构建 gaussian_finalization_test，Expected: Finalize 未定义/声明；旧 peer 提取后的23个 persistent 用例先独立 GREEN，不能以提取引入失败替代 RED。
- [x] **Step 3: 实现前置/排空。** 只读核对成功 system、完整最终 poses/封口组，保存冻结身份；排空旧命令、拒绝尚未 accept 的候选，已 accept 回执按原协议记账后固定 E。预算从接纳起计。
- [x] **Step 4: 实现修复/收尾。** 明确 running/quiescing/repairing/stopping，修复阶段只选择 E，正常截止前无修复接纳在途才转收尾。记录尝试/计数；沿原 Finish 链保存/确认/prune/stop；事后计算结果并写报告，发布错误只改返回发布字段，最后缓存终端结果。
- [x] **Step 5: GREEN。** 受控 test peer 的 hold/真实回执条件控制竞争与时限，不用产品测试钩子。新用例、旧 C1/C2b/协议/写盘及全核心 CTest 通过；真实 worker 验证一次多块 Finalize，GPU 跳过不计成功。
- [x] **Step 6: 提交并 task-done。** `feat: finalize Gaussian submaps against completed poses`；gate 全核心 CTest+完整 Python/CUDA。

### Task 4: 共用 ROS 入口、回放等待与严格验收

**Files:** 修改 src/app/livo_main.cc、tests/ros1/lidar_callback_test.cc、tests/ros2/ros2_node_test.cc、tests/gaussian/ros_entry_smoke.py、tools/run_baseline.py、tools/compare_gaussian_replays.py、tools/3dgs_probe/gaussian_map.py、tests/test_baseline_runner.py、tests/test_compare_gaussian_replays.py。

**Interfaces:** 消费 Finalize 结果；新增 runner 参数 --gaussian-finalize-on-shutdown、--gaussian-finalize-timeout-ms；inspect_gaussian_map 保留旧返回含义，新增独立报告验证函数 `inspect_gaussian_finalization(directory, map_report)`。

- [x] **Step 1: 写 RED。** parser 真实调用拒绝无 persistence/worker、非法预算；进程辅助函数对短寿命/等待退出/强制结束的真实子进程验证等待与采样。比较器伪成功、错 E/前沿/序号/资产集、dirty、缺报告、发布失败、bad stop、强制退出均拒绝；missing 可明确保留。
- [x] **Step 2: 运行 RED。** `python3 -m unittest discover -s tests -p test_baseline_runner.py -v` 与 test_compare_gaussian_replays.py，Expected: 新参数缺失或旧检查错误接受；双 ROS 参数新字段同样 RED→GREEN。
- [x] **Step 3: 实现。** 后端成功后按开关二选一调用，失败置非零但仍结束其他消费者；输出 `livo_atlas gaussian finalization:` 含结果、目标、E/count、durable、worker_stopped、report_persisted、阶段耗时/计数。返回 total_seconds 含报告发布，文件 elapsed 不自证将来。
  runner 最终退出等待=T+90秒，旧模式10秒，单独 shutdown-resources.csv 与 summary时间/forced_termination；显式参数才接线。检查器逐资产/完整 canonical 验证后再核对报告与日志、正常退出，原 C1/C2b 判据继续通过。
- [x] **Step 4: GREEN。** 两组 Python工具测试与完整 Python/CUDA、全核心通过；Noetic/Humble 实际 typed 参数/暂停时钟/零输入 finalization、C1/持久默认、编译OFF早拒绝均通过。Humble 无CUDA只报受控peer。
- [x] **Step 5: 提交并 task-done。** `feat: integrate Gaussian finalization into shared ROS shutdown`；gate 全核心+两组工具测试。

### Task 5: 最终源码验证、一次审阅与迁移记录

**Files:** docs/online-gaussian.md、docs/README.md、spec/plan状态；新 docs/p6-03c3a-validation.md。运行脚本/大型证据仅在包外新阶段。

**Interfaces:** 消费前四项；默认 C2b、Finalize 开关只改变 Gaussian 结束后工作，不能改变前端/后端输出。

- [x] **Step 1: 构建矩阵。** 普通 shared/static OFF/ON、公开头/安装消费者；Noetic/Humble OFF/ON与真实入口/typed参数、Humble ament消费者。记录是否真实GPU，不把受控peer算GPU。
- [x] **Step 2: 候选实测。** 同源码/二进制/共享配置/CPU，10秒 OFF/原持久/Finalize 三组，完整 bag 三组；C1另短段。核验bag/源/库hash；四数组逐值、发布语义、原始文件、后端最终状态一致。Finalize应13已有内容全部current、missing/dirty/unreferenced=0，另新GPU逐块冷恢复；阴性超时/故障明确失败不放宽检查。
- [x] **Step 3: 文档/候选提交及一次独立整范围审阅。** 写准确环境/命令/来源身份、在线与postlude分开、时间/覆盖局限、报告不确定。按 Review Focus 审阅；Important 各先RED再统一修复，不追加复审；Minor记录延期，所有裁决进台账。
- [x] **Step 4: 最终复验。** 修复后重建受影响矩阵，并冻结最后产品源码重跑成组三种模式和C1、真实GPU冷恢复，不能沿用修复前二进制证据。任务级gate重跑全核心+完整Python/CUDA+两工具suite并检查正式报告/hash。
- [x] **Step 5: 提交/task-done/归档。** 文档状态只在实际完成后勾选；保留本分支，归档本计划执行台账和审阅原文后仅清本计划scratch。不得合并/推送/tag，保护刚清理后保留的依赖和旧正式证据。

## Self-Review

Task1→2/3/4 固定普通状态/配置/DTO；Task2→3 固定报告发布且不自证；Task3→4 固定终端结果；
Task4→5 固定CLI与严格验收。五项焦点分别由Task1、Task3、Task2/3、Task3、Task4承担回归。
实现文件按职责拆分，private状态可由执行者选择，不为测试暴露产品注入。原明确定义的预算、
资产协议和默认行为不重写；与设计冲突按spec裁决并记录。先前执行方式/模型切换通知后的
“好啊继续”继续沿用，不另起一轮已授权范围内的审批。
