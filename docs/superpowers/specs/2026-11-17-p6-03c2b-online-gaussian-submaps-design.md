# P6-03c2b：在线 Gaussian 子地图与位姿一致性

2026-11-17，基于 `75c7e17`。本文把
[C2 总设计](2026-11-17-p6-03c2-gaussian-submaps-design.md)中的在线方向细化为实现契约。
[C2a](../../p6-03c2a-validation.md)已经完成；本文是待实施方案，不是新增功能验收。
用户已授权继续按推荐方向推进；下面的具体机制和预算是执行者的设计选择。
按用户要求，完成方案、计划并通知切换模型后，再开始产品编码。

## 目的与边界

保留已确认的历史 Gaussian 工作集，让同一套 SLAM 的子地图分组和绝对位姿快照驱动
高斯存储、加载和修正。当前每次切换或位姿更新都重置模型；本阶段使历史工作集可恢复，
整块移动可复用，组内变形可重新训练，始终标明实际覆盖和版本。

- 只修改 `livo-atlas`。普通 C++ 核心不依赖 ROS、torch 或 CUDA；继续共用
  `src/app/livo_main.cc`，ROS1 Noetic / ROS2 Humble 只负责参数与接入。
- 默认关闭 Gaussian；新增 `gaussian.persist_submaps`，默认 `false`。为 `false` 时保持
  C1 单活动模型、IPC schema 1 和已有预览语义；为 `true` 时启用本文的 IPC schema 2。
  不能把持久化参数为真但 Gaussian 为假的配置静默当作成功。
  另读取 `gaussian.shutdown_timeout_ms`（正整数，默认 3000），连接既有 C++ timeout 字段；
  持久化示例/真实验收显式设为 30000，不暗改 C1 默认值。
- 保持 C2a 资产 schema 1、最多 10 视图/10,000 个高斯、512 帧前沿、每候选训练 20 步。
  这里保存的仍是稀疏训练工作集，不宣称完整覆盖子地图。
- 不实现前后端 session 恢复、worker 自动重连、跨 session 自动接管、跨块联合渲染、
  densification、新的 Gaussian 定位反馈或真值/画质承诺。
- Google C++17，函数实现间空一行，必要处说明坐标与生命周期；不用 Pimpl、通用插件、
  第二套 ROS main 或第二个后台服务。

## 已核对的现状

1. `OnlineGaussianMapper` 的所有者线程决定是否接纳，单后台线程负责重建观测、socket 和
   文件。pending、in-flight 各一个；当前 `Invalidate()` 会丢弃就绪预览并触发 reset。
2. `OnlineGaussianModel.train_candidate()` 原地修改模型；`decide(False)` 清空所有参数。
   因此“切换前才保存”不足以保护先前已接纳结果：切换可能恰好发生在候选训练期间。
3. C2a 已有拥有数据的 `GaussianState`、严格 CPU reader/writer、捕获/新实例恢复；
   writer 区分提交前失败与清单已发布后的 `CommitUncertainError`。
4. `OnlineSubmapUpdate` 提供完整绝对 `PoseSnapshot`，以及最多两条分组修订。
   `pose_changes` 是增量通知，不能充当历史块修正的全部依据。
5. `CompareSubmapPoses()` 要求相同前沿且还比较 support 成员；本阶段的 Gaussian 只训练
   primary，不能直接拿这个接口判断不同前沿上的历史资产。新增领域内的直接比较，
   不改变通用几何接口语义。
6. `KeyframeReadView` 不读完整图像。历史重建使用资产中保留的 RGB/深度，不扩大读接口。
7. 两端 IPC 限制头部 64 KiB、正文 16 MiB。512 个 4×4 float64 位姿不能可靠装进 JSON 头，
   采用受限二进制正文，不提高已有帧长度上限。

## 方案取舍

| 方案 | 收益 | 代价与结论 |
| --- | --- | --- |
| 每个确认都写完整资产，再训练下一候选 | 磁盘直接提供回退 | 每次最多约 193.5 MiB，带来大量写入；本轮不选 |
| GPU 一个模型、CPU 一个确认快照，离开前落盘 | 能抵抗候选原地修改，限制 GPU 驻留和磁盘写频率 | 增加一次有界 CPU 捕获；采用 |
| 同时保留多块 GPU 或完整复制候选 GPU 模型 | 切换快 | 显存随块数或候选复制增长；不符合当前资源目标 |

CPU 快照是确认边界的实现机制，不改变 C1 `decide(False)` 的语义，也不增加通用缓存框架。

## 权威、身份与线程

| 对象 | 权威和所有者 |
| --- | --- |
| 原始关键帧、分组、绝对位姿 | `LivoSystem` / 现有存储 |
| 地图目录、当前可用性、目标绑定、候选接纳 | `OnlineGaussianMapper` 所有者线程 |
| GPU 模型、一个 CPU 确认快照、资产数组 | Python worker，一次处理一个命令 |
| 预览和 `map.json` 的实际写入 | 现有 C++ 后台线程 |
| 不可变三文件资产的实际写入/删除 | Python worker，仅在协商的专用目录中 |

身份分开记录，不能只看 submap ID 或文件名：

- 内容身份：`session_id, submap_id, source_generation, content_job_id`；content_job_id
  是最近一次改变参数的已确认 view/rebuild，reuse 不改变它。目录另记录 binding_job_id，
  表示最近一次接受目标绑定的操作，不能把一次 0 步重绑当成新训练内容。
- 目标绑定：`generation, pose_version, frame_count, optimized_frame_count, parent_frame`，
  以及当前 `parent_from_submap`。同版本新增前沿允许复用旧覆盖。
- 文件身份：本 session 单调递增的 `asset_id`，清单 SHA-256、三文件字节数。
- 目录身份：`map_sequence`，另有当前绝对目标版本和写出时间。

content_job_id / binding_job_id 存在 IPC 回执和地图目录中，不向 C2a schema 1 的根字段
偷偷加字段。目录的清单 SHA-256 将已验证资产绑定到 checkpoint 回执；恢复同时核对
资产的 origin_session_id/submap_id/generation。在线资产不经过离线 bundle，不填占位
source_manifest_sha256 或 bundle_manifest_sha256。

generation 仍为会话中单调递增的失效令牌。IPC schema 2 的任何返回都同时核对
session、job、generation、submap、pose version 和操作种类；迟到的成功不能恢复已失效
的可用状态。存档的 generation 从来不能给新 session 授权。

所有者回调仅做有限元数据校验、至多 512 个成员的比较和邮箱替换。不得在回调里读写资产、
读取原始图像、等待 GPU 或阻塞等待后台锁；沿用 try-lock 与独立计时统计。

## 地图目录与覆盖

新增普通数据 `GaussianMapSnapshot` / `GaussianSubmapEntry`，以及小型
`GaussianMapIndex` 管理目录和绝对目标比较，供 mapper 调用。它们属于可选
`livo_atlas::online_gaussian` 库，不进入默认核心；没有 Pimpl。

每个目录项至少包含：

- logical submap ID、anchor ID、最新 logical primary 范围/修订、active/sealed 状态；
- 可用性 `missing / current / needs_rebuild / failed` 及原因；是否 GPU 驻留；
- 当前目标绑定、最近确认内容身份与 binding_job_id；可选的持久资产描述；
- 资产对应的 primary 局部 body 位姿、实际保留的训练 ID、参数数量；
- 尚在 CPU 内的更新结果与磁盘资产分别记录其 content_job_id，不能混写覆盖；
- 最后一次修复尝试序号，用于有界公平调度。

初次出现但还没训练、被 coalesce 跳过或容量不足的分组，明确为 `missing`，不会制造空的
“已保存地图”。已封存的 logical 组可以有较早前沿的资产：资产的完整 primary 是其确认时
的分组前缀，logical 最新成员与资产覆盖分别输出，不能声称补齐了后续未训练帧。
源 primary 包含所有可能影响当前工作集的已知历史成员，不能从最近 10 个来源 ID 反推。

`GetMapSnapshot()` 返回拥有/共享只读数据的当前内存目录。`map.json` 是带时间与版本的
异步持久目录，最多代表写出时的快照，不标成“始终最新”。其中的资产引用只指向已确认、
持久化成功的内容；内存中更新但未落盘的内容另标为 volatile，不伪装成文件引用。

## 单 GPU 与确认回退

Python 新增专用 `GaussianSubmapWorker`（`submap_worker.py`），由
`online_worker.py` 在 schema 2 分支使用；它组织现有模型和资产接口，不包装所有模型方法。
稳定状态仅有一个 GPU 模型及一个拥有数据的 `confirmed_state`。

1. 新视图用旧模型原地训练，形成 pending candidate。原 `confirmed_state` 保持不变。
2. view/rebuild 接到匹配的 accept：先结束模型 pending，再捕获新的独立 CPU 状态。
   捕获成功后才替换旧 CPU 快照并回复 `commit_ok`。reuse 接纳只替换 confirmed_binding；
   同块保留原确认快照，跨块采用已校验的目标资产状态，不重写训练内容。任何中间错误
   使 Gaussian 失败，不发布半确认目录。
3. 接到 discard：按 C1 规则清空已修改模型，随后从此前 CPU 确认快照恢复；没有旧确认
   快照则保持空模型。先释放旧 GPU 张量再恢复，不能短时并存两个 GPU 模型。
4. 因切换、历史修复或正常停止需要离开 dirty 内容时，先取得有效资产并完成目录引用提交，
   才释放该块的 CPU 快照与 GPU 模型。已经有同一确认内容的资产则复用，不重复写。
5. 同一块整体刚性修正可以在原地重绑；更新模型头与独立 confirmed_binding，CPU 确认
   内容及 canonical primary 保持不变。binding-only 更新不置内容 dirty，已有资产
   可继续供新目录绑定引用，不产生一次多余的数组写入。下一次真正训练后再捕获新内容。

跨块 activate/rebuild 时，“此前确认”明确指原驻留 A，而不是拟加载的 B：保留 A 的 CPU
确认快照与绑定直到候选决策；B 的已加载 CPU 状态至多额外一份，属于单 in-flight 的临时
状态。拒绝时先释放 B 的 GPU，再恢复 A，所有者用 A 的最新绝对目标重新分类；A 可能仍为
current，若 A 已变形则仍为 needs_rebuild。接受 B 后释放 A 的 CPU 状态；不能积累第三块。
从 A 的 CPU 快照回退后，应按 A 的 confirmed_binding 重绑模型头，不使用快照里历史
generation 作为新调度授权。同块修复被拒绝也保留旧内容，但旧内容是否 current 由最新
绝对比较决定，不由“恢复成功”决定。

捕获替换和受限 reader 会有瞬时数组副本；“一个 CPU 确认快照”指稳定驻留，不声称 RSS
只有一份 payload。保存过程中不得修改快照，保存成功前不得训练下一候选。

## 位姿比较与重建

始终对比资产/确认内容的 canonical primary 与最新绝对快照，不逐次累乘回环增量，
也不把每次重绑后的近似值写回 canonical 基准，以防容差累积。

对每个源成员 i：

```text
new_parent_from_S = latest_parent_from_B_anchor
new_S_from_B_i = inverse(new_parent_from_S) * latest_parent_from_B_i
```

按现有 `pose_math::Equal` 的平移范数/旋转角 `1e-9` 比较全部源 primary 局部位姿。
这是刚性复用判断；C2a 格式的 SE(3)/相机一致性校验门限保持不变。

- **局部位姿全部不变：** 立即更新内存绑定的 parent frame、外部位姿与目标版本；历史
  资产的参数/观察文件不改。加载时由 worker 再核对完整源 primary，更新会话内模型头。
  在同 GPU 中重绑不重新训练；从磁盘恢复仍遵循 C2a 的 Adam 重建语义。
- **任意源成员局部变化：** 立即标 `needs_rebuild`，清空其当前就绪引用，旧资产可留作
  重建输入但不能被称为当前地图。即使改变的成员早已被窗口淘汰，也要判为非刚性。
- **同版本新增帧：** 旧块保持有效；新观察单独初始化，不因全局 frame_count 增长
  废弃历史内容。membership/seal 修订不等于位姿变形。

重建从资产或现存确认快照读取最多 10 个保留视图。保留 RGB、mask、depth、K、曝光和
独立 `camera_from_body`，用最新 primary 计算
`camera_from_S = camera_from_body * inverse(new_S_from_body)`，不能再次施加 LiDAR 外参。
新建模型和 Adam，按训练 ID 升序每帧 20 步，最多 200 步，最后给所有者一个整批候选。
中间帧的 model.decide(True) 只是批内执行，不能触发正式捕获/写盘/目录接纳；最终候选
接纳后才替换确认快照。若最终拒绝，回到此前确认内容，再按其最新绝对目标判断可用性；
不能把读取/恢复成功直接升级为 current。

重建仅重训原工作集，不能宣称恢复已淘汰的历史观测对参数的完整影响。实际新 primary
可以包含新增但没训练的成员，覆盖字段继续明确表示哪些视图参与训练。

## IPC schema 2

保留 `<uint32 header_bytes, uint64 body_bytes>` 小端长度前缀、64 KiB/16 MiB 上限和现有
deadline；schema 1 行为不变。schema 2 单独验证允许的类型/字段，禁止混用会话 schema。
新节点开启持久化而旧 worker 不支持时，在首个训练任务前明确报错，不降级。

所有 uint64 头字段继续使用规范十进制字符串，有限计数用 JSON integer，禁止重复 key、
非有限值、object/pickle 或正文长度猜测。路径只在 hello 协商一次：C++ 创建新输出根，
传递其绝对 `state_directory`；Python 校验父目录和专用子目录，独占创建 `states/`。
后续只传 `asset_id`，映射为 `states/<asset_id>/`，不接收任意读写路径。
worker 不读取 ROS 参数，不扫描其他运行目录，不自动接管旧 session 目录。

公共目标头在 C1 身份/前沿之外增加：`anchor_keyframe_id, primary_count, submap_state`。
连续 primary 的 ID 为 anchor+offset；正文记录每个 `submap_from_body`，每条为行优先
16×float64 小端，共 `128*primary_count` 字节，最多 65,536 字节，先核对计数/前沿/精确
长度再分配。首个局部 body 必须单位阵。

| 请求 → 返回 | 额外内容及确认含义 |
| --- | --- |
| hello → hello_ok | schema 2、固定 state_directory、state_schema=1 和明确 persistent_submaps 能力；一次绑定整个会话 |
| activate(empty) → activated | 新 generation/目标、primary 正文；当前没有确认内容，建立空训练目标 |
| activate(reuse/rebuild) → candidate | asset_id 或当前确认内容身份、目标 primary 正文；reuse 0 步，rebuild 20×保留视图数；加载后也必须经过所有者 decision |
| view → candidate | C1 的 9×pixels 观察正文后附 primary 位姿正文；头另加独立 camera_from_body；每个候选 20 步 |
| decision → commit_ok/discard_ok | 与候选精确身份匹配；accept 生成 CPU 确认快照，discard 回到此前确认快照；不等于持久化 |
| checkpoint → checkpoint_ok | 新 asset_id、预期 content_job_id 等确认内容身份；只保存 confirmed_state；返回三文件 bytes、清单 SHA-256、覆盖与参数数目 |
| map_committed → map_committed_ok | C++ 完成目录 fsync 后发送；严格递增 map_sequence 和完整有序 asset_ids（最多 32 个规范 uint64 字符串）；worker 只接受本会话已成功 checkpoint 的资产，记录最后持久引用集合 |
| prune → prune_ok | 仅删除已被新持久目录替代的一个旧 asset_id，带已提交 map_sequence；不操作目录之外文件 |
| stop → stopped | 无 pending，先由 C++ 完成保存/目录/清理链；返回最后持久化身份；异常停止另行报告 incomplete |

candidate 增加 `operation=view/reuse/rebuild`。steps 必须分别为 20、0、20×保留帧数；
rebuild 的累计深度样本上限为 200×最大单视图像素数。reuse 不要求非零梯度或参数更新；
view/rebuild 则记录并校验训练指标。现有比较器不能把 0 步重用当成一次训练成功。
source camera/body 关系来自原始 Keyframe；不能由待校验的 local camera 和 local body
相乘后冒充独立来源。捕获 context 由实际训练的目标/完整 primary 和各视图独立 C_B 组成。

activate 只能在无 pending candidate 时进行。离开另一块/重建前，worker 检查此前确认
内容已有匹配 checkpoint，且最近 map_committed 的引用集合包含它；否则拒绝释放。
本地同块 rigid reuse 不释放模型，不要求先写一个多余的旧绑定资产。
首个空模型不要求 checkpoint。reuse/rebuild 时新目标不可覆盖旧来源身份和 canonical
数组；目标校验失败必须发生在 GPU 分配/旧确认内容释放之前。

## 资产、目录与删除次序

```text
output/
  states/<asset_id>/{manifest.json,gaussians.npz,observations.npz}
  map.json
  preview-<job>.png / .json             # 继续最多 8 对
  status.json
```

资产沿用 C2a `write_state()`，capture_kind=`online_confirmed`。来源 session 为本运行
创建者；目标版本通过目录绑定表达，不能篡改不可变资产来伪装为较新的训练结果。

严格的替换顺序：

1. worker 保存新资产，三文件都成功同步且 reader 可验证，回复 checkpoint_ok。
2. 所有者核对来源内容身份；以**当前绝对目标**重新分类，允许结果成为历史/stale，
   不允许迟到的旧版本覆盖当前可用性。生成新 map_sequence 的拥有数据的目录快照。
3. C++ 后台写 `map.tmp.json`，flush/close/fsync 文件后 rename 为 map.json，再 fsync
   输出目录。所有者收到对应成功后，才算该引用持久提交。
4. 所有者处理步骤 3 成功后发送 map_committed，收到确认后才允许释放/切换；即使没有
   旧资产需要删除也要发送。只有旧 asset_id 不在该集合中才发 prune；完成后才准写下一资产。
   替换期间最多多出一个资产，不做后台无界 GC，不扫描删除用户文件。

任何提交前失败不发布新引用，旧目录/资产保留。资产或目录在 rename 后同步失败都明确
报告 `commit_uncertain`：保留可能已可见的新文件和旧资产，不继续 prune/切换/训练，
仅使 Gaussian 失败，前端继续。这里新增 C++ 目录同步语义，不能沿用仅 close+rename 的
预览 writer 并宣称已有断电持久性。清单 SHA-256由 Python 回传，C++ 索引记录并在读写
时检查格式；真正恢复仍由严格 CPU reader 复验所有文件和数组。

目录第一次写出只描述当前 session；不重新打开追加旧输出。正式自动跨 session 恢复
仍需独立设计，不从这份地图目录推导前端、标定和后端恢复授权。

## 调度、预算与停止

优先完成 candidate decision、安全 checkpoint/目录提交/必要 prune，然后才允许切换。
新观测仍只保留一个 pending；完整目标合并为每个目录项的最新绝对状态，不排队保存
每一个中间位姿版本。未进入 GPU 的中间分组保留 `missing`。

只要同时有前台视图与历史重建，最多执行两个前台 view 后，必须尝试一个历史 repair；
repair 在需要重建的块之间循环选择，不按某块被反复更新就无限回到队首。每次尝试使用
最新目标；若途中更新，完成结果按最新目标重验，stale 尝试也消耗其公平轮次。
必要的 checkpoint/load 是一次任务的前置链，不扩张 pending/in-flight 数量。

目录/状态写入合并为一个待发布快照。在存在训练/修复工作时，两次工作之间最多发布
一次目录/状态，然后必须选择工作；不能因为每次 Observe 都使序号变化而饿死 GPU。
空闲时允许补写最新目录。硬盘/GPU 阻塞下无墙钟公平保证，只给出任务次数上的边界。

初始资源上限是执行者选定的实验预算：

| 资源 | 上限/行为 |
| --- | --- |
| 总 pose frontier / logical 目录项 | 512 帧 / 512 项 |
| 拥有确认内容的 Gaussian 分组 | 32 块，含未落盘的当前块；不自动淘汰旧块 |
| GPU 稳定/瞬时训练模型 | 1；先释放再加载 |
| CPU 稳定确认快照 | 1；跨块 in-flight 最多额外 1 个加载状态，决策后释放/替换；临时副本与 RSS 另测 |
| 单块工作集 | 10 视图，单视图 1M 像素，每来源 1,000 高斯，总 10,000 |
| 文件 | 沿用 512 KiB / 1 MiB / 192 MiB，map.json ≤512 KiB |
| 完整/在写资产目录 | 至多 32 个被引用资产 + 1 个替换/提交中资产；失败后停止新写入 |
| 资产文件逻辑字节总量 | 至多 33×193.5 MiB，约 6.24 GiB；不等于文件系统实际块数 |
| 磁盘保留空间 | 新写入前至少留 1 GiB，并预留一个最大资产大小；不足时明确失败 |
| pending / in-flight | 各 1；修正目标是有界元数据，不是另一条任务队列 |

32 块已占满时不再给新组训练分配槽位，记录 missing/resource_limit 并继续维护已有块；
不得先训练第 33 块后才发现无处保存，也不静默删除旧资产。分组容量与停止/磁盘错误
分别计数。只有无法安全落盘、协议/模型错误或前沿超限时终止 Gaussian，前端保持运行。

正常 Finish 在既有 shutdown_timeout 内优先拒绝未决候选、保存 dirty 确认内容、提交目录
并清理替换资产，完成后再 stop。实施/验收持久化时显式配置足够的 shutdown_timeout
（示例 30 秒），不沿用 C1 的 3 秒默认值来承诺所有保存都完成。超时输出 incomplete/
未落盘内容身份，旧持久资产仍有效；强制退出不能保证保住最近 volatile 更新。
底层系统调用永久阻塞时，仍不能保证严格实时退出。

新增统计覆盖 checkpoint/load/rebind/rebuild/prune、stale、map_sequence、dirty、
indexed/missing/needs_rebuild、容量命中、content/binding/durable job ID、incomplete shutdown，
以及 GPU allocated/reserved、进程 RSS、资产逻辑字节数与所有者回调耗时。

## 实施与验收顺序

1. 纯 C++ 目录/绝对位姿绑定，先证明旧前沿、完整历史成员和迟到结果不会误判。
2. schema 2 的跨语言受限消息；默认 schema 1 完整回归。
3. Python 确认回退、资产切换、rigid reuse 与 bounded rebuild，用实际 CUDA 验证。
4. C++ mapper 接纳/公平调度/持久索引/清理链，再接双 ROS 参数和共用 main。
5. 本机受控 socket 竞争、存储故障、容量边界；Noetic/Humble 构建和真实入口；最后一次
   独立代码审阅。Important 先 RED→GREEN 修复，再以最终源码复验受影响路径。

真实数据使用现有 Retail_Street.bag：同源码、二进制、输入/CPU 设置的 Gaussian OFF 与
持久化 ON 完整回放，严格比较四组前端数组、发布语义、517 个原始文件与后端最终结果。
先短段验证再完整回放；运行输出放包外新目录，串行 nice -n10，建前检查磁盘空间。
源码和 bag 的 SHA-256 随报告记录。另做 C1 模式短段回归，验证新增开关没有改变旧路径。

官方数据的自然闭环不保证触发所需的非刚性/竞争场景；另用真实保存的原始观测和受控
绝对位姿快照，验证全块刚性变换的数组不变、非窗口历史成员变形会重建、V1→V3 跳跃、
训练/写盘期间更新、未训练分组、32/33 槽位、目录提交后同步失败与未引用资产清理。
人工修正明确标为受控实验，不当作真实回环或画质收益。

Humble 至少验构建/安装消费者、参数、共用入口和受控 IPC。真实 GPU 是否可用单独记录，
没有容器 GPU 就不能把假 worker 成功计为 Humble GPU 验收。迁移步骤、命令、失败案例、
新参数和已知预算写进仓库；大型资产/环境/日志留包外。

## 待实施时由构建者决定的小项

内部容器、辅助函数拆分和统计输出的排列可按现有代码选择；不为测试添加产品方法。
头文件/实现文件按 Gaussian 目录、目录持久化按 io 目录归类，避免继续扩大一个巨型文件。
所需新协议字段、确认/持久化次序、覆盖与预算不得靠实现者猜测或静默改变。

## 独立可实施性检查

一次只读 builder check 指出跨块拒绝回退对象不明确，以及没有 prune 时 worker 无法获知
目录持久提交。现已明确保留原驻留 A 到决策结束、按 A 最新目标重判，并增加始终发送的
map_committed 引用集合确认。执行者同时分离 content_job_id / binding_job_id，刚性重绑
不再制造新训练内容或数组写入。检查者确认这些裁决解决其问题，无其他重要未决问题。
上述为执行者技术裁决，不把它们写作用户逐项批准；实施前仍按用户要求通知切换模型。
