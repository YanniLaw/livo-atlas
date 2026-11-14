# P6-02c3a：全局 TF 与当前位姿

2026-11-14，代码基线 `3d3c9d8`，C2b 产品修复代码为 `65c6945`。
状态：**实现、正式主验收及整阶段独立审阅完成；14组构建、12组真实入口、12次官方回放通过；1项验收器Minor延期，实际记录顺序已另行复核**。
实现用法见[全局输出说明](../../global-pose-output.md)，原始证据与范围见[验证记录](../../p6-02c3a-validation.md)。
已完成一次独立可实施性检查，结论为没有影响实施计划的未决问题；该检查不替代用户对方案的确认。
本页保留本轮设计契约；实际完成范围与证据以验证记录为准。
延续用户的普通 C++ 核心、同一 ROS main、Noetic/Humble、Google C++17、直接成员及函数间空行要求。
以下拆分与接口是执行者建议，不记为用户逐项确认。

## 意图与阶段边界

[C2b](../../online-loop-backend.md) 已能在线接纳回环并保存版本化位姿，但 ROS 仍发布原始 odom 位姿。
先使当前定位结果能随已提交优化修正，并建立正确的 TF 拓扑；随后再处理历史轨迹和地图重建的开销。
不向前端状态、IMU bias、原始资产或匹配地图回写修正。

| 子阶段 | 交付 | 状态 |
| --- | --- | --- |
| C3a | `map -> camera_init`、当前全局 body 位姿、普通 C++ 修正值 | 实现、主验收与独立审阅完成 |
| C3b | 使用每帧优化位姿的关键帧轨迹、版本化几何地图与输出来源信息 | 后续单独设计、测量并计划 |

完成 C3a 不等于完成 C3。它不把当前一张 odom 点云整体乘修正当作优化历史地图。
3DGS 继续使用原始观测、局部几何、`PoseSnapshot` 与修订信息；ROS 当前位姿不是新的位姿权威。
本轮不加入 iSAM2、描述子、Gaussian 训练、通用事件总线、新线程、恢复续写或帧名全面迁移。

## 实际代码与路线取舍

- `LivoSystem::ProcessNext` 先提交关键帧/在线成员，再调用普通结果回调；最后才由节点执行 `after_process`。
- `OnlineSubmapManager` 已拥有原始位姿和最近冻结的优化快照，可在原所有者线程直接读取最后优化锚点。
- `GetPoseSnapshot()` 会物化完整前缀；不能在每次 LiDAR 发布中调用它来取得一个刚体修正。
- C2b 的 `Poll` 在一次原始结果发布之后运行，新接纳版本自然从下一次有效原始位姿发布开始可见。
- 两端 `LivoPublisher` 都用 ROS `now()` 为原始 odometry/TF 赋时；结果内的估计时间另有语义。
- 原始旋转先转 Euler，再构造已发布的四元数；这不等价于后端对近似旋转的 SO(3) 投影。

| 路线 | 收益 | 代价 | 本轮选择 |
| --- | --- | --- | --- |
| 一次完成 C3 的 TF、轨迹和整图 | 一次提供完整全局可视化 | 同时引入历史重建、点云读取、版本调度及额外内存问题 | 暂不采用 |
| 先 C3a，再 C3b | 先验证坐标、时间与兼容性，C3a 每次发布只需常数规模数据 | 当前阶段没有优化后的历史轨迹/整图话题 | 执行者推荐 |
| 同时迁移全部旧话题到 map/odom/base_link 命名 | 对新消费者更统一 | 改动既有 frame、工具和回归契约，迁移面更大 | 独立后续工作 |

不为 C3a 新建 callback 分发器或保存第二份 raw 轨迹。C3b 再明确轨迹/地图队列、采样、内存预算与最终导出。

## 坐标与权威

采用 REP-105 的单父节点拓扑及 map/odom 连续性分工；map 的原点沿用本工程初始 odom 的局部定义，
不新增 GNSS、ENU 或重力对齐估计。[REP-105 官方源码](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

```mermaid
flowchart LR
  M["map：已提交后端修正"] -->|C_v| O["camera_init：原始 odom"]
  O -->|O_pub(t)| B["aft_mapped：原始 body / IMU"]
```

不发布额外的 `map -> aft_mapped` TF。`/global_pose` 是消息，不会增加第二条 TF 父边。
版本 0 定义 `C_0=Identity`。正版本 v 令 `k=optimized_frame_count-1`：

```text
C_v = P_v(k) * inverse(ProjectPose(O_raw(k)))
P_live(t) = C_v * O_pub(t)
```

`ProjectPose` 保留平移，在临时副本对已验证的近似旋转作现有 SO(3) 投影。
第一式与 `kProjectedLastOptimizedAnchor` 的尾部传播完全相同；不可改用最新原始帧或任一子地图锚点。
版本 0 不访问 `k`，即使尚未产生关键帧，也能给出单位修正。
正版本的 legacy 尾部策略不能保证严格刚体；全局修正接口明确拒绝，不悄悄替换它的语义。
这不影响原有 legacy 快照的读取与离线能力。

`O_pub(t)` 必须由当次实际发布的原始 odometry/TF 的位置和四元数组成，
不能用 `result.estimate.rot_end` 直接替代，也不修改旧四元数生成规则。
原始矩阵略非正交时，Euler 发布表示与后端投影表示可能有差异：
当前 ROS 全局位姿保证与当次 TF 链一致，历史优化关键帧始终以 `P_v(i)` 为准。
不把这两种估计强行要求为逐元素相等，不把一个尾部修正应用到整条优化历史上。

提交权威仍为既有 prepared 与正式 pose 文件；新增修正值、TF、位姿消息都只是派生输出。

## 普通 C++ 接口与成本

拟在已有 keyframes 库内增加 `mapping/map_correction.h/.cc`，不创建新的安装库：

```cpp
struct MapCorrection {
  std::uint64_t pose_version = 0;
  std::uint64_t frame_count = 0;
  std::uint64_t optimized_frame_count = 0;
  RigidTransform map_from_odom;
};

RigidTransform ApplyMapCorrection(
    const MapCorrection& correction,
    const RigidTransform& published_odom_from_body);

// LivoSystem，及其现有 OnlineSubmapManager 的同名只读方法。
MapCorrection GetMapCorrection() const;
```

`GetMapCorrection` 只读冻结快照头、优化锚点位姿和对应 raw 位姿；frame_count 是当前已提交帧数。
不调用 `GetPoseSnapshot`，不复制历史、不读取磁盘、不获取点云、不进入 GTSAM。
读取和计算时间/额外空间为 O(1)，已有快照/输入的验证仍发生在提交路径。
正版本锚点必须已覆盖，数值有限、修正严格刚体；零版本必须为零优化前缀和单位修正。
局部辅助函数可提取以复用原投影公式，但不改变已有物化结果或尾部校验容差。

`ApplyMapCorrection` 检查上述头、两个严格刚体变换及有限值，返回 `map_from_body`，不修改输入。
外部 SDK 调用者需提供自己实际使用的刚体 odom 位姿；原始滤波矩阵需先按其发布规则形成刚体表示。
非在线模式的 getter 抛 `logic_error`；已提交状态可只读诊断，错误后不由此恢复处理。

节点只在普通结果回调的已更新、非视觉分支读取该值。
这是关键帧提交结束后的同线程 const 读取；不在禁止重入的 OnlineSubmapCallback 中调用系统。
worker 不接触该接口；C2b 的单一通知回调、任务槽和 main 组装保持原职责。

## ROS 配置与发布

拟增加共享 `GlobalOutputOptions`，由 `LivoConfig` 持有，两个 ROS 参数入口同源读取：

```yaml
global_output:
  enabled: false
  publish_tf: true
  map_frame: map
```

Humble 使用相同点分字段：`global_output.enabled`、`global_output.publish_tf`、`global_output.map_frame`。
开启要求 `keyframes.enabled=true`、`submaps.enabled=true`、`submaps.mode=online`。
不要求启用回环或链接 GTSAM：没有已提交优化时发布单位修正；普通 SDK 的合法 projected 更新也可驱动它。

map_frame 采用本工程保守约定：非空 ASCII 字母/数字/下划线及单斜杠分段，无首尾斜杠、空段或空白；
不能等于 `camera_init`、`aft_mapped`、`world`。最后一个名称已用于旧 IMU 传播话题，不能把其原始数据误解释为全局 map。
启用时在输出 I/O 前检查所有约束；关闭时不检查其运行前提、不创建新 publisher、不调用修正 getter。
当前 odom/body 帧名保留；map_frame 只命名新父帧。publish_tf=false 供外部管理 TF 的应用使用，
此时 global_pose 仍属于本工程 map 定义，外部帧定义须由集成方对齐。

实施细化：两个既有回放工具增加 `--global-output`，只启用该参数与必要的在线存储，不隐式开启回环。
新增 TF/pose 的验证记录与旧前端摘要分开，避免为测试更改冻结输入配置；默认启动行为不变。

`LivoPublisher` 构造函数末尾增加默认 `GlobalOutputOptions={}`，
`Publish` 末尾增加默认空的 `optional<MapCorrection>`；旧调用保持可编译。
启用且进入非视觉 updated 分支却缺少修正，属于接线错误，发布前抛异常，不静默降级为单位修正。
两端节点负责提供修正，ROS publisher 只作消息转换和发送；不注册新 main 或新的后台线程。

| 输出 | 类型 / frame | 时机 |
| --- | --- | --- |
| `map_frame -> camera_init` | 动态 TF | 全局输出及 publish_tf 开启，有效原始位姿发布时 |
| `/global_pose` | PoseStamped，frame=map_frame | 全局输出开启，同一次有效原始位姿发布 |
| `camera_init -> aft_mapped` | 原动态 TF | 保留旧数值、发布次数和旧分支 |

新增 pose 发布队列/depth=1；Humble reliable/volatile，Noetic 非 latch，不在进程内保留历史。
不用 Odometry 填充未知全局 twist 或协方差：两种 ROS 的 Odometry 均赋予这些字段具体坐标语义，
当前后端没有提供对应估计。[Noetic 定义](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)、
[Humble 定义](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)

## 时间、可见性与失败

1. 原始 odometry 现有 ROS 发布时间只取得一次，原始 TF、新 map TF、global_pose 复用其整数 sec/nsec。
   不把后端锚点时间、求解完成时间或 `FrontendResult.timestamp` 偷换为这一 ROS 时间。
2. 原始 TF 与新增 map TF 使用同一个现有 broadcaster，一次 vector sendTransform 发送。
   两端已安装 API 都支持向量重载；关闭功能或 publish_tf=false 时保留原单 TF 路径。
3. 本次发送使用一次捕获的 MapCorrection，计算/校验全部完成后才发送。回调内无后台提交，故不会混入另一版本。
   两类 TF 可在同一个 TFMessage 中发送，但 TF 与 global_pose 的跨话题传输不保证原子性。
4. 只有严格正、且大于本 publisher 上一次全局输出时间的 ROS 时间才产生新增输出。
   零时间、暂停造成的重复时间、回退时保留原始发布，跳过全部新增 TF/pose；可限频报告原因。
   不改用 wall clock 冒充 ROS 时间，不改变后端 Poll/等待逻辑。
5. 时间恢复到严格大于最后全局输出时间后，下一次有效原始位姿使用最新已提交修正恢复发布。
   不主动清空外部 TF buffer，不把旧时刻的新修正写回同一个 TF 时间点。
6. Poll 在原始发布后接纳的新版本，只在下一次合法原始位姿发布时广播。
   空闲时不重复旧 body 估计来伪造新测量时间，也不只更新 map 边而留下无法配对的 body 边。
7. Visual、Waiting、单独 IMU propagation 不产生新增输出。C3a 不把 `world` 改名或替换原始云/path 的坐标。
8. 正常最终排空仍以磁盘和 SDK 提交状态为准。最后一次原始发布后接纳的版本、或 ROS 已关闭后排空的版本，
   不保证出现在 ROS 话题；不增加关闭时的虚假样本或“最后一条必达”承诺。

缺失/非法修正不能落成半个新数据结果；计算阶段失败不发送新增消息，由现有 main 报错退出。
传输阶段不能回滚已经发送的 TF，更不能撤销已提交后端位姿。只读 getter 不赋予重试当前输入的权限。
C3a ROS 输出用于当前定位/可视化；PoseStamped 和 TF 没有位姿版本字段，禁止借 ROS1 Header.seq 编码版本。
需要精确版本的 SDK/3DGS 继续消费 MapCorrection/PoseSnapshot；跨 ROS 的版本化传输协议由 C3b 单独确定。

## 验收边界

下述验收已执行并通过，详见[验证记录](../../p6-02c3a-validation.md)；逐任务步骤见[实施计划](../plans/2026-11-14-p6-02c3a-global-pose.md)。

- 原生测试：版本0/无关键帧、正版本 N<M、同版本增长、非交换旋转/平移、近似 raw 旋转、
  失效锚点、非有限输入、legacy 正版本拒绝；修正与已有物化尾部一致，输入及旧快照不变。
- 通过真实 System/Manager 追加和 projected 更新，验证 getter 的版本/前沿、失败后已提交只读语义。
  实现检查确认无完整物化、历史复制、磁盘和额外回调；不以微小时间阈值编写易波动的性能测试。
- 两端实际 publisher 测试：默认话题/数值保持；启用时 TF 树单父、同一整数时间戳、
  非单位修正组合正确、一个 TFMessage 包含两条边、global_pose 与 TF 链相符；publish_tf=false 单独覆盖。
- 专门注入略非正交原始旋转，比较实际 raw quaternion 构成的链，不误用投影 raw 来断言旧话题；
  在原生测试另验证快照的 projected 表示，不能相互替代。
- 零/暂停/回退 ROS 时钟、空闲期间版本更新、恢复后发布、可视/等待/传播分支及关闭排空无新测量，
  通过真实节点确认后端仍能泵送；无效配置在输出 I/O 前拒绝。
- 沿用 SDK 六组合与双 ROS shared/static 全关/双开的构建/安装消费矩阵，检查无新增 GTSAM/ROS 反向依赖；
  全关算法构建亦测试 global_output 开启的单位修正。
- Noetic/Humble 各验证三种完整运行：global OFF/loops OFF、global OFF/loops ON、global ON/loops ON；
  各验证三种3秒短段：global OFF/loops OFF、global ON/loops OFF、global ON/loops ON，共12次。
  全局 OFF 对照 C2b 同平台基线，ON 比较四组前端数组、原始资产、旧发布数值/数量与 body TF 子集；
  新增参数/全局话题/TF 边另验，允许异步调度改变接纳任务数，不放宽算法阈值强造成功。
- ON 从保存的各正式位姿计算期望修正，核对 ROS 修正只取已提交版本且顺序不回退。
  多个版本修正数值相同时仅记录可解释版本范围，不从缺少版本字段的 TF/pose 断言唯一版本；原生测试固定真正版本。
  真实合成链必须产生非单位修正；bag 无接纳仍合法，不能据无真值数据宣称全局精度提升。
  若最终版本在最后样本后才提交，按上述关闭语义记录，不以最后 TF 等于最终文件为错误门槛。
- 新证据放独立阶段目录；串行 nice10/-j1 构建，回放 CPU0/OMP1/1倍速/图像队列32/无RViz。
  原 C2b 归档和 bag 只读，不重用旧日志为新功能验收，不重复索要长 bag。

## C3b 与 3DGS 衔接

C3b 的优化关键帧轨迹必须按每帧 `P_v(i)` 输出；既有 odom Path 即使经 TF 整体移动，也不满足该含义。
地图以局部几何及来源关键帧为本体，用同一提交快照的变换派生，明确 primary/support 防重复融合。
输出必须绑定 pose_version、frame_count、成员/表示修订，异步过期结果不可覆盖较新产品。
地图/轨迹重建成本、资源上限、节流、ROS 版本标识、完成/截断及可持久化导出需要该阶段单独设计。
不得仅因 C3a 增加一个 map TF 就把合并点云当作在线 3DGS 的唯一训练输入。

## 执行者已作的设计选择

本轮建议先做 C3a；保持原始发布表示，以实际发布刚体组合当前全局位姿；使用 PoseStamped；
常数规模 getter 复用原位姿权威；新增输出受单调 ROS 时间门控；关闭后不承诺补发最后版本。
这些选择的代价是历史轨迹/地图和精确 ROS 版本协议留待 C3b，暂停/回退时不刷新新增输出，
当前 ROS body 位姿不等同于优化历史帧，必须在使用说明和验收中明确。
