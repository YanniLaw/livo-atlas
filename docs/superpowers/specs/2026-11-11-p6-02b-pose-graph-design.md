# P6-02b：GTSAM 位姿图设计

2026-11-11。状态：**已确认版本化刚体尾帧策略，实施中，尚未验收**。本文延续
[P6 阶段路线](2026-11-08-online-submaps-design.md#p6-01-之后的阶段路线)和
[P6-02a 几何子地图](2026-11-10-p6-02a-geometry-submaps-design.md)。
这里的实现选择是本轮建议，不表示已验证性能或真实回环效果。

## 目标与范围

建立独立的普通 C++ 位姿图：接收已提交关键帧的原始 body 位姿，构造相邻里程计约束；
接受调用者明确提供的模拟闭环约束，使用 GTSAM 求解；通过现有 `PoseUpdate` 和
`LivoSystem::ApplyPoseUpdate` 交付可检查的位姿版本和几何修正。

本阶段不实现真实回环检索、点云配准、`map→odom` TF、IMU 预积分、前端滤波反馈、
子地图图优化、多 session 重定位或在线 3DGS。模拟约束仅用于验证后端链路，绝不在生产节点中
自动注入，也不宣称它是真实检测结果。普通 ROS 节点不会因编译位姿图库就自动启用优化；
本阶段通过 C++ 集成场景验证实时数据进入和结果应用，后续 P6-02c 再接真实闭环及 ROS 全局输出。

## 可选方案

| 方案 | 优点 | 代价 |
| --- | --- | --- |
| 按需批量求解，显式返回结果 | 边界清楚，容易复现、测试失败与版本提交；无需持久 GTSAM 状态或线程 | 每次求解随图增长，调用时阻塞，不能假定长距离实时性 |
| iSAM2 常驻增量求解 | 面向频繁增量约束，长期可能更高效 | 需管理内部状态、异常后重建、重线性化和消费时序；性能仍须实际测量 |
| 直接把优化器与回环线程嵌入 ROS 节点 | 可直接在节点中运行 | 在尚无真实回环时引入线程与 ROS 生命周期耦合，不利于独立验证 |

建议先采用第一种，并把图输入和结果定义为普通数据，使后续可替换求解实现。
这不是对 iSAM2 性能的结论；只有在真实数据和资源测量支持下才决定何时切换。
GTSAM 的 `ISAM2::update` 会管理增量因子与新变量，具体约束见
[官方接口文档](https://gtsam.org/doxygen/a06323.html)。

## 现有边界

`Keyframe::odom_from_body` 是前端原始位姿，单位米；关键帧 ID 从 0 连续增长。
`OnlineSubmapUpdate` 在原帧和成员修订提交后通知，拥有的数据可短期保留；但回调只能记录或入队，
不能重入 `LivoSystem`，也不能抛异常。`LivoSystem` 同时只登记一个在线回调。
`PoseSnapshot` 的正版本是 map 系，版本 0 是原始 odom 系；`PoseUpdate` 必须提供 `base_version`、
已优化前缀长度及该前缀的全部位姿。已经实现的版本校验、尾帧延续和几何失效机制继续作为权威。

ROS1 主循环主动调用 `ProcessNext`，ROS2 定时器在 executor 内调用同一方法。此阶段不为两端
分别复制后端实现，也不引入第二个 main。需要订阅在线通知的 C++ 使用方负责统一分发，
不能悄悄覆盖已有消费者的唯一回调。

## 模块与接口

新增可选的 `livo_atlas::backend` 普通 C++ 库。公开头使用已有 `RigidTransform`、`PoseSnapshot`
和 `PoseUpdate` 以及标准容器；不暴露 ROS 或 GTSAM 类型，不使用 Pimpl。
类直接持有轻量关键帧位姿和约束；不持有图像、点云、几何快照或 `LivoSystem`。
GTSAM 的图、因子、初值和求解器在求解函数内构造，生命周期不跨求解调用。

```cpp
struct PoseGraphOptions {
  double prior_rotation_sigma_rad = 0.001;
  double prior_translation_sigma_m = 0.001;
  double odometry_rotation_sigma_rad = 0.05;
  double odometry_translation_sigma_m = 0.1;
  int max_iterations = 100;
};

struct LoopConstraint {
  std::uint64_t from_id;
  std::uint64_t to_id;
  RigidTransform from_body_from_to_body;
  double rotation_sigma_rad;
  double translation_sigma_m;
};

struct PoseGraphResult {
  PoseUpdate update;
  double initial_error;
  double final_error;
  std::size_t iterations;
  bool iteration_limit_reached;
};

class PoseGraph {
 public:
  explicit PoseGraph(PoseGraphOptions options);
  void AddKeyframe(std::uint64_t id, const RigidTransform& odom_from_body);
  void AddLoopConstraint(const LoopConstraint& constraint);
  PoseGraphResult Optimize(const PoseSnapshot& current) const;
};
```

上述接口是实现基线；实现可添加少量统计查询，但不提前建立通用优化器插件、消息总线或
后台任务框架。一个图只能由一个串行所有者使用，生命周期绑定同一次运行或同一数据集。
当前整数 ID 和 `PoseSnapshot` 不携带全局数据集标识；调用方必须避免混用不同数据集的同号帧。

## 因子、坐标与噪声

记 `X_i = T_map_Bi`，原始 `O_i = T_odom_Bi`，`O'_i` 是仅供后端求解的旋转投影版本
（见下节）。相邻帧的里程计测量为 `Z_ij = inverse(O'_i) * O'_j`，即 `T_Bi_Bj`。
在图中为 `(i, i+1)` 建立一次 `BetweenFactor<Pose3>`；首帧使用固定测量 `O'_0` 的
有限噪声先验确定坐标规范。
第一版不把支持帧引用当作额外观测，也不从不同子地图重复添加相同里程计因子。

调用者提供的闭环测量同样是 `T_Bi_Bj`，要求 `i < j`、`j > i + 1` 且两个 ID 已存在。
一个有序帧对最多插入一次；不自动倒转约束或变换其噪声。无效旋转、非有限数、非正噪声、
缺失节点和重复约束均在修改图之前拒绝。未来如需多个独立测量、完整协方差或鲁棒核，
另行定义来源身份与噪声坐标，不把本阶段接口当成已完成的真实闭环接口。

所有标准差的精度 `1 / sigma²` 还必须能以有限、非零的 double 表示；否则在插入图之前拒绝。
使用显式对角高斯模型，禁用 GTSAM 对极小标准差的自动硬约束转换。

三个噪声模型均为六维对角、高斯、各向同性的旋转和平移标准差；旋转单位 rad，平移单位 m。
GTSAM `Pose3` 切空间排列是旋转三项在前、平移三项在后；因子实际残差为
`Local(Z_ij, inverse(X_i) * X_j)`，与方向约定必须通过非交换旋转和平移测试。
可参阅 [GTSAM BetweenFactor 文档](https://gtsam.org/doxygen/a06855.html)和
[4.2.2 源码](https://github.com/borglab/gtsam/blob/4.2.2/gtsam/slam/BetweenFactor.h)。

上面的默认数值只是合成实验权重，不是传感器标定结果，也不代表后验协方差。
相邻前端位姿高度相关，不能把 EKF 状态协方差未经推导就当作独立因子噪声；本阶段不输出
置信区间或宣称统计一致性。真实回环必须在 P6-02c 单独设计噪声、异常值和相关性处理。
第一版使用 GTSAM Levenberg–Marquardt 批量求解；`max_iterations` 必须为正，其余停止参数使用
所选 GTSAM 版本的默认值并在验证记录中列出。初始、最终误差均由同一因子图及对应位姿计算。

## 近似旋转的处理

现有 `ValidateExtrinsic` 接受正交残差逐元素最大值及行列式误差不超过 `1e-4` 的旋转，
并保留原始矩阵。后端沿用此输入校验，不放宽非法矩阵，也不改写关键帧、标定或输入快照。
求解时对通过校验的原始帧、闭环测量和快照初值，在临时副本上将旋转投影到最近的 SO(3)：
若 `R = U Σ Vᵀ`，使用 `U diag(1,1,det(UVᵀ)) Vᵀ`；平移不变。GTSAM 因子、首帧先验
和初值统一使用投影后的变换。计算完成后仍只发布新的 map 位姿，不反向更新原始数据。

对严格刚体输入，无回环优化结果在双精度容差内恢复原始轨迹；对通过 `1e-4` 校验的近似输入，
允许结果相对原矩阵产生投影差异，不要求与原始位姿逐字节一致。测试应覆盖轻微非正交输入、
大于允许阈值的输入和投影后重建；前端数组及原始资产仍须精确保持。

现有尾帧延续使用原始矩阵的转置作为逆变换，不会对乘积再投影。两个分别合法的近似矩阵组合后
仍可能超出旋转校验范围：例如锚点和尾帧都是 `diag(1.000045, 1, 1)`，单帧残差约 `9e-5`，
投影后的优化锚点为单位阵时，延续结果的正交残差约 `1.8e-4`。须区分两种情况：

- 如果尾帧在提交候选之前已存在，现有 `MakePoseSnapshot` 在写盘前拒绝，旧版本保持可用。
  将新增帧纳入图、取当前快照并立即提交完整前缀，可以避开这一次尾帧延续。
- 如果先成功提交优化版本，之后再加入这样的尾帧，原始帧和成员索引可能已经提交，
  `Notify` 物化当前位姿时会失败并使在线会话进入终止状态，不能指望后端回调再修复该帧。

本机 P6-02a Noetic 回放的 172 个关键帧中，原始旋转最大正交残差约 `9.54e-14`；假设优化锚点
等于投影后的原始锚点，对 14,706 个有序锚点/尾帧组合只读计算，没有超过现有 `1e-4` 阈值。
该检查没有运行 GTSAM，也未建模优化修正或平移；不能推断其他数据不会遇到边界。
证据和可复跑脚本保存在包外 `validation/P6-02b-20261111/`，迁移电脑时须单独复制；
脚本接受已有 `keyframes/frames` 目录和新的 JSON 输出路径，不改写输入。

本阶段采用兼容的新尾帧策略：策略须随内存快照和磁盘版本传播，
使用统一投影后的锚点和尾帧计算旋转与平移，并保留旧数据的旧策略；只投影结果旋转而沿用
原平移并非同一个刚性校正，误差还可能随距离增长。新格式的旧读者拒绝、ABI 和重物化行为均需测试。
旧策略继续保留其原有行为，并测试上述两种失败；新后端产生的优化版本使用新策略。

### 已选方案：版本化刚体尾帧策略

为新优化版本显式指定新的尾帧策略，默认旧调用仍沿用旧策略。
新策略对通过原校验的原始锚点和尾帧分别进行上述 SO(3) 投影，记为 `P(O_k)` 和 `P(O_j)`，
平移保留原值；再用同一个刚体变换计算两者：

```text
C_tail = T_map_Bk(optimized) * inverse(P(O_k))
T_map_Bj = C_tail * P(O_j),  j > k
```

新策略的优化前缀也先在副本中逐帧通过现有 `1e-4` 校验并投影旋转，输入 `PoseUpdate` 不变。
这样任意合法调用方都能得到真正的刚体校正；新策略快照和文件中的旋转要求正交及行列式误差
不超过 `1e-9`。旧策略仍保持原有容差与计算。

不能只在最后投影 `T_map_Bj` 的旋转：锚点到尾帧距离为 `L` 时，上述反例还会留下约
`0.000045 L` 的平移误差。策略需要在 `PoseUpdate` 和 `PoseSnapshot` 中显式携带，并在同一版本
保持不变。建议新增 `PoseTailPolicy`，取值为 `kLegacyLastOptimizedAnchor` 和
`kProjectedLastOptimizedAnchor`，在两个结构末尾增加默认为旧策略的 `tail_policy` 字段；版本 0
仅允许旧策略。优化器生成新策略候选，已有调用方默认继续生成旧策略候选。
同版本更改策略必须拒绝，即使当前位姿数值碰巧相同。这会改变公开对象布局，下游程序必须重编译。

兼容设计以新位姿文件 `schema_version: 2` 和策略名 `last_optimized_anchor_projected` 区分，
不改变数据集和成员索引的 schema。
新读者同时支持原有 schema 1/旧策略和 schema 2/新策略；旧文件按原公式读取及扩展，
原 schema 1 写入与旧调用行为保留。旧读者遇到 schema 2 应明确拒绝，不能误读为 schema 1。
现有旧策略使用的 `pose_math::Equal` 对非正交矩阵不能可靠识别纯伸缩；新策略需按真实矩阵和
平移误差验证冻结尾帧，不假定旧校验能发现所有破坏。

新策略应从后端首次提交的版本开始使用，不能自动修复已经进入终止状态的旧会话。
需要验证新旧策略跨版本切换、写盘/重读/继续增长、同一版本策略不可变、旧读者拒绝新文件、
近似旋转与非交换旋转/大平移、原资产不变及新增计算开销。

## 快照、提交与失败

`AddKeyframe` 只接受下一个连续 ID，错误或重复输入不修改图。`Optimize` 不修改图，允许再次求解。
它以调用者提供的当前快照作为初值，冻结图中前缀 `N`：`0 < N <= current.frame_count` 且
`N >= current.optimized_frame_count`。版本 0 时，快照前缀与图中的原始位姿须逐系数数值相等
（允许正负零相等），在做 SO(3) 投影之前比较；
正版本使用当前 map 位姿作初值。最终结果只在值、误差和完整前缀均有效时返回。

结果的 `update.base_version = current.version`、`update.optimized_frame_count = N`；
输出 ID 恰好为 `0..N-1`。本阶段求解是同步的，单一所有者在 `Optimize` 返回后、应用前仍可
处理新帧，因而应用时可能已有 `M >= N` 帧；现有路径按最后优化锚点延续尾帧，它们不算已优化。
求解期间并行输入属于未来异步方案，不属于本阶段。若已有新版本提交，旧 base 的结果被现有系统拒绝，
调用者用最新快照重算，不能改写 base 伪装成新结果。新增的约束仍保留在图中，重算不会重复插入。

返回前要求初始与最终图误差有限，且 `final_error <= initial_error + 1e-12 * max(1, initial_error)`；
零误差的无回环图可以不下降。`iterations` 为求解器实际迭代数，`iteration_limit_reached` 只表示
迭代数达到上限，不单独判定是否收敛。达到上限但数值有效时返回带该标志的候选；示例和自动集成测试
默认不提交这种候选，由调用者明确决定是否接受。本阶段不把误差不增等同于收敛或地图正确。
求解失败、误差超限、非有限结果或输入校验失败不发布 `PoseUpdate`，也不改动原始关键帧与已有版本。
`ApplyPoseUpdate` 在既有路径中先写位姿快照，再更新几何和通知；若持久化后几何更新失败，
系统会进入既有终止状态，不能声称图和所有派生数据有跨文件事务或断电耐久保证。

最小在线使用顺序是：在每次 `ProcessNext` 的回调中只复制本次已提交帧的 ID 和原始位姿；
当前一次 `ProcessNext` 最多提交一个关键帧，可使用预先构造的单槽数据，避免在禁止抛异常的
回调内分配；每次调用返回后立即消费，不用会连续处理多帧的 `ProcessAvailable` 积压该单槽。
明确收到约束时取快照、求解、调用 `ApplyPoseUpdate`。
此路径同步阻塞，不在回调内求解或应用。批量图、位姿快照和约束内存随帧数增长；本阶段不承诺
实时上限或无限时长有界，需测量后再决定增量求解、异步队列或持久化策略。

优化前缀最后一帧的校正可写为 `T_map_odom = T_map_B(N-1) * inverse(T_odom_B(N-1))`，
与现有尾帧延续使用同一锚点。该量在本阶段只用于说明坐标关系，不新增 ROS TF；未来发布时仍须
处理时间、坐标系命名和已有 TF 的唯一发布者。

## 构建与兼容

`LIVO_ATLAS_WITH_GTSAM` 默认为 `OFF`。关闭时不查找、链接或运行时加载 GTSAM；没有 GTSAM 的
Noetic/Humble 和普通 SDK 应保持现有行为。打开时显式查找 GTSAM 并构建 backend；普通公开头
仍不包含 GTSAM 类型。enabled 构建的 installed CMake、catkin、ament 以及 shared/static 传递链接
必须实际验证，不能只验证源码树内部链接。普通 SDK 和 ament 消费者使用导出的
`livo_atlas::backend`；catkin 使用既有 `${catkin_LIBRARIES}` 约定，启用时必须按静态链接顺序
提供后端及 GTSAM 的依赖，关闭时不能污染原来的库列表。

初次验收固定使用官方 [GTSAM 4.2.2](https://github.com/borglab/gtsam/releases/tag/4.2.2) 标签，
并记录解析到的提交、编译器、Eigen/Boost、构建选项和实际加载库摘要。
这是一项兼容性验证目标，不是宣称 Noetic/Humble 已经验证：本机 Noetic 是 Ubuntu 20.04、GCC 9.4，
目前没有检测到 GTSAM；其 Eigen 3.3.7、Boost 1.71 已安装。GTSAM 4.2.2 的
[构建说明](https://github.com/borglab/gtsam/blob/4.2.2/INSTALL.md)列出的测试平台较旧，
所以两端必须实测。4.3 是更新的主线，但兼容当前 Noetic 编译器仍需另行验证。
为避免与现有库混用不同 Eigen，验收构建显式使用系统 Eigen；
[4.2.2 构建脚本](https://github.com/borglab/gtsam/blob/4.2.2/cmake/HandleEigen.cmake)
提供 `GTSAM_USE_SYSTEM_EIGEN`。还要检查 GTSAM 和调用方的编译器、C++ 标准、Eigen 版本及
Eigen 对齐选项、`_GLIBCXX_DEBUG` 等 ABI 设置一致，并记录实际编译指令；可移植性验收关闭
`-march=native`，不以构建机原生 CPU 指令作为其他机器可用的依据。本阶段不修改系统安装目录或 vikit。

`package.xml` 不能直接表达用户通过 CMake 选择的可选依赖。初版将默认构建保持无 GTSAM，
启用流程明确要求先准备 GTSAM；不能把这种手工启用说成 rosdep 已自动解析。
如未来需要可分发的默认启用 ROS 二进制包，应单独解决包依赖声明与发布方式。

## 验收

1. 固定小图：无闭环时恢复原始 odom 链；带确定误差的合成闭环降低残差，首帧先验、旋转
   与平移方向正确。错误 ID、方向、噪声、非有限值、重复因子和数值失败不污染图。
2. 实际 `LivoSystem`：处理返回后提交结果；检查版本递增、陈旧结果拒绝、优化前缀增长和尾帧延续，
   并检查几何修正和旧快照。应用错误不改写原始资产或前端状态。
3. 默认关闭与开启构建均测试 ROS1、ROS2 和普通 SDK；测试独立安装消费、实际链接依赖与节点启动。
4. `Retail_Street.bag` 的固定条件回放验证输入数量和前端/旧产品回归。另复制原始关键帧资产到新
   数据集，通过现有 Builder 和可写的 OnlineSubmapStore 新建索引，再注入明确标记的合成约束，
   检查求解、写入的版本和几何变化；原索引重开后只读，不能在原索引或已结束的 `LivoSystem` 上追加。
   活体 `LivoSystem` 交付测试与此离线副本测试分别进行。该 bag 无标注真值，
   不据此报告真实回环精度、重定位能力或在线 3DGS 效果。
5. 记录求解用时、图大小和 RSS 随节点数增长；如批量求解无法满足后续在线需求，
   另行设计增量或异步方案，不靠未测量的假设宣称实时。
