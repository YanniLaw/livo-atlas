# C3b-02 Geometry Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in this session. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 交付带版本和来源、资源受限的几何地图显示结果及对称 ROS1/ROS2 话题。

**Architecture:** 普通 C++ 构建器从固定成员与位姿构建局部体素片段，使用有限 LRU 和流式采样。
系统所有者维护完整成员索引，单一 worker 读取固定 view；同一 main 将结果送到对称 ROS publisher。

**Tech Stack:** C++17、Eigen、现有 GeometryMapper/KeyframeReadView、catkin、Humble rosidl。

**Spec:** [C3b-02设计](../specs/2026-11-15-p6-02c3b2-geometry-map-design.md)。

## Global Constraints

- 只修改 livo-atlas；保留当前 `framework_completion`，不合并、推送或打 tag。
- Google C++17、直接成员、无 Pimpl；函数定义之间空一行，保留用户指定的既有类型名。
- 单一 main；普通 SDK 无 ROS 依赖；不修改原始估计、TF、资产及其数值。
- 默认关闭；0.2 Hz、最多20000帧、100000显示点、300000缓存点、8388608消息字节；有效范围见设计。
- 同一 v/M/N 与完整成员源；v0沿用 GeometryMapper raw 数学，正版本要求 projected tail。
- 单执行/单待处理；单块 primary≤64；一次读取一个块；错误只停止派生组件。
- 只用 `Retail_Street.bag`，重型构建/回放串行、nice10、-j1、OMP_THREAD_LIMIT=1；不声称长距离验收。

## Review Focus

- 旧片段按容差复用、cache=0重建跨体素边界时，来源必须说明两种结果；Task 2。
- 同一v/M sealing 与新v相同采集时间仍要发布，不能依赖 ROS clock递增；Tasks 3/5。
- 首次进入资源不足和失败时清空派生数组，同时原估计可继续；Tasks 3/4。
- Stop与 worker睡眠、慢消费者和系统析构的交错不能丢唤醒或悬空；Task 3。
- Humble静态算法库的生成RMW模块须从空安装前缀加载，普通SDK不能加载ROS；Tasks 4/6。

---

### Task 1: 普通结果与有界采样

**Files:** Create `include/livo_atlas/mapping/geometry_map_snapshot.h`, `src/mapping/geometry_map_sampling.h`, `src/mapping/geometry_map_sampling.cc`, `tests/mapping/geometry_map_sampling_test.cc`; modify `cmake/core.cmake`, `cmake/tests.cmake`。

**Interfaces:**
- Produces: `GeometryMapStatus`、`GeometryMapOptions`、`GeometryMapPoint`、`GeometryMapSourceInfo`、`GeometryMapSnapshot`，字段及默认值按设计。
- Produces: mapping私有 `GeometryMapCandidate`、`GeometryMapSampler(std::size_t)`、`Add(...)`、`Take()`；`GeometryMapPointLimit(options, submap_count)`，预算不足返回0。
- Consumes: `GeometryVoxelKey`、`GeometryOptions`、`PoseFrame`；不含ROS或文件读取。

- [x] 写测试：固定四词散列和完整键tie、不同输入遍历顺序选出同样的最小K、不同块均有代表、K=1、K≥V、空、非有限值、4096+128S+16P边界和整数溢出。
- [x] 配置普通SDK构建，运行新目标并保留缺接口 RED。实现字段、配置校验和至多K项的最大堆；排序按 `(submap_id,kx,ky,kz)`。
- [x] 新测试和普通全套通过；格式检查后提交 `feat: define bounded geometry map snapshots`。

### Task 2: 私有几何构建器与紧凑缓存

**Files:** Create `include/livo_atlas/mapping/geometry_map_builder.h`, `src/mapping/geometry_map_builder.cc`, `tests/mapping/geometry_map_builder_test.cc`; modify `cmake/core.cmake`, `cmake/tests.cmake`。

**Interfaces:**
- Consumes: Task 1值类型及采样器、`GeometryMapper::CloudReader`。
- Produces: `GeometryMapBuilder(GeometryOptions, GeometryMapOptions, GeometryMapper::CloudReader)`；`GeometryMapSnapshot Build(const std::vector<OnlineSubmap>&, std::shared_ptr<const PoseSnapshot>)`；`GeometryMapBuilderStats GetStats() const`。
- 结果来源按设计记录 mapper私有表示修订和实际 `built_pose_version`；构建器不负责producer/revision及传感器时间。

- [x] 用真实合成PCL云和计数reader覆盖：三块、primary一次/support不融合、冷/热两轮、刚体修正复用、组内修正重建、追加、seal、缓存0/淘汰/单块超缓存。
- [x] 写 v0 非正交raw矩阵与 projected-tail 正版本的数值断言；同固定候选不同缓存/遍历得到同采样。
- [x] 写 `5e-10 m` 容差内跨体素边界：缓存命中旧构建、cache=0且已淘汰时新构建允许不同，但各自 `built_pose_version`、表示修订和源数据可复算。
- [x] 运行目标缺接口 RED；实现私有mapper、仅局部键/质心/强度/实际来源的有限LRU，单块流式处理和有界采样。保留表示修订到构建版本的记录，即使片段被淘汰。
- [x] 新测试和普通全套通过；提交 `feat: build versioned geometry map with bounded cache`。

### Task 3: 所有者调度、资源和配置

**Files:** Create `include/livo_atlas/core/geometry_map_output_options.h`, `include/livo_atlas/core/geometry_map_output.h`, `src/core/geometry_map_output.cc`, `tests/core/geometry_map_output_test.cc`; modify `global_output_options.*`, `livo_config.cc`, `tests/core/global_output_options_test.cc`, CMake。

**Interfaces:**
- Consumes: Task 2构建器、`OnlineSubmapUpdate`、固定 `KeyframeReadView`。
- Produces: `GeometryMapOutputOptions`、`ValidateGeometryMapOutputOptions(...)`；`GeometryMapOutput(LivoSystem&, GeometryMapOutputOptions, GeometryOptions, Consumer)`，其中 `Consumer=std::function<void(const GeometryMapSnapshot&)>`；`Observe(...) noexcept`、`Poll()`、`Finish()`、`GetStats() const`。
- 新会话M=0启动；所有者记录增量形成完整索引，worker固定一份全索引、poses和view；首次超预算不保留新增完整快照。

- [x] 测试真实系统输出：空、增帧、seal同v/M仍有revision、同时间版本更新、完整primary索引、support不重复融合、首帧后构造拒绝、非连续/错误revision。
- [x] 控制慢consumer/read：旧完整结果先交付、随后合并到最新；帧超预算与字节预算清空、无新读云、原系统继续；reader/sink失败不递归重试。
- [x] 覆盖同一v旧值修改、固定read前沿不够、重复Finish、构造或析构异常、2000次idle停止/析构；Stop条件在等待mutex下修改。
- [x] 配置测试：仅地图启用且前端geometry关闭时读取并验证同一 `submaps.geometry.*`，不能启动前端mapper；缺storage、max组>64、map_frame>256、各配额边界和关闭时行为。
- [x] 运行新目标 RED；实现所有者小增量索引、单执行/单待处理、可绕限频的首次非Ready、固定view worker和计数。测试通过后提交 `feat: schedule bounded geometry map output`。

### Task 4: 双 ROS 版本化地图和标准云

**Files:** Create `msg/GeometryMapSource.msg`, `msg/GeometryMapParameters.msg`, `msg/GeometryMap.msg`, `src/ros/geometry_messages.h`, 两端 `geometry_map_publisher.h/.cc` 和实际ROS测试；修改 `cmake/ros1.cmake`, `cmake/ros2.cmake`, `cmake/tests.cmake`, `package.xml`、安装消费者。

**Interfaces:**
- Consumes: `GeometryMapSnapshot`；产生对称 `GeometryMapPublisher(node, map_frame)` 和 `Publish(const GeometryMapSnapshot&) const`。
- `/geometry_map` 版本化完整结果，`/global_map` 标准云；小端FLOAT32 XYZI offsets0/4/8/12、step16；source连续区间及header一致。

- [x] 两端实际订阅测试：v0/v>0帧、所有来源字段、限额采样、Late join、Resource/Error的空云、同时间revision和关闭本节点context跳过转换。
- [x] 边界测试：非法/越界ROS时间、float越界、point/source长度及非连续offset；Noetic/Humble实际序列化和真实发布100000点，验证 `4096+128*S+16*P` 上界。
- [x] 外部安装消费者实际序列化/反序列化含64位版本；Humble静态从空安装前缀加载RMW共享模块，SDK_VERSION=0不生成消息。
- [x] 先运行 RED，再实现共享转换及对称publisher；两端完整测试、安装消费通过后提交 `feat: publish versioned geometry maps on ROS`。

### Task 5: 同一 main 与回放采集

**Files:** Modify `src/app/livo_main.cc`, `tools/run_baseline.py`, `tools/run_ros2_baseline.py`, CLI测试；create `tools/geometry_map_records.py`, `tests/test_geometry_map_records.py`，按需补配置说明。

**Interfaces:**
- Consumes: Tasks 3/4；唯一callback组合 backend、trajectory、geometry，Poll依次同序。
- Produces: `--geometry-map` 自动启用全局/online存储，不启用前端几何、trajectory或回环；`geometry-map.json` 复制精确metadata、来源与XYZI。

- [x] CLI RED：新flag、依赖与冲突、仅地图时前端geometry保持关闭、64位元数据与深拷贝；两端真实主入口测试配置生效和publisher存在。
- [x] ROS clock/生命周期测试：重复/回退非负时钟下历史更新但不补造当前TF；正常及异常停止保持publisher/system生命周期。
- [x] 实现同一main组合、geometry开启时Noetic WallRate、Finish顺序和采集工具；运行普通、双ROS、Python测试通过，提交 `feat: wire geometry map output into shared node`。

### Task 6: 验证、文档及整阶段审阅

**Files:** Create `docs/geometry-map-output.md`, `docs/p6-02c3b2-validation.md`; modify `docs/README.md`, 本设计和计划状态；脚本/证据放包外新validation目录。

- [x] 保存最终源码/输入/checker SHA；串行6 SDK及各4 ROS模式，检查实际SDK/消息安装消费者、依赖边界、空静态安装前缀。
- [x] 先以人工非刚性几何、误版本/错索引/错误rank/超配额及容差复用样例验证独立checker；再两平台各三次完整bag及短段缓存0、紧预算、轨迹同开、错误路径。
- [x] 同平台严格比较旧四数组、odom/TF语义、原始关键帧、成员及57个旧产品；按实际build来源、root v/M/N和rank复算新消息。
- [x] 记录冷/热读云、缓存淘汰、点/字节预算、Observe/Poll/worker、延迟、RSS，按可用环境验证RViz正常显示/空云边界并如实报告限制。
- [ ] 编写迁移文档和验证报告、差异/格式/独立头检查；fresh最强模型整阶段只读审阅，Important/Critical按单次 RED→GREEN 修复和相关全套复验，保留本地分支。

## 执行授权与自查

用户在设计与模型切换通知后要求“继续”，且此前指定当前会话直接实施、自动运行验证；沿用 inline execution。
接口按1→2→3→4→5传递，Task 6验证冻结代码。六任务覆盖设计中的表示、缓存、所有权、协议、配置、生命周期与验收。
