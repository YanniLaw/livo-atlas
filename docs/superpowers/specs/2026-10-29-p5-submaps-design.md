# P5-02：子地图分组、持久索引与位姿版本重建

## 目的、范围与授权

在 P5-01 的独立关键帧数据集上建立稳定子地图标识、在线分组和持久索引，并能按原始或
指定优化位姿版本重建子地图几何，供后续后端和重建导出消费。
沿用用户已授权的本会话直接实施、本机主动验证、目录由执行者安排及先文档后代码流程。
本次“继续”按既定 P5 路线落实为此阶段；下面的具体工程选择由执行者负责。

基线为干净 `refactor2` 的 `1d52a866600d9d4118123badf0bbedeca4fa8828`。
仅修改 livo-atlas，Google C++17，保留 VIOManager/Frame/Feature/Pose6D/SubSparseMap/VOXEL_LOCATION。
普通核心、同一个 ROS main 和对称 ROS1/ROS2 接入保持；不改变估计、同步、着色或原有导出公式。
现有 master/tag/其他引用保留，不合并或推送。vikit 当前缺失，保持缺失。
不引入 GTSAM、回环、全局体素融合、3DGS 训练、后台线程、数据库或新第三方依赖。

## 方案选择

1. **采用：子地图保存成员关键帧 ID，几何按指定版本重建。** 原图像、标定和 body 云只存一份，
   成员关系不随优化改变，每帧可独立修正；代价是重建时需读取原帧。
2. 预先合并成一份刚性点云：读取方便，但丢失成员对应关系后难以应用每帧独立修正，
   若同时保存原帧又增加重复资产和更新责任，不符合本阶段目标。

先按连续关键帧数量分组，默认每组 20 帧；这是执行者选择的可复现起点。
不提前加入距离/旋转/空间覆盖多策略。20 帧为完整组，正常 Finish 提交最后的非空不足组。
组之间不重叠，每个已分组关键帧只出现一次；现有 Frame/SubSparseMap 和前端局部地图保持原职责。

## 普通数据和流式分组

`mapping/submap.h/.cc` 归现有 `livo_atlas::keyframes`，不新增物理库：

- `SubmapOptions { bool enabled=false; int max_keyframes=20; }`，最大帧数必须为正，关闭时也校验。
- `Submap { uint64_t id=0; uint64_t anchor_keyframe_id=0; vector<uint64_t> keyframe_ids; }`。
  成员非空、按从小到大连续排列，anchor 必须为第一帧。子地图 id 从 0 连续递增。
- `ValidateSubmapOptions(const SubmapOptions&)`、`ValidateSubmap(const Submap&)`。
- `SubmapBuilder(int max_keyframes)`，`AddKeyframe(uint64_t)->optional<Submap>`，
  `Finish()->optional<Submap>`。输入帧 id 从 0 连续，重复/跳号拒绝且不改变状态。
  满组时交付值快照，Finish 交付非空尾组；重复 Finish 为空，Finish 后 Add 拒绝。

Builder 只保存当前成员 ID，不复制或持有图像、点云、前端状态，也不将分组结果反馈给估计器。

## 存储、兼容和生命周期

新增 `io/submap_store.h/.cc`，归现有 `livo_atlas::io`：

- `SubmapStore(directory, const KeyframeStore&, int max_keyframes)` 创建新目录，必须不存在。
- `SubmapStore(directory, const KeyframeStore&)` 打开既有索引，原成员只读。
- `Append(const Submap&)`、`Read(uint64_t)->Submap`、`size()->size_t`。
- `ReadCloud(uint64_t id, uint64_t pose_version=0)->SubmapCloud`。
- `SubmapCloud { uint64_t id=0; uint64_t pose_version=0; RigidTransform parent_from_submap;
  PointCloud cloud; }`，位于 io 的公开头。

KeyframeStore 为非拥有引用，必须先构造且后析构。串行单写者，打开时建立索引，不自动刷新。
重开不追加成员；也不自动从已有原帧生成、恢复或重分组。这些工作有实际需求后单独设计。

```text
keyframes/                       # P5-01 schema 1 保持
├── dataset.yml
├── frames/...
├── poses/...
└── submaps/                      # 新增，可选，独立 schema 1
    ├── dataset.yml               # schema_version:1, max_keyframes:20
    ├── 0.yml                     # id, anchor_keyframe_id, keyframe_ids
    └── 1.yml
```

整数 schema/max_keyframes 使用 YAML 整数，所有 uint64 ID 使用规范十进制字符串。
子地图文件名固定为规范十进制 ID + `.yml`；`.pending-` 文件忽略，其他未知名称拒绝。
每次 Append 要求新子地图 id 连续、成员紧接上组且已在关联 KeyframeStore 提交、数量不超过上限。
不足组只能是最后一组；其后不再允许追加。创建和写入前验证输入，错误带目录/帧或子地图 ID 上下文。
写入临时文件再 rename，拒绝覆盖已有文件；I/O 失败后该写实例拒绝继续写，已有组可重开读取。
打开时检查未知 schema、ID 空洞、错误 anchor/成员形状、成员不存在或重复覆盖。
Read 重新读该组元数据并核对已建立的成员索引；不默默接受已提交成员被改写。
不提供多进程竞争、路径并发替换、fsync 或断电事务保证，也不自动删除失败现场。

既有 P5-01 数据集不需要迁移：无 submaps 仍可按原接口读取；启用新功能只为新运行创建索引。
旧 schema、旧关键帧和位姿版本格式不变，新增文件不重复保存图像或云。

## 坐标与版本几何

子地图坐标是其 anchor 关键帧的 IMU/body 坐标。单位米，T_A_B 将 B 映射到 A。
版本 0 从各帧 `odom_from_body` 取位姿，返回 `parent_from_submap=T_odom_anchor`。
正版本调用现有 `ReadPoses(version)`，以 ID 查找 `map_from_body`，不假设文件中的姿态顺序；
返回 `parent_from_submap=T_map_anchor`。

对每个成员，在 double 中组合 `T_anchor_body_i=inverse(T_parent_anchor)*T_parent_body_i`，
再一次性变换该帧 body 云，并按成员 ID 顺序拼接到返回 cloud。原资产、标定和前端状态不修改。
不做降采样/去重，不合并或声称转换 normal/curvature，保留 P5-01 的 XYZ/强度契约。
在拼接前检查总点数能由 PCL width 表示，不用截断或溢出伪装成功。

沿用优化版本冻结帧前缀的规则：只要选定版本覆盖本组所有成员，便可重建该组；
不覆盖时在读取成员云前报错，不将新帧混入旧版本。没有另造一套子地图优化位姿格式。
锚点的优化位姿负责子地图整体定位，每帧优化位姿可同时改变组内相对几何。

## 共用系统与参数

`SystemOptions.submaps`，共用配置键 `submaps.enabled=false`、`submaps.max_keyframes=20`。
启用子地图要求 `keyframes.enabled=true`，否则在全部 I/O 前拒绝；参数类型仍由现有两端 reader 负责。
Noetic 斜杠键，Humble 点号键；不增加 ROS 私有估计流程或入口。

LivoSystem 按 options 校验→原关键帧 Store→Builder/SubmapStore→原 Exporter 构造。
选中关键帧只捕获一次，原帧 Append 成功后将 ID 交 Builder，满组时 Append 元数据。
错误向上传递并停止该次运行，不吞掉保存失败。关闭子地图时不执行分组或创建 submaps 目录。
Finish 先完成原 Exporter 的既有 Finish，再提交最后不足组；正常重复调用不产生重复文件。
正常退出须显式 Finish，不宣称异常终止时活动尾组也已提交。

新增 `tools/inspect_submaps.cc` / `livo_atlas_inspect_submaps DATASET_DIRECTORY [POSE_VERSION]`，
实际打开并重建全部组，输出 submaps、frames、points、pose_version JSON。无版本参数用原始位姿。
两个回放工具的 `--save-submaps` 同时启用关键帧和子地图；数据随整个 keyframes 目录归档。
原 `--save-keyframes` 仍只保存关键帧，默认节点接口和 config/launch 文件保持。

## 共用元数据与前轮延期项

两个 Store 实际复用的数值/ID/变换 YAML 读写抽到私有 `src/io/dataset_metadata.h/.cc`，
不安装，不放到 common 或增加通用抽象。KeyframeStore 的 schema、精度及错误类型保持。
同时补齐前轮已知的输入异常路径上下文，测试仍保证验证发生在写入前。
PCD 预检精确关键字集合用无动态析构的常量容器，保留有界内存回归及拒绝前缀/重复字段的行为。

## 验证与收尾

先用冻结 P5-01 的普通二进制在新 TMPDIR/XML 路径验证 180 项基线。
新行为 RED→GREEN：分组/尾组/非法 ID、磁盘重开/损坏/已有文件保护、非平凡旋转与非一致位姿修正的字面值、
乱序优化版本、旧版本覆盖不足、输入错误路径、三模式系统开关及重复 Finish。
shared/static/无 OpenMP 普通 SDK 和 Humble ament 安装移动后消费，并独立编译所有普通公开头。
Noetic/Humble 单元、真实 ROS/DDS、启动与 Python 检查；官方 bag 两端各 3 秒/136 秒，启用子地图。
同环境与冻结 P5-01 的四数组/发布精确一致，短段各 57 旧产品逐字节一致；全部关键帧与组实际 C++ 重读，
默认 20 帧时完整 172 帧应为 9 组，最后 12 帧。另对副本写模拟修正验证几何，不改冻结数据。

新证据 `validation/P5-02-20261029`、容器 `/tmp/livo_atlas_p5_02`，旧证据/原 Log 只读。
nice/-j1，构建与 bag 串行，无 RViz。一次 fresh 整阶段评审，重要问题一次验证修复；
结束保存文档、完整历史 bundle 和空 bare 仓库全部引用恢复检查，保留用户分支。
