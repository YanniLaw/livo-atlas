# 几何子地图使用说明

P6-02a 在已有在线子地图的主关键帧上构建独立几何。前端匹配地图仍按原方式运行；
几何数据不反馈修改前端状态，也不新增 ROS 话题或 TF。
几何子地图是后续回环配准与 Gaussian 初始化的输入基础，本阶段没有 GTSAM、真实回环或在线 3DGS 训练。

后续的版本化几何显示是独立功能，提供 ROS 话题和有限后台采样，见[几何地图显示说明](geometry-map-output.md)。

## 启用

几何默认关闭。需要同时启用关键帧和在线子地图，例如在现有参数配置中增加：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
  max_keyframes: 20
  geometry:
    enabled: true
    voxel_size: 0.2
    min_points_per_voxel: 6
    max_plane_rms: 0.02
    max_plane_ratio: 0.1
    max_voxels_per_submap: 100000
    max_points_per_keyframe: 1000000
    max_resident_submaps: 3
```

ROS1 在节点所在命名空间读取斜杠形式的参数名，如 `submaps/geometry/enabled`；默认 launch
没有额外命名空间时为 `/submaps/geometry/enabled`。ROS2 使用点分形式，如
`submaps.geometry.enabled`。两端读取相同的普通配置。`max_keyframes` 在启用几何时不超过 64，
驻留子地图数量至少为 2；非法配置在创建输出前拒绝。几何关闭时不会读取其余几何选项。
上表数值是初始配置，不代表对所有数据和平台都已调优。
仓库中的 ROS1 和 ROS2 回放工具也可使用 `--geometry-submaps`，自动启用所需的关键帧、在线子地图
和几何参数；实际启动配置仍需按上面的 YAML 设置。

已有运行的 `output/directory/keyframes/` 中继续保存关键帧与在线索引。
程序不自动保存逐帧几何缓存；重启后由已提交的关键帧、成员和位姿按需重建。
源数据和索引必须在导出期间保持停止且不变。

## 普通 C++ 查询

使用独立 SDK 时，`LivoSystem::GetGeometrySubmap(id)` 返回拥有只读几何数据的快照；
`GetGeometryStats()` 返回内部驻留数量、重建和读取计数以及最近操作计时。
在与系统相同的串行执行者上、处理调用返回后查询；不要在在线通知回调中重入系统。
正常 `Finish()` 后仍可查询；未启用时查询抛 `std::logic_error`，未知 ID 抛 `std::out_of_range`。
查询历史块可能同步读取点云并重建，调用者应计入等待时间。

`GeometryMapper` 可作为独立普通 C++ 组件使用。它绑定一个只读取已提交 body 云的 reader，
`Update` 接收同一份 `PoseSnapshot` 和已提交成员。`LivoSystem` 在普通追加或封存时用
`UpdateDelta` 提交前沿和受影响的子地图成员，追加时随附新帧及其修正位姿。优化提交通过
`Update` 检查已有缓存的相对位姿。调用方必须保证 reader 对同一 ID 返回
同一数据集内不可变的观测；跨数据集相同整数 ID 并不代表相同观测。
`KeyframeStore::ReadBodyCloud(id, max_points)` 不打开或验证图像；它验证点数及 PCD 文件，
并在 PCL 分配前限制单点布局不超过 64 字节。普通 `KeyframeStore::Read` 仍验证完整图像资产。

## 坐标与来源

每个子地图的局部坐标系固定为其第一个主关键帧的 body 系。输出中的 `parent_frame` 为
`odom`（版本 0）或 `map`（正版本），`parent_from_submap` 将局部坐标变换到该父坐标系。
每个来源记录保存实际构建时的 `submap_from_body`；body 云已经转换到 body 系，不能重复施加 LiDAR 外参。

每个主关键帧只向所属子地图贡献一次。支持关键帧保留为邻接引用，本阶段不重复融合。
不同子地图仍可能覆盖同一物理表面，不能将分块导出当作已全局去重的地图。
原云、成员和位姿保留为可重建来源；整体刚性修正只更新外部变换，组内相对姿态变化会令相关块重建。

身份包含子地图 ID、成员修订、位姿版本、位姿覆盖帧数和几何表示修订。
表示修订只在当前 `GeometryMapper` 生命周期内单调分配，不能把重启后的同一数字视为同一个缓存版本。
曾经返回的快照保持其旧数据和旧标签；持有大量历史快照的内存由调用者负责，不计入 mapper 驻留上限。
正位姿版本的尾帧按既有最后优化锚点策略延续，不表示这些尾帧参与了优化。

## 几何表示

每个占用体素保留中心、点数、平均强度、点的空间分布协方差和贡献帧 ID。
满足点数、二维展开程度、平面 RMS 和特征值比例条件时，才提供单位法向。
无效面元仍有体素统计；不要把其法向当成有效测量。
法向符号按最大绝对分量取正固定，并非朝向相机或物体外侧。
协方差描述的是点的分布，单位为 m²；它不是位姿/平面参数的不确定度，也不是可直接训练的 Gaussian 形状参数。
单格只描述一个候选面，薄的平行面或运动物体仍可能混入；当前实现不做动态物体过滤。

## 导出

独立核心安装后，可显式导出所选位姿版本：

```bash
livo_atlas_export_geometry /实际输出目录/keyframes /新的几何导出目录 \
  --pose-version 0 --voxel-size 0.2
```

默认版本为 0。正版本必须已经存在，工具会按现有尾帧规则物化至当前原始帧覆盖范围。
未进入在线索引的原始尾帧在 manifest 中报告，但不伪造子地图归属。输出目录必须不存在，
也不能放在源数据集内部；同一路径只允许一个导出写入者。
导出先写同父目录的 `.pending-*`，文件关闭成功后再重命名；失败时保留待检查的临时目录，
不承诺断电时的 fsync 事务耐久性。

```text
导出目录/
├── manifest.yml
└── submaps/<id>/
    ├── metadata.yml
    ├── voxels.csv
    └── contributors.csv
```

manifest 记录数据集来源说明、算法、配置、标定和位姿覆盖；metadata 记录块身份、成员、
父坐标变换及构建来源。来源路径不是全局数据集标识或内容摘要。
`voxels.csv` 中 `plane_valid=0` 时法向列为零，必须检查该标志；
`contributors.csv` 每行关联一个体素与一个主关键帧 ID。两个 CSV 按键/ID 排序，double 使用足够位数输出。

## 时间与资源

已启用时，每次几何更新输出 `geometry_total_seconds` 及 update、读云、完整重建、快照时长。
total 包含本轮的位姿准备和局部几何更新，计时在诊断日志与外部回调之前结束。
这些阶段存在嵌套，不能直接相加。
资源上限用于防止默认缓存无限增长，历史元数据、输入数据和调用者持有的快照仍可能随运行增长。
不同机器和数据应单独测试，测量与实际验收范围见 [P6-02a 验证记录](p6-02a-validation.md)。

参数及坐标决策的完整依据见[设计](superpowers/specs/2026-11-10-p6-02a-geometry-submaps-design.md)。

[C2a 回环准入](loop-admission.md)使用这里的主成员几何：先在当前视图配准，
再在优化预览视图中分别复核固定测量和预测变换。逐源局部位姿变化时必须重建几何。
