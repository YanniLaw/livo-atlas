# 子地图数据集

P5-02 将已选择并保存的关键帧按连续 ID 分组，为后续后端和重建提供稳定子地图标识。
子地图只保存成员关系，图像、标定和 body 点云继续使用[关键帧数据集](keyframe-dataset.md)。
读取时按指定逐帧位姿重建几何，原始数据保持独立。前端体素地图及视觉 `SubSparseMap` 继续承担原职责。

## 启用与运行

两端共用同一配置：

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  max_keyframes: 20
```

两个 enabled 默认都是 false。子地图要求关键帧开启，max_keyframes 必须为正，关闭时也校验。
数量指已选择的关键帧，不能直接当作原始激光扫描数量或固定时间长度。
本阶段仅按数量分组，每组至多 20 帧，组之间不重叠；每组第一帧为 anchor。

Noetic 在另一个终端运行 roscore 后，已 source 工作区的终端执行：

```bash
rosparam set /keyframes/enabled true
rosparam set /submaps/enabled true
rosparam set /submaps/max_keyframes 20
rosparam set /output/directory /absolute/path/to/new-run
roslaunch livo_atlas mapping_avia.launch rviz:=false
```

Humble 将 `config/avia.yaml` 复制为自己的配置文件，加入上述两组配置，仍使用原相机配置：

```bash
ros2 launch livo_atlas mapping_avia.launch.py \
  config:=/absolute/path/to/avia-submaps.yaml \
  use_sim_time:=true reliability:=reliable \
  output_directory:=/absolute/path/to/new-run
```

Humble 直接传参数时使用点号键，例如 `-p submaps.enabled:=true`。
普通 C++ 设置 `SystemOptions.keyframes.enabled=true` 和 `SystemOptions.submaps={true,20}`。
原 config/launch 文件未加入新的默认保存行为。

自动回放工具支持 `--save-submaps`，同时启用关键帧和子地图，并归档完整的 keyframes 目录。
只用 `--save-keyframes` 仍只启用关键帧。工作区根目录的示例：

```bash
# Noetic，先 source devel/setup.bash。
OMP_THREAD_LIMIT=1 python3 src/livo-atlas/tools/run_baseline.py \
  --workspace "$PWD" --bag Retail_Street.bag \
  --output validation/my-submaps-noetic --duration 136 --rate 0.5 --save-submaps

# Humble，先按 ros-builds.md 转换输入并构建。
OMP_THREAD_LIMIT=1 python3 src/livo-atlas/tools/run_ros2_baseline.py \
  --bag Retail_Street.ros2 --output validation/my-submaps-humble \
  --executable install/livo_atlas/lib/livo_atlas/fastlivo_mapping \
  --duration 136 --rate 0.5 --save-submaps
```

所有输出须使用新目录。编译和回放串行，Noetic 的 legacy-output 不支持新数据集保存。
独立 SDK 和两端依赖构建见 [core-build.md](core-build.md)、[ros-builds.md](ros-builds.md)。

## 生命周期和目录格式

`SubmapBuilder` 仅持有成员 ID：连续输入从 0 开始的已提交关键帧 ID，满组返回值快照。
正常 `LivoSystem::Finish()` 在完成原导出器退出后提交最后非空不足组；重复 Finish 不会重复提交。
Finish 后不再输入。正常退出必须显式调用 Finish，异常退出时活动尾组可能尚未提交。
已提交组始终覆盖原始关键帧的一个连续前缀，前缀后的原始帧仍能按 KeyframeStore 读取。

```text
keyframes/
├── dataset.yml                  # 原关键帧 schema 1
├── frames/...
├── poses/...
└── submaps/
    ├── dataset.yml              # 子地图 schema 1、整数 max_keyframes
    ├── 0.yml
    └── 1.yml
```

子地图根元数据只包含 `schema_version: 1` 和 `max_keyframes: 20`。
例如第一组成员文件为：

```yaml
id: "0"
anchor_keyframe_id: "0"
keyframe_ids: ["0", "1", "2"]
```

成员 ID 使用 uint64 规范十进制字符串；这个三帧示例可作为不足组或上限为 3 时的满组。
文件名为 `<id>.yml`，子地图 id 连续，anchor 为成员首帧，成员按 ID 连续且不重复。
不足组只能出现在最后，其后不再追加新组。没有另一份图像或点云，也不覆盖原始位姿。

已有 P5-01 数据无需迁移，可继续使用原 KeyframeStore；只有启用新功能的新运行才创建 submaps。
本阶段不提供重开后追加、从既有帧自动补索引、重分组或多写者功能。

## 坐标和位姿版本

子地图云处于 anchor 的 IMU/body 坐标中，平移单位米。T_A_B 将 B 中的点映射到 A。
`ReadCloud(id, version)` 返回 `SubmapCloud`：id、pose_version、parent_from_submap 和独立 cloud。

| 版本 | 每帧使用的位姿 | 返回的 parent_from_submap |
| --- | --- | --- |
| 0（默认） | 原始 odom_from_body | T_odom_anchor |
| 正整数 | 该版本的 map_from_body | T_map_anchor |

组内每个成员使用 `T_anchor_body_i=inverse(T_parent_anchor)*T_parent_body_i` 重建，
因此逐帧优化既能改变子地图整体位置，也能改变内部相对几何。拼接按成员 ID 顺序进行，不降采样或去重。
cloud 中的点需再用返回的 parent_from_submap 变换一次，才进入对应 odom/map 坐标。
XYZ/强度遵循关键帧契约，附带 normal/curvature 不作为已变换的法向或新测量解释。

正版本沿用 `KeyframeStore::WritePoses/ReadPoses`；没有独立的子地图优化位姿文件。
版本冻结写入时的帧前缀，覆盖本组所有成员才允许读取；覆盖不足时报错，不混用原始位姿补齐。
`ReadCloud(...,0)` 表示使用原始帧位姿，不会调用不支持零版本的 `ReadPoses(0)`。
导出多个组时应对它们选择同一位姿版本。

## 检查与 C++ 消费

普通 SDK 安装后的工具会实际读取并重建所有组：

```bash
/path/to/sdk/bin/livo_atlas_inspect_submaps /path/to/keyframes
/path/to/sdk/bin/livo_atlas_inspect_submaps /path/to/keyframes 1
```

标准输出为 JSON，包含 submaps、frames、points、pose_version。成功返回 0，数据/版本错误返回 1，
参数错误返回 2。只检查数据，不创建优化版本或改写资产。Humble 安装时工具位于包安装前缀的 bin 目录。

链接 `livo_atlas::io` 的普通 C++ 消费者可调用：

```cpp
#include "livo_atlas/io/submap_store.h"

void ReadSubmaps(const std::filesystem::path& directory,
                 std::uint64_t version) {
  livo_atlas::KeyframeStore keyframes(directory);
  livo_atlas::SubmapStore submaps(directory / "submaps", keyframes);
  for (std::size_t id = 0; id < submaps.size(); ++id) {
    const auto membership = submaps.Read(id);
    const auto geometry = submaps.ReadCloud(id, version);
    // 消费成员 ID、anchor 云和对应 parent_from_submap。
  }
}
```

SubmapStore 非拥有地引用传入的 KeyframeStore，后者必须先构造、后析构，并与该索引所属数据集对应。
两个对象均串行使用；索引打开后不自动刷新。停止写入后再复制或重新打开数据。

## 失败保护与跨电脑迁移

无效成员或未提交关键帧在写文件前拒绝，状态和索引不推进。已有目录、已有组或临时文件不会被覆盖。
元数据先写 `.pending-ID.yml`，文件关闭后 rename 提交；I/O 失败后该实例拒绝继续写，之前的组可重开读取。
打开时忽略 pending 文件，检查 schema、ID、anchor、成员形状、连续前缀和关联关键帧。
读取时再核对成员文件，资产缺失/损坏通过真实关键帧读者报告路径；不会自动删除失败现场。
这些保证不包含并发路径替换、多进程事务或断电 fsync 保证。

整体复制 keyframes 目录及运行配置、源码版本到目标电脑，重新编译 SDK 后运行检查工具。
格式不含机器绝对路径。两端与跨环境读取的实际范围见 [P5-02 验证记录](p5-02-validation.md)。
