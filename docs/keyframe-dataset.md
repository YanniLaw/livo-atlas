# 关键帧数据集

P5-01 在共用 `LivoSystem` 中加入关键帧选择和存储。功能默认关闭，Noetic、Humble
和普通 C++ 接口使用同一实现。它提供后续后端与重建所需的数据基础；GTSAM、回环、
全局地图融合和新的 COLMAP/3DGS 导出仍待实现。子地图分组与版本重建见
[子地图说明](submap-dataset.md)。现有 `Frame`、`SubSparseMap` 继续用于视觉跟踪。

## 启用

两端使用同样的配置和默认值：

```yaml
keyframes:
  enabled: false
  translation_threshold: 0.5
  rotation_threshold_deg: 10.0
  time_threshold: 2.0
```

平移单位米、旋转单位度、时间单位秒。首个有效候选选中，之后相对上次选中帧的
三个阈值任一达到便选中；允许零，拒绝负数和非有限值。同一状态时间不重复选中，
时间回退时以该候选重新建立选择基准，ID 继续递增。
LIVO 选择成功的视觉更新；LO/LIO 选择各自成功的激光更新，三种模式都要求非空点云。

Noetic 在一个终端运行 `roscore`，另一个已 source 工作区的终端执行：

```bash
rosparam set /keyframes/enabled true
rosparam set /output/directory /absolute/path/to/new-run
roslaunch livo_atlas mapping_avia.launch rviz:=false
```

Noetic 参数使用斜杠，例如 `/keyframes/time_threshold`。原 launch 仍读取原有两份
配置；也可将上面的组加入自己的 Avia 配置副本，通过现有配置流程加载。

Humble 将 `config/avia.yaml` 复制为自己的配置文件，加入上述组并设 `enabled: true`：

```bash
ros2 launch livo_atlas mapping_avia.launch.py \
  config:=/absolute/path/to/avia-keyframes.yaml \
  use_sim_time:=true reliability:=reliable \
  output_directory:=/absolute/path/to/new-run
```

直接向 Humble 可执行文件传参数时使用点号，如 `-p keyframes.enabled:=true`。
普通 C++ 调用方设置 `SystemOptions::keyframes`，然后正常创建和驱动 `LivoSystem`。
数据集固定保存到 `output.directory/keyframes`；已有该目录时拒绝启动，避免覆盖。
在输出文件创建前检查阈值和已有数据集。

两个基线工具均支持 `--save-keyframes`，退出后将数据集移动到验收目录的 `keyframes/`：

```bash
# Noetic，在工作区根目录且已 source devel/setup.bash。
OMP_THREAD_LIMIT=1 python3 src/livo-atlas/tools/run_baseline.py \
  --workspace "$PWD" --bag Retail_Street.bag --output validation/my-p5-noetic \
  --duration 30 --rate 0.5 --save-keyframes

# Humble，使用已转换的 bag 与实际安装的节点。
OMP_THREAD_LIMIT=1 python3 src/livo-atlas/tools/run_ros2_baseline.py \
  --bag Retail_Street.ros2 --output validation/my-p5-humble \
  --executable install/livo_atlas/lib/livo_atlas/fastlivo_mapping \
  --duration 30 --rate 0.5 --save-keyframes
```

输出目录须为新目录。`--save-products` 仍控制原有 PCD、图像及 COLMAP 产品；
`--save-keyframes` 独立控制本数据集，不能与 Noetic 的 `--legacy-output` 合用。
构建和回放串行进行，详细依赖和构建方式见 [core-build.md](core-build.md) 与
[ros-builds.md](ros-builds.md)。

## 时间、坐标和所有权

所有时间均为秒，以前端已有时间修正后的数值保存：

| 字段 | 含义 |
| --- | --- |
| `timestamp` | 当前前端状态的时刻 |
| `cloud_timestamp` | 产生这份云的 LIO 更新时刻 |
| `image_timestamp` | 可选，有效图像时刻，包含现有偏移及曝光修正 |
| `inverse_exposure_time` | 前端估计的逆曝光量，不能当作硬件快门读数 |

这三个时刻可以不同，不能把它们当作原始 ROS Header 时间或互相替换。
`body` 为 IMU 坐标，`odom` 为前端局部坐标（现有 ROS `camera_init`）。
`T_A_B` 将 B 中的点变换到 A，平移单位米，旋转为行主序 3×3 矩阵：

```text
p_odom = R_odom_body * p_body + t_odom_body
```

`CaptureKeyframe(id, result)` 克隆干净的 `color_image`，将 `world_cloud` 转换到
该快照的 body 坐标并独立复制。强度保留；normal/curvature 不作为额外测量解释。
点云已经过原前端的运动补偿。
图像是前端缩放后的带畸变 BGR8 图像，未叠加跟踪标记；读取后不应再次缩放。
`camera_from_odom` 直接保留对应视觉更新中的相机位姿。
原始相机内参、畸变、缩放因子和两组外参随数据集保存一次。

快照脱离前端的短期图像/点云缓冲区，后续更新不会修改它。消费者按只读约定使用；
结构中的 `cv::Mat` 复制本身仍是浅复制，需要自行修改时先 `clone()`。
本阶段接口要求串行调用，不提供跨线程同步。

## schema 1

```text
keyframes/
├── dataset.yml
├── frames/
│   ├── 0/
│   │   ├── frame.yml
│   │   ├── cloud.pcd
│   │   └── image.png       # 仅视觉帧
│   └── 1/...
└── poses/
    └── 1.yml              # 调用 WritePoses 后才有
```

- `dataset.yml`：整数 `schema_version: 1`、固定 `calibration_id: "0"`，相机的
  model/width/height/scale/fx/fy/cx/cy/distortion，以及 `imu_from_lidar`、`camera_from_lidar`。
- `frame.yml`：id/calibration_id、三个适用的时间、cloud_points、inverse_exposure_time、
  整数 has_image 和 `odom_from_body`；有图像时同时包含 `camera_from_odom`。
- `cloud.pcd`：PCL binary PCD，XYZI 为 float32，几何处于快照 body 坐标。
  `image.png` 为无损 PNG，解码为 BGR8，尺寸与原标定乘 scale 后一致。
- `poses/<version>.yml`：version、frame_count 和 poses 序列，每项包含 id、`map_from_body`。

所有 uint64 ID、计数和版本号使用规范十进制字符串。全部浮点元数据采用 classic
locale 和 `double::max_digits10`（通常 17 位有效数字）字符串，保留 double 精确往返；
图像尺寸和 schema 等控制字段为整数。变换的 rotation 有 9 个字符串，translation 有 3 个。
YAML 使用 OpenCV FileStorage 格式；外部解析器需识别或去掉开头的 `%YAML:1.0` 行。
数据集中资产名固定，未保存机器绝对路径，因此可整体复制到另一台电脑。

## 读取和位姿版本

按[普通 SDK 构建说明](core-build.md)安装后，可以直接检查整个数据集：

```bash
/path/to/install-core/bin/livo_atlas_inspect_keyframes /path/to/keyframes
```

工具实际读取每一帧并用原始位姿重建点云，向标准输出写入 JSON：frames、images、
points、first_timestamp、last_timestamp。成功返回 0，读取失败返回 1，参数数量错误返回 2。
它不写入数据集。构建树中也有同名可执行文件。

仅消费数据时链接 `livo_atlas::io`；只需快照/选择和几何操作可链接
`livo_atlas::keyframes`。完整系统继续链接 `livo_atlas::core`。示例：

```cpp
#include "livo_atlas/io/keyframe_store.h"

void ReadDataset(const std::filesystem::path& directory) {
  livo_atlas::KeyframeStore dataset(directory);
  for (auto id : dataset.ids()) {
    const auto frame = dataset.Read(id);
    const auto cloud_in_odom =
        livo_atlas::TransformKeyframeCloud(frame, frame.odom_from_body);
    // 消费 frame.image、各时间和 cloud_in_odom。
  }
}
```

`KeyframeStore(directory, calibration)` 创建新数据集，要求根目录不存在。
`Append` 要求 ID 从 0 连续递增。`KeyframeStore(directory)` 打开已有数据集，
允许重读和增加优化位姿版本，不能继续追加原始帧。

原始位姿始终来自 `Read(id).odom_from_body`，版本 0 为保留概念，不生成文件，
`ReadPoses(0)`/`WritePoses(0, ...)` 会报错。后端应生成完整的 `KeyframePose` 向量：

```cpp
// poses 中每项为 {id, map_from_body}，须覆盖当前所有已提交帧且 ID 不重复。
dataset.WritePoses(1, poses);
for (const auto& pose : dataset.ReadPoses(1)) {
  const auto frame = dataset.Read(pose.id);
  const auto cloud_in_map =
      livo_atlas::TransformKeyframeCloud(frame, pose.map_from_body);
  // 按同一版本消费 cloud_in_map；原始资产和前端状态不会改变。
}
```

每个版本冻结写入时的完整帧前缀 `frame_count`。原写者后来追加的原始帧不属于旧版本，
应按 `ReadPoses(version)` 返回的 ID 重建，不将当前全部帧强行配入旧版本。
已有版本不可覆盖，新的优化结果使用新版本号。此处提供存储和读取接口，尚无实际优化器。

## 提交与失败语义

一份数据集只允许一个串行写者，不支持多进程并发写入。读取索引在打开时建立，
不会自动刷新。开始复制或打开数据前先停止写入。

`Append` 在 `frames/.pending-ID` 写齐资产和元数据后，通过同目录 rename 提交；
位姿版本也先写临时文件再 rename。接口返回时文件已关闭，不需要另调 Finish。
这不提供 fsync 或突然断电下的事务保证。

打开时忽略 `.pending-` 帧目录，拒绝已提交 ID 空洞或缺失元数据；读取时校验实际资产。
读者拒绝未知 schema、非法数值/变换、错 ID、图像尺寸与类型错误、PCD 数量/XYZI/二进制长度不符。
PCD 在交给 PCL 前检查有界头和实际负载长度，PNG 在解码前检查尺寸与类型。
PCD 只接受本格式使用的精确关键字，拒绝重复或不支持的声明。写入前还要求点云
width/height 均为正，且它们的 uint64 乘积与点数一致，避免提交不能重读的资产。

已有数据集、重复 ID/版本及无效输入不会被覆盖。写入中 I/O 失败会保留之前的已提交帧，
并将该写实例置为失败状态，后续写入拒绝执行；可重新打开读取已提交数据。
程序不会自动删除临时目录或恢复继续追加，请保留失败现场并使用新的运行目录。

跨电脑迁移须同时保存整个数据集、对应源码版本和运行配置；目标电脑重编 SDK 并运行检查工具。
schema 1 的跨 Noetic/Humble 读取和真实回放范围见 [P5-01 验证记录](p5-01-validation.md)。
