# 离线 COLMAP 研究数据导出

P5-03 将[关键帧数据集](keyframe-dataset.md)的图像、相机位姿和 LiDAR 种子导出到独立目录。
可以选择原始 odom 位姿或已有的优化位姿版本；原数据、前端和[子地图](submap-dataset.md)保持独立。
导出是普通 C++ 功能，位于现有 io 库，默认节点不触发它。

## 构建与调用

依赖和普通 SDK 构建见 [core-build.md](core-build.md)，双 ROS 构建见 [ros-builds.md](ros-builds.md)。
工作区根目录的普通安装示例：

```bash
cmake -S src/livo-atlas -B build-sdk \
  -DLIVO_ATLAS_ROS_VERSION=0 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install-sdk"
cmake --build build-sdk -- -j1
cmake --install build-sdk

install-sdk/bin/livo_atlas_export_colmap \
  /absolute/path/to/run/keyframes /absolute/path/to/new-export
```

输入应已正常停止写入，输出目录必须不存在。现有目录即使为空也拒绝覆盖。
导出工具与两个数据集检查工具一起安装进 bin。Humble colcon 的安装路径为
`install/livo_atlas/bin/livo_atlas_export_colmap`，普通 SDK 为自己的安装前缀/bin。
两端使用同一工具源文件和普通接口；运行它不需要启动 ROS、COLMAP 或 CUDA。

```text
livo_atlas_export_colmap DATASET_DIRECTORY OUTPUT_DIRECTORY [POSE_VERSION] [MAX_POINTS_PER_IMAGE]
```

默认版本为0，每图最多2000个种子。版本是规范非负十进制整数，点数上限为1到UINT32_MAX-1；不接受符号或前导零。
例如，已有位姿版本1时另导出一个目录：

```bash
install-sdk/bin/livo_atlas_export_colmap \
  /absolute/path/to/run/keyframes /absolute/path/to/export-version-1 1 1500
```

正版本由 KeyframeStore::WritePoses 写入；前端不会自行生成后端优化版本。
它只导出该版本冻结帧前缀内的视觉关键帧，之后追加的帧不混入；位姿记录顺序可以不同。
成功返回0并输出 JSON（images、points、pose_version）。参数错误返回2，数据/文件错误返回1。

## 输出和关联

```text
new-export/
├── images/0.png
├── images/2.png
├── sparse/0/cameras.txt
├── sparse/0/images.txt
├── sparse/0/points3D.txt
├── frames.csv
└── export.yml
```

文件名使用源关键帧ID；上面的缺号可来自无图像的 LO/LIO 帧，COLMAP 图像ID仍从1连续编号。
frames.csv 保存 keyframe_id、image_id、image_name、timestamp、cloud_timestamp、image_timestamp、inverse_exposure_time。
浮点数保留 double 往返精度；时间来自传感器快照，不能用文件时间或文件名字典序代替。
逆曝光为前端估计原值，没有当作实际快门时间或用于修改图像亮度。

export.yml 是完成标记，记录 schema1、位姿版本、odom/map父坐标、图像/点数、点数上限、
原始相机配置和外参，以及 pixel_convention=colmap_corner、rectification=centered_pinhole。
旋转表示另声明rotation_projection=nearest_so3。
它在全部图像、模型和CSV关闭成功后，经临时文件rename提交。没有完成标记的目录应视为未完成，
先检查错误原因，再自行选择新的输出目录；工具保留失败现场，重试不覆盖。

## 相机坐标与像素

T_A_B 将B中的点变换到A。点的单位为米，源快照的body坐标为IMU/body。
版本0的父坐标为odom，正版本为map；不声明它们一定是ENU或NED。

历史快照允许微小的旋转舍入误差。导出内部先把已通过校验的相机、原body和所选body旋转
投影到最近SO(3)，平移保持，再用同一刚体表示变换相机与点；不改写原始矩阵。
以下T表示该导出刚体表示。先恢复 T_C_B，再使用所选版本的body位姿：

```text
T_C_B = T_C_odom(raw) * T_odom_B(raw)
T_C_parent = T_C_B * inverse(T_parent_B(version))
p_parent = T_parent_B(version) * p_body
```

images.txt 中是世界/父坐标到相机的 T_C_parent，四元数为Hamilton的w,x,y,z。
相机中心为 -R.transpose()*t；相机X向右、Y向下、Z向前。不要直接把其中的t当成相机位置。
这样每帧优化会同时修正相机和本帧点云，保留原始快照中实际使用的相机/机体关系。

源内参先遵循已有数据集契约缩放一次，图像尺寸为 int(width*scale)、int(height*scale)。
输出采用居中的虚拟针孔相机，焦距保持缩放后的fx/fy，像素约定明确区分：

| 项目 | 主点/像素中心约定 |
| --- | --- |
| OpenCV 整流目标K | 主点((width-1)/2,(height-1)/2)，左上像素中心为(0,0) |
| COLMAP cameras.txt | 主点(width/2,height/2)，左上像素中心为(0.5,0.5) |
| images.txt 观测 | COLMAP角点坐标；floor(u),floor(v)对应图像数组索引 |

Pinhole使用全部k1/k2/p1/p2，EquidistantCamera使用OpenCV fisheye四系数，均输出无畸变PINHOLE。
不再缩放或裁剪图像，主点重映射会改变部分边缘可见范围。边缘无法取得完整源双线性邻域时，不用于点着色。
源图像为BGR8，PNG通过OpenCV正常编码；点颜色按RGB写出。

官方3DGS相机读取路径只使用焦距和尺寸计算FOV，居中相机可对应它的对称投影。
像素约定参考[COLMAP标定转换](https://colmap.github.io/faq.html#using-calibration-from-opencv-kalibr-or-other-tools)，
读取行为参考[官方3DGS实现](https://github.com/graphdeco-inria/gaussian-splatting/blob/54c035f7834b564019656c3e3fcc3646292f727d/scene/dataset_readers.py)。

## 初始化点与研究边界

种子来自各视觉帧自己的body云。仅保留相机前方、图内且整流有效的点，同一像素取最近深度，
按像素行优先顺序确定性采样到每图上限以内。不进行跨帧融合，也不复制全部原始点云。
points3D.txt 中每点有一个真实投影关联，images.txt 保存反向关联；ERROR=0表示所构造的投影关系，
不代表BA残差或测量精度。这些是LiDAR初始化种子，不是多视角三角化结果。

没有可见点的单张图像可保留空观测行；全部图像都无种子时导出失败。
单张图像或很小的相机运动仍可用于格式检查，训练需要用户选择合适的覆盖和视角。
本阶段验证外部数据消费，不运行3DGS训练、重定位、回环或GTSAM优化。

## C++ 消费与跨电脑迁移

```cpp
#include "livo_atlas/io/colmap_exporter.h"

const auto summary = livo_atlas::ExportColmap(
    "/data/keyframes", "/data/new-export", {0, 2000});
```

包外程序通过 find_package(livo_atlas CONFIG REQUIRED)，链接 livo_atlas::io 即可。
可以移动安装前缀再消费；跨工具链/平台应重新编译SDK，步骤见构建文档。
停止写入后整体复制导出目录，图像、模型和CSV的相对关联仍有效，不依赖原机器绝对路径。

已有文件、源/目标重叠和普通符号链接别名会在写入前拒绝。
串行使用，不提供并发路径替换、多写者、自动恢复或断电fsync保证。
实测环境、外部读取器固定版本和源资产保护证据见 [P5-03验证记录](p5-03-validation.md)。
