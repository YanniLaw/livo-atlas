# P5-03：离线 COLMAP 研究数据导出

## 目标与授权

用户已认可先完成研究数据导出、再推进 GTSAM 后端和实际 3DGS 研究，并要求文档先行、当前会话直接实施、主动验证。
沿用此前由执行者安排工程细节的授权；本阶段具体格式和实现选择由执行者负责，先记录，再实现。
本阶段在 P5-02 的关键帧/子地图及位姿版本基础上提供普通 C++ 导出接口和安装后的命令行工具。
交付可被 COLMAP 和官方 3DGS 数据读取代码消费的数据，不宣称已完成训练、回环或后端。

基线为干净 refactor2 的 55646e15233554d756f826b8017aa9aeb6df60e1。
只修改 livo-atlas；Google C++17，保留六个指定类型名、同一 ROS main 和现有前端算法。
保留 master、现有标签及其他引用，不合并/推送。正常工作区原地实施，旧阶段证据及原 Log 只读，vikit 缺失保持。
新证据只写 validation/P5-03-20261030 与容器 /tmp/livo_atlas_p5_03；nice/-j1、OMP_THREAD_LIMIT=1，构建/回放串行，无 RViz。

## 方案和范围

采用普通 C++ 离线导出器，读取已停止写入的数据集，归入现有 livo_atlas::io；CLI 仅做参数解析和摘要。
它复用 KeyframeStore、位姿版本和私有元数据帮助函数，不增加物理库、运行时依赖或线程。
另两种途径分别是在线 ROS 回调中输出，以及另建 Python 数据读取链；它们需要处理运行调度或再维护一套输入格式，留待实际需要。
默认节点/参数/launch、数据集 schema 1、估计和已有导出不改变。
COLMAP/3DGS 的外部读取代码仅作为包外验收工具，不成为链接或构建依赖；CUDA/训练不属于本阶段。

## 普通接口与文件职责

新增 include/livo_atlas/io/colmap_exporter.h、src/io/colmap_exporter.cc：

```cpp
struct ColmapExportOptions {
  std::uint64_t pose_version = 0;
  std::uint32_t max_points_per_image = 2000;
};
struct ColmapExportSummary {
  std::uint64_t images = 0;
  std::uint64_t points = 0;
};
ColmapExportSummary ExportColmap(const std::filesystem::path& dataset,
                                const std::filesystem::path& output,
                                const ColmapExportOptions& options = {});
```

接口只接收路径和普通配置。相机整流、投影、成员选择和文件写入在此组件内闭合；帮助函数保持私有。
若实现文件过长，可将相机整流/可见点投影移入同目录私有文件，不创建通用框架。
新增 tools/export_colmap.cc，安装 livo_atlas_export_colmap，Linux/macOS 使用现有工具的相对 RPATH 方式。
测试归 tests/io/colmap_exporter_test.cc；安装消费者实际调用公开接口。

## 输入和位姿语义

输入必须是有效 KeyframeStore。版本 0 使用原始 odom_from_body，正版本使用 ReadPoses(version)。
正版本只导出其冻结前缀中的帧，以 ID 查找位姿，不依赖文件中记录顺序，也不混入之后追加的原始帧。
即使没有可导出的图像，显式指定的正版本仍须先验证存在。只导出带图像的关键帧，缺图的 LO/LIO 帧跳过；没有视觉帧时报错。
所有被选中前缀的原始资产在创建输出前预检；写出时仍按帧读取，不把整个序列图像和云留在内存。

图像实测位姿 camera_from_odom 是原始快照的一部分，应保持其相机坐标约定和相机/机体关联。
历史记录允许标定/四元数舍入导致的微小非正交误差。仅在导出内部，将已通过原校验的每个相机/原body/选定body旋转
通过SVD/polar投影到最近SO(3)，平移保持；以下T均使用这一刚体表示，再一致地组合相机和点。
源记录不变，导出清单声明rotation_projection=nearest_so3，不宣称输出矩阵与原近似矩阵逐元素相同。
T_A_B 将 B 映射到 A；对每帧先恢复 T_C_B = T_C_O(raw) * T_O_B(raw)，再计算：

```text
T_C_P(version) = T_C_B * inverse(T_P_B(version))
p_P = T_P_B(version) * p_B
```

P 在原始版本为 odom，在正版本为 map。版本 0 与快照原始相机位姿一致；优化时相机和该帧云接受同一机体修正。
原始外参仍写入导出清单供追溯；不将配置舍入后的另一份相机位姿替换已捕获的图像位姿。
COLMAP images.txt 保存 T_C_P，四元数为 Hamilton w,x,y,z；归一化后输出，相机中心为 -R.transpose()*t。
轨迹更新不反馈 EKF，原始相机、云、标定和位姿版本均不改写。

## 图像、标定与颜色

统一输出去畸变 PINHOLE 图像，RGB PNG 通过 OpenCV BGR8 编码，不手工交换图像通道。
输出尺寸与源快照的已缩放尺寸相同：int(width*scale)、int(height*scale)。源 fx/fy/cx/cy 按既有契约乘 scale 一次。
输出 fx/fy 保持缩放后的值，主点居中：OpenCV 整流目标为((width-1)/2,(height-1)/2)，
COLMAP 的角点坐标约定下写为(width/2,height/2)，两者相差半个像素。观测坐标使用 COLMAP 约定，取 floor 后对应图像数组索引。
官方3DGS相机路径只从焦距和图像尺寸计算FOV，不消费主点偏移；居中的虚拟相机使该路径的射线与输出图像一致。
不裁剪，不再次缩放；主点重映射可能改变边缘可见范围，源图像/标定保持。Pinhole 使用 OpenCV 常规四系数，EquidistantCamera 使用 fisheye 四系数。
所有系数都参与整流，不沿用前端历史 k1 阈值分支；这项离线图像处理不改变前端。
预计算 float 映射，以缩放后的源 K、居中的 OpenCV 目标 K 和单位整流旋转 remap；无法从原图取得完整双线性邻域的像素视为无效，不用于点云着色。
原 BGR 图像保持只读；点的 RGB 由去畸变图像中 BGR 像素按 2,1,0 取出。
逆曝光估计仅原值记录，不假定它等于物理快门时间，不自动修改图像亮度。

## 点云与观测

只将视觉帧自身的 body 云用于初始化点，不做跨帧融合或虚构 SfM 匹配。
用所导出的相机位姿投影本帧变换后的世界点；仅保留 z>0、有限且落在有效像素内的点。
像素使用 floor(u),floor(v)，同一像素保留最小相机 z，深度相等时保留输入较早者。
按图像行优先遍历可见像素，以 ceil(visible/max_points_per_image) 的步长确定性采样；每图不超过上限。
上限必须为正且小于 UINT32_MAX，默认 2000。不含前向投影种子的整帧仍保留图像和空观测行；全部帧无种子时导出失败。
非法全局变换/非有限结果报错；出视野或无法表示为有效投影的点跳过，转换为整数前先检查有限和范围。

每个保留种子独立编号，世界坐标为 double，不复制原点的 normal/curvature 语义。
points3D.txt 中写 RGB、重投影 ERROR=0，以及一个真实投影关联 (IMAGE_ID,POINT2D_IDX)。
此误差表示构造的单视角投影关系，不代表 BA 残差或定位精度；这些点是 LiDAR 初始化种子，不是多视角三角化成果。
images.txt 第二行写同一组 x,y,POINT3D_ID，点索引从0开始；两个方向必须严格对应。
图像 ID 从1连续编号，不复用可能超过32位的关键帧 ID；点 ID 从1连续，保护 COLMAP 的无效 ID 边界与计数溢出。
图像文件名为原始关键帧 ID 的规范十进制字符串加 .png；关联表保留原 ID 与时间，消费者不可据文件名字典序推断采集时间。

## 输出布局和追溯

```text
output/
├── images/<keyframe_id>.png
├── sparse/0/cameras.txt       # 一台 PINHOLE 相机，ID=1
├── sparse/0/images.txt        # 图像/位姿及关联投影
├── sparse/0/points3D.txt      # 带颜色的 LiDAR 种子及单图关联
├── frames.csv                # keyframe_id,image_id,image_name,timestamp,
│                             # cloud_timestamp,image_timestamp,inverse_exposure_time
└── export.yml                # 完成标记及来源契约
```

浮点输出用 classic locale 和 max_digits10。CSV 文件名只有数字和 .png，数值字段保持可无歧义读取。
export.yml 使用独立 schema 1；pose_version/images/points/max_points_per_image 为规范十进制字符串，
parent_frame 为 odom/map，images_undistorted=1，camera_model=PINHOLE，pixel_convention=colmap_corner，
rectification=centered_pinhole，rotation_projection=nearest_so3，source_camera 含原相机全部配置，
imu_from_lidar/camera_from_lidar 保留原始外参；数字/变换复用现有私有元数据格式。
清单不记录机器绝对路径。它只在全部图像、模型、关联文件关闭成功后写临时文件再 rename 为 export.yml。
没有 PLY 或深度文件的重复存储；官方 3DGS 读取器可从 points3D.txt 生成自己的 PLY。

## 失败与路径保护

输出必须为新目录，不覆盖文件或已有目录（包括空目录和符号链接）。
拒绝输出在源数据集内部，以及其他会重叠输入数据的路径；现有父目录不因此获得覆盖授权。
选项、位姿版本、视觉帧/选中资产预检和路径关系验证发生在创建输出前。
开始写后发生 I/O 故障则抛带路径的异常，保留失败现场，没有完成清单；重复调用拒绝覆盖。
不自动删除不完整目录，不提供恢复、并发写者、并发路径替换或断电 fsync 承诺。
正常停止写入后整体复制 output，所有相对关联保持可用。

## CLI 与兼容

```text
livo_atlas_export_colmap DATASET_DIRECTORY OUTPUT_DIRECTORY [POSE_VERSION] [MAX_POINTS_PER_IMAGE]
```

默认版本0、每图2000点。参数必须是范围内的规范非负十进制字符串，点上限为正；参数错误返回2。
数据/路径/写入错误返回1，成功返回0且标准输出 JSON：images、points、pose_version。
Noetic/Humble/普通 SDK 均构建和安装；只链接 io，不增加订阅/参数/main 分支。
默认前端不会调用导出器，原有关键帧和子地图检查工具保持。

## 验收

1. 冻结 P5-02 的22个普通测试二进制在新 TMPDIR/XML 下运行205项基线，旧证据只读。
2. RED→GREEN 字面值测试：非单位外参、原始与乱序优化位姿、旧版本帧前缀、非视觉帧、颜色与时间关联、
   Pinhole/fisheye 及 k1=0/k2非0、缩放一次、近远点遮挡、视野/深度/无效边缘和上限、源/已有目标保护、失败无完成标记。
3. 普通全套/公开头、Noetic/Humble全套；shared/static/无OMP SDK 及 ament 迁移后实际调用导出接口和 CLI。
4. 在冻结 P5-02 两端短/完整数据上分别导出原始版本；完整副本另写倒序非刚性修正版本。
   官方 COLMAP/3DGS 原始读取实现实际解析相机、图像、点及双向关联，检查所有图像尺寸/颜色、重投影、相机中心和时间对应。
   所有原数据资产 SHA 不变；优化改变相机/几何，图像与来源关联稳定。外部读取器版本/SHA/许可记入新证据。
5. 最终两端官方 bag 回归与冻结 P5-02 配对，原数组/发布及原始关键帧资产保持；短段旧产品逐字节一致。
6. 跨电脑说明、完整历史 bundle/空 bare 全引用恢复；一轮独立评审，重要问题一次 RED→GREEN 修复，全套相关验证。

主点/像素约定修订依据：[COLMAP与OpenCV标定转换](https://colmap.github.io/faq.html#using-calibration-from-opencv-kalibr-or-other-tools)、
[3DGS相机读取](https://github.com/graphdeco-inria/gaussian-splatting/blob/54c035f7834b564019656c3e3fcc3646292f727d/scene/dataset_readers.py)。

参考：[COLMAP格式](https://colmap.github.io/format.html)、
[3DGS相机/数据读取](https://github.com/graphdeco-inria/gaussian-splatting/blob/main/scene/dataset_readers.py)、
[3DGS COLMAP解析](https://github.com/graphdeco-inria/gaussian-splatting/blob/main/scene/colmap_loader.py)。
