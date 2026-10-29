# P5-03 离线 COLMAP 导出实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将已有关键帧按选定位姿版本离线导出为可被 COLMAP/官方 3DGS 读取的相机、图像和带颜色种子点。

**Architecture:** 相机整流/投影放 src/io 的私有 ColmapCamera；公开 ExportColmap 负责只读输入、模型写入和完成标记，
两者归现有 io 库。CLI 只调用公开接口，默认前端不触发导出。已有两个数据集格式及数学流程保持。

**Tech Stack:** Google C++17、CMake3.16、现有 Eigen/OpenCV/PCL、Noetic/Humble；外部原始读取器仅包外验收。

**Spec:** docs/superpowers/specs/2026-10-30-p5-colmap-export-design.md

## Global Constraints

- 基线55646e15233554d756f826b8017aa9aeb6df60e1，当前授权的 refactor2 原地实施，仅 livo-atlas。
- Google C++17；六个指定类型名、同一 ROS main、前端估计/同步/着色和已有导出保持；无新物理库/依赖/线程。
- 原 keyframes/submaps schema1 和资产保持；源数据停止写入后只读，输出新目录；无后端/回环/CUDA/训练。
- 新证据 validation/P5-03-20261030、容器/tmp/livo_atlas_p5_03；旧证据/原Log只读、vikit缺失保持。
- nice/-j1、OMP_THREAD_LIMIT=1，编译与回放串行，无RViz；不合并/推送/改master、标签和其他引用。
- 输出已缩放的去畸变 PINHOLE 图像、T_C_parent 位姿、真实单视角投影关联；max_points_per_image默认2000。
- 输出完成以最后 rename 的 export.yml 为准；已有输出不覆盖、失败现场不自动删除。

## Review Focus

1. 相机快照与机体位姿的关联在原始/优化版本间保持，非刚性逐帧修正、乱序记录及旧版本冻结前缀应一致。
2. 非零高阶畸变而k1为0、fisheye中心、非整数缩放尺寸及无效整流边缘，输出图像必须与PINHOLE内参和颜色匹配。
3. LiDAR种子的前向深度/像素遮挡/采样与双向track索引是否一致；极端有限值不得引发非有限输出或整数转换溢出。
4. 空/无图/无可见点、缺失/损坏资产及不存在版本是否明确失败，且不把不完整输出伪装为可用数据。
5. 现有目录、符号链接、规范化后的源/目标重叠和真实I/O失败是否保护输入和用户文件，移动安装后实际读取是否成立。

### Task 1: 私有相机整流和点投影

**Files:** 新 src/io/colmap_camera.h/.cc、tests/io/colmap_camera_test.cc；修改 cmake/core.cmake/tests.cmake。

**Interfaces:** namespace livo_atlas 的私有 ColmapSample {Eigen::Vector3d position; Eigen::Vector2d pixel; cv::Vec3b rgb;};
ColmapView {RigidTransform camera_from_parent; cv::Mat image; vector<ColmapSample> samples;};
ColmapCamera(const CameraConfig&)、pinhole()->const CameraConfig&（已缩放，scale1，零畸变）、
MakeView(const Keyframe&, const RigidTransform& parent_from_body, uint32_t max_points)->ColmapView。
私有头不安装，测试只给该目标增加 src include 路径；不导出通用相机基类。

- [ ] Step1：写相机/几何 RED。原始相机配置12×8、scale0.5、fx8/fy10/cx6/cy4，输出6×4、4/5/3/2。
  原body Rz90,t=(1,2,3)，相机body Rz90,t=(0,0,1)，因此快照相机world为I,t=(-1,-2,-2)。
  body点(.5,0,3)导出world=(1,2.5,6)，pixel=(3,2.625)，图像BGR(x,y)=(10+x,20+y,30+x+y)，种子RGB=(35,22,13)。
  优化body为I,t=(10,0,0)，新相机world应Rz90,t=(0,-10,1)，点为(10.5,0,3)，投影与颜色保持。
  新头不存在时构建应失败；记录 camera-red.log。
- [ ] Step2：添加图像/采样 RED：Pinhole k1=0而k2/p1/p2非0与独立OpenCV参考图像一致；fisheye四系数及光轴有限；
  11×9按0.5缩放为5×4；同像素近点遮挡远点、深度相等取前者、负深度/出界/整流边缘不产生种子；上限1/2确定且不超限。
  原图/原云不修改，返回图像独立；零或保留的最大uint32上限、错图像形状/类型、无图帧、不合法/溢出的变换拒绝。
- [ ] Step3：实现私有相机，使用double K、全部畸变系数和CV_32FC1映射；每帧世界点先变换，使用最终归一化相机姿态投影。
  双线性源有效性独立于像素颜色。可见像素只存深度/点索引，行优先按规定步长取点，转换整数前检查范围。
- [ ] Step4：Google格式，普通全套（原205项及新测试）GREEN，44个普通头保持。task gate camera；提交。

Expected：字面值几何、图像与颜色契约通过，源资产和现有算法行为保持。
Commit：feat: prepare calibrated COLMAP images and lidar seeds。

### Task 2: 普通导出器与持久模型

**Files:** 新 include/livo_atlas/io/colmap_exporter.h、src/io/colmap_exporter.cc、tests/io/colmap_exporter_test.cc；
修改 cmake/core.cmake/tests.cmake。

**Interfaces:** 按Spec的ColmapExportOptions/ColmapExportSummary/ExportColmap(dataset,output,options={})。
消费Task1 ColmapCamera/MakeView，不在调度、ROS或相机前端增加分支。

- [ ] Step1：写公开接口/文件 RED，初始头不存在应失败，记录 exporter-red.log。
  使用Task1字面外参：imu_from_lidar=I,t=(1,0,0)，camera_from_lidar=Rz90,t=(0,1,1)，与快照相机body一致。
  读实际文件验证PINHOLE=1、6×4/4/5/3/2、T_C_world、world点/RGB、每点track的IMAGE_ID/零起始POINT2D_IDX以及images.txt逆关联。
  CSV三时间和0.8逆曝光无精度损失，export.yml是完成标记，记录原标定/外参和原始或map版本。
- [ ] Step2：写版本与生命周期 RED：视觉帧0、非视觉帧1、视觉帧2映射图像ID1/2及0.png/2.png；
  冻结版本1覆盖0/1且记录倒序，之后追加2并移除其云，导出版本1仍成功只含图像0，版本0因损坏失败。
  两帧不同修正改变两份相机/种子几何，图像与关联时间保持；源文件字节及已有位姿文件保持。
- [ ] Step3：写错误与路径 RED：空集、全LO、显式缺失版本、坏资产先于输出创建拒绝；全无种子没有完成标记。
  新输出限制包括已有空目录/含哨兵目录、现有文件、符号链接以及含..或符号链接父目录的源内部路径。
  Unix fork子进程使用真实RLIMIT_FSIZE=0及忽略SIGXFSZ触发文件写入失败；无export.yml、原数据可读、重试不覆盖现场。
- [ ] Step4：实现只读输入/版本ID映射、选中资产预检、逐帧输出、规范图像ID和计数保护、classic/max_digits10文本与CSV。
  metadata复用Decimal/WriteTransform/WriteYaml；模型和图像关闭成功后.pending-export.yml→export.yml，异常增加目标路径。
- [ ] Step5：Google格式，普通全套GREEN，45公开头独立编译通过。task gate exporter；提交。

Expected：完整真实模型可读、版本不混用、原文件保持，错误不伪装成功。
Commit：feat: export versioned keyframes as COLMAP datasets。

### Task 3: 安装CLI与外部消费入口

**Files:** 新 tools/export_colmap.cc；修改 cmake/core.cmake、tests/installed_consumer/main.cc；需要时新增该消费者自己的小型export_scenario.h。
包外新 check_export_cli.py 和外部读取器验收工具。

**Interfaces:** livo_atlas_export_colmap DATASET_DIRECTORY OUTPUT_DIRECTORY [POSE_VERSION] [MAX_POINTS_PER_IMAGE]。
成功JSON(images,points,pose_version)且返回0；参数错误2；数据/文件失败1。安装进bin，复用相对RPATH设置。

- [ ] Step1：先用字面数据和安装路径写CLI RED：目标不存在或安装缺失失败；记录 cli-red.log。
  raw/优化版本及上限1成功；负数/非规范前导零/尾字符/溢出/上限0或UINT32_MAX/错误参数数量返回2且无输出。
  输入缺失/输出已有/无视觉帧/不存在版本返回1，无图时不能吞掉不存在版本错误。
- [ ] Step2：实现薄CLI，复用ExportColmap，不另写数学/路径策略；CMake在普通/Noetic/Humble均构建安装。
  安装消费者创建自己的小数据集，调用公共接口，读完成标记和PNG/模型确认原始/优化导出；不依赖源目录。
- [ ] Step3：shared/static/无OMP三种SDK迁移消费与45头、Noetic/Humble完整构建/测试和ament迁移消费。
  保存真实源码与节点+14自有库摘要；原两个检查工具及新CLI字面数据都通过。task gate installed；提交。

Expected：同一普通实现被两端及包外安装消费者调用，运行时无需源树、COLMAP或CUDA。
Commit：feat: install the offline COLMAP export tool。

### Task 4: 外部实际读取、实包回归与复现文档

**Files:** 新 docs/colmap-export.md、docs/p5-03-validation.md；修改 README.md/docsREADME/core-build/ros-builds 的当前入口和安装说明。
包外读取器版本记录、实际数据导出、逐点几何/时间验证和全阶段脚本。

**Interfaces:** 用实际移动SDK的CLI导出冻结P5-02两端short/full；完整数据副本写入倒序非刚性位姿版本，导出另一个新目录。
外部读取实现只下载到本阶段并固定版本/SHA及许可，来源COLMAP官方read_write_model.py及3DGS官方colmap_loader.py。

- [ ] Step1：官方读取代码实际解析每个导出目录；核对相机/所有PNG/点/双向关联、正深度、重投影和相机中心、版本/时间/曝光。
  完整数据各172图像，短段各2；优化前后相机及点改变，PNG/时间关联稳定，所有冻结和原始副本资产摘要不变。
  记录外部真实读取范围，准确区分数据消费和训练；不得用自己编写的解析器冒充第三方读取。
- [ ] Step2：最终Noetic/Humble各short/full官方bag串行回归；原四数组/发布与冻结P5-02精确保持、短段各57旧产品字节一致，
  原始关键帧资产保持。只写本阶段，绑定实际源码/15运行产物摘要。task gate acceptance。
- [ ] Step3：写使用/布局/坐标/图像/种子与单视角限制/版本/完成标记及跨电脑复现，校正文档总索引旧P5-02计数。
  Google检查、原Log/缺失vikit/保护引用核对；提交并将最终HEAD与验收绑定。

Expected：真实外部消费与两端数值回归均通过，用户能在另一台电脑按文档重建和导出。
Commit：docs: validate portable COLMAP exports across ROS versions。

## 收尾

一次fresh整阶段独立评审；Critical/Important一次RED→GREEN修复加全套相关验证，无二次评审，Minor留档。
归档且只归档本计划执行记录；完整历史--all bundle、空bare全引用/HEAD/历史/fsck恢复核对。
保留refactor2及正常工作区，不合并、推送或改标签。
