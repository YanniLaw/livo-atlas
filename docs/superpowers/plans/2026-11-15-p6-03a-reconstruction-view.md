# P6-03a Reconstruction View Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in this session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 以已有关键帧和位姿快照生成拥有数据的去畸变图像、有效区域和稀疏相机 z 深度视图。

**Architecture:** `livo_atlas::io` 的普通 C++ 值和自由函数复用现有私有 `ColmapCamera`，按一致快照构建全分辨率可见深度及独立掩码；不改前端、ROS 主入口或引入 GPU 依赖。

**Tech Stack:** C++17、Eigen、OpenCV、PCL、GTest、CMake、ROS1 Noetic、ROS2 Humble。

**Spec:** [P6-03a 观测准备设计](../specs/2026-11-15-p6-03a-reconstruction-view-design.md)。

## Global Constraints

- 仅修改 livo-atlas；当前 `framework_completion` 分支、当前会话执行，不合并、推送或打 tag。
- 编码前已通知用户；Google C++17、直接成员、函数实现之间空一行，不使用 Pimpl。
- `ReconstructionView` 数据与输入不共享图像/点云存储；结果 `cv::Mat` 副本仍按只读约定使用。
- 输出 `CameraConfig` 使用 OpenCV 整数像素中心，内部 COLMAP 角点坐标只在边界转换；图像 BGR8，两个掩码为独立的 0/1 `CV_8UC1`。
- 深度为米制相机正向 z，`CV_32FC1`，缺失处为 0 且深度掩码为 0；不把缺失当作零深度监督。
- 限制 `max_pixels` 和 `max_cloud_points` 均由调用方明确指定，先检查再分配/投影；只限制单次输入。
- 版本 0 为 odom，正版本为 map；版本 0 当前帧与原始关键帧位姿在 1e-9 m/1e-9 rad 内一致。
- 不接入在线训练队列、渲染器、CUDA、LibTorch、ROS 新话题，也不更改现有 COLMAP 产品。
- 真正训练器、异步过期提交、子地图成员修订与高斯优化属于后续 P6-03 阶段。
- 构建和数据验证串行：`nice -n 10`、`-j1`、`OMP_THREAD_LIMIT=1`；只读原始 bag 和历史结果。

## File Structure

- `include/livo_atlas/io/reconstruction_view.h`：普通拥有数据的视图值、输入上限和公共构造函数。
- `src/io/reconstruction_view.cc`：资源与快照检查，调用现有投影并构造深度/两种掩码。
- `src/io/colmap_camera.h`：仅添加供同一 IO 库使用的有效图像区域只读访问。
- `cmake/core.cmake`：将实现编入现有 `livo_atlas::io`，仅其私有编译路径可访问既有 pose helper。
- `tests/io/colmap_camera_test.cc`：在现有真实投影测试旁覆盖公共视图行为和失败边界。
- `tests/installed_consumer/export_scenario.h`：实际安装消费者调用新版 API，覆盖导出链接。
- `docs/reconstruction-view.md`、`docs/p6-03a-validation.md`、`docs/README.md`：跨机器使用与实测记录。
- 临时数据探针、真实数据消费者、隔离构建与日志放在包外 `validation/P6-03a-20261115/`。

## Review Focus

- 畸变映射无效区域与没有测得深度是两种含义；须分别测试掩码和值，不能混淆。Task 1。
- 前方多点落到同一像素、点在相机后方或图像外时，深度只来自最近可见点。Task 1。
- 版本 0 伪造位姿、未覆盖 ID、无图像、非法标定或资源上限都应明确拒绝。Task 1。
- 非平凡位姿修正只更新相机父系变换，不应使同一帧局部深度变化。Task 1。
- 大图/大点云、共享 OpenCV 所有权、无有效深度及小数边界不应产生意外修改或无限输出。Task 1。

---

### Task 1: 普通 C++ 视图构造及直接测试

**Files:** Create `include/livo_atlas/io/reconstruction_view.h`, `src/io/reconstruction_view.cc`；modify
`src/io/colmap_camera.h`, `cmake/core.cmake`, `tests/io/colmap_camera_test.cc`。

**Interfaces:** Consumes `CameraConfig`, `Keyframe`, `PoseSnapshot` and
`ReconstructionViewLimits{max_pixels, max_cloud_points}`; produces `ReconstructionView` and
`ReconstructionView MakeReconstructionView(const CameraConfig&, const Keyframe&, const PoseSnapshot&, ReconstructionViewLimits)` as defined verbatim in the spec. The private `ColmapCamera` exposes `const cv::Mat& valid_mask() const` to this implementation; the public result clones it.

- [x] **Step 1: 建立基线。** 在包外新目录配置独立 SDK，不改以前的构建树：

  ```bash
  cmake -S . -B "$PHASE/core" -DLIVO_ATLAS_ROS_VERSION=0 -DCMAKE_BUILD_TYPE=Release \
    -DLIVO_ATLAS_USE_OPENMP=OFF -DLIVO_ATLAS_WITH_GTSAM=ON \
    -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON -DSophus_DIR="$SOPHUS_DIR" \
    -DGTSAM_DIR="$GTSAM_DIR" -DCMAKE_INSTALL_PREFIX="$PHASE/sdk-install"
  nice -n 10 cmake --build "$PHASE/core" --target colmap_camera_test -- -j1
  OMP_THREAD_LIMIT=1 "$PHASE/core/colmap_camera_test"
  ```

  Expected: existing COLMAP camera tests pass. `PHASE` is a new absolute evidence directory;
  on this host `SOPHUS_DIR=/home/yanni/3rd-party/Sophus/build` and
  `GTSAM_DIR=/home/yanni/livo-atlas_ws/validation/P6-02b-20261111/gtsam-install-noetic/lib/cmake/GTSAM`.

- [x] **Step 2: 写失败测试。** 在现有 `Config()`、`Frame()` 夹具旁构造版本 0 的一帧 `PoseSnapshot`
  （`frame_count=1`，位姿 `{0, frame.odom_from_body}`），添加公共头并测试：

  ```cpp
  const auto result = MakeReconstructionView(Config(), frame, poses, {24, 1});
  EXPECT_EQ(result.camera.width, 6);
  EXPECT_EQ(result.camera.height, 4);
  EXPECT_EQ(result.camera.cx, 2.5);
  EXPECT_EQ(result.camera.cy, 1.5);
  EXPECT_EQ(result.image_bgr.type(), CV_8UC3);
  EXPECT_EQ(result.image_mask.at<unsigned char>(2, 3), 1);
  EXPECT_EQ(result.depth_mask.at<unsigned char>(2, 3), 1);
  EXPECT_FLOAT_EQ(result.depth_m.at<float>(2, 3), 4.0f);
  EXPECT_EQ(result.depth_mask.at<unsigned char>(0, 0), 0);
  EXPECT_FLOAT_EQ(result.depth_m.at<float>(0, 0), 0.0f);
  EXPECT_EQ(result.valid_depth_pixels, 1u);
  ```

  再增加具体用例：前后两个点投影同一像素时最近 z=4；相机后方/越界点不增加深度；输入图像
  有效但无可见点时深度掩码全 0。测试返回视图后修改源 `cv::Mat` 和原点云，结果保持不变。

- [x] **Step 3: 写失败边界与版本测试。** 版本 1 使用 `base_version=0`、`frame_count=1`、
  `optimized_frame_count=1`、`parent_frame=kMap` 和平移 `(10,0,0)` 的修正位姿；相机变换与现有
  `ColmapCameraTest.AppliesTheSameBodyCorrectionToTheCameraAndTheSeeds` 相同，掩码/深度与版本 0
  一致。分别检查：版本 0 位姿不匹配、ID 超前沿、没有图像、无效标定、`{23,1}` 像素上限、
  `{24,0}` 点数上限、超过点数限制及非有限输入都抛 `std::invalid_argument`；无效资源在构建映射前
  拒绝。加入 EquidistantCamera 去畸变和已有有效区域边界测试。通过 `PinholeCamera` 使用
  返回的 `CameraConfig` 投影，验证整数像素中心与深度数组索引一致。

- [x] **Step 4: 运行 RED。** `nice -n 10 cmake --build "$PHASE/core" --target colmap_camera_test -- -j1`。
  Expected: 编译因新公共头/函数尚不存在而失败；保存完整日志。

- [x] **Step 5: 实现。** 定义 spec 中的值类型和函数；`ValidateCameraConfig`/`ValidateKeyframe`/
  `ValidatePoseSnapshot` 配合前置容量检查，版本 0 使用现有 `pose_math::Equal`；调用已有
  `ColmapCamera::MakeView`，以 `width*height` 作为点数上限使其不采样，逐样本生成 `depth_m`/
  `depth_mask` 并拒绝转换后非有限或非正的 float 深度，拷贝图像有效区域；返回内参的
  主点各减 0.5 以符合现有 OpenCV 像素中心约定。只给 `livo_atlas_io`
  增加 `src` 私有 include 目录；公开头只包含普通 C++、Eigen/OpenCV/PCL 依赖。

- [x] **Step 6: 运行 GREEN 与核心回归。** 构建并运行 `colmap_camera_test`，再串行构建完整
  SDK 并运行 `OMP_THREAD_LIMIT=1 cmake -E chdir "$PHASE/core" ctest --output-on-failure --timeout 30 -j1`。
  Expected: 新旧相机用例及所有注册核心测试通过。格式检查使用 Google 风格并保留 include 分组，
  同时手查所有修改文件的函数定义之间有空行；`git diff --check` 无输出。

- [x] **Step 7: 提交。** `git add` 本任务列出的源码、头文件、CMake 和测试，提交
  `feat: prepare versioned reconstruction views`。

### Task 2: 安装消费、双 ROS 构建、真实数据与交付记录

**Files:** Modify `tests/installed_consumer/export_scenario.h`, `docs/README.md` and this plan/spec status；
create `docs/reconstruction-view.md`, `docs/p6-03a-validation.md`。真实数据测试程序和日志仅放在 `$PHASE`。

**Interfaces:** Consumes the installed `livo_atlas::io` API from Task 1 and existing
`validation/startup-sync-20261115/replay-noetic-full-on/keyframes` dataset; produces checked, separately retained evidence and portable instructions. No ROS node calls the new API in this stage.

- [x] **Step 1: 验证安装消费。** 扩展已有安装消费者的 COLMAP 场景，分别使用版本 0/1 构造视图，
  检查相机父系、4 米有效深度、掩码和独立所有权；安装 `"$PHASE/core"` 到 `"$PHASE/sdk-install"`，
  从 `tests/installed_consumer` 创建独立外部 CMake 工程并运行。Expected: 现有 LO/LIO/VIO、在线
  子地图、几何和 COLMAP/视图消费均通过；不借用构建树里的目标。

- [x] **Step 2: 真实数据独立对照。** 在 `$PHASE` 用安装后的 SDK 构建只读小程序，读取已提交
  172 帧和版本 0 快照，调用新 API（例如像素上限 1,000,000，点数上限 100,000），记录逐帧
  有效像素数和实际处理时间。输出前 3 帧掩码/float 深度到包外证据目录；与已冻结的
  `validation/P6-03-design-20261115/` 独立 Python 投影及 COLMAP 对照逐像素比较，允许 float32
  深度舍入，掩码要求完全一致。记录输入 SHA，不改原始数据。Expected: 172 帧完成、像素数一致、
  前 3 帧掩码完全一致且有效深度误差不超过 1e-5 米；报告耗时和 RSS 但不宣传在线训练吞吐。

- [x] **Step 3: 双 ROS 受影响构建。** 将提交后的源码快照分别放入独立 Noetic catkin 工作区和
  Humble 构建目录，不复用或覆盖上一阶段的证据构建。各自构建真实 `fastlivo_mapping` 和
  `colmap_camera_test`，运行该测试；Humble source 消息依赖及系统环境见 `docs/ros-builds.md`。
  Expected: 两端均编译通过并通过全部相机/新视图测试；`ldd` 无 vikit、CUDA、torch 或缺失库。
  这些构建只说明 CPU 观测接口兼容，不声称已运行 GPU 训练。由于未改变运行路径，完整 bag 不重复。

- [ ] **Step 4: 文档及最终审阅。** 记录用法、坐标、像素中心、时间、内存上限和可移植构建；
  写入真实数据覆盖、测试、独立对照及未实现的在线 mapper。先提交文档，运行任务完成检查；
  然后按 executing-plans 进行一次整个阶段的独立审阅，处理 Critical/Important 并记录范围。

## Self-review

设计中的观测格式、v0/正版本、两类掩码、空投影、有限规模、所有权和真实数据对照均归入
Task 1/2。接口命名在两个任务间一致；并发训练调度、GPU backend、持久化和 map correction
交付有意留给后续，不在本计划里伪造接口。
