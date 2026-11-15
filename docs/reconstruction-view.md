# 重建观测视图

`MakeReconstructionView` 将一个已保存或在线交付的视觉关键帧及一致的位姿快照，转换成拥有独立
存储的去畸变图像、有效区域和稀疏 LiDAR 深度。它可作为后续在线 Gaussian mapper 的输入，
本身不训练或渲染高斯，也不创建 ROS 话题或后台线程。

## 使用

接口位于 `livo_atlas/io/reconstruction_view.h`，链接 `livo_atlas::io`：

```cpp
#include <cstdint>

#include "livo_atlas/io/reconstruction_view.h"

void UseObservation(const livo_atlas::CameraConfig& calibration,
                    const livo_atlas::Keyframe& keyframe,
                    const livo_atlas::PoseSnapshot& poses) {
  const auto view = livo_atlas::MakeReconstructionView(
      calibration, keyframe, poses, {1000000, 100000});
  const int x = 0;
  const int y = 0;
  if (view.depth_mask.at<std::uint8_t>(y, x) != 0) {
    const float depth_in_meters = view.depth_m.at<float>(y, x);
    (void)depth_in_meters;
  }
}
```

最后两个数依次是单次图像像素数和输入点云点数上限；应根据相机、点云和设备预算显式设置，
不能设为 0。超限、无图像、非法标定、图像尺寸不符或快照无效会抛异常。传入的标定必须与关键帧的
`calibration_id` 对应；接口保留该 ID，但不能仅凭相机参数判断它来自哪个标定文件。

可从 `OnlineSubmapUpdate` 保存其 `keyframe` 与 `poses` 后，在消费者自己的线程中构造视图；
回调不得重入 `LivoSystem` 或抛异常。复制 `cv::Mat` 可能共享视图自身的存储，异步消费者应按
只读方式使用。此阶段没有提供队列或多消费者注册机制；输出中的版本和前沿供未来消费者检查，
该函数不会自动阻止一个稍后过期的训练结果被提交。前沿增长本身不代表已有帧的位姿变化；
未来消费者应结合实际位姿版本及成员修订判断，不能只比较帧总数。

## 坐标、像素与深度

`camera_from_parent` 将父坐标中的点变换到相机坐标。`parent_frame` 为 `kOdom` 表示原始里程计
系，`kMap` 表示优化地图系；同时保留 `pose_version`、`frame_count` 与 `optimized_frame_count`。
版本 0 的当前帧位姿必须与关键帧原始位姿一致。正版本的尾帧可能由优化锚点延续，不能因此说它们
都参加了优化。`keyframe_id`、`calibration_id` 和 `pose_version` 的作用域是单个运行或数据集；
跨会话合并时必须由调用方另外区分来源，本接口没有生成全局会话 ID。

`camera` 已转换为中心针孔相机，分辨率和焦距只缩放一次，畸变为零。它沿用本工程相机类的
OpenCV 整数像素中心约定：图像第 `(x,y)` 个像素的中心坐标为 `(x,y)`。若交给使用 COLMAP
角点坐标的代码，应将主点 `cx`、`cy` 各加 0.5；内部深度栅格与这一转换一致。
`image_bgr` 是 `CV_8UC3`，不是 RGB；需要 RGB 的训练器应自行转换。

`image_mask` 与 `depth_mask` 均为 `CV_8UC1`，取值 0 或 1，含义不同：前者表示去畸变后图像
采样有效，后者表示对应像素有 LiDAR 测量。`depth_m` 是 `CV_32FC1`，有效值为相机光轴正向 z，
单位米；没有测量的位置存 0，使用时必须由 `depth_mask` 排除。相同像素的多个点只保留最近 z；
合法图像可以没有任何可见深度。`valid_depth_pixels` 是有效深度像素数量；若将 0/1 掩码
作为灰度图显示，可先乘以 255。

深度从该帧的局部点云直接投影；全局位姿修正只改变 `camera_from_parent` 等位姿信息，不应
改变同一观测的深度像素。既有 COLMAP 导出保留父坐标往返的计算方式，因此恰好落在像素边界的
点可能因浮点舍入产生不同的像素归属。

返回值保留状态、点云和图像三个时间，以及前端估计的 `inverse_exposure_time`。后者不是硬件
曝光时长。当前关键帧没有完整逐点时间和连续轨迹，该视图不重新执行逐点运动补偿。

## 限制和验证

单次资源上限约束输入像素和点数；源图像本身、完整位姿快照、其他队列与未来 GPU 内存不在这一
上限内。函数每次构造去畸变映射并进行投影；若后续实际吞吐不足，应先测量再决定是否缓存。
不同版本的视图不能替代未来 mapper 的过期任务检查、子地图修正和训练状态管理。

已有 COLMAP 导出协议和原始关键帧文件保持不变。测试与 Retail_Street 的真实投影对照见
[P6-03a 验证记录](p6-03a-validation.md)，完整构建及依赖见[独立核心说明](core-build.md)。
