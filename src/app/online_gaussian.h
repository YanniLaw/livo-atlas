#ifndef LIVO_ATLAS_SRC_APP_ONLINE_GAUSSIAN_H_
#define LIVO_ATLAS_SRC_APP_ONLINE_GAUSSIAN_H_

#include <cstdint>
#include <memory>

namespace ros {
class NodeHandle;
}  // namespace ros

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace livo_atlas {

class GaussianRenderOutput;
class LivoSystem;
class OnlineGaussianMapper;
struct LivoConfig;
struct OnlineGaussianOptions;
struct OnlineSubmapDelta;

namespace ros1 {
class GaussianRenderPublisher;
}  // namespace ros1

namespace ros2 {
class GaussianRenderPublisher;
}  // namespace ros2

namespace app {

#if LIVO_ATLAS_ROS_VERSION == 1
using RosNode = ::ros::NodeHandle;
using GaussianRenderPublisher = ros1::GaussianRenderPublisher;
#elif LIVO_ATLAS_ROS_VERSION == 2
using RosNode = rclcpp::Node;
using GaussianRenderPublisher = ros2::GaussianRenderPublisher;
#endif

class OnlineGaussian {
 public:
  // Check build support and paths before constructing the LIVO system.
  static void ValidateConfig(const LivoConfig& config);

  OnlineGaussian(RosNode& node, LivoSystem& system, const LivoConfig& config);
  ~OnlineGaussian();

  OnlineGaussian(const OnlineGaussian&) = delete;
  OnlineGaussian& operator=(const OnlineGaussian&) = delete;

  void ObserveDelta(const OnlineSubmapDelta& update);
  void Poll();
  void FinishPublication();
  bool Finish(const OnlineGaussianOptions& options);

 private:
#if LIVO_ATLAS_ONLINE_GAUSSIAN
  std::unique_ptr<OnlineGaussianMapper> mapper_;
  // The publisher outlives the output worker, including stack unwinding.
  std::unique_ptr<GaussianRenderPublisher> publisher_;
  std::unique_ptr<GaussianRenderOutput> publication_;
  std::uint64_t reported_render_job_ = 0;
  std::uint64_t reported_render_wait_ = 0;
#endif
};

}  // namespace app
}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_APP_ONLINE_GAUSSIAN_H_
