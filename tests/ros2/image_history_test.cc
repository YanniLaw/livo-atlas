#if defined(__linux__)
#include <dlfcn.h>
#endif
#include <gtest/gtest.h>
#include <rcl/publisher.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/ros2/livo_node.h"
#include "livo_atlas/ros2/node_options.h"
#include "ros2/fixtures.h"

#if defined(__linux__)
namespace {
std::atomic<std::size_t> image_history{0};
}  // namespace

// Fast DDS discovery reports an unknown history depth. Observe the actual RCL
// publisher handle while forwarding the real initialization unchanged.
extern "C" rcl_ret_t rcl_publisher_init(
    rcl_publisher_t* publisher, const rcl_node_t* node,
    const rosidl_message_type_support_t* type_support, const char* topic,
    const rcl_publisher_options_t* options) {
  static const auto original = reinterpret_cast<decltype(&rcl_publisher_init)>(
      dlsym(RTLD_NEXT, "rcl_publisher_init"));
  if (!original) std::abort();
  const auto result = original(publisher, node, type_support, topic, options);
  if (result == RCL_RET_OK &&
      std::strcmp(rcl_publisher_get_topic_name(publisher), "/rgb_img") == 0) {
    image_history = rcl_publisher_get_actual_qos(publisher)->depth;
  }
  return result;
}
#endif

namespace livo_atlas::ros2 {
TEST(Ros2NodeTest, ConfiguresTheActualImagePublisherHistory) {
#if defined(__linux__)
  for (int depth : {1, 32}) {
    SCOPED_TRACE(depth);
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("livo-image-history-" + std::to_string(depth) + "-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    {
      auto parameters = test::Parameters(directory.string());
      if (depth != 1)
        parameters.emplace_back("publish.image_queue_size", depth);
      const auto name = "image_history_test_" + std::to_string(depth);
      auto node = std::make_shared<rclcpp::Node>(
          name, rclcpp::NodeOptions().parameter_overrides(parameters));
      const auto options = ReadNodeOptions(*node);
      LivoSystem system(options.livo.system);
      image_history = 0;
      LivoNode mapper(*node, options, system);
      EXPECT_EQ(image_history.load(), static_cast<std::size_t>(depth));
      system.Finish();
    }
    std::filesystem::remove_all(directory);
  }
#else
  GTEST_SKIP() << "RCL initialization observation requires Linux interposition";
#endif
}
}  // namespace livo_atlas::ros2
