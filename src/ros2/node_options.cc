#include "livo_atlas/ros2/node_options.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace livo_atlas::ros2 {
namespace {
class RosParameterReader final : public ParameterReader {
 public:
  explicit RosParameterReader(rclcpp::Node& node) : node_(node) {}
  bool Boolean(const std::string& name, bool fallback) override {
    return Declare(name, rclcpp::ParameterValue(fallback)).as_bool();
  }
  int Integer(const std::string& name, int fallback) override {
    return Narrow(name,
                  Declare(name, rclcpp::ParameterValue(fallback)).as_int());
  }
  double Number(const std::string& name, double fallback) override {
    const auto value = Declare(name, rclcpp::ParameterValue(fallback), true);
    double result;
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      result = value.as_int();
    else if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      result = value.as_double();
    else
      throw std::invalid_argument(name + " must be numeric");
    if (!std::isfinite(result))
      throw std::invalid_argument(name + " must be finite");
    return result;
  }
  std::string String(const std::string& name,
                     const std::string& fallback) override {
    return Declare(name, rclcpp::ParameterValue(fallback)).as_string();
  }
  std::vector<double> NumberArray(const std::string& name) override {
    const auto value =
        Declare(name, rclcpp::ParameterValue(std::vector<double>{}), true);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
      return value.as_double_array();
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto& integers = value.as_integer_array();
      return {integers.begin(), integers.end()};
    }
    throw std::invalid_argument(name + " must be a numeric array");
  }
  std::vector<int> IntegerArray(const std::string& name,
                                const std::vector<int>& fallback) override {
    const auto value =
        Declare(name, rclcpp::ParameterValue(std::vector<std::int64_t>(
                          fallback.begin(), fallback.end())));
    std::vector<int> result;
    for (auto integer : value.as_integer_array())
      result.push_back(Narrow(name, integer));
    return result;
  }
  std::string Name(const std::string& name) const override { return name; }

 private:
  int Narrow(const std::string& name, std::int64_t value) const {
    if (value < std::numeric_limits<int>::min() ||
        value > std::numeric_limits<int>::max())
      throw std::invalid_argument(name + " is outside the integer range");
    return static_cast<int>(value);
  }
  rclcpp::Parameter Declare(const std::string& name,
                            const rclcpp::ParameterValue& value,
                            bool numeric = false) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.dynamic_typing = numeric;
    node_.declare_parameter(name, value, descriptor);
    return node_.get_parameter(name);
  }
  rclcpp::Node& node_;
};
}  // namespace
NodeOptions ReadNodeOptions(rclcpp::Node& node) {
  RosParameterReader reader(node);
  NodeOptions result;
  result.livo = LoadLivoConfig(reader);
  for (const auto& topic : {result.livo.lidar_topic, result.livo.imu_topic,
                            result.livo.image_topic})
    node.get_node_topics_interface()->resolve_topic_name(topic);
  result.reliability = reader.String("qos.reliability", result.reliability);
  result.imu_depth = reader.Integer("qos.imu_depth", result.imu_depth);
  result.lidar_depth = reader.Integer("qos.lidar_depth", result.lidar_depth);
  result.image_depth = reader.Integer("qos.image_depth", result.image_depth);
  if ((result.reliability != "best_effort" &&
       result.reliability != "reliable") ||
      result.imu_depth <= 0 || result.lidar_depth <= 0 ||
      result.image_depth <= 0)
    throw std::invalid_argument(
        "qos reliability must be best_effort/reliable and depths must be "
        "positive");
  return result;
}
}  // namespace livo_atlas::ros2
