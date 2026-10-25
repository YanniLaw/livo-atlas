#include "livo_atlas/ros1/node_options.h"

#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace livo_atlas::ros1 {
namespace {
class RosParameterReader final : public ParameterReader {
 public:
  explicit RosParameterReader(ros::NodeHandle& node)
      : node_(node), camera_("~") {}
  bool Boolean(const std::string& name, bool fallback) override {
    const auto value = Read(name);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInvalid) return fallback;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
      Bad(name, "boolean");
    return static_cast<bool>(value);
  }
  int Integer(const std::string& name, int fallback) override {
    const auto value = Read(name);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInvalid) return fallback;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) Bad(name, "integer");
    return static_cast<int>(value);
  }
  double Number(const std::string& name, double fallback) override {
    const auto value = Read(name);
    const double result = value.getType() == XmlRpc::XmlRpcValue::TypeInvalid
                              ? fallback
                              : Numeric(name, value);
    if (!std::isfinite(result)) Bad(name, "finite numeric value");
    return result;
  }
  std::string String(const std::string& name,
                     const std::string& fallback) override {
    const auto value = Read(name);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInvalid) return fallback;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeString) Bad(name, "string");
    return static_cast<std::string>(value);
  }
  std::vector<double> NumberArray(const std::string& name) override {
    const auto value = Read(name);
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
      Bad(name, "numeric array");
    std::vector<double> result;
    result.reserve(value.size());
    for (int i = 0; i < value.size(); ++i)
      result.push_back(Numeric(name, value[i]));
    return result;
  }
  std::vector<int> IntegerArray(const std::string& name,
                                const std::vector<int>& fallback) override {
    const auto value = Read(name);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInvalid) return fallback;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
      Bad(name, "integer array");
    std::vector<int> result;
    result.reserve(value.size());
    for (int i = 0; i < value.size(); ++i) {
      if (value[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
        Bad(name, "integer array");
      result.push_back(static_cast<int>(value[i]));
    }
    return result;
  }
  std::string Name(const std::string& name) const override {
    return Handle(name).resolveName(Key(name));
  }

 private:
  [[noreturn]] void Bad(const std::string& name,
                        const std::string& expected) const {
    throw std::invalid_argument(Name(name) + ": expected a " + expected);
  }
  double Numeric(const std::string& name,
                 const XmlRpc::XmlRpcValue& value) const {
    double result;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      result = static_cast<int>(value);
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
      result = static_cast<double>(value);
    else
      Bad(name, "numeric value");
    if (!std::isfinite(result)) Bad(name, "finite numeric value");
    return result;
  }
  static std::string Key(std::string name) {
    std::replace(name.begin(), name.end(), '.', '/');
    return name;
  }
  const ros::NodeHandle& Handle(const std::string& name) const {
    const bool camera = name.rfind("cam_", 0) == 0 || name == "scale" ||
                        name == "k1" || name == "k2" || name == "k3" ||
                        name == "k4";
    return camera ? camera_ : node_;
  }
  XmlRpc::XmlRpcValue Read(const std::string& name) const {
    XmlRpc::XmlRpcValue result;
    Handle(name).getParam(Key(name), result);
    return result;
  }
  ros::NodeHandle& node_;
  ros::NodeHandle camera_;
};
}  // namespace
NodeOptions ReadNodeOptions(ros::NodeHandle& node) {
  RosParameterReader reader(node);
  NodeOptions result;
  result.livo = LoadLivoConfig(reader);
  result.livo.system.imu_time_range = {0, 4294967296.0};
  for (const auto& topic :
       {result.livo.lidar_topic, result.livo.imu_topic,
        result.livo.image_topic})
    node.resolveName(topic);
  return result;
}
}  // namespace livo_atlas::ros1
