#ifndef LIVO_ATLAS_SRC_ROS_COMMON_TIME_CONVERSION_H_
#define LIVO_ATLAS_SRC_ROS_COMMON_TIME_CONVERSION_H_

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace livo_atlas::ros_detail {

inline std::int64_t RosNanoseconds(double time, const char* context,
                                   const char* lowercase_context) {
  if (!std::isfinite(time) || time < 0 || time >= 2147483648.0)
    throw std::invalid_argument(std::string(context) +
                                " time must be in [0,2^31) seconds");
  const auto seconds = static_cast<std::int64_t>(std::floor(time));
  const auto result =
      seconds * 1000000000LL + std::llround((time - seconds) * 1000000000.0);
  if (result >= 2147483648LL * 1000000000LL)
    throw std::invalid_argument(std::string("Rounded ") + lowercase_context +
                                " time exceeds ROS range");
  return result;
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_COMMON_TIME_CONVERSION_H_
