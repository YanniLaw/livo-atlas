#ifndef LIVO_ATLAS_SRC_IO_LOOP_OPTIONS_H_
#define LIVO_ATLAS_SRC_IO_LOOP_OPTIONS_H_

#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <opencv2/core/persistence.hpp>

#include "io/dataset_metadata.h"
#include "livo_atlas/backend/loop_closure.h"

namespace livo_atlas::loop_options {

void ValidateKeys(const cv::FileNode& group,
                  const std::set<std::string>& allowed);

template <typename Fields, typename Options>
void ReadFields(const cv::FileNode& group, Options& options, Fields fields) {
  if (group.empty()) return;
  std::set<std::string> names;
  fields(options, [&](const char* name, auto&) { names.insert(name); });
  ValidateKeys(group, names);
  fields(options, [&](const char* name, auto& value) {
    const auto node = group[name];
    if (node.empty()) return;
    if (!node.isReal() && !node.isInt())
      throw std::invalid_argument("Loop option must be numeric");
    const double number = static_cast<double>(node);
    using Value = std::decay_t<decltype(value)>;
    if (!std::isfinite(number))
      throw std::invalid_argument("Loop option must be finite");
    if constexpr (std::is_integral_v<Value>) {
      // Check the exact-double range before any integer conversion.
      if (number < 0 || number > 9007199254740991.0 ||
          std::floor(number) != number ||
          number >= static_cast<double>(std::numeric_limits<Value>::max())) {
        throw std::invalid_argument("Loop count option is out of range");
      }
    }
    value = static_cast<Value>(number);
  });
}

template <typename Value>
void WriteField(cv::FileStorage& yaml, const char* name, Value value) {
  if constexpr (std::is_integral_v<Value>)
    yaml << name << std::to_string(value);
  else
    yaml << name << dataset_metadata::Decimal(value);
}

void ReadSearch(const cv::FileNode& node, LoopSearchOptions& options);
void ReadRegistration(const cv::FileNode& node,
                      LoopRegistrationOptions& options);
void Write(cv::FileStorage& yaml, LoopSearchOptions search,
           LoopRegistrationOptions registration);

}  // namespace livo_atlas::loop_options

#endif  // LIVO_ATLAS_SRC_IO_LOOP_OPTIONS_H_
