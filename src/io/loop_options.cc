#include "io/loop_options.h"

#include "backend/loop_option_fields.h"

namespace livo_atlas::loop_options {
using loop_option_fields::RegistrationFields;
using loop_option_fields::SearchFields;

void ValidateKeys(const cv::FileNode& group,
                  const std::set<std::string>& allowed) {
  if (!group.isMap())
    throw std::invalid_argument("Loop option group must be a map");
  std::set<std::string> seen;
  for (const auto& node : group) {
    if (!allowed.count(node.name()))
      throw std::invalid_argument("Unknown loop option: " + node.name());
    if (!seen.insert(node.name()).second)
      throw std::invalid_argument("Duplicate loop option: " + node.name());
  }
}

void ReadSearch(const cv::FileNode& node, LoopSearchOptions& options,
                bool serialized) {
  ReadFields(
      node, options, [](auto& o, auto visit) { SearchFields(o, visit); },
      serialized);
}

void ReadRegistration(const cv::FileNode& node,
                      LoopRegistrationOptions& options, bool serialized) {
  ReadFields(
      node, options, [](auto& o, auto visit) { RegistrationFields(o, visit); },
      serialized);
}

void Write(cv::FileStorage& yaml, LoopSearchOptions search,
           LoopRegistrationOptions registration) {
  const auto write = [&](const char* name, auto value) {
    WriteField(yaml, name, value);
  };
  yaml << "search"
       << "{";
  SearchFields(search, write);
  yaml << "}"
       << "registration"
       << "{";
  RegistrationFields(registration, write);
  yaml << "}";
}

}  // namespace livo_atlas::loop_options
