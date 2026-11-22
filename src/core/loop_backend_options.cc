#include "core/loop_backend_options.h"

#include <stdexcept>
#include <string>
#include <type_traits>

#include "backend/loop_option_fields.h"

namespace livo_atlas {
namespace {

auto ReadFields(ParameterReader& reader, std::string prefix) {
  return [&reader, prefix](const char* field, auto& value) {
    const auto name = prefix + field;
    using Value = std::decay_t<decltype(value)>;
    if constexpr (std::is_integral_v<Value>) {
      const int count = reader.Integer(name, static_cast<int>(value));
      if constexpr (std::is_unsigned_v<Value>) {
        if (count < 0)
          throw std::invalid_argument(reader.Name(name) +
                                      " must be nonnegative");
      }
      value = static_cast<Value>(count);
    } else {
      value = reader.Number(name, value);
    }
  };
}

}  // namespace

LoopAdmissionOptions ReadOnlineLoopOptions(ParameterReader& reader) {
  namespace fields = loop_option_fields;
  LoopAdmissionOptions options;
  fields::SearchFields(options.search, ReadFields(reader, "loop_closure.search."));
  fields::RegistrationFields(options.registration,
                             ReadFields(reader, "loop_closure.registration."));
  fields::AdmissionFields(options, ReadFields(reader, "loop_closure.admission."));
  fields::GraphFields(options.graph, ReadFields(reader, "pose_graph."));
  fields::GeometryFields(options.geometry,
                         ReadFields(reader, "loop_closure.geometry."));
  return options;
}

}  // namespace livo_atlas
