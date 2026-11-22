#ifndef LIVO_ATLAS_COMMON_LOGGING_H_
#define LIVO_ATLAS_COMMON_LOGGING_H_

#include <glog/logging.h>

namespace livo_atlas {

// Call once at the executable entry point.
inline void InitializeLogging(const char* program) {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(program);
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_COMMON_LOGGING_H_
