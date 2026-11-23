#ifndef LIVO_ATLAS_SRC_COMMON_TIMING_H_
#define LIVO_ATLAS_SRC_COMMON_TIMING_H_

#include <chrono>
#include <iomanip>

#include "livo_atlas/common/logging.h"

namespace livo_atlas {

using SteadyClock = std::chrono::steady_clock;

inline SteadyClock::time_point StartTiming(bool enabled) {
  return enabled ? SteadyClock::now() : SteadyClock::time_point{};
}

inline void LogStageSeconds(const char* stage, double seconds) {
  LOG(INFO) << "livo_atlas timing stage=" << stage << " seconds=" << std::fixed
            << std::setprecision(9) << seconds;
}

inline void LogStageTiming(const char* stage, SteadyClock::duration elapsed) {
  LogStageSeconds(stage, std::chrono::duration<double>(elapsed).count());
}

inline void LogStageTiming(const char* stage, SteadyClock::time_point start) {
  if (start != SteadyClock::time_point{}) {
    LogStageTiming(stage, SteadyClock::now() - start);
  }
}

}  // namespace livo_atlas

#endif  // LIVO_ATLAS_SRC_COMMON_TIMING_H_
