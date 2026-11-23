#ifndef LIVO_ATLAS_SRC_ROS_COMMON_PUBLICATION_TIMING_H_
#define LIVO_ATLAS_SRC_ROS_COMMON_PUBLICATION_TIMING_H_

#include "common/timing.h"
#include "livo_atlas/frontend/synchronized_measurements.h"

namespace livo_atlas::ros_detail {

class PublicationTiming {
 public:
  PublicationTiming(bool enabled, UpdateKind kind)
      : start_(StartTiming(enabled)), visual_(kind == UpdateKind::kVisual) {}

  template <typename Publish>
  void Send(Publish publish) {
    const auto start = StartTiming(start_ != SteadyClock::time_point{});
    publish();
    if (start != SteadyClock::time_point{}) {
      publish_time_ += SteadyClock::now() - start;
    }
  }

  void Report() const {
    if (start_ == SteadyClock::time_point{}) return;
    const auto total = SteadyClock::now() - start_;
    LogStageTiming(visual_ ? "ros_vio_message_build" : "ros_lio_message_build",
                   total - publish_time_);
    LogStageTiming(visual_ ? "ros_vio_publish" : "ros_lio_publish",
                   publish_time_);
  }

 private:
  SteadyClock::time_point start_;
  bool visual_;
  SteadyClock::duration publish_time_{};
};

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_COMMON_PUBLICATION_TIMING_H_
