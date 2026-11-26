#ifndef LIVO_ATLAS_SRC_APP_SHUTDOWN_SIGNAL_H_
#define LIVO_ATLAS_SRC_APP_SHUTDOWN_SIGNAL_H_

#include <signal.h>

namespace livo_atlas::app {

// One owner per process. ROS must leave signal handling to this object so its
// context remains alive while the application finishes and publishes outputs.
class ShutdownSignal {
 public:
  ShutdownSignal();
  ~ShutdownSignal();
  ShutdownSignal(const ShutdownSignal&) = delete;
  ShutdownSignal& operator=(const ShutdownSignal&) = delete;

  bool requested() const;

 private:
  struct sigaction previous_interrupt_{};
  struct sigaction previous_terminate_{};
};

}  // namespace livo_atlas::app

#endif  // LIVO_ATLAS_SRC_APP_SHUTDOWN_SIGNAL_H_
