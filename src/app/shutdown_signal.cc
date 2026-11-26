#include "app/shutdown_signal.h"

#include <atomic>
#include <cerrno>
#include <system_error>

namespace livo_atlas::app {
namespace {

// A handler may run on any ROS/worker thread. A lock-free atomic supports both
// that signal context and reads by the owner or the executor's wall timer.
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> shutdown_requested{false};

void RequestShutdown(int) {
  shutdown_requested.store(true, std::memory_order_relaxed);
}

}  // namespace

ShutdownSignal::ShutdownSignal() {
  shutdown_requested.store(false, std::memory_order_relaxed);
  struct sigaction action{};
  action.sa_handler = RequestShutdown;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  if (sigaction(SIGINT, &action, &previous_interrupt_) != 0) {
    throw std::system_error(errno, std::generic_category(),
                            "Install SIGINT handler");
  }
  if (sigaction(SIGTERM, &action, &previous_terminate_) != 0) {
    const int error = errno;
    sigaction(SIGINT, &previous_interrupt_, nullptr);
    throw std::system_error(error, std::generic_category(),
                            "Install SIGTERM handler");
  }
}

ShutdownSignal::~ShutdownSignal() {
  sigaction(SIGTERM, &previous_terminate_, nullptr);
  sigaction(SIGINT, &previous_interrupt_, nullptr);
}

bool ShutdownSignal::requested() const {
  return shutdown_requested.load(std::memory_order_relaxed);
}

}  // namespace livo_atlas::app
