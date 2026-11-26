#include "app/shutdown_signal.h"

#include <gtest/gtest.h>

#include <csignal>

namespace livo_atlas::app {
namespace {

volatile std::sig_atomic_t previous_handler_calls = 0;

void PreviousHandler(int) { ++previous_handler_calls; }

TEST(ShutdownSignalTest, SignalsRequestStopWithoutTerminatingTheProcess) {
  ShutdownSignal signal;
  EXPECT_FALSE(signal.requested());
  std::raise(SIGINT);
  EXPECT_TRUE(signal.requested());
  std::raise(SIGTERM);
  std::raise(SIGINT);
  EXPECT_TRUE(signal.requested());
}

TEST(ShutdownSignalTest, RestoresHandlersAndResetsTheNextOwner) {
  const auto previous = std::signal(SIGTERM, PreviousHandler);
  previous_handler_calls = 0;
  {
    ShutdownSignal signal;
    std::raise(SIGTERM);
    EXPECT_TRUE(signal.requested());
    EXPECT_EQ(previous_handler_calls, 0);
  }
  std::raise(SIGTERM);
  EXPECT_EQ(previous_handler_calls, 1);
  {
    ShutdownSignal signal;
    EXPECT_FALSE(signal.requested());
  }
  std::signal(SIGTERM, previous);
}

}  // namespace
}  // namespace livo_atlas::app
