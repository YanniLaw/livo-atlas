#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "gaussian/render_output_test_helpers.h"
#include "livo_atlas/gaussian/gaussian_render_output.h"

// Linker wrapping gates the real validation boundary without a test hook in
// production. This target compiles the unchanged controller source itself.
extern "C" void RealValidate(const livo_atlas::GaussianRenderSnapshot&) asm(
    "__real__ZN10livo_atlas30ValidateGaussianRenderSnapshot"
    "ERKNS_22GaussianRenderSnapshotE");
extern "C" void WrappedValidate(const livo_atlas::GaussianRenderSnapshot&) asm(
    "__wrap__ZN10livo_atlas30ValidateGaussianRenderSnapshot"
    "ERKNS_22GaussianRenderSnapshotE");

namespace {
std::mutex gate_mutex;
std::condition_variable gate_condition;
bool gate_enabled = false;
bool gate_entered = false;
bool gate_released = false;
}  // namespace

void WrappedValidate(const livo_atlas::GaussianRenderSnapshot& source) {
  RealValidate(source);
  if (source.status != livo_atlas::GaussianRenderStatus::kReady) return;
  std::unique_lock<std::mutex> lock(gate_mutex);
  if (!gate_enabled) return;
  gate_entered = true;
  gate_condition.notify_all();
  gate_condition.wait(lock, [] { return gate_released; });
  gate_enabled = false;
}

namespace livo_atlas {
namespace {
using namespace std::chrono_literals;

TEST(GaussianRenderAdmissionTest, SupersededValidationCannotStartACallback) {
  for (bool finish : {false, true}) {
    {
      std::lock_guard<std::mutex> lock(gate_mutex);
      gate_enabled = true;
      gate_entered = gate_released = false;
    }
    std::vector<GaussianRenderStatus> received;
    GaussianRenderOutputOptions options;
    options.enabled = true;
    GaussianRenderOutput output(options, [&](const auto& source) {
      received.push_back(source.status);
      return true;
    });
    auto source = test::ReadyRenderSnapshot();
    output.Observe(source);
    bool entered;
    {
      std::unique_lock<std::mutex> lock(gate_mutex);
      entered = gate_condition.wait_for(lock, 3s, [] { return gate_entered; });
    }
    std::thread stopping;
    if (finish) {
      stopping = std::thread([&] { output.Finish(); });
      const auto deadline = std::chrono::steady_clock::now() + 3s;
      while (output.GetStats().last_revision < 2 &&
             std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(1ms);
    } else {
      source.render.reset();
      ++source.map_sequence;
      source.status = GaussianRenderStatus::kInvalidated;
      source.reason = "source_changed";
      output.Observe(source);
    }
    {
      std::lock_guard<std::mutex> lock(gate_mutex);
      gate_released = true;
      gate_condition.notify_all();
    }
    if (stopping.joinable()) stopping.join();
    output.Finish();
    EXPECT_TRUE(entered);
    EXPECT_EQ(std::count(received.begin(), received.end(),
                         GaussianRenderStatus::kReady),
              0);
    EXPECT_GE(output.GetStats().coalesced, 1u);
    EXPECT_EQ(output.GetStats().pending, 0u);
    EXPECT_EQ(output.GetStats().in_flight, 0u);
  }
}

}  // namespace
}  // namespace livo_atlas
