#include "livo_atlas/backend/loop_worker.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

#include "livo_atlas/io/keyframe_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class Gate {
 public:
  void Enter() {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    condition_.notify_all();
    condition_.wait(lock, [&] { return released_; });
  }

  bool WaitForEntry() {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 5s, [&] { return entered_; });
  }

  void Release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false, released_ = false;
};

class ReleaseOnExit {
 public:
  explicit ReleaseOnExit(Gate& gate) : gate_(gate) {}

  ~ReleaseOnExit() { gate_.Release(); }

 private:
  Gate& gate_;
};

class LoopWorkerTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("c2b-worker-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
    frames_ = std::make_unique<KeyframeStore>(root_, calibration);
  }

  void TearDown() override { fs::remove_all(root_); }

  LoopTask Task(std::uint64_t id) {
    LoopAdmissionInput input;
    input.raw_poses.push_back({});
    input.raw_poses.front().translation.x() = 3;
    return {id, std::move(input), {}, frames_->CaptureReadView()};
  }

  fs::path root_;
  std::unique_ptr<KeyframeStore> frames_;
};

TEST_F(LoopWorkerTest, OnePayloadUntilTaken) {
  Gate gate;
  std::atomic<int> runs{0};
  LoopWorker worker([&](const LoopTask&) {
    ++runs;
    gate.Enter();
    return LoopAdmissionResult{};
  });
  ReleaseOnExit release(gate);
  EXPECT_TRUE(worker.idle());
  worker.Wait();
  auto original = Task(7);
  worker.Submit(original);
  ASSERT_TRUE(gate.WaitForEntry());
  original.task_id = 99;
  original.input.raw_poses.front().translation.x() = 123;
  EXPECT_FALSE(worker.idle());
  EXPECT_THROW(worker.Submit(Task(8)), std::logic_error);
  EXPECT_FALSE(worker.Take());
  gate.Release();
  worker.Wait();
  EXPECT_FALSE(worker.idle());
  EXPECT_THROW(worker.Submit(Task(8)), std::logic_error);
  auto first = worker.Take();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->task.task_id, 7u);
  EXPECT_DOUBLE_EQ(first->task.input.raw_poses.front().translation.x(), 3);
  EXPECT_TRUE(first->admission);
  EXPECT_FALSE(first->error);
  EXPECT_TRUE(worker.idle());
  EXPECT_FALSE(worker.Take());
  worker.Submit(Task(8));
  worker.Wait();
  auto second = worker.Take();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->task.task_id, 8u);
  EXPECT_EQ(runs, 2);
}

TEST_F(LoopWorkerTest, StopWaitsAndWakesWithoutDetach) {
  Gate gate;
  std::atomic<bool> runner_finished{false};
  LoopWorker worker([&](const LoopTask&) {
    gate.Enter();
    runner_finished = true;
    return LoopAdmissionResult{};
  });
  std::promise<void> stop_started;
  auto started = stop_started.get_future();
  std::future<bool> stopped;
  // Release before the future/worker destructors even if an ASSERT fails.
  ReleaseOnExit release(gate);
  worker.Submit(Task(0));
  ASSERT_TRUE(gate.WaitForEntry());
  stopped = std::async(std::launch::async, [&] {
    stop_started.set_value();
    worker.Stop();
    return runner_finished.load();
  });
  ASSERT_EQ(started.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(stopped.wait_for(50ms), std::future_status::timeout);
  gate.Release();
  ASSERT_EQ(stopped.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(stopped.get());
  worker.Wait();
  worker.Stop();
  EXPECT_FALSE(worker.idle());
  EXPECT_THROW(worker.Submit(Task(1)), std::logic_error);
}

TEST_F(LoopWorkerTest, PropagatesBadAllocAndUnknown) {
  for (bool allocation : {false, true}) {
    LoopWorker worker([&](const LoopTask&) -> LoopAdmissionResult {
      if (allocation) throw std::bad_alloc();
      throw 17;
    });
    worker.Submit(Task(0));
    worker.Wait();
    auto result = worker.Take();
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->error);
    EXPECT_FALSE(result->admission);
    if (allocation) {
      EXPECT_THROW(std::rethrow_exception(result->error), std::bad_alloc);
    } else {
      try {
        std::rethrow_exception(result->error);
        ADD_FAILURE() << "Unknown exception must survive transport";
      } catch (int value) {
        EXPECT_EQ(value, 17);
      }
    }
    EXPECT_THROW(worker.Submit(Task(1)), std::logic_error);
  }
}

TEST(LoopWorkerContractTest, RejectsEmptyRunner) {
  EXPECT_THROW(LoopWorker(LoopTaskRunner{}), std::invalid_argument);
}

}  // namespace
}  // namespace livo_atlas
