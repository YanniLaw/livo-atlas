#include "livo_atlas/core/trajectory_output.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "core/fixtures.h"

namespace livo_atlas {
namespace {
using Clock = std::chrono::steady_clock;

class TrajectoryOutputTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = test::Options(FrontendMode::kLidarOnly);
    options_.keyframes.enabled = options_.submaps.enabled = true;
    options_.keyframes.time_threshold = 0;
    options_.submap_mode = SubmapMode::kOnline;
    options_.submaps.max_keyframes = 2;
    system_ = std::make_unique<LivoSystem>(options_);
  }

  void TearDown() override {
    Release();
    if (system_) system_->SetOnlineSubmapCallback({});
    output_.reset();
    if (system_) EXPECT_NO_THROW(system_->Finish());
    system_.reset();
    std::filesystem::remove_all(options_.exporter.directory);
  }

  void Start(int limit = 20, TrajectoryOutput::Consumer consumer = {}) {
    TrajectoryOutputOptions options;
    options.enabled = true;
    options.max_frequency_hz = 10;
    options.max_keyframes = limit;
    if (!consumer) {
      consumer = [this](const TrajectorySnapshot& snapshot) {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        condition_.notify_all();
        condition_.wait(lock, [&] { return !blocked_; });
        snapshots_.push_back(snapshot);
        delivered_at_.push_back(Clock::now());
        condition_.notify_all();
      };
    }
    output_ = std::make_unique<TrajectoryOutput>(*system_, options, consumer);
    system_->SetOnlineSubmapCallback(
        [this](const auto& update) { output_->Observe(update); });
  }

  void FeedTo(std::uint64_t count) {
    while (system_->GetPoseSnapshot()->frame_count < count && scan_ < 30) {
      ASSERT_TRUE(system_->AddLidar(test::Scan(100 + scan_++ * .25)));
      system_->ProcessAvailable({});
    }
    ASSERT_EQ(system_->GetPoseSnapshot()->frame_count, count);
  }

  bool Pump(const std::function<bool()>& ready) {
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (Clock::now() < deadline) {
      output_->Poll();
      if (ready()) return true;
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait_for(lock, std::chrono::milliseconds(1));
    }
    return ready();
  }

  bool WaitForSize(std::size_t count) {
    return Pump([&] {
      if (output_->GetStats().delivered < count) return false;
      std::lock_guard<std::mutex> lock(mutex_);
      return snapshots_.size() >= count;
    });
  }

  TrajectorySnapshot Snapshot(std::size_t i) {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshots_.at(i);
  }

  void Release() {
    std::lock_guard<std::mutex> lock(mutex_);
    blocked_ = false;
    condition_.notify_all();
  }

  void Optimize() {
    const auto source = system_->GetPoseSnapshot();
    PoseUpdate update{source->version, source->frame_count, source->poses,
                      PoseTailPolicy::kProjectedLastOptimizedAnchor};
    for (auto& pose : update.poses)
      pose.map_from_body.translation.x() += pose.id + 1;
    system_->ApplyPoseUpdate(update);
  }

  SystemOptions options_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<TrajectoryOutput> output_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<TrajectorySnapshot> snapshots_;
  std::vector<Clock::time_point> delivered_at_;
  bool blocked_ = false;
  bool entered_ = false;
  int scan_ = 0;
};

TEST_F(TrajectoryOutputTest, EmptyThenGrowingAndOptimizedAtTheSameSensorTime) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  const auto empty = Snapshot(0);
  EXPECT_EQ(empty.status, TrajectoryStatus::kReady);
  EXPECT_TRUE(empty.keyframes.empty());
  EXPECT_EQ(empty.revision, 1);
  EXPECT_EQ(empty.producer_id.size(), 32);
  EXPECT_EQ(empty.producer_id.find_first_not_of("0123456789abcdef"),
            std::string::npos);
  FeedTo(2);
  ASSERT_TRUE(WaitForSize(2));
  const auto raw = Snapshot(1);
  ASSERT_EQ(raw.keyframes.size(), 2);
  Optimize();
  ASSERT_TRUE(WaitForSize(3));
  const auto optimized = Snapshot(2);
  ASSERT_EQ(optimized.keyframes.size(), 2);
  EXPECT_EQ(optimized.pose_version, 1);
  EXPECT_EQ(optimized.parent_frame, PoseFrame::kMap);
  EXPECT_EQ(optimized.producer_id, raw.producer_id);
  EXPECT_GT(optimized.revision, raw.revision);
  EXPECT_DOUBLE_EQ(optimized.keyframes.back().timestamp,
                   raw.keyframes.back().timestamp);
  EXPECT_NEAR(optimized.keyframes[0].parent_from_body.translation.x() -
                  raw.keyframes[0].parent_from_body.translation.x(),
              1, 1e-9);
  EXPECT_NEAR(optimized.keyframes[1].parent_from_body.translation.x() -
                  raw.keyframes[1].parent_from_body.translation.x(),
              2, 1e-9);
  FeedTo(3);
  ASSERT_TRUE(WaitForSize(4));
  EXPECT_EQ(Snapshot(3).pose_version, 1);
  EXPECT_EQ(Snapshot(3).optimized_frame_count, 2);
  EXPECT_EQ(Snapshot(3).frame_count, 3);
  system_->Finish();
  output_->Finish();
  output_->Finish();
  EXPECT_EQ(output_->GetStats().delivered, 4);
  EXPECT_TRUE(output_->GetStats().finished);
}

TEST_F(TrajectoryOutputTest, SlowConsumerDoesNotStarveCompletedRevisions) {
  blocked_ = true;
  Start();
  output_->Poll();
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(condition_.wait_for(lock, std::chrono::seconds(3),
                                    [&] { return entered_; }));
  }
  FeedTo(1);
  output_->Poll();
  FeedTo(2);
  output_->Poll();
  EXPECT_EQ(output_->GetStats().peak_work_payloads, 1);
  EXPECT_EQ(output_->GetStats().peak_pending, 1);
  Release();
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(0).frame_count, 0);
  EXPECT_EQ(Snapshot(1).frame_count, 2);
  EXPECT_GT(Snapshot(1).revision, Snapshot(0).revision + 1);
  EXPECT_GT(output_->GetStats().coalesced, 0);
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_GE(delivered_at_[1] - delivered_at_[0], std::chrono::milliseconds(90));
}

TEST_F(TrajectoryOutputTest, CachesOnlyTimestampMetadataForTheFixedPrefix) {
  Start();
  FeedTo(1);
  ASSERT_TRUE(WaitForSize(1));
  const auto raw = Snapshot(0);
  const auto metadata =
      options_.exporter.directory / "keyframes/frames/0/frame.yml";
  Optimize();
  const auto saved = metadata.string() + ".saved";
  std::filesystem::rename(metadata, saved);
  const bool ready = WaitForSize(2);
  std::filesystem::rename(saved, metadata);
  ASSERT_TRUE(ready);
  EXPECT_EQ(Snapshot(1).status, TrajectoryStatus::kReady);
  EXPECT_DOUBLE_EQ(Snapshot(1).keyframes[0].timestamp,
                   raw.keyframes[0].timestamp);
}

TEST_F(TrajectoryOutputTest,
       ResourceLimitKeepsFrontierAndDoesNotReadMissingFiles) {
  Start(1);
  FeedTo(2);
  std::filesystem::remove_all(options_.exporter.directory /
                              "keyframes/frames/0");
  ASSERT_TRUE(WaitForSize(1));
  const auto result = Snapshot(0);
  EXPECT_EQ(result.status, TrajectoryStatus::kResourceLimited);
  EXPECT_EQ(result.frame_count, 2);
  EXPECT_TRUE(result.keyframes.empty());
  EXPECT_EQ(output_->GetStats().resource_limited, 1);
  EXPECT_EQ(output_->GetStats().failed, 0);
  system_->Finish();
  EXPECT_NO_THROW(output_->Finish());
}

TEST_F(TrajectoryOutputTest, ReadFailureStopsOnlyTheDerivedOutput) {
  Start();
  FeedTo(1);
  std::filesystem::remove(options_.exporter.directory /
                          "keyframes/frames/0/frame.yml");
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, TrajectoryStatus::kError);
  EXPECT_TRUE(Snapshot(0).keyframes.empty());
  EXPECT_FALSE(Snapshot(0).error.empty());
  EXPECT_LE(Snapshot(0).error.size(), 1024);
  ASSERT_TRUE(Pump([&] { return output_->GetStats().failed == 1; }));
  FeedTo(2);
  EXPECT_NO_THROW(system_->Finish());
  EXPECT_NO_THROW(output_->Finish());
  EXPECT_EQ(output_->GetStats().delivered, 1);
}

TEST_F(TrajectoryOutputTest, DefaultBudgetDoesNotRetainAnOversizedSnapshot) {
  Start(20000);
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 20001;
  poses->poses.resize(20001);
  for (std::size_t i = 0; i < poses->poses.size(); ++i) poses->poses[i].id = i;
  std::weak_ptr<const PoseSnapshot> borrowed = poses;
  OnlineSubmapUpdate update;
  update.poses = std::move(poses);
  output_->Observe(update);
  update.poses.reset();
  EXPECT_TRUE(borrowed.expired());
  // No such disk prefix exists: the resource result must not query it.
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, TrajectoryStatus::kResourceLimited);
  EXPECT_EQ(Snapshot(0).frame_count, 20001);
  EXPECT_EQ(output_->GetStats().timestamp_reads, 0);
}

TEST_F(TrajectoryOutputTest, SinkFailureIsNotRetriedOrPropagatedToTheSystem) {
  std::atomic<int> calls{0};
  Start(20, [&](const auto&) {
    ++calls;
    throw std::runtime_error("sink failed");
  });
  ASSERT_TRUE(Pump([&] { return output_->GetStats().failed == 1; }));
  FeedTo(2);
  EXPECT_NO_THROW(system_->Finish());
  EXPECT_NO_THROW(output_->Finish());
  EXPECT_EQ(calls.load(), 1);
  EXPECT_EQ(output_->GetStats().delivered, 0);
  EXPECT_FALSE(output_->GetStats().error.empty());
}

TEST_F(TrajectoryOutputTest, DestructorDoesNotReadTheExpiredSystem) {
  Start();
  FeedTo(1);
  output_->Poll();
  system_->SetOnlineSubmapCallback({});
  system_->Finish();
  system_.reset();
  EXPECT_NO_THROW(output_.reset());
}

TEST_F(TrajectoryOutputTest, StopsWhileTheWorkerReturnsToIdle) {
  TrajectoryOutputOptions options;
  options.enabled = true;
  // Race both stop paths against the worker returning from delivery to wait.
  for (int i = 0; i < 2000; ++i) {
    TrajectoryOutput output(*system_, options, [](const auto&) {});
    output.Poll();
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (output.GetStats().delivered == 0 && Clock::now() < deadline)
      std::this_thread::yield();
    ASSERT_EQ(output.GetStats().delivered, 1);
    if (i % 2 == 0) output.Finish();
  }
}

TEST_F(TrajectoryOutputTest, RestartHasANewProducerAndAResetRevision) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  const auto first = Snapshot(0);
  system_->SetOnlineSubmapCallback({});
  output_.reset();
  Start();
  ASSERT_TRUE(Pump([&] { return output_->GetStats().delivered == 1; }));
  EXPECT_NE(first.producer_id, Snapshot(1).producer_id);
  EXPECT_EQ(Snapshot(1).revision, 1);
}

TEST_F(TrajectoryOutputTest, InvalidNotificationDoesNotEscapeTheObserver) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  auto bad = std::make_shared<PoseSnapshot>();
  bad->tail_policy = static_cast<PoseTailPolicy>(42);
  OnlineSubmapUpdate notification;
  notification.poses = bad;
  EXPECT_NO_THROW(output_->Observe(notification));
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, TrajectoryStatus::kError);
  EXPECT_TRUE(Snapshot(1).keyframes.empty());
  EXPECT_GT(Snapshot(1).revision, Snapshot(0).revision);
}

TEST_F(TrajectoryOutputTest, ReadFrontierMismatchDeliversAnErrorState) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  auto bad = std::make_shared<PoseSnapshot>();
  bad->frame_count = 1;
  bad->poses = {{0, {}}};
  OnlineSubmapUpdate notification;
  notification.poses = bad;
  output_->Observe(notification);
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, TrajectoryStatus::kError);
  EXPECT_EQ(Snapshot(1).frame_count, 1);
  EXPECT_TRUE(Snapshot(1).keyframes.empty());
  EXPECT_NO_THROW(system_->Finish());
}

}  // namespace
}  // namespace livo_atlas
