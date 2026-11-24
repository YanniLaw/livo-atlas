#include "livo_atlas/core/geometry_map_output.h"

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

#include <gtest/gtest.h>

#include "core/fixtures.h"

namespace livo_atlas {
namespace {
using Clock = std::chrono::steady_clock;

class GeometryMapOutputTest : public testing::Test {
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
    if (system_) {
      system_->SetOnlineSubmapCallback({});
      system_->SetOnlineSubmapDeltaCallback({});
    }
    output_.reset();
    if (system_) EXPECT_NO_THROW(system_->Finish());
    system_.reset();
    std::filesystem::remove_all(options_.exporter.directory);
  }

  void Start(int max_frames = 20, int max_bytes = 8388608,
             GeometryMapOutput::Consumer consumer = {},
             bool incremental = false) {
    GeometryMapOutputOptions output_options;
    output_options.enabled = true;
    output_options.max_frequency_hz = 2;
    output_options.max_keyframes = max_frames;
    output_options.map.max_message_bytes = max_bytes;
    if (!consumer) {
      consumer = [this](const GeometryMapSnapshot& snapshot) {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        condition_.notify_all();
        condition_.wait(lock, [&] { return !blocked_; });
        snapshots_.push_back(snapshot);
        condition_.notify_all();
      };
    }
    output_ = std::make_unique<GeometryMapOutput>(*system_, output_options,
                                                  options_.geometry, consumer);
    if (incremental) {
      system_->SetOnlineSubmapDeltaCallback(
          [this](const auto& update) { output_->ObserveDelta(update); });
    } else {
      system_->SetOnlineSubmapCallback(
          [this](const auto& update) { output_->Observe(update); });
    }
  }

  void FeedTo(std::uint64_t count) {
    while (system_->GetPoseSnapshot()->frame_count < count && scan_ < 30) {
      ASSERT_TRUE(system_->AddLidar(test::Scan(100 + scan_++ * .25)));
      system_->ProcessAvailable({});
    }
    ASSERT_EQ(system_->GetPoseSnapshot()->frame_count, count);
  }

  bool Pump(const std::function<bool()>& ready) {
    const auto deadline = Clock::now() + std::chrono::seconds(4);
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

  GeometryMapSnapshot Snapshot(std::size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshots_.at(index);
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
  std::unique_ptr<GeometryMapOutput> output_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<GeometryMapSnapshot> snapshots_;
  bool blocked_ = false;
  bool entered_ = false;
  int scan_ = 0;
};

TEST_F(GeometryMapOutputTest, DeltasCoalesceAndCaptureTheCommittedVersion) {
  Start(20, 8388608, {}, /*incremental=*/true);
  FeedTo(2);
  Optimize();
  FeedTo(3);
  ASSERT_TRUE(WaitForSize(1));
  const auto current = Snapshot(0);
  EXPECT_EQ(current.status, GeometryMapStatus::kReady);
  EXPECT_EQ(current.pose_version, 1u);
  EXPECT_EQ(current.frame_count, 3u);
  EXPECT_EQ(current.optimized_frame_count, 2u);
  ASSERT_EQ(current.sources.size(), 2u);
  EXPECT_EQ(current.sources[1].first_keyframe_id, 2u);
  EXPECT_GT(output_->GetStats().coalesced, 0u);
  system_->Finish();
  output_->Finish();
  ASSERT_EQ(output_->GetStats().delivered, 2u);
  EXPECT_EQ(Snapshot(1).sources.back().state, OnlineSubmapState::kSealed);
}

TEST_F(GeometryMapOutputTest, RejectsADeltaWithoutMatchingCommittedPoses) {
  Start(20, 8388608, {}, /*incremental=*/true);
  FeedTo(1);
  OnlineSubmapDelta delta;
  delta.frontier = {1,
                    0,
                    1,
                    1,
                    PoseFrame::kMap,
                    PoseTailPolicy::kProjectedLastOptimizedAnchor};
  output_->ObserveDelta(delta);
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kError);
  EXPECT_TRUE(Snapshot(0).sources.empty());
}

TEST_F(GeometryMapOutputTest, EmptyGrowthOptimizationAndFinalSeal) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kReady);
  EXPECT_TRUE(Snapshot(0).points.empty());
  FeedTo(3);
  ASSERT_TRUE(WaitForSize(2));
  const auto raw = Snapshot(1);
  ASSERT_EQ(raw.sources.size(), 2u);
  EXPECT_EQ(raw.frame_count, 3u);
  EXPECT_EQ(raw.sources[0].primary_keyframe_count, 2u);
  EXPECT_EQ(raw.sources[1].primary_keyframe_count, 1u);
  EXPECT_EQ(raw.sources[1].first_keyframe_id, 2u);
  Optimize();
  ASSERT_TRUE(WaitForSize(3));
  const auto corrected = Snapshot(2);
  EXPECT_EQ(corrected.pose_version, 1u);
  EXPECT_DOUBLE_EQ(corrected.timestamp, raw.timestamp);
  EXPECT_GT(corrected.revision, raw.revision);
  system_->Finish();
  output_->Finish();
  output_->Finish();
  ASSERT_EQ(output_->GetStats().delivered, 4u);
  const auto sealed = Snapshot(3);
  EXPECT_EQ(sealed.pose_version, corrected.pose_version);
  EXPECT_EQ(sealed.frame_count, corrected.frame_count);
  EXPECT_GT(sealed.revision, corrected.revision);
  EXPECT_EQ(sealed.sources.back().state, OnlineSubmapState::kSealed);
  EXPECT_TRUE(output_->GetStats().finished);
  EXPECT_EQ(output_->GetStats().last_optimized_frame_count, 3u);
  EXPECT_EQ(output_->GetStats().last_submap_count, 2u);
}

TEST_F(GeometryMapOutputTest, SlowConsumerDeliversOldThenLatest) {
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
  FeedTo(3);
  output_->Poll();
  Release();
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(0).frame_count, 0u);
  EXPECT_EQ(Snapshot(1).frame_count, 3u);
  EXPECT_GT(Snapshot(1).revision, Snapshot(0).revision + 1);
  EXPECT_EQ(output_->GetStats().peak_work_payloads, 1u);
  EXPECT_EQ(output_->GetStats().peak_pending, 1u);
  EXPECT_GT(output_->GetStats().coalesced, 0u);
}

TEST_F(GeometryMapOutputTest,
       FrameLimitClearsWithoutReadingAndDoesNotStopSystem) {
  Start(1);
  FeedTo(2);
  std::filesystem::remove_all(options_.exporter.directory /
                              "keyframes/frames/0");
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kResourceLimited);
  EXPECT_EQ(Snapshot(0).frame_count, 2u);
  EXPECT_TRUE(Snapshot(0).sources.empty());
  EXPECT_TRUE(Snapshot(0).points.empty());
  EXPECT_EQ(output_->GetStats().geometry.geometry.cloud_reads, 0u);
  EXPECT_NO_THROW(system_->Finish());
  EXPECT_NO_THROW(output_->Finish());
}

TEST_F(GeometryMapOutputTest, ByteLimitClearsWithoutReading) {
  Start(20, 4096);
  FeedTo(1);
  std::filesystem::remove_all(options_.exporter.directory /
                              "keyframes/frames/0");
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kResourceLimited);
  EXPECT_EQ(output_->GetStats().geometry.geometry.cloud_reads, 0u);
}

TEST_F(GeometryMapOutputTest, FinalSealAdvancesResourceLimitedRevision) {
  Start(1, 8388608, {}, /*incremental=*/true);
  FeedTo(3);
  ASSERT_TRUE(WaitForSize(1));
  const auto before = Snapshot(0);
  ASSERT_EQ(before.status, GeometryMapStatus::kResourceLimited);
  ASSERT_EQ(before.frame_count, 3u);
  ASSERT_EQ(before.submap_count, 2u);
  system_->Finish();
  output_->Finish();
  ASSERT_EQ(output_->GetStats().delivered, 2u);
  const auto after = Snapshot(1);
  EXPECT_EQ(after.status, GeometryMapStatus::kResourceLimited);
  EXPECT_EQ(after.frame_count, before.frame_count);
  EXPECT_EQ(after.submap_count, before.submap_count);
  EXPECT_GT(after.revision, before.revision);
}

TEST_F(GeometryMapOutputTest, FirstResourceStateBypassesFrequencyLimit) {
  Start(1);
  ASSERT_TRUE(WaitForSize(1));
  FeedTo(2);
  const auto start = Clock::now();
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, GeometryMapStatus::kResourceLimited);
  EXPECT_LT(Clock::now() - start, std::chrono::milliseconds(400));
}

TEST_F(GeometryMapOutputTest, InvalidMembershipIsReportedWithoutThrowing) {
  Start();
  FeedTo(1);
  ASSERT_TRUE(WaitForSize(1));
  OnlineSubmapUpdate bad;
  bad.poses = system_->GetPoseSnapshot();
  OnlineSubmap group;
  group.primary.id = 2;
  group.primary.keyframe_ids = {0};
  bad.revisions.push_back(group);
  EXPECT_NO_THROW(output_->Observe(bad));
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, GeometryMapStatus::kError);
  EXPECT_TRUE(Snapshot(1).sources.empty());
}

TEST_F(GeometryMapOutputTest, RejectsReadViewBehindThePoseFrontier) {
  Start();
  ASSERT_TRUE(WaitForSize(1));
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 1;
  poses->poses = {{0, {}}};
  OnlineSubmap group;
  group.primary.keyframe_ids = {0};
  OnlineSubmapUpdate update;
  update.poses = poses;
  update.revisions.push_back(group);
  std::weak_ptr<const PoseSnapshot> borrowed = poses;
  output_->Observe(update);
  update.poses.reset();
  poses.reset();
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, GeometryMapStatus::kError);
  EXPECT_EQ(Snapshot(1).frame_count, 1u);
  ASSERT_TRUE(Pump([&] { return borrowed.expired(); }));
}

TEST_F(GeometryMapOutputTest, OversizedInputIsNotRetained) {
  Start(1);
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 2;
  poses->poses = {{0, {}}, {1, {}}};
  std::weak_ptr<const PoseSnapshot> borrowed = poses;
  OnlineSubmapUpdate update;
  update.poses = poses;
  output_->Observe(update);
  update.poses.reset();
  poses.reset();
  EXPECT_TRUE(borrowed.expired());
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kResourceLimited);
}

TEST_F(GeometryMapOutputTest, DestructorDoesNotReadExpiredSystem) {
  Start();
  FeedTo(1);
  output_->Poll();
  system_->SetOnlineSubmapCallback({});
  system_->Finish();
  system_.reset();
  EXPECT_NO_THROW(output_.reset());
}

TEST_F(GeometryMapOutputTest, ReadFailureIsIsolated) {
  Start();
  FeedTo(1);
  std::filesystem::remove(options_.exporter.directory /
                          "keyframes/frames/0/frame.yml");
  ASSERT_TRUE(WaitForSize(1));
  EXPECT_EQ(Snapshot(0).status, GeometryMapStatus::kError);
  EXPECT_TRUE(Snapshot(0).points.empty());
  EXPECT_FALSE(Snapshot(0).error.empty());
  ASSERT_TRUE(Pump([&] { return output_->GetStats().failed == 1; }));
  FeedTo(2);
  EXPECT_NO_THROW(system_->Finish());
  EXPECT_NO_THROW(output_->Finish());
  EXPECT_EQ(output_->GetStats().delivered, 1u);
}

TEST_F(GeometryMapOutputTest, SinkFailureIsNotRetried) {
  std::atomic<int> calls{0};
  Start(20, 8388608, [&](const auto&) {
    ++calls;
    throw std::runtime_error("sink failed");
  });
  ASSERT_TRUE(Pump([&] { return output_->GetStats().failed == 1; }));
  FeedTo(1);
  system_->Finish();
  output_->Finish();
  EXPECT_EQ(calls.load(), 1);
}

TEST_F(GeometryMapOutputTest, RejectsAttachingAfterFirstFrame) {
  FeedTo(1);
  GeometryMapOutputOptions output_options;
  output_options.enabled = true;
  EXPECT_THROW(GeometryMapOutput(*system_, output_options, options_.geometry,
                                 [](const auto&) {}),
               std::invalid_argument);
}

TEST_F(GeometryMapOutputTest, SameVersionOldPoseCannotChange) {
  Start();
  FeedTo(1);
  ASSERT_TRUE(WaitForSize(1));
  auto changed = std::make_shared<PoseSnapshot>(*system_->GetPoseSnapshot());
  changed->poses[0].map_from_body.translation.x() += 1;
  OnlineSubmapUpdate notification;
  notification.poses = changed;
  EXPECT_NO_THROW(output_->Observe(notification));
  ASSERT_TRUE(WaitForSize(2));
  EXPECT_EQ(Snapshot(1).status, GeometryMapStatus::kError);
}

TEST_F(GeometryMapOutputTest, StopsWhileWorkerReturnsToIdle) {
  GeometryMapOutputOptions output_options;
  output_options.enabled = true;
  for (int i = 0; i < 2000; ++i) {
    GeometryMapOutput output(*system_, output_options, options_.geometry,
                             [](const auto&) {});
    output.Poll();
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (output.GetStats().delivered == 0 && Clock::now() < deadline)
      std::this_thread::yield();
    ASSERT_EQ(output.GetStats().delivered, 1u);
    if (i % 2 == 0) output.Finish();
  }
}

}  // namespace
}  // namespace livo_atlas
