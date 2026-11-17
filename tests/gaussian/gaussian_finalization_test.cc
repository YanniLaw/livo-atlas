#include <gtest/gtest.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "core/loop_backend_fixtures.h"
#include "gaussian/submap_peer.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

namespace {
std::atomic<bool> fail_report_sync{false}, fail_catalog_sync{false};
std::string sync_directory;
}  // namespace

// Affect only this test's run, and only after the selected rename is visible.
extern "C" int fsync(int fd) {
  if (fail_report_sync || fail_catalog_sync) {
    char path[4096];
    const auto link = "/proc/self/fd/" + std::to_string(fd);
    const auto size = readlink(link.c_str(), path, sizeof(path));
    const bool visible =
        (fail_report_sync &&
         std::filesystem::exists(sync_directory + "/finalization.json")) ||
        (fail_catalog_sync &&
         std::filesystem::exists(sync_directory + "/map.json"));
    if (visible && size > 0 && std::string(path, size) == sync_directory) {
      errno = EIO;
      return -1;
    }
  }
  return static_cast<int>(syscall(SYS_fsync, fd));
}

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using namespace std::chrono_literals;

std::string Bytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}

class GaussianFinalizationTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("gs-final-" + std::to_string(getpid()) + "-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(root_);
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.frontend.calibration.camera = Camera();
    options.frontend.visual.grid_size = 1;
    options.frontend.visual.grid_rows = 1;
    options.exporter.directory = root_ / "dataset";
    options.keyframes.enabled = options.submaps.enabled = true;
    options.submap_mode = SubmapMode::kOnline;
    options.submaps.max_keyframes = 2;
    options.online_submaps.support_keyframes = 0;
    system_ = std::make_unique<LivoSystem>(options);
    options_.enabled = options_.persist_submaps = true;
    options_.socket_path = root_ / "peer.sock";
    options_.output_directory = root_ / "output";
    options_.connect_timeout = 2s;
    options_.job_timeout = 5s;
    options_.shutdown_timeout = 2s;
    peer_ = std::make_unique<test::SubmapPeer>(options_.socket_path);
    mapper_ =
        std::make_unique<OnlineGaussianMapper>(*system_, Camera(), options_);
    system_->SetOnlineSubmapCallback([&](const auto& update) {
      if (deliver_)
        mapper_->Observe(update);
      else
        delayed_.push_back(update);
    });
  }

  void TearDown() override {
    cancel_ = true;
    if (peer_)
      peer_->hold_candidate = peer_->hold_commit = peer_->hold_checkpoint =
          peer_->hold_map_ack = peer_->hold_empty = false;
    for (auto& thread : releases_)
      if (thread.joinable()) thread.join();
    fail_report_sync = fail_catalog_sync = false;
    mapper_.reset();
    peer_.reset();
    system_.reset();
    fs::remove_all(root_);
  }

  static CameraConfig Camera() {
    return {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
  }

  void Append(bool image = true) {
    Keyframe frame;
    frame.id = system_->GetKeyframeReadView().frame_count();
    frame.timestamp = frame.cloud_timestamp = 100 + frame.id;
    if (image) {
      frame.image_timestamp = frame.timestamp;
      frame.camera_from_odom = RigidTransform{};
      frame.image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(10, 20, 30));
    }
    PointCloud::Ptr cloud(new PointCloud);
    Point point{};
    point.z = 4;
    cloud->push_back(point);
    frame.cloud = cloud;
    // Existing test utility commits actual dataset files and core
    // notifications.
    LoopBackendTestPeer::Append(*system_, std::move(frame));
  }

  void Correct(bool deform = true, std::uint64_t member = 1,
               std::uint64_t optimized = 0) {
    const auto current = system_->GetPoseSnapshot();
    PoseUpdate update;
    update.base_version = current->version;
    update.optimized_frame_count = optimized ? optimized : current->frame_count;
    update.poses = current->poses;
    update.poses.resize(update.optimized_frame_count);
    update.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
    for (auto& pose : update.poses) pose.map_from_body.translation.y() += 1;
    if (deform) update.poses.at(member).map_from_body.translation.x() += .1;
    system_->ApplyPoseUpdate(update);
  }

  bool Wait(const std::function<bool()>& condition) {
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while (std::chrono::steady_clock::now() < deadline) {
      mapper_->Poll();
      if (condition()) return true;
      if (mapper_->GetStats().failed) {
        ADD_FAILURE() << mapper_->GetStats().error;
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    ADD_FAILURE() << "Timed out waiting for controlled peer";
    return false;
  }

  void Confirm() {
    const auto id = system_->GetKeyframeReadView().frame_count() - 1;
    ASSERT_TRUE(Wait([&] {
      const auto map = mapper_->GetMapSnapshot();
      for (const auto& entry : map->entries)
        if (!entry.frame_ids.empty() && entry.frame_ids.back() == id)
          return true;
      return false;
    }));
  }

  void Populate(int frames = 4) {
    for (int i = 0; i < frames; ++i) {
      Append();
      Confirm();
    }
  }

  void Release(std::atomic<bool>& hold, std::function<bool()> ready,
               std::chrono::milliseconds delay = 20ms) {
    releases_.emplace_back([&, ready, delay] {
      while (!cancel_ && !ready()) std::this_thread::sleep_for(1ms);
      if (!cancel_) std::this_thread::sleep_for(delay);
      hold = false;
    });
  }

  GaussianFinalizationResult Finalize(std::chrono::milliseconds timeout = 2s) {
    system_->Finish();
    return mapper_->Finalize(timeout);
  }

  void ExpectSuccess(const GaussianFinalizationResult& result,
                     std::size_t count) {
    EXPECT_EQ(result.status, GaussianFinalizationStatus::kSucceeded)
        << result.reason;
    EXPECT_TRUE(result.target_valid);
    EXPECT_TRUE(result.eligible_set_frozen);
    EXPECT_TRUE(result.report_persisted) << result.report_error;
    EXPECT_TRUE(result.worker_stopped);
    EXPECT_TRUE(result.durable);
    EXPECT_EQ(result.eligible_submap_ids.size(), count);
    EXPECT_EQ(result.current, count);
    EXPECT_EQ(result.needs_rebuild, 0u);
    EXPECT_EQ(result.dirty, 0u);
    EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
    EXPECT_TRUE(
        fs::is_regular_file(options_.output_directory / "finalization.json"));
  }

  fs::path root_;
  OnlineGaussianOptions options_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<test::SubmapPeer> peer_;
  std::unique_ptr<OnlineGaussianMapper> mapper_;
  bool deliver_ = true;
  std::vector<OnlineSubmapUpdate> delayed_;
  std::atomic<bool> cancel_{false};
  std::vector<std::thread> releases_;
};

TEST_F(GaussianFinalizationTest, RequiresCompletedSourceAndValidBudget) {
  Populate(1);
  EXPECT_EQ(mapper_->Finalize(2s).status, GaussianFinalizationStatus::kFailed);
  system_->SealInput();
  EXPECT_EQ(mapper_->Finalize(2s).status, GaussianFinalizationStatus::kFailed);
  system_->Finish();
  for (const auto budget : {0ms, -1ms, 600001ms})
    EXPECT_EQ(mapper_->Finalize(budget).status,
              GaussianFinalizationStatus::kFailed);
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  ExpectSuccess(mapper_->Finalize(2s), 1);
}

TEST_F(GaussianFinalizationTest, RepairsAllConfirmedBlocksAndFlushesLastDirty) {
  Populate(4);
  const auto raw = Bytes(root_ / "dataset/keyframes/frames/1/frame.yml");
  Correct();
  const auto result = Finalize();
  ExpectSuccess(result, 2);
  EXPECT_EQ(result.pose_version, 1u);
  EXPECT_EQ(result.after.rebuilds - result.before.rebuilds, 1u);
  EXPECT_EQ(result.repair_attempts, 1u);
  EXPECT_EQ(result.asset_ids.size(), 2u);
  EXPECT_EQ(Bytes(root_ / "dataset/keyframes/frames/1/frame.yml"), raw);
}

TEST_F(GaussianFinalizationTest, FinalPoseIncludesBackendTail) {
  Populate(4);
  Correct(true, 1, 3);
  const auto result = Finalize();
  ExpectSuccess(result, 2);
  EXPECT_EQ(result.frame_count, 4u);
  EXPECT_EQ(result.optimized_frame_count, 3u);
}

TEST_F(GaussianFinalizationTest, EmptyAndMissingStayExplicit) {
  const auto result = Finalize();
  ExpectSuccess(result, 0);
  EXPECT_EQ(result.missing, 0u);
  EXPECT_EQ(result.after.accepted, 0u);
}

TEST_F(GaussianFinalizationTest, MissingViewsAreNotInvented) {
  Append(false);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().skipped_no_image == 1; }));
  const auto result = Finalize();
  ExpectSuccess(result, 0);
  EXPECT_EQ(result.missing, 1u);
}

TEST_F(GaussianFinalizationTest, CapacityKeepsMissing) {
  Populate(64);
  Append();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().resource_limited > 0; }));
  const auto result = Finalize(5s);
  ExpectSuccess(result, 32);
  EXPECT_EQ(result.missing, 1u);
}

TEST_F(GaussianFinalizationTest, RepeatedFinalizeDoesNotRetrain) {
  Populate(2);
  Correct();
  const auto first = Finalize();
  ExpectSuccess(first, 1);
  const auto report = Bytes(options_.output_directory / "finalization.json");
  const auto second = mapper_->Finalize(1ms);
  EXPECT_EQ(second.map_sequence, first.map_sequence);
  EXPECT_EQ(second.after.accepted, first.after.accepted);
  EXPECT_EQ(Bytes(options_.output_directory / "finalization.json"), report);
  mapper_->Finish();
}

TEST_F(GaussianFinalizationTest, FastFinishStaysFast) {
  Populate(2);
  Correct();
  system_->Finish();
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().rebuilds, 0u);
  EXPECT_FALSE(fs::exists(options_.output_directory / "finalization.json"));
  EXPECT_EQ(mapper_->Finalize(2s).reason, "already_finished");
}

TEST_F(GaussianFinalizationTest, PendingCandidateIsNotPromoted) {
  peer_->hold_candidate = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->views == 1; }));
  Release(peer_->hold_candidate, [] { return true; });
  const auto result = Finalize();
  ExpectSuccess(result, 0);
  EXPECT_EQ(result.missing, 1u);
  EXPECT_EQ(peer_->discards, 1);
}

TEST_F(GaussianFinalizationTest, AlreadyAcceptedCommitJoinsEligibleSet) {
  peer_->hold_commit = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->decisions == 1; }));
  Release(peer_->hold_commit, [] { return true; });
  const auto result = Finalize();
  ExpectSuccess(result, 1);
  EXPECT_EQ(result.after.accepted, 1u);
}

TEST_F(GaussianFinalizationTest,
       RejectedForegroundCannotBlockHistoricalFinalization) {
  Populate(2);
  Correct();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().rebuilds == 1; }));
  peer_->hold_candidate = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->views == 3; }));
  Correct();
  Release(peer_->hold_candidate, [] { return true; });
  const auto result = Finalize();
  ExpectSuccess(result, 1);
  EXPECT_EQ(result.missing, 1u);
  EXPECT_EQ(result.repair_attempts, 1u);
}

TEST_F(GaussianFinalizationTest, EntryDuringCheckpointAndAck) {
  Populate(2);
  peer_->hold_checkpoint = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->checkpoints == 1; }));
  peer_->hold_map_ack = true;
  Release(peer_->hold_checkpoint, [] { return true; });
  const auto prior_events = peer_->Events();
  const auto prior =
      std::count(prior_events.begin(), prior_events.end(), "map_committed:");
  Release(peer_->hold_map_ack, [&, prior] {
    const auto events = peer_->Events();
    return std::count(events.begin(), events.end(), "map_committed:") > prior;
  });
  const auto result = Finalize();
  ExpectSuccess(result, 1);
  EXPECT_GE(peer_->acknowledgements, 1);
  EXPECT_EQ(result.missing, 1u);
}

TEST_F(GaussianFinalizationTest, FrozenTargetMismatchCannotSucceed) {
  deliver_ = false;
  Append();
  system_->Finish();
  const auto invalid = mapper_->Finalize(2s);
  EXPECT_EQ(invalid.status, GaussianFinalizationStatus::kFailed);
  EXPECT_FALSE(invalid.target_valid);
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  for (const auto& update : delayed_) mapper_->Observe(update);
  ExpectSuccess(mapper_->Finalize(2s), 0);
}

TEST_F(GaussianFinalizationTest, RigidContentDoesNotRewriteArrays) {
  Populate(2);
  Correct(false);
  const auto result = Finalize();
  ExpectSuccess(result, 1);
  EXPECT_EQ(result.after.rebuilds, result.before.rebuilds);
  EXPECT_EQ(result.after.accepted, result.before.accepted);
  EXPECT_EQ(result.pose_version, 1u);
}

TEST_F(GaussianFinalizationTest, QuiescingTimeoutDoesNotFreezeEmptySet) {
  peer_->hold_candidate = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->views == 1; }));
  Release(
      peer_->hold_candidate, [] { return true; }, 200ms);
  const auto result = Finalize(50ms);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut)
      << result.reason;
  EXPECT_FALSE(result.eligible_set_frozen);
  EXPECT_EQ(result.current, 0u);
  EXPECT_TRUE(result.durable);
}

TEST_F(GaussianFinalizationTest, CandidateAfterDeadlineIsRejected) {
  Populate(2);
  Correct();
  peer_->hold_candidate = true;
  Release(
      peer_->hold_candidate, [&] { return peer_->rebuilds > 0; }, 300ms);
  const auto result = Finalize(150ms);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut)
      << result.reason;
  EXPECT_EQ(result.needs_rebuild, 1u);
  EXPECT_GT(peer_->discards, 0);
  EXPECT_TRUE(result.durable);
}

TEST_F(GaussianFinalizationTest, LateAcceptedCommitStaysTimedOut) {
  Populate(1);
  peer_->hold_commit = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->decisions == 2; }));
  Release(
      peer_->hold_commit, [] { return true; }, 200ms);
  const auto result = Finalize(50ms);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut);
  EXPECT_FALSE(result.eligible_set_frozen);
  EXPECT_EQ(result.after.accepted, 2u);
  EXPECT_TRUE(result.durable);
}

TEST_F(GaussianFinalizationTest, CleanupMayOutliveRepairBudget) {
  Populate(1);
  peer_->hold_checkpoint = true;
  Release(
      peer_->hold_checkpoint, [&] { return peer_->checkpoints > 0; }, 400ms);
  const auto result = Finalize(200ms);
  ExpectSuccess(result, 1);
  EXPECT_LT(result.repair_seconds, .2);
  EXPECT_GE(result.shutdown_seconds, .35);
}

TEST_F(GaussianFinalizationTest, CompletionCheckAfterDeadlineCannotSucceed) {
  if (!std::getenv("LIVO_TEST_FINALIZATION_DEADLINE"))
    GTEST_SKIP()
        << "Run finalization_deadline_test.py for controlled descheduling";
  Populate(1);
  const auto result = Finalize(200ms);
  // The external debugger pauses at the completion check, after the loop's
  // initial time check. The pause must occur before cleanup is admitted.
  EXPECT_GE(result.repair_seconds, .3);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut);
  EXPECT_TRUE(result.eligible_set_frozen);
  EXPECT_TRUE(result.durable);
  EXPECT_TRUE(result.worker_stopped);
  EXPECT_TRUE(result.report_persisted) << result.report_error;
  EXPECT_EQ(result.current, 1u);
}

TEST_F(GaussianFinalizationTest, RebuildDispatchAfterDeadlineIsSkipped) {
  if (!std::getenv("LIVO_TEST_FINALIZATION_DEADLINE"))
    GTEST_SKIP()
        << "Run finalization_deadline_test.py for controlled descheduling";
  Populate(2);
  Correct();
  const auto result = Finalize(200ms);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut);
  EXPECT_TRUE(result.durable);
  EXPECT_TRUE(result.worker_stopped);
  EXPECT_EQ(result.repair_attempts, 0u);
  EXPECT_EQ(peer_->rebuilds, 0);
  EXPECT_EQ(result.needs_rebuild, 1u);
}

TEST_F(GaussianFinalizationTest, CandidateValidationAfterDeadlineIsRejected) {
  if (!std::getenv("LIVO_TEST_FINALIZATION_DEADLINE"))
    GTEST_SKIP()
        << "Run finalization_deadline_test.py for controlled descheduling";
  Populate(2);
  Correct();
  const auto result = Finalize(200ms);
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kTimedOut);
  EXPECT_TRUE(result.durable);
  EXPECT_TRUE(result.worker_stopped);
  EXPECT_EQ(result.after.accepted, result.before.accepted);
  EXPECT_EQ(result.needs_rebuild, 1u);
  EXPECT_EQ(peer_->discards, 1);
}

TEST_F(GaussianFinalizationTest, WrongStopCannotSucceed) {
  Populate(1);
  peer_->wrong_stop = true;
  const auto result = Finalize();
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kFailed);
  EXPECT_FALSE(result.worker_stopped);
  EXPECT_TRUE(mapper_->GetStats().incomplete_shutdown);
}

TEST_F(GaussianFinalizationTest, UncertainMapCannotSucceed) {
  Populate(1);
  sync_directory = options_.output_directory.string();
  fail_catalog_sync = true;
  const auto result = Finalize();
  fail_catalog_sync = false;
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kFailed);
  EXPECT_FALSE(result.durable);
  EXPECT_EQ(peer_->prunes, 0);
}

TEST_F(GaussianFinalizationTest, ReportFailureKeepsActualConvergence) {
  Populate(1);
  fs::create_directory(options_.output_directory / "finalization.tmp.json");
  const auto result = Finalize();
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kSucceeded);
  EXPECT_FALSE(result.report_persisted);
  EXPECT_FALSE(result.report_error.empty());
  EXPECT_TRUE(result.durable);
  EXPECT_TRUE(result.worker_stopped);
}

TEST_F(GaussianFinalizationTest, VisibleReportDoesNotCertifyItsDirectorySync) {
  Populate(1);
  sync_directory = options_.output_directory.string();
  fail_report_sync = true;
  const auto result = Finalize();
  fail_report_sync = false;
  EXPECT_EQ(result.status, GaussianFinalizationStatus::kSucceeded);
  EXPECT_FALSE(result.report_persisted);
  EXPECT_EQ(result.report_error, "commit_uncertain");
  EXPECT_TRUE(
      fs::is_regular_file(options_.output_directory / "finalization.json"));
  EXPECT_TRUE(result.durable);
}

TEST_F(GaussianFinalizationTest, RealPythonWorkerFinalizesCompletedSubmaps) {
  const char* socket = std::getenv("LIVO_REAL_GAUSSIAN_SOCKET");
  const char* output = std::getenv("LIVO_REAL_GAUSSIAN_OUTPUT");
  if (!socket || !output)
    GTEST_SKIP() << "Run separately with the actual CUDA worker";
  mapper_.reset();
  peer_.reset();
  options_.socket_path = socket;
  options_.output_directory = output;
  options_.job_timeout = 30s;
  options_.shutdown_timeout = 10s;
  mapper_ =
      std::make_unique<OnlineGaussianMapper>(*system_, Camera(), options_);
  Populate(4);
  Correct();
  const auto result = Finalize(20s);
  ExpectSuccess(result, 2);
  EXPECT_GE(result.after.rebuilds - result.before.rebuilds, 1u);
}

}  // namespace
}  // namespace livo_atlas
