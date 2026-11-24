#include "livo_atlas/core/loop_backend.h"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "core/loop_backend_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

class Gate {
 public:
  bool Enter(const LoopTask& task) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (entered_) return true;
    entered_ = true;
    frontier_ = task.input.poses.frame_count;
    condition_.notify_all();
    return condition_.wait_for(lock, std::chrono::seconds(5),
                               [&] { return released_; });
  }

  bool Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::seconds(5),
                               [&] { return entered_; });
  }

  void Release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  std::uint64_t frontier() const { return frontier_; }

 private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false, released_ = false;
  std::uint64_t frontier_ = 0;
};

struct ReleaseOnExit {
  Gate& gate;

  ~ReleaseOnExit() { gate.Release(); }
};

class LoopBackendTest : public testing::Test {
 protected:
  void SetUp() override {
    options_ = loop_test::OnlineOptions();
    system_ = std::make_unique<LivoSystem>(options_);
    cloud_ = loop_test::DenseCloud();
  }

  void TearDown() override {
    system_.reset();
    fs::remove_all(options_.exporter.directory);
  }

  void Append(std::uint64_t first, std::uint64_t end) {
    Enqueue(first, end);
    LoopBackendTestPeer::FlushKeyframes(*system_);
  }

  void Enqueue(std::uint64_t first, std::uint64_t end) {
    for (auto id = first; id < end; ++id)
      LoopBackendTestPeer::Enqueue(*system_, loop_test::DenseFrame(id, cloud_));
  }

  void Observe(LoopBackend& backend) {
    system_->SetOnlineSubmapCallback(
        [&backend](const auto& update) { backend.Observe(update); });
  }

  bool Pump(LoopBackend& backend) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!backend.GetStats().completed &&
           std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(system_->ProcessNext({}), SyncStatus::kWaiting);
      backend.Poll();
      std::this_thread::yield();
    }
    return backend.GetStats().completed > 0;
  }

  fs::path Dataset() { return options_.exporter.directory / "keyframes"; }

  SystemOptions options_;
  PointCloud::ConstPtr cloud_;
  std::unique_ptr<LivoSystem> system_;
};

TEST_F(LoopBackendTest, LatestOnlyButCompleteMetadata) {
  Gate gate;
  const auto options = loop_test::NearbyOptions();
  auto backend =
      LoopBackendTestPeer::Create(*system_, options, [&](const auto& task) {
        gate.Enter(task);
        return RunLoopTask(task, options);
      });
  ReleaseOnExit release{gate};
  Observe(*backend);
  Append(0, 4);
  backend->Poll();
  ASSERT_TRUE(gate.Wait());
  EXPECT_EQ(gate.frontier(), 4u);
  for (std::uint64_t id = 4; id < 8; ++id) {
    Enqueue(id, id + 1);
    backend->Poll();
  }
  gate.Release();
  ASSERT_TRUE(Pump(*backend));
  EXPECT_GE(system_->GetPoseSnapshot()->version, 1u);
  EXPECT_EQ(system_->GetPoseSnapshot()->frame_count, 8u);
  backend->Finish();
  const auto stats = backend->GetStats();
  EXPECT_EQ(stats.frame_count, 8u);
  EXPECT_EQ(stats.peak_pending, 1u);
  EXPECT_EQ(stats.peak_work_payloads, 1u);
  EXPECT_GT(stats.skipped, 0u);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, WaitingPumpAcceptsResult) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  Observe(backend);
  Append(0, 4);
  backend.Poll();
  ASSERT_TRUE(Pump(backend));
  EXPECT_EQ(backend.GetStats().accepted, 1u);
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 1u);
  backend.Finish();
}

TEST_F(LoopBackendTest, FullQueueCanDeliverAResultAndSealInput) {
  Gate gate;
  ReleaseOnExit release{gate};
  auto options = loop_test::NearbyOptions();
  options.max_pending_keyframes = 4;
  LoopBackendTestPeer::InstallManaged(
      *system_, options, [&](const auto& task) {
        gate.Enter(task);
        return RunLoopTask(task, options);
      });
  Append(0, 4);
  EXPECT_EQ(system_->ProcessNext({}), SyncStatus::kWaiting);
  ASSERT_TRUE(gate.Wait());
  EXPECT_EQ(gate.frontier(), 4u);
  Enqueue(4, 8);
  EXPECT_EQ(system_->ProcessNext({}), SyncStatus::kWaiting);
  EXPECT_EQ(system_->GetLoopStats()->peak_queued_keyframes, 4u);
  gate.Release();
  system_->SealInput();
  system_->Finish();

  EXPECT_EQ(system_->GetLoopStats()->frame_count, 8u);
  EXPECT_GT(system_->GetLoopStats()->accepted, 0u);
  EXPECT_GT(system_->GetLoopStats()->skipped, 0u);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, OverflowPreservesOnlyTheCommittedPrefix) {
  Gate gate;
  ReleaseOnExit release{gate};
  auto options = loop_test::NearbyOptions();
  options.max_pending_keyframes = 4;
  LoopBackendTestPeer::InstallManaged(*system_, options, [&](const auto& task) {
    gate.Enter(task);
    return RunLoopTask(task, options);
  });
  Append(0, 4);
  EXPECT_EQ(system_->ProcessNext({}), SyncStatus::kWaiting);
  ASSERT_TRUE(gate.Wait());
  Enqueue(4, 8);
  EXPECT_THROW(Enqueue(8, 9), std::overflow_error);
  EXPECT_EQ(system_->GetKeyframeReadView().frame_count(), 4u);
  EXPECT_EQ(system_->GetPoseFrontier().frame_count, 4u);
  EXPECT_FALSE(fs::exists(Dataset() / "frames/4"));
  EXPECT_FALSE(fs::exists(Dataset() / "frames/8"));
  EXPECT_THROW(system_->ProcessNext({}), std::overflow_error);
  gate.Release();
  EXPECT_THROW(system_->Finish(), std::overflow_error);

  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
      << testing::PrintToString(report.issues);
  EXPECT_EQ(report.committed_version, 0u);
}

TEST_F(LoopBackendTest, FrontendContinuesWhileArchiveWriterIsBusy) {
  const auto gate = std::make_shared<Gate>();
  ReleaseOnExit release{*gate};
  const auto options = loop_test::NearbyOptions();
  LoopBackendTestPeer::InstallManaged(
      *system_, options, [gate, options](const auto& task) {
        gate->Enter(task);
        return RunLoopTask(task, options);
      });
  const auto owner = std::this_thread::get_id();
  system_->SetOnlineSubmapCallback([this, owner](const auto& update) {
    EXPECT_EQ(std::this_thread::get_id(), owner);
    EXPECT_EQ(system_->GetKeyframeReadView().frame_count(),
              update.poses->frame_count);
    EXPECT_EQ(system_->GetMapCorrection().pose_version, update.poses->version);
  });
  Append(0, 4);
  ASSERT_TRUE(gate->Wait());
  const auto before = system_->GetKeyframeReadView();
  int estimates = 0;
  const auto collect = [&](const auto&, const auto&, double) { ++estimates; };
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(system_->AddLidar(test::Scan(100 + .25 * i)));
    while (system_->ProcessNext(collect) != SyncStatus::kWaiting) {
    }
  }
  EXPECT_GT(estimates, 0);
  EXPECT_EQ(system_->GetPoseFrontier().frame_count, 4u);
  EXPECT_FALSE(fs::exists(Dataset() / "frames/4"));

  gate->Release();
  system_->Finish();
  EXPECT_GT(system_->GetKeyframeReadView().frame_count(), 4u);
  EXPECT_THROW(before.ReadTimestamp(4), std::out_of_range);
}

TEST_F(LoopBackendTest, BackgroundPoolWritesWhileLoopQueryIsBlocked) {
  for (const int threads : {4, 6}) {
    SCOPED_TRACE(threads);
    auto config = options_;
    config.exporter.directory /= "threads-" + std::to_string(threads);
    config.num_background_threads = threads;
    const auto gate = std::make_shared<Gate>();
    LivoSystem system(config);
    ReleaseOnExit release{*gate};
    const auto loop = loop_test::NearbyOptions();
    LoopBackendTestPeer::InstallManaged(
        system, loop, [gate, loop](const LoopTask& task) {
          EXPECT_TRUE(gate->Enter(task))
              << "Archive work did not progress while the loop was blocked";
          return RunLoopTask(task, loop);
        });
    for (std::uint64_t id = 0; id < 4; ++id)
      LoopBackendTestPeer::Enqueue(system, loop_test::DenseFrame(id, cloud_));
    LoopBackendTestPeer::FlushKeyframes(system);
    ASSERT_TRUE(gate->Wait());
    ASSERT_EQ(gate->frontier(), 4u);

    // The loop task holds one pool worker. The serial writer uses another.
    LoopBackendTestPeer::Append(system, loop_test::DenseFrame(4, cloud_));
    EXPECT_EQ(system.GetKeyframeReadView().frame_count(), 5u);
    EXPECT_NO_THROW(system.SealInput());
    gate->Release();
    system.Finish();
    EXPECT_TRUE(system.IsFinished());
    const auto report =
        InspectLoopDataset(config.exporter.directory / "keyframes");
    EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent);
  }
}

TEST(LoopBackendOptionsTest, RejectsInvalidThreadCountsBeforeIo) {
  EXPECT_EQ(SystemOptions{}.num_background_threads, 4);
  for (const int threads : {0, -1}) {
    auto options = loop_test::OnlineOptions();
    options.num_background_threads = threads;
    EXPECT_THROW(LivoSystem{options}, std::invalid_argument);
    EXPECT_FALSE(fs::exists(options.exporter.directory));
  }
}

TEST_F(LoopBackendTest, DetectionOnlyFinishesWithoutPublishingAPoseUpdate) {
  auto options = loop_test::NearbyOptions();
  options.pose_graph_enabled = false;
  LoopBackend backend(*system_, options);
  Observe(backend);
  Append(0, 4);
  backend.Poll();
  ASSERT_TRUE(Pump(backend));
  backend.Finish();

  const auto stats = backend.GetStats();
  EXPECT_GT(stats.detected, 0u);
  EXPECT_EQ(stats.accepted, 0u);
  EXPECT_EQ(stats.completed, stats.detected + stats.rejected);
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 0u);
  EXPECT_TRUE(fs::is_empty(Dataset() / "loop_closure" / "prepared"));
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, CrossedBudgetDoesNotCommit) {
  Gate gate;
  auto options = loop_test::NearbyOptions();
  options.max_keyframes = 4;
  auto backend =
      LoopBackendTestPeer::Create(*system_, options, [&](const auto& task) {
        gate.Enter(task);
        return RunLoopTask(task, options);
      });
  ReleaseOnExit release{gate};
  Observe(*backend);
  Append(0, 4);
  backend->Poll();
  ASSERT_TRUE(gate.Wait());
  Enqueue(4, 6);
  gate.Release();
  backend->Finish();
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 0u);
  EXPECT_EQ(backend->GetStats().submitted, 1u);
  EXPECT_GE(backend->GetStats().resource_limited, 2u);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, TrueProposalCommitsBeforeFinalFinish) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  int notifications = 0;
  system_->SetOnlineSubmapCallback([&](const auto& update) {
    backend.Observe(update);
    if (!update.pose_changes.empty()) {
      ++notifications;
      EXPECT_TRUE(fs::is_regular_file(
          Dataset() / "loop_closure/prepared" /
          (std::to_string(update.poses->version) + ".yml")));
    }
  });
  Append(0, 4);
  backend.Finish();
  backend.Finish();
  EXPECT_GE(backend.GetStats().accepted, 1u);
  EXPECT_EQ(notifications, backend.GetStats().accepted);
  EXPECT_EQ(system_->GetPoseSnapshot()->version, backend.GetStats().accepted);
  EXPECT_THROW(system_->ProcessNext({}), std::logic_error);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, LiveStandaloneBackendOwnsFinalization) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  Observe(backend);
  Append(0, 4);
  EXPECT_THROW(system_->Finish(), std::logic_error);
  EXPECT_FALSE(system_->IsFinished());
  backend.Finish();
  EXPECT_TRUE(system_->IsFinished());
  EXPECT_GT(backend.GetStats().accepted, 0u);
  EXPECT_NO_THROW(system_->Finish());
}

TEST_F(LoopBackendTest, SystemOwnsBackendWithOrWithoutAnObserver) {
  auto options = options_;
  options.exporter.directory /= "managed";
  options.loop_closure = loop_test::NearbyOptions();
  LivoSystem system(options);
  ASSERT_TRUE(system.GetLoopStats());
  EXPECT_THROW(system.ApplyPoseUpdate({}), std::logic_error);
  int keyframes = 0;
  system.SetOnlineSubmapCallback([&](const auto& update) {
    if (update.keyframe) ++keyframes;
  });
  for (std::uint64_t id = 0; id < 4; ++id) {
    LoopBackendTestPeer::Append(system, loop_test::DenseFrame(id, cloud_));
  }
  EXPECT_EQ(keyframes, 4);
  EXPECT_EQ(system.ProcessNext({}), SyncStatus::kWaiting);
  system.SetOnlineSubmapCallback({});
  system.Finish();
  system.Finish();
  EXPECT_TRUE(system.IsFinished());
  const auto stats = system.GetLoopStats();
  EXPECT_GT(stats->submitted, 0u);
  EXPECT_GE(stats->accepted, 1u);
  EXPECT_EQ(system.GetPoseSnapshot()->version, stats->accepted);
  const auto report = InspectLoopDataset(options.exporter.directory / "keyframes");
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopBackendTest, LateBackendFailureInvalidatesCompletion) {
  system_->Finish();
  ASSERT_TRUE(system_->IsFinished());
  LoopBackendTestPeer::FailCompletedSystem(*system_);
  EXPECT_FALSE(system_->IsFinished());
  EXPECT_THROW(system_->Finish(), std::runtime_error);
}

TEST_F(LoopBackendTest, PostPublicationFailureIsCommittedAndTerminal) {
  for (int variant = 0; variant < 2; ++variant) {
    auto local_options = options_;
    local_options.exporter.directory /= std::to_string(variant);
    LivoSystem system(local_options);
    LoopBackend backend(system, loop_test::NearbyOptions());
    const auto dataset = local_options.exporter.directory / "keyframes";
    system.SetOnlineSubmapCallback([&](const auto& update) {
      backend.Observe(update);
      if (!update.pose_changes.empty()) {
        if (variant == 0)
          throw std::runtime_error("notification after publication");
        fs::create_directory(dataset / "loop_closure/queries/.pending-1.yml");
      }
    });
    for (std::uint64_t id = 0; id < 4; ++id)
      LoopBackendTestPeer::Append(system, loop_test::DenseFrame(id, cloud_));
    backend.Poll();
    std::string error;
    try {
      backend.Finish();
    } catch (const std::exception& e) {
      error = e.what();
    }
    ASSERT_FALSE(error.empty());
    EXPECT_EQ(system.GetPoseSnapshot()->version, 1u);
    if (variant == 1) {
      EXPECT_EQ(backend.GetStats().accepted, 1u);
      EXPECT_EQ(backend.GetStats().completed, 1u);
    }
    const auto report = InspectLoopDataset(dataset);
    EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
        << testing::PrintToString(report.issues);
    EXPECT_EQ(report.committed_version, 1u);
    for (int retry = 0; retry < 3; ++retry) {
      try {
        if (retry == 0) backend.Finish();
        if (retry == 1) backend.Poll();
        if (retry == 2) system.SealInput();
        FAIL() << "Failed session continued";
      } catch (const std::exception& e) {
        EXPECT_EQ(e.what(), error);
      }
    }
  }
}

TEST_F(LoopBackendTest, ExclusivePoseWriterAndObserverFailure) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  EXPECT_THROW(system_->ApplyPoseUpdate({}), std::logic_error);
  EXPECT_THROW(LoopBackend(*system_, loop_test::NearbyOptions()),
               std::logic_error);
  EXPECT_NO_THROW(backend.Observe({}));
  EXPECT_THROW(backend.Poll(), std::invalid_argument);
  EXPECT_THROW(system_->AddLidar(test::Scan(100)), std::invalid_argument);
  EXPECT_THROW(backend.Finish(), std::invalid_argument);
}

TEST_F(LoopBackendTest, StandalonePollStopsAfterSystemOverflow) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  for (int scan = 0; scan < 64; ++scan) {
    ASSERT_TRUE(system_->AddLidar(test::Scan(100)));
  }
  ASSERT_THROW(system_->AddLidar(test::Scan(100)), std::overflow_error);
  EXPECT_THROW(backend.Poll(), std::overflow_error);
  EXPECT_THROW(backend.Finish(), std::overflow_error);
  EXPECT_FALSE(system_->IsFinished());
  EXPECT_TRUE(fs::exists(Dataset() / "loop_closure/failure.yml"));
}

TEST_F(LoopBackendTest, UnknownWorkerExceptionIsPreserved) {
  auto backend = LoopBackendTestPeer::Create(
      *system_, loop_test::NearbyOptions(),
      [](const auto&) -> LoopAdmissionResult { throw 29; });
  Observe(*backend);
  Append(0, 4);
  backend->Poll();
  try {
    backend->Finish();
    FAIL();
  } catch (int value) {
    EXPECT_EQ(value, 29);
  }
  EXPECT_THROW(system_->Finish(), int);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete);
  EXPECT_EQ(report.committed_version, 0u);
}

TEST_F(LoopBackendTest, DestructorNeverCommits) {
  {
    LoopBackend backend(*system_, loop_test::NearbyOptions());
    Observe(backend);
    Append(0, 4);
    backend.Poll();
    system_->SetOnlineSubmapCallback({});
  }
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 0u);
  system_->Finish();
  EXPECT_THROW(system_->ApplyPoseUpdate({}), std::logic_error);
  EXPECT_EQ(InspectLoopDataset(Dataset()).status,
            LoopRecoveryStatus::kIncomplete);
}

TEST_F(LoopBackendTest, RejectsMalformedRunnerNoiseBeforePreparing) {
  const auto options = loop_test::NearbyOptions();
  auto backend = LoopBackendTestPeer::Create(
      *system_, options, [options](const auto& task) {
        auto result = RunLoopTask(task, options);
        if (result.proposal)
          result.proposal->loop.constraint.translation_sigma_m = .3;
        return result;
      });
  Observe(*backend);
  Append(0, 4);
  backend->Poll();
  EXPECT_THROW(backend->Finish(), std::invalid_argument);
  EXPECT_EQ(system_->GetPoseSnapshot()->version, 0u);
  EXPECT_FALSE(fs::exists(Dataset() / "loop_closure/prepared/1.yml"));
}

TEST_F(LoopBackendTest, MissingObserverCannotProduceACompleteSession) {
  LoopBackend backend(*system_, loop_test::NearbyOptions());
  Append(0, 4);
  EXPECT_THROW(backend.Finish(), std::invalid_argument);
  EXPECT_THROW(system_->ProcessNext({}), std::invalid_argument);
  EXPECT_EQ(InspectLoopDataset(Dataset()).status,
            LoopRecoveryStatus::kIncomplete);
}

TEST_F(LoopBackendTest, PreparationAndPoseWriteFailuresLeaveNoFalseCommit) {
  for (int variant = 0; variant < 2; ++variant) {
    auto local = options_;
    local.exporter.directory /= "before-pose-" + std::to_string(variant);
    LivoSystem system(local);
    LoopBackend backend(system, loop_test::NearbyOptions());
    system.SetOnlineSubmapCallback(
        [&](const auto& update) { backend.Observe(update); });
    const auto dataset = local.exporter.directory / "keyframes";
    const auto folder = variant == 0 ? dataset / "loop_closure/prepared"
                                     : dataset / "online_submaps/poses";
    fs::create_directory(folder / ".pending-1.yml");
    for (std::uint64_t id = 0; id < 4; ++id)
      LoopBackendTestPeer::Append(system, loop_test::DenseFrame(id, cloud_));
    backend.Poll();
    EXPECT_THROW(backend.Finish(), std::exception);
    EXPECT_EQ(system.GetPoseSnapshot()->version, 0u);
    const auto report = InspectLoopDataset(dataset);
    EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
        << testing::PrintToString(report.issues);
    EXPECT_EQ(report.committed_version, 0u);
    EXPECT_EQ(report.uncommitted_prepared.size(),
              static_cast<std::size_t>(variant));
  }
}

TEST_F(LoopBackendTest, AcceptedLoopBudgetStopsDispatchWithoutStoppingInput) {
  auto options = loop_test::NearbyOptions();
  options.max_loops = 1;
  LoopBackend backend(*system_, options);
  Observe(backend);
  Append(0, 4);
  backend.Poll();
  ASSERT_TRUE(Pump(backend));
  Append(4, 7);
  backend.Poll();
  backend.Finish();
  EXPECT_EQ(backend.GetStats().accepted, 1u);
  EXPECT_EQ(backend.GetStats().submitted, 1u);
  EXPECT_EQ(backend.GetStats().frame_count, 7u);
  const auto report = InspectLoopDataset(Dataset());
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}
}  // namespace
}  // namespace livo_atlas
