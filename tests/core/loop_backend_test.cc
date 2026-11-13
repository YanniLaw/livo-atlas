#include "livo_atlas/core/loop_backend.h"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include "core/loop_backend_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

class Gate {
 public:
  void Enter(const LoopTask& task) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (entered_) return;
    entered_ = true;
    frontier_ = task.input.poses.frame_count;
    condition_.notify_all();
    condition_.wait(lock, [&] { return released_; });
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

  void TearDown() override { fs::remove_all(options_.exporter.directory); }

  void Append(std::uint64_t first, std::uint64_t end) {
    for (auto id = first; id < end; ++id)
      LoopBackendTestPeer::Append(*system_, loop_test::DenseFrame(id, cloud_));
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
    Append(id, id + 1);
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
  Append(4, 6);
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
