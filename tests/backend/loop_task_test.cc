#include "livo_atlas/backend/loop_task.h"

#include <gtest/gtest.h>

#if defined(__unix__)
#include <unistd.h>
#endif

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <future>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "backend/loop_dataset_fixtures.h"
#include "common/thread_pool.h"
#include "loop_closure/loop_query.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::map<std::string, std::string> Bytes(const fs::path& root) {
  std::map<std::string, std::string> result;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream stream(entry.path(), std::ios::binary);
    result.emplace(fs::relative(entry.path(), root).string(),
                   std::string(std::istreambuf_iterator<char>(stream), {}));
  }
  return result;
}

class LoopTaskTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("c2b-loop-task-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
    frames_ = std::make_unique<KeyframeStore>(root_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    store_ = std::make_unique<OnlineSubmapStore>(root_ / "online_submaps",
                                                 *frames_, 1, options);
    builder_ = std::make_unique<OnlineSubmapBuilder>(1, options);
    cloud_ = loop_test::DenseCloud();
    for (std::uint64_t id = 0; id < 4; ++id) Append(id);
  }

  void TearDown() override { fs::remove_all(root_); }

  void Append(std::uint64_t id) {
    auto frame = loop_test::DenseFrame(id, cloud_);
    frames_->Append(frame);
    for (const auto& group :
         builder_->AddKeyframe(id, frame.timestamp, frame.odom_from_body))
      store_->AppendRevision(group);
  }

  LoopTask Task(std::uint64_t count = 4) {
    auto view = frames_->CaptureReadView();
    LoopAdmissionInput input;
    input.query_id = 2;
    input.poses = store_->MaterializePoseSnapshot(0, count);
    for (const auto& pose : input.poses.poses)
      input.raw_poses.push_back(pose.map_from_body);
    for (std::uint64_t id = 0; id + 1 < count; ++id) {
      auto group = store_->Read(id);
      input.submaps.push_back({group, 10.0 + id, 10.0 + id,
                               input.poses.poses[id].map_from_body, 0, count,
                               0});
    }
    std::vector<double> timestamps;
    for (std::uint64_t id = 0; id < count; ++id) timestamps.push_back(10 + id);
    return {5, std::move(input), std::move(timestamps), std::move(view)};
  }

  fs::path root_;
  PointCloud::ConstPtr cloud_;
  std::unique_ptr<KeyframeStore> frames_;
  std::unique_ptr<OnlineSubmapStore> store_;
  std::unique_ptr<OnlineSubmapBuilder> builder_;
};

// Drive native Tasks in a chosen order, without depending on OS scheduling.
class ManualThreadPool : public ThreadPoolInterface {
 public:
  std::weak_ptr<livo_atlas::Task> Schedule(
      std::unique_ptr<livo_atlas::Task> task) override {
    if (++schedule_calls_ == failure_call_) throw std::bad_alloc();
    auto shared = std::shared_ptr<livo_atlas::Task>(std::move(task));
    waiting_.emplace(shared.get(), shared);
    SetThreadPool(shared.get());
    return shared;
  }

  std::size_t ready_count() const { return ready_.size(); }

  std::size_t waiting_count() const { return waiting_.size(); }

  void FailAt(std::size_t call) { failure_call_ = call; }

  void RunLastReady() {
    auto task = std::move(ready_.back());
    ready_.pop_back();
    Execute(task.get());
  }

 private:
  void NotifyDependenciesCompleted(livo_atlas::Task* task) override {
    ready_.push_back(std::move(waiting_.at(task)));
    waiting_.erase(task);
  }

  std::size_t schedule_calls_ = 0;
  std::size_t failure_call_ = 0;
  std::map<livo_atlas::Task*, std::shared_ptr<livo_atlas::Task>> waiting_;
  std::vector<std::shared_ptr<livo_atlas::Task>> ready_;
};

TEST_F(LoopTaskTest, FailedQueryDoesNotTimeSkippedBatches) {
  for (std::uint64_t id = 4; id < 9; ++id) Append(id);
  auto task = Task(9);
  task.input.query_id = 7;
  fs::remove(root_ / "frames/5/cloud.pcd");
  const auto options = loop_test::NearbyOptions();
  ManualThreadPool pool;
  auto payload = std::make_shared<LoopTaskResult>(
      LoopTaskResult{task, {}, {}});
  testing::internal::CaptureStderr();
  ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool, 4,
                    [](auto) {}, true);
  while (pool.ready_count()) pool.RunLastReady();
  const auto log = testing::internal::GetCapturedStderr();
  ASSERT_TRUE(payload->error);
  const std::string stage = "stage=loop_verification_batch seconds=";
  const auto first = log.find(stage);
  ASSERT_NE(first, std::string::npos);
  EXPECT_EQ(log.find(stage, first + stage.size()), std::string::npos);
}

TEST_F(LoopTaskTest, ForwardReverseShareCovariances) {
  const auto task = Task();
  const auto options = loop_test::NearbyOptions();
  ManualThreadPool pool;
  auto payload = std::make_shared<LoopTaskResult>(
      LoopTaskResult{task, {}, {}});
  testing::internal::CaptureStderr();
  ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool, 4,
                    [](auto) {}, true);
  while (pool.ready_count()) pool.RunLastReady();
  const auto log = testing::internal::GetCapturedStderr();
  ASSERT_FALSE(payload->error);
  ASSERT_TRUE(payload->admission);
  ASSERT_TRUE(payload->admission->proposal);
  ASSERT_NE(log.find("stage=loop_gicp_reverse seconds="), std::string::npos);
  for (const auto* name : {"source", "target"}) {
    const auto stage = std::string("stage=loop_gicp_covariance_") + name;
    const auto first = log.find(stage);
    ASSERT_NE(first, std::string::npos);
    EXPECT_EQ(log.find(stage, first + stage.size()), std::string::npos);
  }
}

TEST_F(LoopTaskTest, QueryStageTimingIsOptIn) {
  const auto task = Task();
  const auto options = loop_test::NearbyOptions();
  const auto expected = RunLoopTask(task, options);
  for (const bool enabled : {false, true}) {
    ManualThreadPool pool;
    auto payload = std::make_shared<LoopTaskResult>(
        LoopTaskResult{task, {}, {}});
    testing::internal::CaptureStderr();
    ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool,
                      4, [](auto) {}, enabled);
    while (pool.ready_count()) pool.RunLastReady();
    const auto log = testing::internal::GetCapturedStderr();
    ASSERT_FALSE(payload->error);
    ASSERT_TRUE(payload->admission);
    EXPECT_EQ(payload->admission->status, expected.status);
    ASSERT_EQ(payload->admission->candidates.size(), 1u);
    ASSERT_EQ(expected.candidates.size(), 1u);
    const auto& actual = payload->admission->candidates.front().verification;
    const auto& reference = expected.candidates.front().verification;
    ASSERT_TRUE(actual.target_from_source);
    ASSERT_TRUE(reference.target_from_source);
    EXPECT_TRUE(actual.target_from_source->rotation.isApprox(
        reference.target_from_source->rotation, 1e-12));
    EXPECT_TRUE(actual.target_from_source->translation.isApprox(
        reference.target_from_source->translation, 1e-12));
    EXPECT_EQ(actual.forward.iterations, reference.forward.iterations);
    EXPECT_EQ(actual.reverse.iterations, reference.reverse.iterations);
    for (const auto* stage : {"loop_geometry_read", "loop_geometry_build",
                              "loop_cloud_read", "loop_geometry_snapshot",
                              "loop_prepare_batch", "loop_candidate",
                              "loop_verification_batch", "loop_admission",
                              "loop_query_compute", "loop_geometry_recheck",
                              "loop_input_prepare", "loop_gicp_forward",
                              "loop_gicp_reverse", "loop_quality_forward",
                              "loop_quality_reverse", "loop_gicp_source_tree",
                              "loop_gicp_covariance_source",
                              "loop_gicp_covariance_target",
                              "loop_gicp_iterations", "loop_gicp_estimate",
                              "loop_gicp_transform_check"}) {
      const auto found = log.find(std::string("stage=") + stage + " seconds=");
      EXPECT_EQ(found != std::string::npos, enabled) << stage;
    }
  }
}

TEST_F(LoopTaskTest, ParallelCandidatesRespectBudgetAndSearchOrder) {
  for (std::uint64_t id = 4; id < 9; ++id) Append(id);
  auto task = Task(9);
  task.input.query_id = 7;
  for (const int threads : {1, 4, 6}) {
    for (const int resident : {2, 3, 6}) {
      SCOPED_TRACE(threads);
      SCOPED_TRACE(resident);
      auto options = loop_test::NearbyOptions();
      options.geometry.max_resident_submaps = resident;
      const auto expected = RunLoopTask(task, options);
      ASSERT_EQ(expected.candidates.size(), 5u);
      auto payload = std::make_shared<LoopTaskResult>(
          LoopTaskResult{task, {}, {}});
      auto prepared = PrepareLoopTask(task, options);
      ManualThreadPool pool;
      int completed = 0;
      ScheduleLoopQuery(payload, options, std::move(prepared), pool, threads,
                        [&](auto result) {
                          EXPECT_EQ(result, payload);
                          ++completed;
                        });
      ASSERT_EQ(pool.ready_count(), 1u);
      pool.RunLastReady();  // The first preparation makes its candidates ready.
      const auto budget = static_cast<std::size_t>(
          std::min(std::max(1, threads - 1), resident - 1));
      ASSERT_EQ(pool.ready_count(), std::min(budget, std::size_t{5}));
      for (int step = 0; pool.ready_count() && step < 100; ++step) {
        EXPECT_LE(pool.ready_count(), budget);
        EXPECT_EQ(completed, 0);
        pool.RunLastReady();  // Finish candidates in reverse readiness order.
      }
      ASSERT_EQ(completed, 1);
      ASSERT_FALSE(payload->error);
      ASSERT_TRUE(payload->admission);
      const auto& actual = *payload->admission;
      EXPECT_EQ(actual.status, expected.status);
      ASSERT_EQ(actual.candidates.size(), expected.candidates.size());
      for (std::size_t i = 0; i < actual.candidates.size(); ++i) {
        const auto& a = actual.candidates[i];
        const auto& b = expected.candidates[i];
        EXPECT_EQ(a.candidate.target_id, b.candidate.target_id);
        EXPECT_EQ(a.verification.status, b.verification.status);
        ASSERT_EQ(a.verification.forward.metrics.has_value(),
                  b.verification.forward.metrics.has_value());
        if (a.verification.forward.metrics) {
          EXPECT_EQ(a.verification.forward.metrics->rmse_m,
                    b.verification.forward.metrics->rmse_m);
        }
      }
    }
  }
}

TEST_F(LoopTaskTest, PoolShutdownDrainsAllDependentBatches) {
  for (std::uint64_t id = 4; id < 9; ++id) Append(id);
  auto task = Task(9);
  task.input.query_id = 7;
  const auto options = loop_test::NearbyOptions();
  for (const int threads : {1, 4, 6}) {
    std::promise<std::shared_ptr<LoopTaskResult>> completed;
    auto future = completed.get_future();
    {
      ThreadPool pool(threads);
      auto payload = std::make_shared<LoopTaskResult>(
          LoopTaskResult{task, {}, {}});
      ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool,
                        threads, [&](auto result) {
                          completed.set_value(std::move(result));
                        });
      // No further scheduling is allowed once destruction starts.
    }
    ASSERT_EQ(future.wait_for(std::chrono::seconds(0)),
              std::future_status::ready);
    const auto result = future.get();
    EXPECT_FALSE(result->error);
    ASSERT_TRUE(result->admission);
    EXPECT_EQ(result->admission->candidates.size(), 5u);
  }
}

TEST_F(LoopTaskTest, PartialRegistrationCanDrainAfterEverySubmissionFailure) {
  for (std::uint64_t id = 4; id < 9; ++id) Append(id);
  auto task = Task(9);
  task.input.query_id = 7;
  const auto options = loop_test::NearbyOptions();
  // Five candidates in three batches use 3 preparation + 5 verification +
  // 3 merge + 1 completion Tasks. Fail each submission in turn.
  for (std::size_t failure = 1; failure <= 12; ++failure) {
    SCOPED_TRACE(failure);
    ManualThreadPool pool;
    pool.FailAt(failure);
    auto payload = std::make_shared<LoopTaskResult>(
        LoopTaskResult{task, {}, {}});
    int completed = 0;
    try {
      ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool,
                        4, [&](auto) { ++completed; });
      FAIL() << "Expected submission failure";
    } catch (const std::bad_alloc&) {
      payload->error = std::current_exception();
    }
    const auto failure_result = payload->error;
    for (int step = 0; pool.ready_count() && step < 100; ++step)
      pool.RunLastReady();
    EXPECT_EQ(pool.ready_count(), 0u);
    EXPECT_EQ(pool.waiting_count(), 0u);
    EXPECT_EQ(completed, 0);
    EXPECT_EQ(payload->error, failure_result);
    EXPECT_FALSE(payload->admission);
  }
}

TEST_F(LoopTaskTest, EmptyAndBudgetLimitedQueriesDoNotReadGeometry) {
  for (std::uint64_t id = 0; id < 4; ++id)
    fs::remove(root_ / "frames" / std::to_string(id) / "cloud.pcd");
  for (const bool limited : {false, true}) {
    auto task = Task();
    auto options = loop_test::NearbyOptions();
    if (limited) {
      options.max_keyframes = 3;
    } else {
      task.input.query_id = 0;
    }
    auto payload = std::make_shared<LoopTaskResult>(
        LoopTaskResult{task, {}, {}});
    ManualThreadPool pool;
    int completed = 0;
    ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool, 1,
                      [&](auto) { ++completed; });
    ASSERT_EQ(pool.ready_count(), 1u);
    pool.RunLastReady();
    EXPECT_EQ(pool.ready_count(), 0u);
    ASSERT_EQ(completed, 1);
    ASSERT_FALSE(payload->error);
    ASSERT_TRUE(payload->admission);
    EXPECT_EQ(payload->admission->status,
              limited ? LoopAdmissionStatus::kResourceLimit
                      : LoopAdmissionStatus::kNoCandidate);
  }
}

TEST_F(LoopTaskTest, CandidateReadFailureStillCompletesDependencyGraph) {
  const auto task = Task();
  fs::remove(root_ / "frames/0/cloud.pcd");
  const auto options = loop_test::NearbyOptions();
  auto payload = std::make_shared<LoopTaskResult>(
      LoopTaskResult{task, {}, {}});
  ManualThreadPool pool;
  int completed = 0;
  ScheduleLoopQuery(payload, options, PrepareLoopTask(task, options), pool, 4,
                    [&](auto) { ++completed; });
  for (int step = 0; pool.ready_count() && step < 100; ++step)
    pool.RunLastReady();
  EXPECT_EQ(completed, 1);
  ASSERT_TRUE(payload->error);
  EXPECT_THROW(std::rethrow_exception(payload->error), std::runtime_error);
}

TEST_F(LoopTaskTest, DispatchFrontierAndGeometryViews) {
  const auto task = Task();
  Append(4);
  // Later live index changes must not be scanned by the captured task.
  std::ofstream(root_ / "online_submaps/dataset.yml") << "invalid";
  fs::remove(root_ / "frames/4/cloud.pcd");
  const auto before = Bytes(root_);
  const auto result = RunLoopTask(task, loop_test::NearbyOptions());
  ASSERT_EQ(result.status, LoopAdmissionStatus::kProposed);
  ASSERT_TRUE(result.proposal);
  EXPECT_EQ(result.proposal->update.optimized_frame_count, 4u);
  EXPECT_EQ(result.proposal->update.poses.size(), 4u);
  EXPECT_EQ(result.proposal->loop.target.primary.id, 0u);
  EXPECT_EQ(result.proposal->loop.source.primary.id, 2u);
  EXPECT_EQ(task.input.poses.version, 0u);
  EXPECT_DOUBLE_EQ(task.input.raw_poses.back().translation.x(), .06);
  EXPECT_EQ(Bytes(root_), before);

  std::vector<OnlineSubmap> groups;
  for (const auto& info : task.input.submaps) groups.push_back(info.membership);
  GeometryOptions geometry;
  geometry.enabled = true;
  const auto direct = ProposeLoopUpdate(
      task.input,
      [&](std::uint64_t id, const PoseSnapshot& poses) {
        GeometryMapper mapper(geometry, [&](std::uint64_t frame) {
          return task.frames.ReadBodyCloud(frame,
                                           geometry.max_points_per_keyframe);
        });
        mapper.Update(groups, std::make_shared<const PoseSnapshot>(poses));
        return mapper.GetSubmap(id);
      },
      loop_test::NearbyOptions());
  ASSERT_TRUE(direct.proposal);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_LT((direct.proposal->update.poses[i].map_from_body.translation -
               result.proposal->update.poses[i].map_from_body.translation)
                  .norm(),
              1e-9);
  }
  // A later task may keep the same version while extending its frontier.
  const auto later = RunLoopTask(Task(5), loop_test::NearbyOptions());
  ASSERT_TRUE(later.proposal);
  EXPECT_EQ(later.proposal->update.optimized_frame_count, 5u);
  EXPECT_EQ(Bytes(root_), before);
}

TEST_F(LoopTaskTest, KeepsGraphBudgetAheadOfCloudReads) {
  auto task = Task();
  fs::remove(root_ / "frames/0/cloud.pcd");
  auto options = loop_test::NearbyOptions();
  options.max_keyframes = 3;
  EXPECT_EQ(RunLoopTask(task, options).status,
            LoopAdmissionStatus::kResourceLimit);
}

TEST_F(LoopTaskTest, EnforcesConfiguredGeometryCloudLimit) {
  auto options = loop_test::NearbyOptions();
  options.geometry.max_points_per_keyframe = 1;
  EXPECT_THROW(RunLoopTask(Task(), options), std::runtime_error);
}

TEST_F(LoopTaskTest, RejectsDifferentTaskAndReadFrontiers) {
  auto task = Task();
  task.input.poses.frame_count = 3;
  task.input.poses.poses.resize(3);
  task.input.raw_poses.resize(3);
  task.timestamps.resize(3);
  for (auto& info : task.input.submaps) info.frame_count = 3;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
  task = Task();
  task.timestamps.pop_back();
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
}

TEST_F(LoopTaskTest, RejectsChangedCommittedSourceMetadata) {
  auto task = Task();
  task.input.raw_poses[0].translation.x() = .01;
  task.input.poses.poses[0].map_from_body.translation.x() = .01;
  task.input.submaps[0].parent_from_submap.translation.x() = .01;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
  task = Task();
  task.timestamps[0] = 10.25;
  task.input.submaps[0].first_timestamp = 10.25;
  task.input.submaps[0].last_timestamp = 10.25;
  EXPECT_THROW(RunLoopTask(task, loop_test::NearbyOptions()),
               std::invalid_argument);
}

#if defined(__unix__)
TEST_F(LoopTaskTest, RejectsMalformedMembershipBeforeIndexingTimestamps) {
  auto task = Task();
  task.input.submaps[0].membership.primary.keyframe_ids = {
      std::uint64_t{1} << 52, 0};
  // A malformed ID must reach validation, rather than an unchecked read.
  ASSERT_EXIT(
      {
        try {
          RunLoopTask(task, loop_test::NearbyOptions());
        } catch (const std::invalid_argument&) {
          _exit(0);
        }
        _exit(1);
      },
      testing::ExitedWithCode(0), "");
}
#endif

}  // namespace
}  // namespace livo_atlas
