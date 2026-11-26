#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <opencv2/core/persistence.hpp>
#include <thread>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

namespace livo_atlas {

class OnlineGaussianMapperTestPeer {
 public:
  static void StopThread(OnlineGaussianMapper& mapper) {
    mapper.stopping_ = true;
    mapper.condition_.notify_all();
    if (mapper.thread_.joinable()) mapper.thread_.join();
    mapper.stopping_ = false;
    mapper.connected_ = true;
  }

  static void Confirm(OnlineGaussianMapper& mapper) {
    mapper.generation_ = 1;
    const auto target = mapper.map_index_.MakeTarget(0, 1);
    mapper.map_index_.RecordConfirmed(target, GaussianOperation::kView, 1,
                                      {0, 2}, 2);
    mapper.remote_target_ = target;
    mapper.acknowledged_map_ = mapper.map_index_.sequence();
  }

  static bool Dispatch(OnlineGaussianMapper& mapper, bool content,
                       int attempts) {
    if (content) {
      mapper.pending_.emplace();
      mapper.pending_->submap.primary.id = 0;
    } else
      mapper.pending_.reset();
    mapper.evaluation_content_attempts_ = attempts;
    mapper.acknowledged_map_ = mapper.map_index_.sequence();
    return mapper.DispatchEvaluation(mapper.map_index_.Snapshot());
  }

  static bool Contended(OnlineGaussianMapper& mapper) {
    std::promise<void> entered, released;
    auto ready = entered.get_future();
    auto release = released.get_future();
    std::thread holder([&] {
      std::lock_guard<std::mutex> lock(mapper.mutex_);
      entered.set_value();
      release.wait();
    });
    ready.wait();
    const bool reserved =
        mapper.DispatchEvaluation(mapper.map_index_.Snapshot());
    released.set_value();
    holder.join();
    return reserved;
  }

  static void Complete(OnlineGaussianMapper& mapper) {
    auto command = *mapper.command_;
    mapper.command_.reset();
    mapper.busy_ = false;
    OnlineGaussianMapper::Reply reply;
    reply.kind = OnlineGaussianMapper::ReplyKind::kEvaluated;
    reply.command = command;
    reply.evaluation_report_sha256 = std::string(64, 'b');
    mapper.HandleEvaluationReply(reply);
  }

  static bool RenderGetsNextTurn(OnlineGaussianMapper& mapper) {
    if (mapper.evaluation_turn_) return false;
    mapper.render_pending_ = true;
    mapper.render_attempts_ = 2;
    mapper.acknowledged_map_ = mapper.map_index_.sequence();
    mapper.DispatchSubmaps();
    return mapper.command_ &&
           mapper.command_->kind == OnlineGaussianMapper::CommandKind::kRender;
  }

  static void Journal(OnlineGaussianMapper& mapper) {
    mapper.WriteEvaluationStatus(mapper.evaluation_status_);
  }

  static void Fail(OnlineGaussianMapper& mapper) {
    mapper.Fail("controlled_preflight_budget_failure");
  }

  static void ManyErrors(OnlineGaussianMapper& mapper) {
    auto state = mapper.evaluation_status_.front();
    mapper.evaluation_status_.clear();
    for (int i = 0; i < 64; ++i) {
      state.query.camera.keyframe_id = i;
      state.status = "in_flight";
      state.job_id = i + 1;
      mapper.evaluation_status_.push_back(state);
      OnlineGaussianMapper::Reply reply;
      reply.kind = OnlineGaussianMapper::ReplyKind::kEvaluationError;
      reply.command.evaluation.emplace();
      reply.command.evaluation->camera.keyframe_id = i;
      reply.command.evaluation->sources.job_id = i + 1;
      reply.detail = std::string(2048, '\\');
      mapper.HandleEvaluationReply(reply);
    }
  }

  static void FinishWithQueuedReceipt(OnlineGaussianMapper& mapper) {
    OnlineGaussianMapper::Reply reply;
    reply.kind = OnlineGaussianMapper::ReplyKind::kEvaluated;
    reply.command = *mapper.command_;
    reply.evaluation_report_sha256 = std::string(64, 'b');
    mapper.command_.reset();
    mapper.reply_ = reply;
    std::promise<void> ready;
    auto entered = ready.get_future();
    std::thread holder([&] {
      std::lock_guard<std::mutex> lock(mapper.mutex_);
      ready.set_value();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    });
    entered.wait();
    mapper.Finish();
    holder.join();
  }

  static void Stop(OnlineGaussianMapper& mapper) { mapper.StopEvaluation(); }

  static void Cleanup(OnlineGaussianMapper& mapper) {
    mapper.failed_ = true;
    mapper.connected_ = false;
  }
};

namespace {
namespace fs = std::filesystem;

class EvaluationSchedulerTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("ev-plan-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(root_);
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.exporter.directory = root_ / "keyframes";
    options.keyframes.enabled = options.submaps.enabled = true;
    options.submap_mode = SubmapMode::kOnline;
    system_ = std::make_unique<LivoSystem>(options);
    OnlineGaussianOptions gaussian;
    gaussian.enabled = gaussian.persist_submaps =
        gaussian.joint_render_enabled = true;
    gaussian.evaluation_enabled = gaussian.online_evaluation_enabled = true;
    gaussian.shutdown_timeout = std::chrono::milliseconds(5);
    gaussian.socket_path = root_ / "peer.sock";
    gaussian.output_directory = root_ / "output";
    gaussian.evaluation_split_file = root_ / "split.json";
    std::ofstream out(gaussian.evaluation_split_file);
    out << R"({"schema":1,"kind":"livo_atlas.gaussian_split","split_id":")"
        << std::string(64, 'a') << R"(","source":{},"frames":[)";
    for (int id = 0; id < 3; ++id) {
      if (id) out << ',';
      out << "{\"keyframe_id\":\"" << id
          << "\",\"submap_id\":\"0\",\"calibration_id\":\"0\",\"role\":\""
          << (id == 1 ? "heldout" : "train")
          << "\",\"state_timestamp\":" << 100 + id
          << ",\"cloud_timestamp\":" << 100 + id
          << ",\"image_timestamp\":" << 100 + id;
      if (id == 1)
        out << R"(,"query":1,"width":8,"height":6,"intrinsics":[4,0,3.5,0,4,2.5,0,0,1],"camera_from_body":[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])";
      out << '}';
    }
    out << "]}";
    out.close();
    mapper_ = std::make_unique<OnlineGaussianMapper>(
        *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}},
        gaussian);
    OnlineGaussianMapperTestPeer::StopThread(*mapper_);
    poses_ = std::make_shared<PoseSnapshot>();
    poses_->frame_count = 3;
    for (int id = 0; id < 3; ++id) {
      RigidTransform pose;
      pose.translation.x() = id;
      poses_->poses.push_back({std::uint64_t(id), pose});
    }
    group_.primary = {0, 0, {0}};
    mapper_->Observe({{}, {group_}, poses_, {}});
    group_.primary.keyframe_ids = {0, 1};
    group_.membership_revision = 2;
    mapper_->Observe({{}, {group_}, poses_, {}});
    group_.primary.keyframe_ids = {0, 1, 2};
    group_.membership_revision = 3;
    mapper_->Observe({{}, {group_}, poses_, {}});
    OnlineGaussianMapperTestPeer::Confirm(*mapper_);
  }

  void TearDown() override {
    OnlineGaussianMapperTestPeer::Cleanup(*mapper_);
    mapper_.reset();
    system_.reset();
    fs::remove_all(root_);
  }

  void Seal() {
    group_.state = OnlineSubmapState::kSealed;
    group_.membership_revision = 4;
    mapper_->Observe({{}, {group_}, poses_, {}});
  }

  fs::path root_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<OnlineGaussianMapper> mapper_;
  std::shared_ptr<PoseSnapshot> poses_;
  OnlineSubmap group_;
};

TEST_F(EvaluationSchedulerTest, SealAndTwoContentAttemptsPrecedeScoring) {
  ASSERT_EQ(mapper_->GetEvaluationStatus().size(), 1u);
  EXPECT_FALSE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  Seal();
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "pending");
  EXPECT_FALSE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, true, 1));
  EXPECT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, true, 2));
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "in_flight");
}

TEST_F(EvaluationSchedulerTest, BusyMailboxKeepsTheScoringOpportunity) {
  Seal();
  EXPECT_TRUE(OnlineGaussianMapperTestPeer::Contended(*mapper_));
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "pending");
  EXPECT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "in_flight");
}

TEST_F(EvaluationSchedulerTest, HistoricalReplySurvivesNewerPoseFrontier) {
  Seal();
  ASSERT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  poses_ = std::make_shared<PoseSnapshot>(*poses_);
  poses_->version = 1;
  poses_->optimized_frame_count = 3;
  poses_->parent_frame = PoseFrame::kMap;
  for (auto& pose : poses_->poses) pose.map_from_body.translation.y() += 10;
  mapper_->Observe({{}, {}, poses_, {}});
  OnlineGaussianMapperTestPeer::Complete(*mapper_);
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "succeeded");
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].report_sha256,
            std::string(64, 'b'));
}

TEST_F(EvaluationSchedulerTest, StopRetainsInFlightResultAndItsReceipt) {
  Seal();
  ASSERT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  OnlineGaussianMapperTestPeer::Stop(*mapper_);
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "in_flight");
  OnlineGaussianMapperTestPeer::Complete(*mapper_);
  OnlineGaussianMapperTestPeer::Journal(*mapper_);
  cv::FileStorage journal(
      (root_ / "output/evaluations/online-status.json").string(),
      cv::FileStorage::READ);
  EXPECT_EQ(static_cast<std::string>(journal["status"]), "complete");
  EXPECT_EQ(static_cast<std::string>(journal["queries"][0]["report_sha256"]),
            std::string(64, 'b'));
}

TEST_F(EvaluationSchedulerTest,
       ScoringYieldsTheNextSharedOpportunityToPreview) {
  Seal();
  ASSERT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  OnlineGaussianMapperTestPeer::Complete(*mapper_);
  EXPECT_TRUE(OnlineGaussianMapperTestPeer::RenderGetsNextTurn(*mapper_));
}

TEST_F(EvaluationSchedulerTest,
       FailureBeforeScoringIsNeverAnOrdinaryCancellation) {
  OnlineGaussianMapperTestPeer::Fail(*mapper_);
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "failed");
  cv::FileStorage journal(
      (root_ / "output/evaluations/online-status.json").string(),
      cv::FileStorage::READ);
  EXPECT_EQ(static_cast<std::string>(journal["status"]), "failed");
}

TEST_F(EvaluationSchedulerTest, FullFailurePlanFitsTheJournalBudget) {
  OnlineGaussianMapperTestPeer::ManyErrors(*mapper_);
  EXPECT_NO_THROW(OnlineGaussianMapperTestPeer::Journal(*mapper_));
}

TEST_F(EvaluationSchedulerTest, ShutdownConsumesTheAlreadyValidatedReceipt) {
  Seal();
  ASSERT_TRUE(OnlineGaussianMapperTestPeer::Dispatch(*mapper_, false, 0));
  OnlineGaussianMapperTestPeer::FinishWithQueuedReceipt(*mapper_);
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "succeeded");
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].report_sha256,
            std::string(64, 'b'));
}

TEST_F(EvaluationSchedulerTest, StopCancelsWaitingQueries) {
  OnlineGaussianMapperTestPeer::Stop(*mapper_);
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].status, "cancelled");
  EXPECT_EQ(mapper_->GetEvaluationStatus()[0].reason, "shutdown_before_seal");
}

}  // namespace
}  // namespace livo_atlas
