#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "core/fixtures.h"
#include "gaussian/submap_peer.h"
#include "gaussian/wire_protocol.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

namespace {
std::atomic<bool> fail_joint_publication{false};
}  // namespace

// Exercise a real local publication failure without modifying the GPU worker.
extern "C" int rename(const char* from, const char* to) noexcept {
  const std::string name(from);
  if (fail_joint_publication && name.find("joint-render-") !=
                                    std::string::npos &&
      name.find(".tmp.json") != std::string::npos) {
    errno = EIO;
    return -1;
  }
  return static_cast<int>(syscall(SYS_rename, from, to));
}

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using namespace std::chrono_literals;

using test::SubmapPeer;
using test::Text;

class OnlineGaussianRenderTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("gs-map-" + std::to_string(getpid()) + "-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(root_);
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.exporter.directory = root_ / "dataset";
    options.keyframes.enabled = options.submaps.enabled = true;
    options.submap_mode = SubmapMode::kOnline;
    system_ = std::make_unique<LivoSystem>(options);
    options_.enabled = options_.persist_submaps = true;
    options_.socket_path = root_ / "peer.sock";
    options_.output_directory = root_ / "output";
    options_.joint_render_enabled = true;
    options_.joint_render_interval = 100ms;
    options_.joint_render_timeout = 1s;
    options_.shutdown_timeout = 2s;
    options_.job_timeout = 5s;
    options_.connect_timeout = 2s;
    peer_ = std::make_unique<SubmapPeer>(options_.socket_path);
    mapper_ = std::make_unique<OnlineGaussianMapper>(
        *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}},
        options_);
  }

  void TearDown() override {
    fail_joint_publication = false;
    if (peer_)
      peer_->hold_candidate = peer_->hold_commit = peer_->hold_checkpoint =
          peer_->hold_map_ack = peer_->hold_empty = peer_->hold_render = false;
    mapper_.reset();
    peer_.reset();
    system_.reset();
    fs::remove_all(root_);
  }

  std::shared_ptr<Keyframe> Frame(std::uint64_t id, bool image = true) {
    auto frame = std::make_shared<Keyframe>();
    frame->id = id;
    frame->timestamp = frame->cloud_timestamp = 100 + id;
    frame->image_timestamp = 100 + id;
    frame->camera_from_odom = RigidTransform{};
    if (rounded_camera_) frame->camera_from_odom->rotation(0, 0) += 2e-6;
    if (image) frame->image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(10, 20, 30));
    PointCloud::Ptr cloud(new PointCloud);
    Point point{};
    point.z = 4;
    cloud->push_back(point);
    frame->cloud = cloud;
    return frame;
  }

  void Append(bool new_group = false, bool image = true) {
    const auto id = poses_->frame_count;
    auto next = std::make_shared<PoseSnapshot>(*poses_);
    next->poses.push_back({id, {}});
    ++next->frame_count;

    std::vector<OnlineSubmap> revisions;
    if (groups_.empty() || new_group) {
      if (!groups_.empty()) {
        auto& previous = groups_.back();
        previous.state = OnlineSubmapState::kSealed;
        ++previous.membership_revision;
        revisions.push_back(previous);
      }
      OnlineSubmap group;
      group.primary = {groups_.size(), id, {id}};
      groups_.push_back(group);
    } else {
      groups_.back().primary.keyframe_ids.push_back(id);
      ++groups_.back().membership_revision;
    }
    revisions.push_back(groups_.back());
    poses_ = next;
    mapper_->Observe({Frame(id, image), revisions, poses_, {}});
  }

  void Correct(bool deform = false, std::uint64_t member = 0) {
    auto next = std::make_shared<PoseSnapshot>(*poses_);
    next->base_version = next->version;
    ++next->version;
    next->optimized_frame_count = next->frame_count;
    next->parent_frame = PoseFrame::kMap;
    for (auto& pose : next->poses) pose.map_from_body.translation.y() += 1;
    if (deform) next->poses.at(member).map_from_body.translation.x() += .1;
    poses_ = next;
    mapper_->Observe({{}, {}, poses_, {}});
  }

  bool Wait(const std::function<bool()>& predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    while (std::chrono::steady_clock::now() < deadline) {
      mapper_->Poll();
      if (predicate()) return true;
      if (mapper_->GetStats().failed) {
        ADD_FAILURE() << mapper_->GetStats().error
                      << " peer=" << (peer_ ? peer_->Error() : "real worker");
        return false;
      }
      std::this_thread::sleep_for(1ms);
    }
    ADD_FAILURE() << "Timeout; peer="
                  << (peer_ ? peer_->Error() : "real worker");
    return false;
  }

  GaussianSubmapEntry Entry(std::size_t id) {
    return mapper_->GetMapSnapshot()->entries.at(id);
  }

  void Confirm() {
    ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted > accepted_; }));
    accepted_ = mapper_->GetStats().accepted;
  }

  fs::path root_;
  OnlineGaussianOptions options_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<SubmapPeer> peer_;
  std::unique_ptr<OnlineGaussianMapper> mapper_;
  std::shared_ptr<const PoseSnapshot> poses_ = std::make_shared<PoseSnapshot>();
  std::vector<OnlineSubmap> groups_;
  std::uint64_t accepted_ = 0;
  bool rounded_camera_ = false;
};


TEST_F(OnlineGaussianRenderTest, AcceptsCurrentOwnedResultThenInvalidates) {
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().accepted == 1; }));
  const auto first = mapper_->GetLatestRender();
  ASSERT_TRUE(first);
  EXPECT_EQ(first, mapper_->GetLatestRender());
  EXPECT_EQ(first->request.query_keyframe_id, 0u);
  EXPECT_EQ(first->request.blocks.size(), 1u);
  const auto trained = mapper_->GetStats().accepted;
  Append();
  EXPECT_FALSE(mapper_->GetLatestRender());
  ASSERT_TRUE(Wait([&] {
    return mapper_->GetStats().accepted > trained;
  }));
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().accepted >= 2; }));
  EXPECT_EQ(mapper_->GetLatestRender()->request.query_keyframe_id, 1u);
  Correct();
  EXPECT_FALSE(mapper_->GetLatestRender());
  mapper_->Finish();
  EXPECT_FALSE(mapper_->GetLatestRender());
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
}

TEST_F(OnlineGaussianRenderTest, StaleInflightResultCannotBeRelabeled) {
  peer_->hold_render = true;
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return peer_->renders == 1; }));
  Append();
  Correct();
  peer_->hold_render = false;
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().stale == 1; }));
  EXPECT_FALSE(mapper_->GetLatestRender());
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().accepted >= 1; }));
  ASSERT_TRUE(mapper_->GetLatestRender());
  EXPECT_EQ(mapper_->GetLatestRender()->request.pose_version, 1u);
  EXPECT_EQ(mapper_->GetLatestRender()->request.frame_count, 2u);
}

TEST_F(OnlineGaussianRenderTest, MandatoryPersistenceAndDecisionPrecedeRender) {
  Append();
  Confirm();
  Append(true);
  Confirm();
  ASSERT_TRUE(Wait([&] {
    const auto render = mapper_->GetLatestRender();
    return render && render->request.query_submap_id == 1;
  }));
  EXPECT_EQ(mapper_->GetLatestRender()->request.blocks.size(), 2u);
  const auto events = peer_->Events();
  const auto checkpoint =
      std::find(events.begin(), events.end(), "checkpoint:0");
  const auto ack = std::find(checkpoint, events.end(), "map_committed:");
  const auto decision = std::find(ack, events.end(), "decision:1");
  const auto render = std::find(decision, events.end(), "joint_render:");
  EXPECT_NE(checkpoint, events.end());
  EXPECT_NE(ack, events.end());
  EXPECT_NE(decision, events.end());
  EXPECT_NE(render, events.end());
}

TEST_F(OnlineGaussianRenderTest, OomDisablesOnlyRenderingAndTrainingContinues) {
  peer_->oom_render = true;
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().failed == 1; }));
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  EXPECT_FALSE(mapper_->GetLatestRender());
  const auto submitted = mapper_->GetRenderStats().submitted;
  Append();
  Confirm();
  EXPECT_EQ(mapper_->GetRenderStats().submitted, submitted);
}

TEST_F(OnlineGaussianRenderTest, MissingNegotiatedCapabilityFailsGaussian) {
  // A fresh mapper is required to exercise negotiation.
  mapper_->Finish();
  mapper_.reset();
  peer_.reset();
  fs::remove_all(options_.output_directory);
  peer_ = std::make_unique<SubmapPeer>(options_.socket_path);
  peer_->no_render_capability = true;
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}}, options_);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!mapper_->GetStats().failed &&
         std::chrono::steady_clock::now() < deadline) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_FALSE(mapper_->GetLatestRender());
}

TEST_F(OnlineGaussianRenderTest, TimeoutIsFatalRatherThanRecoverableOom) {
  peer_->hold_render = true;
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return peer_->renders == 1; }));
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!mapper_->GetStats().failed &&
         std::chrono::steady_clock::now() < deadline) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_FALSE(mapper_->GetLatestRender());
}

TEST_F(OnlineGaussianRenderTest, TwoContentAttemptsYieldOnceThenTrainAgain) {
  peer_->hold_commit = true;
  peer_->hold_render = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->decisions == 1; }));
  Append();
  peer_->hold_commit = false;
  Confirm();
  peer_->hold_commit = true;
  ASSERT_TRUE(Wait([&] { return peer_->decisions == 2; }));
  Append();
  peer_->hold_commit = false;
  Confirm();
  ASSERT_TRUE(Wait([&] { return peer_->renders == 1; }));
  EXPECT_EQ(peer_->views, 2);
  peer_->hold_render = false;
  ASSERT_TRUE(Wait([&] { return peer_->views == 3; }));
  Confirm();
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
}

TEST_F(OnlineGaussianRenderTest, ChangingMapAcknowledgementsCannotStarveViews) {
  Append();
  Confirm();
  Append(true);
  Confirm();
  ASSERT_TRUE(Wait([&] {
    const auto render = mapper_->GetLatestRender();
    return render && render->request.query_submap_id == 1;
  }));
  ASSERT_TRUE(Wait([&] {
    return mapper_->GetRenderStats().saved ==
           mapper_->GetRenderStats().accepted;
  }));
  std::this_thread::sleep_for(110ms);
  const auto before = peer_->views.load();
  auto events = peer_->Events();
  auto seen = std::count(events.begin(), events.end(), "map_committed:");
  peer_->hold_map_ack = true;
  Append();
  for (int attempt = 0; attempt < 10 && peer_->views < before + 3; ++attempt) {
    ASSERT_TRUE(Wait([&] {
      events = peer_->Events();
      return std::count(events.begin(), events.end(), "map_committed:") >
                 seen || peer_->views >= before + 3;
    }));
    if (peer_->views >= before + 3) break;
    seen = std::count(events.begin(), events.end(), "map_committed:");
    // Every acknowledgement is already obsolete when the owner receives it.
    Append();
    const auto acknowledged = peer_->acknowledgements.load();
    peer_->hold_map_ack = false;
    ASSERT_TRUE(Wait([&] {
      return peer_->acknowledgements > acknowledged;
    }));
    peer_->hold_map_ack = true;
  }
  EXPECT_GE(peer_->views, before + 3);
  peer_->hold_map_ack = false;
}

TEST_F(OnlineGaussianRenderTest,
       EmptyActivationDoesNotBlockHistoricalRenderAfterPoseOnlyChange) {
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().saved == 1; }));
  Append(true, false);
  ASSERT_TRUE(Wait([&] {
    return mapper_->GetStats().skipped_no_image == 1;
  }));
  Correct();
  ASSERT_TRUE(Wait([&] {
    const auto render = mapper_->GetLatestRender();
    return render && render->request.pose_version == 1;
  }));
  EXPECT_EQ(mapper_->GetLatestRender()->request.query_keyframe_id, 0u);
  EXPECT_EQ(mapper_->GetStats().skipped_no_image, 1u);
}

TEST_F(OnlineGaussianRenderTest, RealWorkerRendersAssetsDuringHistoryRepair) {
  const char* socket = std::getenv("LIVO_REAL_GAUSSIAN_SOCKET");
  const char* output = std::getenv("LIVO_REAL_GAUSSIAN_OUTPUT");
  if (!socket || !output) GTEST_SKIP() << "Real GPU worker was not requested";
  mapper_->Finish();
  mapper_.reset();
  peer_.reset();
  options_.socket_path = socket;
  options_.output_directory = output;
  options_.connect_timeout = 20s;
  options_.shutdown_timeout = 30s;
  options_.joint_render_timeout = 5s;
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}}, options_);
  for (int i = 0; i < 6; ++i) {
    Append(i == 2 || i == 4);
    Confirm();
    ASSERT_TRUE(Wait([&] {
      const auto result = mapper_->GetLatestRender();
      return result && result->request.query_keyframe_id == std::uint64_t(i);
    }));
  }
  Correct(true, 1);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().rebuilds >= 1; }));
  ASSERT_TRUE(Wait([&] {
    const auto result = mapper_->GetLatestRender();
    return result && result->request.pose_version == 1 &&
           result->request.query_submap_id == 2;
  }));
  const auto result = mapper_->GetLatestRender();
  ASSERT_EQ(result->request.blocks.size(), 2u);
  EXPECT_EQ(result->request.blocks[0].target.group.primary.id, 1u);
  EXPECT_EQ(result->request.blocks[1].target.group.primary.id, 2u);
  EXPECT_FALSE(result->request.blocks[0].resident);
  EXPECT_FALSE(result->request.blocks[1].resident);
  EXPECT_TRUE(Entry(0).resident);
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_TRUE(Entry(0).asset_current);
  EXPECT_TRUE(Entry(1).asset_current);
  EXPECT_TRUE(Entry(2).asset_current);
}

TEST_F(OnlineGaussianRenderTest, SavesAtMostFourOwnedHistoricalPairs) {
  std::ofstream(options_.output_directory / "unrelated.json") << "keep";
  for (int i = 0; i < 6; ++i) {
    Append();
    Confirm();
    ASSERT_TRUE(Wait([&] {
      return mapper_->GetRenderStats().saved == std::uint64_t(i + 1);
    }));
    EXPECT_LE(mapper_->GetRenderStats().peak_pending, 1u);
    EXPECT_LE(mapper_->GetRenderStats().peak_in_flight, 1u);
  }
  int images = 0, metadata = 0;
  for (const auto& path : fs::directory_iterator(options_.output_directory)) {
    const auto name = path.path().filename().string();
    if (name.rfind("joint-render-", 0) != 0) continue;
    images += path.path().extension() == ".png";
    metadata += path.path().extension() == ".json";
    EXPECT_EQ(name.find(".tmp."), std::string::npos);
  }
  EXPECT_EQ(images, 4);
  EXPECT_EQ(metadata, 4);
  EXPECT_TRUE(fs::exists(options_.output_directory / "unrelated.json"));
}

TEST_F(OnlineGaussianRenderTest, WriteFailureDisablesOnlyJointPreview) {
  peer_->hold_render = true;
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return peer_->renders == 1; }));
  const auto name = "joint-render-" + std::to_string(peer_->render_job.load());
  fs::create_directory(options_.output_directory / (name + ".tmp.json"));
  peer_->hold_render = false;
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().failed == 1; }));
  EXPECT_TRUE(mapper_->GetRenderStats().disabled);
  EXPECT_FALSE(mapper_->GetLatestRender());
  EXPECT_FALSE(fs::exists(options_.output_directory / (name + ".json")));
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  Append();
  Confirm();
  mapper_->Finish();
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
}

TEST_F(OnlineGaussianRenderTest, RealWorkerContinuesAfterPreviewRenameFailure) {
  const char* socket = std::getenv("LIVO_REAL_GAUSSIAN_SOCKET");
  const char* output = std::getenv("LIVO_REAL_GAUSSIAN_OUTPUT");
  if (!socket || !output) GTEST_SKIP() << "Real GPU worker was not requested";
  mapper_->Finish();
  mapper_.reset();
  peer_.reset();
  options_.socket_path = socket;
  options_.output_directory = output;
  options_.connect_timeout = 20s;
  options_.shutdown_timeout = 30s;
  options_.joint_render_timeout = 5s;
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}}, options_);
  fail_joint_publication = true;
  Append();
  Confirm();
  ASSERT_TRUE(Wait([&] { return mapper_->GetRenderStats().failed == 1; }));
  EXPECT_TRUE(mapper_->GetRenderStats().disabled);
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  EXPECT_FALSE(mapper_->GetLatestRender());
  Append();
  Confirm();
  EXPECT_EQ(mapper_->GetStats().effective_training, 2u);
  mapper_->Finish();
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_TRUE(Entry(0).asset_current);
  for (const auto& path : fs::directory_iterator(options_.output_directory))
    EXPECT_NE(path.path().filename().string().rfind("joint-render-", 0), 0u);
}

}  // namespace
}  // namespace livo_atlas
