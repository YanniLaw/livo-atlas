#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "core/fixtures.h"
#include "gaussian/wire_protocol.h"
#include "livo_atlas/core/livo_system.h"

namespace {
std::atomic<bool> fail_map_sync{false};
std::string map_sync_path;
}  // namespace

// Fail only the selected run's directory fsync after map.json becomes visible.
extern "C" int fsync(int fd) {
  if (fail_map_sync) {
    char path[4096];
    const auto name = "/proc/self/fd/" + std::to_string(fd);
    const auto size = readlink(name.c_str(), path, sizeof(path));
    if (size > 0 && std::string(path, size) == map_sync_path) {
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

std::string Text(const cv::FileNode& node) {
  return static_cast<std::string>(node);
}

// This peer exercises actual framing and files, while controlling only timings
// and returned model telemetry. Real arrays and CUDA are tested by Python.
class SubmapPeer {
 public:
  explicit SubmapPeer(const fs::path& path) : path_(path) {
    listener_ = socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const auto name = path.string();
    std::copy(name.begin(), name.end(), address.sun_path);
    if (listener_ < 0 ||
        bind(listener_, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) ||
        listen(listener_, 1))
      throw std::runtime_error("Cannot start submap peer");
    thread_ = std::thread(&SubmapPeer::Run, this);
  }

  ~SubmapPeer() {
    stopping = true;
    if (client_ >= 0) shutdown(client_, SHUT_RDWR);
    shutdown(listener_, SHUT_RDWR);
    if (thread_.joinable()) thread_.join();
    if (client_ >= 0) close(client_);
    close(listener_);
    fs::remove(path_);
  }

  std::vector<std::string> Events() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  std::atomic<bool> hold_candidate{false}, hold_commit{false},
      hold_checkpoint{false}, hold_map_ack{false}, corrupt_checkpoint{false},
      wrong_stop{false};
  std::atomic<int> views{0}, checkpoints{0}, acknowledgements{0}, prunes{0},
      discards{0}, reuses{0}, rebuilds{0}, commits{0}, decisions{0};
  std::atomic<bool> stopping{false};
  std::string Error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

 private:
  void Hold(const std::atomic<bool>& flag) {
    while (flag && !stopping) std::this_thread::sleep_for(1ms);
  }

  void Run() {
    client_ = accept(listener_, nullptr, nullptr);
    try {
      std::string session;
      fs::path states;
      std::vector<std::string> confirmed, proposed;
      std::string source_generation, content_job;
      for (;;) {
        const auto message = ReadGaussianMessage(client_, 30s);
        cv::FileStorage input(message.header_json,
                              cv::FileStorage::READ | cv::FileStorage::MEMORY);
        const auto kind = Text(input["type"]);
        cv::FileStorage out(".json", cv::FileStorage::WRITE |
                                         cv::FileStorage::MEMORY |
                                         cv::FileStorage::FORMAT_JSON);
        if (kind == "hello") session = Text(input["session_id"]);
        out << "schema" << 2 << "session_id" << session;
        for (const auto* field :
             {"job_id", "generation", "pose_version", "submap_id", "operation"})
          if (!input[field].empty())
            out << field
                << (kind == "stop" && wrong_stop &&
                            std::string(field) == "generation"
                        ? "999999"
                        : Text(input[field]));
        {
          std::lock_guard<std::mutex> lock(mutex_);
          events_.push_back(
              kind + ":" +
              (input["submap_id"].empty() ? "" : Text(input["submap_id"])));
        }
        if (kind == "hello") {
          states = Text(input["state_directory"]);
          fs::create_directory(states);
          out << "type"
              << "hello_ok"
              << "mode"
              << "persistent_submaps"
              << "state_schema" << 1;
          for (const auto* key :
               {"python", "torch", "gsplat", "cuda", "device"})
            out << key << "controlled";
          out << "gpu_free_bytes"
              << "5000000000"
              << "gpu_total_bytes"
              << "8000000000";
        } else if (kind == "activate" || kind == "view") {
          const auto action = kind == "view" ? "view" : Text(input["action"]);
          if (action == "empty") {
            confirmed.clear();
            out << "type"
                << "activated";
          } else {
            proposed = confirmed;
            if (action == "view") {
              ++views;
              proposed.push_back(Text(input["keyframe_id"]));
              if (proposed.size() > 10) proposed.erase(proposed.begin());
            } else {
              if (!input["asset_id"].empty()) {
                cv::FileStorage saved(
                    (states / Text(input["asset_id"]) / "peer.json").string(),
                    cv::FileStorage::READ);
                proposed.clear();
                for (const auto& id : saved["frames"])
                  proposed.push_back(Text(id));
              }
              if (action == "reuse")
                ++reuses;
              else
                ++rebuilds;
              out << "operation" << action;
            }
            if (action == "view")
              out << "operation"
                  << "view";
            if (kind == "activate") {
              pending_source_ = Text(input["source_generation"]);
              pending_content_ = Text(input["content_job_id"]);
            }
            Hold(hold_candidate);
            out << "type"
                << "candidate"
                << "frame_ids"
                << "[";
            for (const auto& id : proposed) out << id;
            out << "]";
            for (const auto* key : {"membership_revision", "frame_count",
                                    "optimized_frame_count"})
              out << key << Text(input[key]);
            const int width =
                kind == "view" ? static_cast<int>(input["width"]) : 8;
            const int height =
                kind == "view" ? static_cast<int>(input["height"]) : 6;
            out << "width" << width << "height" << height << "steps"
                << (action == "reuse"
                        ? 0
                        : action == "view"
                              ? 20
                              : 20 * static_cast<int>(proposed.size()))
                << "gaussian_count" << static_cast<int>(proposed.size())
                << "supervised_depth_pixels" << (action == "reuse" ? 0 : 20)
                << "rgb_l1" << .2 << "depth_l1_m" << .1 << "nonzero_gradient"
                << (action == "reuse" ? 0 : 1) << "parameter_updated"
                << (action == "reuse" ? 0 : 1) << "capture_time" << 100.;
            WriteGaussianMessage(
                client_,
                {out.releaseAndGetString(),
                 std::vector<std::uint8_t>(width * height * 3, 40)},
                3s);
            continue;
          }
        } else if (kind == "decision") {
          ++decisions;
          Hold(hold_commit);
          if (Text(input["action"]) == "accept") {
            confirmed = proposed;
            ++commits;
            if (Text(input["operation"]) != "reuse") {
              source_generation = Text(input["generation"]);
              content_job = Text(input["job_id"]);
            } else if (!pending_source_.empty()) {
              source_generation = pending_source_;
              content_job = pending_content_;
            }
            out << "type"
                << "commit_ok"
                << "source_generation" << source_generation << "content_job_id"
                << content_job << "frame_ids"
                << "[";
            for (const auto& id : confirmed) out << id;
            out << "]"
                << "gaussian_count" << static_cast<int>(confirmed.size());
          } else {
            ++discards;
            out << "type"
                << "discard_ok";
          }
        } else if (kind == "checkpoint") {
          ++checkpoints;
          Hold(hold_checkpoint);
          const auto asset = Text(input["asset_id"]);
          fs::create_directory(states / asset);
          cv::FileStorage saved((states / asset / "peer.json").string(),
                                cv::FileStorage::WRITE);
          saved << "frames"
                << "[";
          for (const auto& id : confirmed) saved << id;
          saved << "]";
          saved.release();
          out << "type"
              << "checkpoint_ok"
              << "asset_id" << asset << "source_generation"
              << Text(input["source_generation"]) << "content_job_id"
              << (corrupt_checkpoint ? "999999" : Text(input["content_job_id"]))
              << "manifest_sha256" << std::string(64, 'b') << "manifest_bytes"
              << "2048"
              << "gaussian_bytes"
              << "1024"
              << "observation_bytes"
              << "4096"
              << "frame_ids"
              << "[";
          for (const auto& id : confirmed) out << id;
          out << "]"
              << "gaussian_count" << static_cast<int>(confirmed.size());
        } else if (kind == "map_committed") {
          if (input["asset_ids"].size()) Hold(hold_map_ack);
          ++acknowledgements;
          out << "type"
              << "map_committed_ok"
              << "map_sequence" << Text(input["map_sequence"]) << "asset_ids"
              << "[";
          for (const auto& id : input["asset_ids"]) out << Text(id);
          out << "]";
        } else if (kind == "prune") {
          ++prunes;
          fs::remove_all(states / Text(input["asset_id"]));
          out << "type"
              << "prune_ok"
              << "map_sequence" << Text(input["map_sequence"]) << "asset_id"
              << Text(input["asset_id"]);
        } else if (kind == "stop") {
          out << "type"
              << "stopped"
              << "map_sequence" << Text(input["map_sequence"]);
          if (!input["incomplete"].empty())
            out << "incomplete" << static_cast<int>(input["incomplete"]);
          WriteGaussianMessage(client_, {out.releaseAndGetString(), {}}, 3s);
          return;
        } else {
          throw std::runtime_error("Unexpected request " + kind);
        }
        if (kind == "activate" && Text(input["action"]) != "empty") {
          pending_source_ = Text(input["source_generation"]);
          pending_content_ = Text(input["content_job_id"]);
        }
        WriteGaussianMessage(client_, {out.releaseAndGetString(), {}}, 3s);
      }
    } catch (const std::exception& exception) {
      if (!stopping) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = exception.what();
      }
    }
  }

  fs::path path_;
  int listener_ = -1;
  std::atomic<int> client_{-1};
  mutable std::mutex mutex_;
  std::vector<std::string> events_;
  std::string pending_source_, pending_content_, error_;
  std::thread thread_;
};

class PersistentGaussianMapperTest : public testing::Test {
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
    options_.shutdown_timeout = 2s;
    options_.job_timeout = 5s;
    options_.connect_timeout = 2s;
    peer_ = std::make_unique<SubmapPeer>(options_.socket_path);
    mapper_ = std::make_unique<OnlineGaussianMapper>(
        *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}},
        options_);
  }

  void TearDown() override {
    if (peer_)
      peer_->hold_candidate = peer_->hold_commit = peer_->hold_checkpoint =
          peer_->hold_map_ack = false;
    fail_map_sync = false;
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

TEST_F(PersistentGaussianMapperTest,
       RoundedCalibrationMatchesReconstructionCamera) {
  rounded_camera_ = true;
  Append();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted == 1; }));
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
}

TEST_F(PersistentGaussianMapperTest,
       FirstCheckpointIsAcknowledgedBeforeLeaving) {
  Append();
  Confirm();
  peer_->hold_map_ack = true;
  Append(true);
  ASSERT_TRUE(Wait([&] { return peer_->checkpoints == 1; }));
  ASSERT_TRUE(Wait([&] {
    return fs::exists(options_.output_directory / "map.json") &&
           Entry(0).asset.has_value();
  }));
  for (int i = 0; i < 30; ++i) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(peer_->views, 1);
  peer_->hold_map_ack = false;
  ASSERT_TRUE(Wait([&] { return peer_->views == 2; }));
  ASSERT_TRUE(Wait([&] { return Entry(1).content.has_value(); }));
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_TRUE(Entry(0).asset);
  EXPECT_TRUE(Entry(1).asset);
  const auto events = peer_->Events();
  EXPECT_LT(std::find(events.begin(), events.end(), "checkpoint:0"),
            std::find(events.begin(), events.end(), "activate:1"));
}

TEST_F(PersistentGaussianMapperTest, EmptyActivationReleasesPriorResident) {
  Append();
  Confirm();
  peer_->hold_candidate = true;
  Append(true);
  ASSERT_TRUE(Wait([&] { return peer_->views == 2; }));
  EXPECT_TRUE(Entry(0).asset);
  EXPECT_FALSE(Entry(0).resident);
  EXPECT_FALSE(Entry(1).resident);
}

TEST_F(PersistentGaussianMapperTest,
       SwitchDuringMutatedCandidateSavesPreviousConfirmation) {
  Append();
  Confirm();
  const auto prior = Entry(0).content;
  peer_->hold_candidate = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->views == 2; }));
  Append(true);
  Correct(true, 1);
  peer_->hold_candidate = false;
  ASSERT_TRUE(
      Wait([&] { return peer_->discards >= 1 && Entry(0).asset.has_value(); }));
  EXPECT_EQ(Entry(0).asset->content, *prior);
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
}

TEST_F(PersistentGaussianMapperTest,
       PoseUpdateDuringSaveCannotRepublishOldCurrent) {
  Append();
  Confirm();
  Append();
  Confirm();
  peer_->hold_checkpoint = true;
  Append(true);
  ASSERT_TRUE(Wait([&] { return peer_->checkpoints == 1; }));
  Correct(true, 1);
  EXPECT_EQ(Entry(0).availability, GaussianAvailability::kNeedsRebuild);
  peer_->hold_checkpoint = false;
  ASSERT_TRUE(Wait([&] { return Entry(0).asset.has_value(); }));
  EXPECT_FALSE(Entry(0).asset_current);
  EXPECT_EQ(Entry(0).asset_source.pose_version, 0u);
}

TEST_F(PersistentGaussianMapperTest, OldCheckpointCannotReplaceNewerContent) {
  Append();
  Confirm();
  peer_->corrupt_checkpoint = true;
  Append(true);
  ASSERT_TRUE(Wait([&] { return peer_->checkpoints == 1; }));
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!mapper_->GetStats().failed &&
         std::chrono::steady_clock::now() < deadline) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_FALSE(Entry(0).asset);
  EXPECT_EQ(peer_->prunes, 0);
}

TEST_F(PersistentGaussianMapperTest, RigidRebindDoesNotResetOrRewriteArrays) {
  Append();
  Confirm();
  const auto content = Entry(0).content;
  Correct();
  ASSERT_TRUE(Wait(
      [&] { return peer_->reuses == 1 && mapper_->GetStats().accepted >= 2; }));
  EXPECT_EQ(Entry(0).content, content);
  EXPECT_EQ(Entry(0).binding.pose_version, 1u);
  EXPECT_EQ(peer_->checkpoints, 0);
  EXPECT_EQ(peer_->rebuilds, 0);
  EXPECT_EQ(mapper_->GetLatestPreview()->steps, 0);
}

TEST_F(PersistentGaussianMapperTest,
       EvictedMemberChangeInvalidatesImmediately) {
  for (int i = 0; i < 12; ++i) {
    Append();
    Confirm();
  }
  ASSERT_EQ(Entry(0).frame_ids.front(), 2u);
  Correct(true, 1);
  EXPECT_EQ(Entry(0).availability, GaussianAvailability::kNeedsRebuild);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  ASSERT_TRUE(Wait([&] {
    return peer_->rebuilds == 1 &&
           Entry(0).availability == GaussianAvailability::kCurrent;
  }));
  EXPECT_EQ(Entry(0).frame_ids.front(), 2u);
}

TEST_F(PersistentGaussianMapperTest, TwoViewsThenHistoricalRepair) {
  Append();
  Confirm();
  Append();
  Confirm();
  Append(true);
  Confirm();
  Correct(true, 1);
  // Continuously offer foreground views, retaining only the newest pending one.
  for (int i = 0; i < 60 && peer_->rebuilds == 0; ++i) {
    Append();
    for (int j = 0; j < 5; ++j) {
      mapper_->Poll();
      std::this_thread::sleep_for(1ms);
    }
  }
  ASSERT_TRUE(Wait([&] { return peer_->rebuilds >= 1; }));
  const auto events = peer_->Events();
  const auto repair =
      std::find(events.rbegin(), events.rend(), "activate:0").base();
  EXPECT_LE(std::count(events.begin(), repair, "view:1"), 3);
  EXPECT_EQ(mapper_->GetStats().peak_pending, 1u);
  EXPECT_EQ(mapper_->GetStats().peak_in_flight, 1u);
}

TEST_F(PersistentGaussianMapperTest, RepeatedStatusUpdatesDoNotStarveWork) {
  Append();
  for (int i = 0; i < 200 && peer_->views == 0; ++i) {
    mapper_->Observe({{}, {}, poses_, {}});
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(Wait([&] { return peer_->views == 1; }));
  EXPECT_LT(peer_->acknowledgements, 10);
}

TEST_F(PersistentGaussianMapperTest, SkippedGroupsAreMissing) {
  Append(false, false);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().skipped_no_image == 1; }));
  Append(true);
  Confirm();
  EXPECT_EQ(Entry(0).availability, GaussianAvailability::kMissing);
  EXPECT_FALSE(Entry(0).content);
}

TEST_F(PersistentGaussianMapperTest, CapacityKeepsOldAssets) {
  for (int i = 0; i < 32; ++i) {
    Append(i != 0);
    Confirm();
  }
  Append(true);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().resource_limited >= 1; }));
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().failed, 0u);
  EXPECT_FALSE(Entry(32).content);
  EXPECT_EQ(Entry(32).reason, "resource_limit");
  for (int i = 0; i < 32; ++i) EXPECT_TRUE(Entry(i).asset);
}

TEST_F(PersistentGaussianMapperTest, StopFlushesDirtyOrReportsIncomplete) {
  Append();
  Confirm();
  mapper_->Finish();
  EXPECT_TRUE(Entry(0).asset);
  EXPECT_FALSE(Entry(0).dirty);
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
}

TEST_F(PersistentGaussianMapperTest, WrongStopIdentityReportsIncomplete) {
  Append();
  Confirm();
  peer_->wrong_stop = true;
  mapper_->Finish();
  EXPECT_TRUE(mapper_->GetStats().incomplete_shutdown);
}

TEST_F(PersistentGaussianMapperTest,
       StopTimeoutReportsUnpersistedConfirmation) {
  Append();
  Confirm();
  peer_->hold_checkpoint = true;
  mapper_->Finish();
  EXPECT_TRUE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_TRUE(Entry(0).dirty);
  EXPECT_FALSE(Entry(0).asset);
}

TEST_F(PersistentGaussianMapperTest,
       LateCommitIsRecordedOnlyAsHistoricalContent) {
  Append();
  Confirm();
  peer_->hold_commit = true;
  Append();
  ASSERT_TRUE(Wait([&] { return peer_->decisions == 2; }));
  Correct(true, 1);
  peer_->hold_candidate = true;
  peer_->hold_commit = false;
  ASSERT_TRUE(Wait([&] { return Entry(0).frame_ids.size() == 2; }));
  EXPECT_EQ(Entry(0).availability, GaussianAvailability::kNeedsRebuild);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  EXPECT_EQ(Entry(0).content_target.pose_version, 0u);
}

TEST_F(PersistentGaussianMapperTest, RealPythonWorkerPersistsAndRepairs) {
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
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}}, options_);
  for (int i = 0; i < 12; ++i) {
    Append();
    Confirm();
  }
  EXPECT_TRUE(mapper_->GetLatestPreview()->nonzero_gradient);
  EXPECT_TRUE(mapper_->GetLatestPreview()->parameter_updated);
  Append(true);
  Confirm();
  EXPECT_TRUE(Entry(0).asset);
  const auto content = Entry(1).content;
  Correct();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().rebinds == 1; }));
  EXPECT_EQ(Entry(1).content, content);
  EXPECT_EQ(mapper_->GetLatestPreview()->steps, 0);
  Correct(true, 1);
  EXPECT_EQ(Entry(0).availability, GaussianAvailability::kNeedsRebuild);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().rebuilds == 1; }));
  EXPECT_EQ(Entry(0).frame_ids.front(), 2u);
  mapper_->Finish();
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  EXPECT_EQ(mapper_->GetStats().checkpoints, 3u);
  EXPECT_EQ(mapper_->GetStats().prunes, 1u);
  EXPECT_TRUE(Entry(0).asset_current);
  EXPECT_TRUE(Entry(1).asset_current);
  EXPECT_EQ(Entry(0).asset_source.pose_version, 2u);
}

TEST_F(PersistentGaussianMapperTest,
       RealPythonWorkerRejectsFirstViewThenLoadsHistory) {
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
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      *system_, CameraConfig{"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}}, options_);
  Append();
  Confirm();
  Append();
  Confirm();
  const auto submitted = mapper_->GetStats().submitted;
  Append(true);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().submitted > submitted; }));
  // Correct before taking the candidate from the mailbox, regardless of GPU
  // timing. B's first view must be rejected; A must then load from its asset.
  Correct(true, 1);
  ASSERT_TRUE(Wait([&] { return Entry(1).content.has_value(); }));
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().rebuilds >= 1; }));
  EXPECT_GE(mapper_->GetStats().stale, 1u);
  EXPECT_GE(mapper_->GetStats().loads, 1u);
  mapper_->Finish();
  EXPECT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  EXPECT_FALSE(mapper_->GetStats().incomplete_shutdown);
  EXPECT_TRUE(Entry(0).asset_current);
  EXPECT_TRUE(Entry(1).asset_current);
}

TEST_F(PersistentGaussianMapperTest,
       UncertainIndexPreventsPruneAndFurtherWrites) {
  Append();
  Confirm();
  Append(true);
  Confirm();
  Correct();
  ASSERT_TRUE(Wait([&] { return peer_->reuses >= 1; }));
  map_sync_path = options_.output_directory.string();
  fail_map_sync = true;
  Append();
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!mapper_->GetStats().failed &&
         std::chrono::steady_clock::now() < deadline) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_NE(mapper_->GetStats().error.find("uncertain"), std::string::npos);
  EXPECT_EQ(peer_->prunes, 0);
  EXPECT_TRUE(fs::exists(options_.output_directory / "map.json"));
}

}  // namespace
}  // namespace livo_atlas
