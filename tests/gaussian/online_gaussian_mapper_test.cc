#include "livo_atlas/core/online_gaussian_options.h"
#include "livo_atlas/gaussian/online_gaussian_mapper.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include "core/fixtures.h"
#include "gaussian/wire_protocol.h"
#include "livo_atlas/core/livo_system.h"
#if LIVO_ATLAS_TEST_WITH_BACKEND
#include "core/loop_backend_fixtures.h"
#endif

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
using namespace std::chrono_literals;

std::string String(const cv::FileNode& node) {
  return static_cast<std::string>(node);
}

// A real socket peer whose candidate and confirmation can be delayed by the
// test. The source observation remains owned by the mapper while it waits.
class FakeWorker {
 public:
  explicit FakeWorker(const fs::path& path) : path_(path) {
    listener_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listener_ < 0) throw std::runtime_error("socket failed");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    const auto name = path.string();
    if (name.size() >= sizeof(address.sun_path))
      throw std::runtime_error("socket path too long");
    std::copy(name.begin(), name.end(), address.sun_path);
    if (bind(listener_, reinterpret_cast<sockaddr*>(&address),
             sizeof(address)) ||
        listen(listener_, 1))
      throw std::runtime_error("listen failed");
    thread_ = std::thread(&FakeWorker::Run, this);
  }

  ~FakeWorker() {
    stopping_ = true;
    hold_candidate_ = false;
    hold_confirmation_ = false;
    const int client = client_.load();
    if (client >= 0) shutdown(client, SHUT_RDWR);
    shutdown(listener_, SHUT_RDWR);
    if (thread_.joinable()) thread_.join();
    if (client >= 0) close(client);
    close(listener_);
    fs::remove(path_);
  }

  void HoldCandidate() { hold_candidate_ = true; }

  void ReleaseCandidate() { hold_candidate_ = false; }

  void HoldConfirmation() { hold_confirmation_ = true; }

  void ReleaseConfirmation() { hold_confirmation_ = false; }

  void CorruptCoverage() { corrupt_coverage_ = true; }

  void WrongSession() { wrong_session_ = true; }

  void CloseOnView() { close_on_view_ = true; }

  int views() const { return views_; }

  int decisions() const { return decisions_; }

  int resets() const { return resets_; }

  int discards() const { return discards_; }

  bool disconnected() const {
    const int client = client_.load();
    if (client < 0) return false;
    char byte;
    const auto count = recv(client, &byte, 1, MSG_PEEK | MSG_DONTWAIT);
    return count == 0 || (count < 0 && errno == ECONNRESET);
  }

  std::uint64_t last_reset_generation() const { return last_reset_generation_; }

  std::vector<std::uint64_t> received_frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_frames_;
  }

 private:
  static cv::FileStorage Parse(const GaussianMessage& message) {
    return cv::FileStorage(message.header_json,
                           cv::FileStorage::READ | cv::FileStorage::MEMORY |
                               cv::FileStorage::FORMAT_JSON);
  }

  static void Start(cv::FileStorage& writer, const std::string& kind,
                    const std::string& session) {
    writer << "schema" << 1 << "type" << kind << "session_id" << session;
  }

  void Run() {
    client_ = accept(listener_, nullptr, nullptr);
    if (client_ < 0) return;
    try {
      const auto hello = ReadGaussianMessage(client_, 10s);
      auto parsed = Parse(hello);
      const auto session = String(parsed["session_id"]);
      cv::FileStorage reply(".json", cv::FileStorage::WRITE |
                                         cv::FileStorage::MEMORY |
                                         cv::FileStorage::FORMAT_JSON);
      Start(reply, "hello_ok", wrong_session_ ? std::string(32, 'b') : session);
      reply << "python"
            << "test"
            << "torch"
            << "test"
            << "gsplat"
            << "test"
            << "cuda"
            << "test"
            << "device"
            << "test"
            << "gpu_free_bytes"
            << "5000000000"
            << "gpu_total_bytes"
            << "8000000000";
      WriteGaussianMessage(client_, {reply.releaseAndGetString(), {}}, 2s);
      std::vector<std::string> frames;
      for (;;) {
        const auto message = ReadGaussianMessage(client_, 30s);
        auto input = Parse(message);
        const auto kind = String(input["type"]);
        if (kind == "stop") return;
        cv::FileStorage output(".json", cv::FileStorage::WRITE |
                                            cv::FileStorage::MEMORY |
                                            cv::FileStorage::FORMAT_JSON);
        if (kind == "reset") {
          Start(output, "reset_ok", session);
          output << "generation" << String(input["generation"])
                 << "pose_version" << String(input["pose_version"]);
          if (!input["submap_id"].empty())
            output << "submap_id" << String(input["submap_id"]);
          frames.clear();
          last_reset_generation_ = std::stoull(String(input["generation"]));
          ++resets_;
          WriteGaussianMessage(client_, {output.releaseAndGetString(), {}}, 2s);
          continue;
        }
        if (kind != "view") throw std::runtime_error("unexpected request");
        const auto frame = String(input["keyframe_id"]);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          received_frames_.push_back(std::stoull(frame));
        }
        ++views_;
        if (close_on_view_) {
          shutdown(client_, SHUT_RDWR);
          return;
        }
        while (hold_candidate_ && !stopping_) std::this_thread::sleep_for(1ms);
        if (stopping_) return;
        frames.push_back(frame);
        if (frames.size() > 10) frames.erase(frames.begin());
        Start(output, "candidate", session);
        for (const char* key :
             {"job_id", "generation", "submap_id", "pose_version",
              "membership_revision", "frame_count", "optimized_frame_count"})
          output << key << String(input[key]);
        const int width = static_cast<int>(input["width"]);
        const int height = static_cast<int>(input["height"]);
        output << "width" << width << "height" << height << "frame_ids"
               << "[";
        if (corrupt_coverage_ && frames.size() > 1) {
          output << frames.back();
        } else {
          for (const auto& id : frames) output << id;
        }
        output << "]"
               << "steps" << 20 << "rgb_l1" << .2 << "depth_l1_m" << .1
               << "supervised_depth_pixels" << 20 << "gaussian_count" << 1;
        GaussianMessage candidate{
            output.releaseAndGetString(),
            std::vector<std::uint8_t>(width * height * 3, 40)};
        WriteGaussianMessage(client_, candidate, 2s);
        const auto decision = ReadGaussianMessage(client_, 10s);
        auto chosen = Parse(decision);
        ++decisions_;
        while (hold_confirmation_ && !stopping_)
          std::this_thread::sleep_for(1ms);
        if (stopping_) return;
        const bool accept = String(chosen["action"]) == "accept";
        if (!accept) {
          ++discards_;
          frames.clear();
        }
        cv::FileStorage confirmation(".json", cv::FileStorage::WRITE |
                                                  cv::FileStorage::MEMORY |
                                                  cv::FileStorage::FORMAT_JSON);
        Start(confirmation, accept ? "commit_ok" : "discard_ok", session);
        for (const char* key :
             {"job_id", "generation", "submap_id", "pose_version"})
          confirmation << key << String(chosen[key]);
        WriteGaussianMessage(client_, {confirmation.releaseAndGetString(), {}},
                             2s);
      }
    } catch (const std::exception&) {
      // Shutdown and the mapper's deliberate failure tests close the socket.
    }
  }

  fs::path path_;
  int listener_ = -1;
  std::atomic<int> client_{-1};
  std::atomic<bool> stopping_{false}, hold_candidate_{false},
      hold_confirmation_{false};
  std::atomic<bool> corrupt_coverage_{false}, wrong_session_{false},
      close_on_view_{false};
  std::atomic<int> views_{0}, decisions_{0}, resets_{0}, discards_{0};
  std::atomic<std::uint64_t> last_reset_generation_{0};
  mutable std::mutex mutex_;
  std::vector<std::uint64_t> received_frames_;
  std::thread thread_;
};

class OnlineGaussianMapperTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("livo-gs-" + std::to_string(getpid()) + "-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(root_);
    auto options = test::Options(FrontendMode::kLidarOnly);
    options.exporter.directory = root_ / "dataset";
    options.keyframes.enabled = options.submaps.enabled = true;
    options.submap_mode = SubmapMode::kOnline;
    system_ = std::make_unique<LivoSystem>(options);
    camera_ = {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
    options_.enabled = true;
    options_.socket_path = root_ / "peer.sock";
    options_.output_directory = root_ / "output";
    options_.connect_timeout = 3s;
    options_.transfer_timeout = 2s;
    options_.job_timeout = 5s;
    peer_ = std::make_unique<FakeWorker>(options_.socket_path);
  }

  void TearDown() override {
    mapper_.reset();
    peer_.reset();
    system_.reset();
    fs::remove_all(root_);
  }

  void Start() {
    mapper_ =
        std::make_unique<OnlineGaussianMapper>(*system_, camera_, options_);
  }

  std::shared_ptr<PoseSnapshot> Poses(std::uint64_t frame,
                                      std::uint64_t version = 0) {
    auto poses = std::make_shared<PoseSnapshot>();
    poses->version = version;
    poses->base_version = version ? version - 1 : 0;
    poses->frame_count = frame + 1;
    poses->optimized_frame_count = version ? frame + 1 : 0;
    poses->parent_frame = version ? PoseFrame::kMap : PoseFrame::kOdom;
    if (version)
      poses->tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
    for (std::uint64_t id = 0; id <= frame; ++id)
      poses->poses.push_back({id, {}});
    return poses;
  }

  std::shared_ptr<Keyframe> Frame(std::uint64_t frame) {
    auto keyframe = std::make_shared<Keyframe>();
    keyframe->id = frame;
    keyframe->timestamp = keyframe->cloud_timestamp = 100 + frame;
    keyframe->image_timestamp = 100 + frame;
    keyframe->camera_from_odom = RigidTransform{};
    keyframe->image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(10, 20, 30));
    PointCloud::Ptr cloud(new PointCloud);
    Point point{};
    point.z = 4;
    cloud->push_back(point);
    keyframe->cloud = cloud;
    return keyframe;
  }

  void Observe(std::uint64_t frame, std::uint64_t version = 0) {
    auto poses = Poses(frame, version);
    auto keyframe = Frame(frame);
    OnlineSubmap group;
    group.membership_revision = frame + 1;
    for (std::uint64_t id = 0; id <= frame; ++id)
      group.primary.keyframe_ids.push_back(id);
    mapper_->Observe({keyframe, {group}, poses, {}});
  }

  void PoseOnly(std::uint64_t frame, std::uint64_t version) {
    mapper_->Observe({{}, {}, Poses(frame, version), {}});
  }

  template <typename Predicate>
  bool Wait(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 30s;
    while (std::chrono::steady_clock::now() < deadline) {
      mapper_->Poll();
      if (predicate()) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  bool WaitForDisconnect() {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (peer_->disconnected()) return true;
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  fs::path root_;
  CameraConfig camera_;
  OnlineGaussianOptions options_;
  std::unique_ptr<LivoSystem> system_;
  std::unique_ptr<FakeWorker> peer_;
  std::unique_ptr<OnlineGaussianMapper> mapper_;
};

TEST(OnlineGaussianOptionsTest, ValidatesOnlyEnabledConfiguration) {
  OnlineGaussianOptions options;
  EXPECT_NO_THROW(ValidateOnlineGaussianOptions(options));
  options.enabled = true;
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
  options.socket_path = "/tmp/livo-gaussian.sock";
  options.output_directory = "/tmp/livo-gaussian-output";
  EXPECT_NO_THROW(ValidateOnlineGaussianOptions(options));
  options.job_timeout = std::chrono::milliseconds(0);
  EXPECT_THROW(ValidateOnlineGaussianOptions(options), std::invalid_argument);
}

TEST_F(OnlineGaussianMapperTest,
       CoalescesPendingWithoutBlockingOrInvalidatingFrontier) {
  peer_->HoldCandidate();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->views() == 1; }));
  const auto start = std::chrono::steady_clock::now();
  Observe(1);
  Observe(2);
  mapper_->Poll();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  peer_->ReleaseCandidate();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted == 2; }));
  EXPECT_EQ(peer_->received_frames(), (std::vector<std::uint64_t>{0, 2}));
  const auto preview = mapper_->GetLatestPreview();
  ASSERT_TRUE(preview);
  EXPECT_EQ(preview->frame_ids, (std::vector<std::uint64_t>{0, 2}));
  EXPECT_EQ(mapper_->GetStats().coalesced, 1u);
  EXPECT_EQ(mapper_->GetStats().peak_pending, 1u);
  const auto stats = mapper_->GetStats();
  EXPECT_EQ(std::accumulate(stats.latency_buckets.begin(),
                            stats.latency_buckets.end(), std::uint64_t{0}),
            stats.accepted);
  EXPECT_GE(stats.max_latency_seconds, stats.last_latency_seconds);
  EXPECT_GE(preview->latency_seconds, 0);
}

TEST_F(OnlineGaussianMapperTest,
       PoseChangeAfterAcceptNeverPublishesOldConfirmation) {
  peer_->HoldConfirmation();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->decisions() == 1; }));
  PoseOnly(0, 1);
  PoseOnly(0, 2);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  peer_->ReleaseConfirmation();
  ASSERT_TRUE(Wait([&] {
    const auto preview = mapper_->GetLatestPreview();
    return preview && preview->pose_version == 2;
  }));
  EXPECT_GE(peer_->resets(), 2);
  EXPECT_EQ(peer_->last_reset_generation(), mapper_->GetStats().generation);
  EXPECT_GE(mapper_->GetStats().stale, 1u);
  EXPECT_EQ(mapper_->GetLatestPreview()->frame_ids,
            (std::vector<std::uint64_t>{0}));
}

#if LIVO_ATLAS_TEST_WITH_BACKEND
TEST_F(OnlineGaussianMapperTest,
       BackendFinishDeliversItsFinalPoseBeforeMapperShutdown) {
  auto options = loop_test::OnlineOptions();
  options.exporter.directory = root_ / "loop-dataset";
  LivoSystem system(options);
  LoopBackend backend(system, loop_test::NearbyOptions());
  mapper_ = std::make_unique<OnlineGaussianMapper>(
      system, options.frontend.calibration.camera, options_);
  system.SetOnlineSubmapCallback([&](const auto& update) {
    backend.Observe(update);
    mapper_->Observe(update);
  });
  const auto cloud = loop_test::DenseCloud();
  for (std::uint64_t id = 0; id < 4; ++id)
    LoopBackendTestPeer::Append(system, loop_test::DenseFrame(id, cloud));
  backend.Finish();
  ASSERT_GT(system.GetPoseSnapshot()->version, 0u);
  EXPECT_EQ(mapper_->GetStats().pose_version,
            system.GetPoseSnapshot()->version);
  ASSERT_TRUE(Wait([&] {
    return peer_->resets() > 0 &&
           peer_->last_reset_generation() == mapper_->GetStats().generation;
  }));
  EXPECT_FALSE(mapper_->GetLatestPreview());
  mapper_->Finish();
}
#endif

TEST_F(OnlineGaussianMapperTest,
       PoseChangeBeforeCandidateDiscardsAndResetsLatestVersion) {
  peer_->HoldCandidate();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->views() == 1; }));
  PoseOnly(0, 1);
  PoseOnly(0, 2);
  peer_->ReleaseCandidate();
  ASSERT_TRUE(Wait([&] {
    const auto preview = mapper_->GetLatestPreview();
    return preview && preview->pose_version == 2;
  }));
  EXPECT_EQ(peer_->discards(), 1);
  EXPECT_EQ(peer_->last_reset_generation(), mapper_->GetStats().generation);
  EXPECT_EQ(mapper_->GetLatestPreview()->frame_ids,
            (std::vector<std::uint64_t>{0}));
}

TEST_F(OnlineGaussianMapperTest,
       SealedSubmapRemainsCurrentUntilNewPrimaryFrame) {
  Start();
  OnlineSubmap first;
  first.primary.keyframe_ids = {0};
  auto sealed = first;
  sealed.state = OnlineSubmapState::kSealed;
  sealed.membership_revision = 2;
  mapper_->Observe({Frame(0), {first, sealed}, Poses(0), {}});
  ASSERT_TRUE(Wait([&] { return mapper_->GetLatestPreview() != nullptr; }));
  EXPECT_EQ(mapper_->GetLatestPreview()->submap_id, 0u);
  OnlineSubmap second;
  second.primary.id = 1;
  second.primary.anchor_keyframe_id = 1;
  second.primary.keyframe_ids = {1};
  second.support_keyframe_ids = {0};
  mapper_->Observe({Frame(1), {sealed, second}, Poses(1), {}});
  EXPECT_FALSE(mapper_->GetLatestPreview());
  ASSERT_TRUE(Wait([&] {
    const auto preview = mapper_->GetLatestPreview();
    return preview && preview->submap_id == 1;
  }));
  EXPECT_EQ(mapper_->GetLatestPreview()->frame_ids,
            (std::vector<std::uint64_t>{1}));
}

TEST_F(OnlineGaussianMapperTest, RetainsEightCompletePreviewPairs) {
  Start();
  for (std::uint64_t frame = 0; frame < 10; ++frame) {
    Observe(frame);
    ASSERT_TRUE(Wait([&] {
      return mapper_->GetStats().accepted == frame + 1 &&
             fs::exists(options_.output_directory /
                        ("preview-" + std::to_string(frame + 1) + ".json"));
    }));
  }
  std::size_t images = 0, metadata = 0;
  for (const auto& entry : fs::directory_iterator(options_.output_directory)) {
    const auto name = entry.path().filename().string();
    if (name.rfind("preview-", 0) != 0) continue;
    if (entry.path().extension() == ".png") ++images;
    if (entry.path().extension() == ".json") ++metadata;
  }
  EXPECT_EQ(images, 8u);
  EXPECT_EQ(metadata, 8u);
  EXPECT_FALSE(fs::exists(options_.output_directory / "preview-1.png"));
  EXPECT_TRUE(fs::exists(options_.output_directory / "preview-10.png"));
}

TEST_F(OnlineGaussianMapperTest, RejectsCandidateCoverageNotActuallySubmitted) {
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted == 1; }));
  peer_->CorruptCoverage();
  Observe(1);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().failed == 1; }));
  EXPECT_NE(mapper_->GetStats().error.find("coverage"), std::string::npos);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  EXPECT_EQ(mapper_->GetStats().in_flight, 0u);
  EXPECT_TRUE(WaitForDisconnect());
}

TEST_F(OnlineGaussianMapperTest,
       RejectsReplacementOfAnExistingSubmapMembership) {
  Start();
  Observe(0);
  OnlineSubmap invalid;
  invalid.primary.anchor_keyframe_id = 1;
  invalid.primary.keyframe_ids = {1};
  mapper_->Observe({Frame(1), {invalid}, Poses(1), {}});
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_FALSE(mapper_->GetLatestPreview());
}

TEST_F(OnlineGaussianMapperTest, WrongSessionAndLostPeerStayInsideMapper) {
  peer_->WrongSession();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().failed == 1; }));
  EXPECT_NE(mapper_->GetStats().error.find("session"), std::string::npos);
  EXPECT_NO_THROW(Observe(1));
  EXPECT_FALSE(mapper_->GetLatestPreview());
}

TEST_F(OnlineGaussianMapperTest, WorkerTimeoutDoesNotBlockFrontend) {
  options_.job_timeout = 100ms;
  peer_->HoldCandidate();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->views() == 1; }));
  // The background timeout must close the connection without another Poll.
  EXPECT_TRUE(WaitForDisconnect());
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().failed == 1; }));
  const auto start = std::chrono::steady_clock::now();
  Observe(1);
  mapper_->Poll();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  peer_->ReleaseCandidate();
}

TEST_F(OnlineGaussianMapperTest, FinishClearsInFlightAfterInterruptingWorker) {
  options_.shutdown_timeout = 50ms;
  peer_->HoldCandidate();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->views() == 1; }));
  const auto start = std::chrono::steady_clock::now();
  mapper_->Finish();
  peer_->ReleaseCandidate();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  EXPECT_TRUE(mapper_->GetStats().finished);
  EXPECT_EQ(mapper_->GetStats().in_flight, 0u);
  EXPECT_EQ(mapper_->GetStats().pending, 0u);
}

TEST_F(OnlineGaussianMapperTest, SkipsMissingImageAndDepthBeforeSendingViews) {
  Start();
  auto missing_image = Frame(0);
  missing_image->image.release();
  missing_image->image_timestamp.reset();
  OnlineSubmap group;
  group.primary.keyframe_ids = {0};
  mapper_->Observe({missing_image, {group}, Poses(0), {}});
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().skipped_no_image == 1; }));
  auto missing_depth = Frame(1);
  PointCloud::Ptr behind(new PointCloud);
  Point point{};
  point.z = -4;
  behind->push_back(point);
  missing_depth->cloud = behind;
  group.primary.keyframe_ids = {0, 1};
  group.membership_revision = 2;
  mapper_->Observe({missing_depth, {group}, Poses(1), {}});
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().skipped_no_depth == 1; }));
  Observe(2);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted == 1; }));
  EXPECT_EQ(peer_->received_frames(), (std::vector<std::uint64_t>{2}));
  EXPECT_EQ(mapper_->GetLatestPreview()->frame_ids,
            (std::vector<std::uint64_t>{2}));
}

TEST_F(OnlineGaussianMapperTest, FrontierLimitDisablesOnlyTheMapper) {
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().accepted == 1; }));
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 513;
  mapper_->Observe({{}, {}, poses, {}});
  EXPECT_EQ(mapper_->GetStats().resource_limited, 1u);
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_EQ(mapper_->GetStats().in_flight, 0u);
  EXPECT_TRUE(WaitForDisconnect());
  EXPECT_NO_THROW(mapper_->Poll());
}

TEST_F(OnlineGaussianMapperTest, FrontierLimitCancelsAnInFlightView) {
  peer_->HoldCandidate();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return peer_->views() == 1; }));
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 513;
  mapper_->Observe({{}, {}, poses, {}});
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  EXPECT_EQ(mapper_->GetStats().in_flight, 0u);
  EXPECT_EQ(mapper_->GetStats().pending, 0u);
  EXPECT_TRUE(WaitForDisconnect());
  peer_->ReleaseCandidate();
}

TEST_F(OnlineGaussianMapperTest, PrimarySubmapMayContainMoreThan64Frames) {
  Start();
  for (std::uint64_t frame = 0; frame <= 64; ++frame) Observe(frame);
  EXPECT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  EXPECT_EQ(mapper_->GetStats().observed, 65u);
  EXPECT_EQ(mapper_->GetStats().pending, 1u);
}

TEST_F(OnlineGaussianMapperTest, PeerEofDoesNotEscapeObservationOrPolling) {
  peer_->CloseOnView();
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().failed == 1; }));
  EXPECT_EQ(mapper_->GetStats().in_flight, 0u);
  EXPECT_NO_THROW(Observe(1));
  EXPECT_NO_THROW(mapper_->Poll());
}

TEST_F(OnlineGaussianMapperTest,
       SlowStatusWriterEventuallyCatchesLatestGeneration) {
  Start();
  const auto fifo = options_.output_directory / "status.tmp.json";
  ASSERT_EQ(mkfifo(fifo.c_str(), 0600), 0);
  Observe(0);
  EXPECT_TRUE(Wait([&] { return peer_->resets() >= 1; }));
  // Let the background thread enter the FIFO open before changing state.
  for (int i = 0; i < 30; ++i) {
    mapper_->Poll();
    std::this_thread::sleep_for(1ms);
  }
  PoseOnly(0, 1);
  PoseOnly(0, 2);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  const int unblock = open(fifo.c_str(), O_RDWR | O_NONBLOCK);
  ASSERT_GE(unblock, 0);
  const bool caught_up = Wait([&] {
    const auto path = options_.output_directory / "status.json";
    if (!fs::is_regular_file(path)) return false;
    cv::FileStorage status(
        path.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    return status.isOpened() && String(status["pose_version"]) == "2";
  });
  close(unblock);
  EXPECT_TRUE(caught_up);
}

TEST_F(OnlineGaussianMapperTest, RealPythonWorkerConfirmsAndResetsGpuModel) {
  const char* socket = std::getenv("LIVO_REAL_GAUSSIAN_SOCKET");
  const char* output = std::getenv("LIVO_REAL_GAUSSIAN_OUTPUT");
  if (!socket || !output) GTEST_SKIP() << "Real GPU worker was not requested";
  peer_.reset();
  options_.socket_path = socket;
  options_.output_directory = output;
  options_.connect_timeout = 20s;
  options_.job_timeout = 30s;
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] {
    return mapper_->GetStats().accepted == 1 || mapper_->GetStats().failed;
  }));
  ASSERT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  const auto initial = mapper_->GetLatestPreview();
  ASSERT_TRUE(initial);
  EXPECT_TRUE(initial->nonzero_gradient);
  EXPECT_TRUE(initial->parameter_updated);
  EXPECT_GT(initial->supervised_depth_pixels, 0);
  EXPECT_GT(initial->gpu_peak_allocated_bytes, 0u);
  PoseOnly(0, 1);
  EXPECT_FALSE(mapper_->GetLatestPreview());
  ASSERT_TRUE(Wait([&] {
    const auto preview = mapper_->GetLatestPreview();
    return (preview && preview->pose_version == 1) ||
           mapper_->GetStats().failed;
  }));
  ASSERT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  EXPECT_EQ(mapper_->GetLatestPreview()->frame_ids,
            (std::vector<std::uint64_t>{0}));
  auto poses = Poses(1, 1);
  poses->optimized_frame_count = 1;
  OnlineSubmap group;
  group.primary.keyframe_ids = {0, 1};
  group.membership_revision = 2;
  mapper_->Observe({Frame(1), {group}, poses, {}});
  ASSERT_TRUE(Wait([&] {
    const auto preview = mapper_->GetLatestPreview();
    return (preview && preview->frame_ids.size() == 2) ||
           mapper_->GetStats().failed;
  }));
  ASSERT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  const auto preview = mapper_->GetLatestPreview();
  EXPECT_EQ(preview->frame_ids, (std::vector<std::uint64_t>{0, 1}));
  EXPECT_EQ(preview->steps, 20);
  EXPECT_TRUE(preview->nonzero_gradient);
  EXPECT_TRUE(preview->parameter_updated);
  std::cout << "Real GPU preview: depth_samples="
            << preview->supervised_depth_pixels << " rgb_l1=" << preview->rgb_l1
            << " depth_l1_m=" << preview->depth_l1_m
            << " gpu_peak_bytes=" << preview->gpu_peak_allocated_bytes
            << " worker_peak_rss_bytes=" << preview->worker_peak_rss_bytes
            << '\n';
  mapper_->Finish();
}

TEST_F(OnlineGaussianMapperTest, RealPythonWorkerExitsAfterOwnerFailure) {
  const char* socket = std::getenv("LIVO_REAL_GAUSSIAN_SOCKET");
  const char* output = std::getenv("LIVO_REAL_GAUSSIAN_OUTPUT");
  if (!socket || !output) GTEST_SKIP() << "Real GPU worker was not requested";
  peer_.reset();
  options_.socket_path = socket;
  options_.output_directory = output;
  options_.connect_timeout = 20s;
  options_.job_timeout = 30s;
  Start();
  Observe(0);
  ASSERT_TRUE(Wait([&] {
    return mapper_->GetStats().accepted == 1 || mapper_->GetStats().failed;
  }));
  ASSERT_EQ(mapper_->GetStats().failed, 0u) << mapper_->GetStats().error;
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 513;
  mapper_->Observe({{}, {}, poses, {}});
  EXPECT_EQ(mapper_->GetStats().failed, 1u);
  // The Python worker removes its socket when it observes the disconnect.
  const auto deadline = std::chrono::steady_clock::now() + 8s;
  while (fs::exists(socket) && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(1ms);
  EXPECT_FALSE(fs::exists(socket));
  EXPECT_FALSE(mapper_->GetStats().finished);
}

TEST_F(OnlineGaussianMapperTest, PersistentModeRejectsLegacyWorker) {
  options_.persist_submaps = true;
  Start();
  ASSERT_TRUE(Wait([&] { return mapper_->GetStats().failed == 1; }));
  EXPECT_NE(mapper_->GetStats().error.find("mode mismatch"), std::string::npos);
  EXPECT_FALSE(fs::exists(options_.output_directory / "states"));
  EXPECT_FALSE(fs::exists(options_.output_directory / "map.json"));
  EXPECT_EQ(system_->GetPoseSnapshot()->frame_count, 0u);
}

TEST_F(OnlineGaussianMapperTest, RefusesExistingOutputDirectory) {
  fs::create_directory(options_.output_directory);
  EXPECT_THROW(Start(), std::exception);
  EXPECT_TRUE(fs::is_empty(options_.output_directory));
}

TEST_F(OnlineGaussianMapperTest, RefusesOutputInsideSourceDataset) {
  options_.output_directory =
      system_->GetKeyframeReadView().directory() / "previews";
  EXPECT_THROW(Start(), std::exception);
  EXPECT_FALSE(fs::exists(options_.output_directory));
}

}  // namespace
}  // namespace livo_atlas
