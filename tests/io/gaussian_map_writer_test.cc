#include "livo_atlas/io/gaussian_map_writer.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <system_error>

#include "livo_atlas/gaussian/gaussian_map_index.h"

namespace {

enum class Fault {
  kNone,
  kFileSync,
  kFileClose,
  kRename,
  kDirectorySync,
  kParentSync
};
Fault fault = Fault::kNone;
std::string output_path;
std::string temporary_name = "map.tmp.json";

std::string DescriptorPath(int fd) {
  char value[4096];
  const auto name = "/proc/self/fd/" + std::to_string(fd);
  const auto size = readlink(name.c_str(), value, sizeof(value));
  return size < 0 ? "" : std::string(value, static_cast<std::size_t>(size));
}

}  // namespace

// Faults affect only the test's selected output, while the real syscall still
// handles all other descriptors. No product-only injection interface is needed.
extern "C" int fsync(int fd) {
  const auto path = DescriptorPath(fd);
  const bool fail =
      (fault == Fault::kFileSync &&
       path == output_path + "/" + temporary_name) ||
      (fault == Fault::kDirectorySync && path == output_path) ||
      (fault == Fault::kParentSync &&
       path == std::filesystem::path(output_path).parent_path().string());
  if (fail) {
    errno = EIO;
    return -1;
  }
  return static_cast<int>(syscall(SYS_fsync, fd));
}

extern "C" int close(int fd) {
  const bool fail = fault == Fault::kFileClose &&
                    DescriptorPath(fd) == output_path + "/" + temporary_name;
  const auto result = static_cast<int>(syscall(SYS_close, fd));
  if (fail) {
    errno = EIO;
    return -1;
  }
  return result;
}

extern "C" int rename(const char* old_path, const char* new_path) noexcept {
  if (fault == Fault::kRename &&
      std::string(old_path) == output_path + "/" + temporary_name) {
    errno = EIO;
    return -1;
  }
  return static_cast<int>(syscall(SYS_rename, old_path, new_path));
}

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string Bytes(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), {}};
}

GaussianMapSnapshot Snapshot() {
  constexpr char kSession[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  GaussianMapIndex index(kSession);
  auto poses = std::make_shared<PoseSnapshot>();
  poses->frame_count = 1;
  poses->poses.push_back({0, {}});
  OnlineSubmap first;
  first.primary = {0, 0, {0}};
  index.Observe({{}, {first}, poses, {}});
  index.RecordConfirmed(index.MakeTarget(0, 1), GaussianOperation::kView, 1,
                        {0}, 1);
  GaussianAssetReference asset;
  asset.asset_id = 1;
  asset.content = *index.Snapshot().entries[0].content;
  asset.manifest_sha256 = std::string(64, 'b');
  asset.manifest_bytes = 2048;
  asset.gaussian_bytes = 1024;
  asset.observation_bytes = 4096;
  index.RecordCheckpoint(asset);
  auto next = std::make_shared<PoseSnapshot>(*poses);
  next->frame_count = 2;
  RigidTransform second;
  second.translation.x() = 1;
  next->poses.push_back({1, second});
  first.membership_revision = 2;
  first.state = OnlineSubmapState::kSealed;
  OnlineSubmap group;
  group.primary = {1, 1, {1}};
  index.Observe({{}, {first, group}, next, {}});
  index.RecordConfirmed(index.MakeTarget(1, 2), GaussianOperation::kView, 2,
                        {1}, 1);
  asset.asset_id = 2;
  asset.content = *index.Snapshot().entries[1].content;
  index.RecordCheckpoint(asset);
  auto latest = std::make_shared<PoseSnapshot>(*next);
  latest->version = 3;
  latest->base_version = 2;
  latest->optimized_frame_count = 2;
  latest->parent_frame = PoseFrame::kMap;
  for (auto& value : latest->poses) {
    value.map_from_body.rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    value.map_from_body.translation = {4, static_cast<double>(value.id) - 2, 1};
  }
  index.Observe({{}, {}, latest, {}});
  return index.Snapshot();
}

class GaussianMapWriterTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("gaussian-index-" + std::to_string(getpid()) + "-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    output_ = root_ / "output";
    fs::create_directories(output_ / "states/1");
    std::ofstream(output_ / "states/1/marker") << "immutable asset";
    fs::create_directories(root_ / "raw");
    std::ofstream(root_ / "raw/image.png") << "immutable source";
    std::ofstream(output_ / "notes.txt") << "unrelated file";
    output_path = output_.string();
    snapshot_ = Snapshot();
  }

  void TearDown() override {
    fault = Fault::kNone;
    output_path.clear();
    temporary_name = "map.tmp.json";
    fs::remove_all(root_);
  }

  fs::path root_;
  fs::path output_;
  GaussianMapSnapshot snapshot_;
};

TEST_F(GaussianMapWriterTest, FullMetadataBudgetFitsIndexFile) {
  auto map = snapshot_;
  const auto prototype = snapshot_.entries[0];
  map.entries.clear();
  map.pose_version = map.optimized_frame_count = 0;
  map.frame_count = 512;
  map.parent_frame = PoseFrame::kOdom;
  for (std::uint64_t i = 0; i < 32; ++i) {
    auto entry = prototype;
    auto& group = entry.group;
    group.primary = {i, i * 16, {}};
    for (std::uint64_t id = i * 16; id < (i + 1) * 16; ++id)
      group.primary.keyframe_ids.push_back(id);
    group.state =
        i == 31 ? OnlineSubmapState::kActive : OnlineSubmapState::kSealed;
    group.membership_revision = 16 + (i != 31);
    auto& target = entry.binding;
    target.group = group;
    target.generation = i + 1;
    target.pose_version = target.optimized_frame_count = 0;
    target.frame_count = 512;
    target.parent_frame = PoseFrame::kOdom;
    target.parent_from_submap = {};
    target.parent_from_submap.translation = {static_cast<double>(i), .123456789,
                                             .0123456789};
    target.submap_from_body.clear();
    for (int j = 0; j < 16; ++j) {
      RigidTransform pose;
      const double angle = j * .0123456789;
      pose.rotation =
          Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      pose.translation = {j * .123456789, j * .0123456789, j * .00123456789};
      target.submap_from_body.push_back(pose);
    }
    entry.content_target = entry.asset_source = target;
    entry.content = GaussianContentId{map.session_id, i, i + 1, i + 1};
    entry.binding_job_id = i + 1;
    entry.asset->asset_id = i + 1;
    entry.asset->content = *entry.content;
    entry.frame_ids =
        entry.asset_frame_ids = {group.primary.keyframe_ids.back()};
    entry.gaussian_count = entry.asset_gaussian_count = 1;
    entry.resident = i == 31;
    entry.reason = "saved \"quoted\" paths \\ copies";
    map.entries.push_back(std::move(entry));
  }
  try {
    WriteGaussianMapIndex(output_, map);
  } catch (const std::exception& error) {
    FAIL() << error.what();
  }
  RecordProperty("full_metadata_bytes",
                 std::to_string(fs::file_size(output_ / "map.json")));
  EXPECT_LE(fs::file_size(output_ / "map.json"), 512u * 1024);
  cv::FileStorage json(Bytes(output_ / "map.json"),
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  EXPECT_EQ(static_cast<std::string>(json["entries"][0]["reason"]),
            map.entries[0].reason);
}

TEST_F(GaussianMapWriterTest, FirstIndexCanReferenceCheckpointWithoutPruning) {
  WriteGaussianMapIndex(output_, snapshot_);
  cv::FileStorage json(Bytes(output_ / "map.json"),
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  ASSERT_TRUE(json.isOpened());
  EXPECT_EQ(static_cast<std::string>(json["kind"]), "livo_atlas.gaussian_map");
  EXPECT_EQ(static_cast<int>(json["schema"]), 1);
  EXPECT_EQ(json["entries"].size(), 2u);
  EXPECT_EQ(static_cast<std::string>(json["entries"][0]["asset"]["asset_id"]),
            "1");
  EXPECT_TRUE(fs::is_directory(output_ / "states/1"));
  EXPECT_FALSE(fs::exists(output_ / "map.tmp.json"));
}

TEST_F(GaussianMapWriterTest, WritesExactSourceAndBindingCoverage) {
  auto& entry = snapshot_.entries[0];
  entry.content->source_generation = 5;
  entry.content->content_job_id = 9;
  entry.content_target = entry.binding;
  entry.content_target.generation = 5;
  entry.binding.generation = 5;
  entry.binding_job_id = 9;
  entry.dirty = true;
  WriteGaussianMapIndex(output_, snapshot_);
  cv::FileStorage json(Bytes(output_ / "map.json"),
                       cv::FileStorage::READ | cv::FileStorage::MEMORY);
  const auto first = json["entries"][0];
  EXPECT_EQ(static_cast<std::string>(first["binding"]["pose_version"]), "3");
  EXPECT_EQ(static_cast<std::string>(first["content"]["content_job_id"]), "9");
  EXPECT_EQ(static_cast<std::string>(first["asset"]["content_job_id"]), "1");
  EXPECT_EQ(static_cast<std::string>(first["asset_source"]["frame_count"]),
            "1");
  EXPECT_EQ(static_cast<std::string>(first["asset_source"]["pose_version"]),
            "0");
  EXPECT_EQ(static_cast<int>(first["volatile"]), 1);
  EXPECT_EQ(first["trained_frame_ids"].size(), 1u);
}

TEST_F(GaussianMapWriterTest,
       RejectsOversizedOrMalformedIndexBeforePublishing) {
  WriteGaussianMapIndex(output_, snapshot_);
  const auto original = Bytes(output_ / "map.json");
  for (int kind = 0; kind < 5; ++kind) {
    auto invalid = snapshot_;
    if (kind == 0) invalid.frame_count = 513;
    if (kind == 1) invalid.entries[0].asset->manifest_sha256 = "bad";
    if (kind == 2) invalid.entries[0].reason.assign(600000, 'x');
    if (kind == 3) invalid.entries[0].asset_current = false;
    if (kind == 4) invalid.entries[1].asset->asset_id = 1;
    EXPECT_THROW(WriteGaussianMapIndex(output_, invalid),
                 std::invalid_argument);
    EXPECT_EQ(Bytes(output_ / "map.json"), original);
    EXPECT_FALSE(fs::exists(output_ / "map.tmp.json"));
  }
}

TEST_F(GaussianMapWriterTest, FailureBeforeRenameKeepsPreviousIndex) {
  WriteGaussianMapIndex(output_, snapshot_);
  const auto original = Bytes(output_ / "map.json");
  ++snapshot_.map_sequence;
  for (const auto selected :
       {Fault::kFileSync, Fault::kFileClose, Fault::kRename}) {
    fault = selected;
    EXPECT_THROW(WriteGaussianMapIndex(output_, snapshot_), std::system_error);
    fault = Fault::kNone;
    EXPECT_EQ(Bytes(output_ / "map.json"), original);
    EXPECT_FALSE(fs::exists(output_ / "map.tmp.json"));
  }
}

TEST_F(GaussianMapWriterTest,
       DirectorySyncFailureReportsVisibleUncertainCommit) {
  for (const auto selected : {Fault::kDirectorySync, Fault::kParentSync}) {
    ++snapshot_.map_sequence;
    fault = selected;
    EXPECT_THROW(WriteGaussianMapIndex(output_, snapshot_),
                 GaussianMapCommitUncertainError);
    fault = Fault::kNone;
    cv::FileStorage json(Bytes(output_ / "map.json"),
                         cv::FileStorage::READ | cv::FileStorage::MEMORY);
    ASSERT_TRUE(json.isOpened());
    EXPECT_EQ(static_cast<std::string>(json["map_sequence"]),
              std::to_string(snapshot_.map_sequence));
    EXPECT_EQ(Bytes(output_ / "states/1/marker"), "immutable asset");
  }
}

TEST_F(GaussianMapWriterTest, SourceFilesAndUnrelatedPathsRemainUntouched) {
  WriteGaussianMapIndex(output_, snapshot_);
  EXPECT_EQ(Bytes(root_ / "raw/image.png"), "immutable source");
  EXPECT_EQ(Bytes(output_ / "notes.txt"), "unrelated file");
  EXPECT_EQ(Bytes(output_ / "states/1/marker"), "immutable asset");
}

TEST_F(GaussianMapWriterTest, OmittedFinalGroupCannotBePublished) {
  snapshot_.entries.pop_back();
  EXPECT_THROW(WriteGaussianMapIndex(output_, snapshot_),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_ / "map.json"));
}

TEST_F(GaussianMapWriterTest, TruncatedFinalGroupCannotBePublished) {
  snapshot_.frame_count = 3;
  for (auto& entry : snapshot_.entries) entry.binding.frame_count = 3;
  EXPECT_THROW(WriteGaussianMapIndex(output_, snapshot_),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_ / "map.json"));
}

TEST_F(GaussianMapWriterTest, ExistingTemporaryFifoIsNotOpenedOrRemoved) {
  const auto pending = output_ / "map.tmp.json";
  ASSERT_EQ(mkfifo(pending.c_str(), 0600), 0);
  EXPECT_THROW(WriteGaussianMapIndex(output_, snapshot_), std::system_error);
  EXPECT_TRUE(fs::is_fifo(pending));
  EXPECT_FALSE(fs::exists(output_ / "map.json"));
}

GaussianFinalizationResult Finalization() {
  GaussianFinalizationResult result;
  result.status = GaussianFinalizationStatus::kSucceeded;
  result.target_valid = result.eligible_set_frozen = true;
  result.session_id = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
  result.frame_count = 2;
  result.eligible_submap_ids = {0, 1};
  result.current = 2;
  result.durable = result.worker_stopped = true;
  result.map_sequence = 42;
  result.asset_ids = {1, 2};
  result.before = {2, 2, 0, 0};
  result.after = {4, 4, 2, 2};
  result.repair_attempts = 2;
  result.repair_seconds = .1;
  result.shutdown_seconds = .2;
  result.elapsed_seconds = .3;
  result.total_seconds = .4;
  return result;
}

TEST_F(GaussianMapWriterTest, FinalizationReportBindsOutcomeWithoutSelfProof) {
  auto result = Finalization();
  result.report_persisted = true;
  WriteGaussianFinalizationResult(output_, result);
  cv::FileStorage report((output_ / "finalization.json").string(),
                         cv::FileStorage::READ);
  ASSERT_TRUE(report.isOpened());
  EXPECT_EQ(static_cast<int>(report["schema"]), 1);
  EXPECT_EQ(static_cast<std::string>(report["kind"]),
            "livo_atlas.gaussian_finalization");
  EXPECT_EQ(static_cast<std::string>(report["status"]), "succeeded");
  EXPECT_EQ(static_cast<std::string>(report["map_sequence"]), "42");
  EXPECT_EQ(static_cast<std::string>(report["session_id"]), result.session_id);
  EXPECT_EQ(report["eligible_submap_ids"].size(), 2u);
  EXPECT_EQ(static_cast<std::string>(report["eligible_submap_ids"][0]), "0");
  EXPECT_EQ(static_cast<int>(report["current"]), 2);
  EXPECT_EQ(static_cast<std::string>(report["after"]["rebuilds"]), "2");
  EXPECT_TRUE(report["report_persisted"].empty());
  EXPECT_TRUE(report["report_error"].empty());
  EXPECT_TRUE(report["total_seconds"].empty());
  EXPECT_LE(fs::file_size(output_ / "finalization.json"), 16u * 1024);
  EXPECT_EQ(Bytes(output_ / "states/1/marker"), "immutable asset");
}

TEST_F(GaussianMapWriterTest, FinalizationFitsMaximumContentBudget) {
  auto result = Finalization();
  result.frame_count = 512;
  result.current = 32;
  result.eligible_submap_ids.clear();
  result.asset_ids.clear();
  for (std::uint64_t i = 0; i < 32; ++i) {
    result.eligible_submap_ids.push_back(i);
    result.asset_ids.push_back(i + 1);
  }
  result.after.accepted = result.after.effective_training = 32;
  result.reason.assign(1024, '\"');
  WriteGaussianFinalizationResult(output_, result);
  EXPECT_LE(fs::file_size(output_ / "finalization.json"), 16u * 1024);
}

TEST_F(GaussianMapWriterTest,
       FinalizationRejectsInvalidMetadataBeforePublication) {
  const auto original = Finalization();
  for (int variant = 0; variant < 7; ++variant) {
    auto value = original;
    if (variant == 0) value.eligible_submap_ids.assign(33, 0);
    if (variant == 1) value.asset_ids = {1, 1};
    if (variant == 2) value.asset_ids = {0, 2};
    if (variant == 3) value.elapsed_seconds = -1;
    if (variant == 4)
      value.status = static_cast<GaussianFinalizationStatus>(99);
    if (variant == 5) value.eligible_set_frozen = false;
    if (variant == 6) value.reason.assign(1025, 'x');
    EXPECT_THROW(WriteGaussianFinalizationResult(output_, value),
                 std::invalid_argument)
        << variant;
    EXPECT_FALSE(fs::exists(output_ / "finalization.json"));
    EXPECT_FALSE(fs::exists(output_ / "finalization.tmp.json"));
  }
}

TEST_F(GaussianMapWriterTest, FinalizationKeepsTimeoutAndDurabilitySeparate) {
  auto value = Finalization();
  value.status = GaussianFinalizationStatus::kTimedOut;
  value.reason = "deadline";
  WriteGaussianFinalizationResult(output_, value);
  cv::FileStorage report((output_ / "finalization.json").string(),
                         cv::FileStorage::READ);
  EXPECT_EQ(static_cast<std::string>(report["status"]), "timed_out");
  EXPECT_EQ(static_cast<int>(report["durable"]), 1);
}

TEST_F(GaussianMapWriterTest, FinalizationPreservesOldReportBeforeRename) {
  temporary_name = "finalization.tmp.json";
  auto value = Finalization();
  WriteGaussianFinalizationResult(output_, value);
  const auto before = Bytes(output_ / "finalization.json");
  ++value.map_sequence;
  for (const auto failure :
       {Fault::kFileSync, Fault::kFileClose, Fault::kRename}) {
    fault = failure;
    EXPECT_THROW(WriteGaussianFinalizationResult(output_, value),
                 std::system_error);
    fault = Fault::kNone;
    EXPECT_EQ(Bytes(output_ / "finalization.json"), before);
    EXPECT_FALSE(fs::exists(output_ / temporary_name));
  }
}

TEST_F(GaussianMapWriterTest, FinalizationPreservesVisibleReportAfterRename) {
  temporary_name = "finalization.tmp.json";
  auto value = Finalization();
  for (const auto failure : {Fault::kDirectorySync, Fault::kParentSync}) {
    fault = failure;
    EXPECT_THROW(WriteGaussianFinalizationResult(output_, value),
                 GaussianMapCommitUncertainError);
    fault = Fault::kNone;
    EXPECT_TRUE(fs::is_regular_file(output_ / "finalization.json"));
    EXPECT_EQ(Bytes(output_ / "states/1/marker"), "immutable asset");
  }
}

TEST_F(GaussianMapWriterTest, FinalizationDoesNotTouchExistingTemporaryLinks) {
  const auto temporary = output_ / "finalization.tmp.json";
  ASSERT_EQ(mkfifo(temporary.c_str(), 0600), 0);
  EXPECT_THROW(WriteGaussianFinalizationResult(output_, Finalization()),
               std::system_error);
  EXPECT_TRUE(fs::is_fifo(temporary));
  fs::remove(temporary);
  fs::create_symlink(output_ / "notes.txt", temporary);
  EXPECT_THROW(WriteGaussianFinalizationResult(output_, Finalization()),
               std::system_error);
  EXPECT_EQ(Bytes(output_ / "notes.txt"), "unrelated file");
}

}  // namespace
}  // namespace livo_atlas
