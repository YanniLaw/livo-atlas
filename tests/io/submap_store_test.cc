#include "livo_atlas/io/submap_store.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
std::string ReadBytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(stream), {}};
}
class SubmapStoreTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("submap-store-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {0, 0, 0, 0}};
    keyframes_ =
        std::make_unique<KeyframeStore>(root_ / "keyframes", calibration);
    directory_ = root_ / "keyframes/submaps";
  }
  void TearDown() override { fs::remove_all(root_); }
  Keyframe Frame(std::uint64_t id) {
    return CaptureKeyframe(id,
                           test_data::KeyframeResult(UpdateKind::kLidarOnly));
  }
  void AddFrames(std::size_t count) {
    while (keyframes_->ids().size() < count)
      keyframes_->Append(Frame(keyframes_->ids().size()));
  }
  void WriteGroup(const std::string& contents) {
    std::ofstream(directory_ / "0.yml") << "%YAML:1.0\n---\n" << contents;
  }
  fs::path root_, directory_;
  std::unique_ptr<KeyframeStore> keyframes_;
};

TEST_F(SubmapStoreTest, ReopensAnEmptyIndexAndCommittedFullAndPartialGroups) {
  AddFrames(5);
  SubmapStore writer(directory_, *keyframes_, 2);
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).size(), 0u);
  writer.Append({0, 0, {0, 1}});
  writer.Append({1, 2, {2, 3}});
  writer.Append({2, 4, {4}});
  SubmapStore reader(directory_, *keyframes_);
  EXPECT_EQ(reader.size(), 3u);
  EXPECT_EQ(reader.Read(2).keyframe_ids, (std::vector<std::uint64_t>{4}));
  EXPECT_EQ(reader.Read(1).anchor_keyframe_id, 2u);
  EXPECT_THROW(reader.Append({3, 5, {5}}), std::logic_error);
  EXPECT_THROW(reader.Read(3), std::out_of_range);
}

TEST_F(SubmapStoreTest, ProtectsExistingDirectoriesAndValidatesLimitsBeforeIo) {
  fs::create_directory(directory_);
  std::ofstream(directory_ / "keep") << "original";
  EXPECT_THROW(SubmapStore(directory_, *keyframes_, 2), std::exception);
  EXPECT_EQ(ReadBytes(directory_ / "keep"), "original");
  const auto fresh = root_ / "invalid";
  EXPECT_THROW(SubmapStore(fresh, *keyframes_, 0), std::invalid_argument);
  EXPECT_FALSE(fs::exists(fresh));
  EXPECT_EQ(KeyframeStore(root_ / "keyframes").ids().size(), 0u);
}

TEST_F(SubmapStoreTest, InvalidMembershipDoesNotWriteOrAdvanceTheIndex) {
  AddFrames(5);
  SubmapStore writer(directory_, *keyframes_, 2);
  for (const auto& group : std::vector<Submap>{{1, 0, {0, 1}},
                                               {0, 1, {0, 1}},
                                               {0, 0, {}},
                                               {0, 0, {0, 0}},
                                               {0, 0, {0, 2}},
                                               {0, 0, {0, 1, 2}}})
    EXPECT_THROW(writer.Append(group), std::invalid_argument);
  EXPECT_EQ(writer.size(), 0u);
  EXPECT_FALSE(fs::exists(directory_ / "0.yml"));
  EXPECT_FALSE(fs::exists(directory_ / ".pending-0.yml"));
  writer.Append({0, 0, {0, 1}});
  EXPECT_THROW(writer.Append({0, 0, {0, 1}}), std::invalid_argument);
  EXPECT_THROW(writer.Append({1, 3, {3, 4}}), std::invalid_argument);
  EXPECT_THROW(writer.Append({1, 2, {2, 3, 4}}), std::invalid_argument);
  EXPECT_THROW(writer.Append({1, 4294967296ULL, {4294967296ULL}}),
               std::invalid_argument);
  writer.Append({1, 2, {2, 3}});
  EXPECT_EQ(writer.size(), 2u);
}

TEST_F(SubmapStoreTest, APartialGroupSealsTheCommittedMembershipPrefix) {
  AddFrames(3);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0}});
  EXPECT_THROW(writer.Append({1, 1, {1, 2}}), std::invalid_argument);
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).size(), 1u);
}

TEST_F(SubmapStoreTest, RejectsMissingOrModifiedCommittedGroupMetadata) {
  AddFrames(4);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  writer.Append({1, 2, {2, 3}});
  const auto original = ReadBytes(directory_ / "0.yml");
  fs::rename(directory_ / "0.yml", root_ / "saved.yml");
  EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  std::ofstream(directory_ / "0.yml") << original;
  WriteGroup("id: \"0\"\nanchor_keyframe_id: \"0\"\nkeyframe_ids: [\"0\"]\n");
  EXPECT_THROW(writer.Read(0), std::runtime_error);
  EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
}

TEST_F(SubmapStoreTest, RejectsBadSchemaAnchorsAndNoncanonicalOrHugeIds) {
  AddFrames(2);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  const auto metadata = ReadBytes(directory_ / "dataset.yml");
  std::ofstream(directory_ / "dataset.yml")
      << "%YAML:1.0\n---\nschema_version: 999\nmax_keyframes: 2\n";
  EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
  std::ofstream(directory_ / "dataset.yml") << metadata;
  for (const auto& contents :
       {"id: \"4294967296\"\nanchor_keyframe_id: \"0\"\nkeyframe_ids: "
        "[\"0\",\"1\"]\n",
        "id: \"00\"\nanchor_keyframe_id: \"0\"\nkeyframe_ids: [\"0\",\"1\"]\n",
        "id: \"0\"\nanchor_keyframe_id: \"1\"\nkeyframe_ids: [\"0\",\"1\"]\n",
        "id: \"0\"\nanchor_keyframe_id: \"0\"\nkeyframe_ids: [\"0\",\"0\"]\n",
        "id: \"0\"\nanchor_keyframe_id: "
        "\"18446744073709551615\"\nkeyframe_ids: "
        "[\"18446744073709551615\"]\n"}) {
    WriteGroup(contents);
    EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
  }
}

TEST_F(SubmapStoreTest, IgnoresPendingFilesAndRefusesToOverwriteThem) {
  AddFrames(2);
  SubmapStore writer(directory_, *keyframes_, 2);
  const auto pending = directory_ / ".pending-0.yml";
  std::ofstream(pending) << "interrupted";
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).size(), 0u);
  EXPECT_THROW(writer.Append({0, 0, {0, 1}}), std::runtime_error);
  EXPECT_EQ(ReadBytes(pending), "interrupted");
  fs::remove(pending);
  EXPECT_THROW(writer.Append({0, 0, {0, 1}}), std::logic_error);
}

TEST_F(SubmapStoreTest, RejectsOverflowAliasesOfSchemaAndGroupLimits) {
  AddFrames(2);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  const auto path = directory_ / "dataset.yml";
  const auto original = ReadBytes(path);
  for (const auto* version : {"4294967297", "-4294967295"}) {
    std::ofstream(path) << "%YAML:1.0\n---\nschema_version: " << version
                        << "\nmax_keyframes: 2\n";
    EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
  }
  for (const auto* limit : {"4294967298", "-4294967294"}) {
    std::ofstream(path) << "%YAML:1.0\n---\nschema_version: 1\nmax_keyframes: "
                        << limit << "\n";
    EXPECT_THROW(SubmapStore(directory_, *keyframes_), std::runtime_error);
  }
  std::ofstream(path)
      << "%YAML:1.0\n---\nschema_version: 01\n"
         "max_keyframes: 0x2 # 4294967298 must stay a comment\n";
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).Read(0).keyframe_ids.size(),
            2u);
  std::ofstream(path) << original;
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).Read(0).keyframe_ids.size(),
            2u);
}

TEST_F(SubmapStoreTest, WriteFailureKeepsCommittedGroupsAndLatchesWriter) {
  AddFrames(4);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  const auto original = ReadBytes(directory_ / "0.yml");
  fs::rename(directory_, root_ / "saved-submaps");
  std::ofstream(directory_) << "blocking file";
  EXPECT_THROW(writer.Append({1, 2, {2, 3}}), std::runtime_error);
  fs::remove(directory_);
  fs::rename(root_ / "saved-submaps", directory_);
  EXPECT_THROW(writer.Append({1, 2, {2, 3}}), std::logic_error);
  EXPECT_EQ(ReadBytes(directory_ / "0.yml"), original);
  EXPECT_EQ(SubmapStore(directory_, *keyframes_).size(), 1u);
}

TEST_F(SubmapStoreTest, RebuildsGeometryFromUnorderedPerFramePoseVersions) {
  auto first = Frame(0), second = Frame(1);
  PointCloud::Ptr cloud(new PointCloud(*second.cloud));
  cloud->front().x = 2;
  second.cloud = cloud;
  second.odom_from_body.translation << 1, 4, 3;
  keyframes_->Append(first);
  keyframes_->Append(second);
  const auto original = ReadBytes(root_ / "keyframes/frames/1/cloud.pcd");
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  const auto raw = writer.ReadCloud(0);
  ASSERT_EQ(raw.cloud.size(), 2u);
  EXPECT_EQ(raw.pose_version, 0u);
  EXPECT_FLOAT_EQ(raw.cloud[0].x, 1);
  EXPECT_FLOAT_EQ(raw.cloud[1].x, 4);
  EXPECT_EQ(raw.parent_from_submap.translation, Eigen::Vector3d(1, 2, 3));
  RigidTransform anchor, moved;
  anchor.translation << 10, 0, 0;
  moved.rotation = first.odom_from_body.rotation;
  moved.translation << 10, 5, 0;
  keyframes_->WritePoses(1, {{1, moved}, {0, anchor}});
  const auto optimized = SubmapStore(directory_, *keyframes_).ReadCloud(0, 1);
  EXPECT_EQ(optimized.pose_version, 1u);
  EXPECT_EQ(optimized.parent_from_submap.translation,
            Eigen::Vector3d(10, 0, 0));
  EXPECT_EQ(optimized.parent_from_submap.rotation, Eigen::Matrix3d::Identity());
  ASSERT_EQ(optimized.cloud.size(), 2u);
  EXPECT_FLOAT_EQ(optimized.cloud[0].x, 1);
  EXPECT_FLOAT_EQ(optimized.cloud[1].x, 0);
  EXPECT_FLOAT_EQ(optimized.cloud[1].y, 7);
  EXPECT_FLOAT_EQ(optimized.cloud[1].z, 0);
  EXPECT_FLOAT_EQ(optimized.cloud[1].intensity, 42);
  EXPECT_EQ(ReadBytes(root_ / "keyframes/frames/1/cloud.pcd"), original);
  EXPECT_EQ(keyframes_->Read(1).odom_from_body.translation,
            Eigen::Vector3d(1, 4, 3));
}

TEST_F(SubmapStoreTest, OldPoseFrontierRejectsUncoveredGroupsBeforeAssetReads) {
  AddFrames(2);
  const RigidTransform pose;
  keyframes_->WritePoses(1, {{1, pose}, {0, pose}});
  AddFrames(4);
  SubmapStore writer(directory_, *keyframes_, 2);
  writer.Append({0, 0, {0, 1}});
  writer.Append({1, 2, {2, 3}});
  EXPECT_EQ(writer.ReadCloud(0, 1).cloud.size(), 2u);
  fs::remove(root_ / "keyframes/frames/2/cloud.pcd");
  try {
    writer.ReadCloud(1, 1);
    FAIL() << "Uncovered submap must fail";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("cover"), std::string::npos);
  }
  EXPECT_THROW(writer.ReadCloud(1), std::runtime_error);
  EXPECT_THROW(writer.ReadCloud(0, 99), std::runtime_error);
}
}  // namespace
}  // namespace livo_atlas
