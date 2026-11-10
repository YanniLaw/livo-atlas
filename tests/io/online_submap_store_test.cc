#include "livo_atlas/io/online_submap_store.h"

#include <chrono>
#include <fstream>
#include <iterator>
#include <memory>

#include <gtest/gtest.h>
#include <opencv2/core/persistence.hpp>

#include "mapping/keyframe_fixtures.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
std::string Bytes(const fs::path& p) {
  std::ifstream input(p);
  return {std::istreambuf_iterator<char>(input), {}};
}
class OnlineSubmapStoreTest : public testing::Test {
 protected:
  void SetUp() override {
    root = fs::temp_directory_path() /
           ("online-store-" +
            std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {0, 0, 0, 0}};
    frames = std::make_unique<KeyframeStore>(root / "keyframes", calibration);
    path = root / "keyframes/online_submaps";
    Add(3);
  }

  void TearDown() override { fs::remove_all(root); }

  void Add(int count) {
    while (frames->ids().size() < static_cast<std::size_t>(count)) {
      auto frame =
          CaptureKeyframe(frames->ids().size(),
                          test_data::KeyframeResult(UpdateKind::kLidarOnly));
      frame.odom_from_body = {};
      frame.odom_from_body.translation.x() = frame.id;
      frames->Append(frame);
    }
  }
  fs::path root, path;
  std::unique_ptr<KeyframeStore> frames;
};
TEST_F(OnlineSubmapStoreTest, RevisionsRoundTripAndRemainImmutable) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  for (int i = 0; i < 3; ++i)
    for (const auto& revision :
         builder.AddKeyframe(i, i, frames->ReadRawBodyPose(i)))
      writer.AppendRevision(revision);
  const auto first = Bytes(path / "records/0/1.yml");
  OnlineSubmapStore reader(path, *frames);
  EXPECT_EQ(reader.size(), 2u);
  EXPECT_EQ(reader.indexed_frame_count(), 3u);
  EXPECT_EQ(reader.Read(0).state, OnlineSubmapState::kSealed);
  EXPECT_EQ(reader.Read(1).state, OnlineSubmapState::kActive);
  EXPECT_EQ(reader.Read(1).support_keyframe_ids,
            (std::vector<std::uint64_t>{0, 1}));
  EXPECT_EQ(Bytes(path / "records/0/1.yml"), first);
  EXPECT_THROW(reader.AppendRevision(reader.Read(1)), std::logic_error);
}

TEST_F(OnlineSubmapStoreTest, RawPoseReadDoesNotLoadAssets) {
  auto frame = CaptureKeyframe(3, test_data::KeyframeResult());
  frames->Append(frame);
  const auto directory = root / "keyframes/frames/3";
  ASSERT_TRUE(fs::remove(directory / "image.png"));
  ASSERT_TRUE(fs::remove(directory / "cloud.pcd"));
  EXPECT_TRUE(frames->ReadRawBodyPose(3).translation.isApprox(
      frame.odom_from_body.translation));
  EXPECT_THROW(frames->Read(3), std::runtime_error);
  EXPECT_THROW(frames->ReadRawBodyPose(10), std::out_of_range);
  std::ofstream(directory / "frame.yml") << "%YAML:1.0\n---\nid: \"2\"\n";
  EXPECT_THROW(frames->ReadRawBodyPose(3), std::runtime_error);
}

TEST_F(OnlineSubmapStoreTest, PartialCommitRemainsInspectable) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  for (int i = 0; i < 2; ++i)
    for (const auto& revision : builder.AddKeyframe(i, i, {}))
      writer.AppendRevision(revision);
  EXPECT_EQ(OnlineSubmapStore(path, *frames).indexed_frame_count(), 2u);
  fs::rename(path / "records", path / "saved");
  std::ofstream(path / "records") << "blocked";
  auto next = builder.AddKeyframe(2, 2, {}).front();
  std::string failure;
  try {
    writer.AppendRevision(next);
    FAIL();
  } catch (const std::runtime_error& e) {
    failure = e.what();
  }
  fs::remove(path / "records");
  fs::rename(path / "saved", path / "records");
  try {
    writer.AppendRevision(next);
    FAIL();
  } catch (const std::runtime_error& e) {
    EXPECT_EQ(e.what(), failure);
  }
  EXPECT_EQ(OnlineSubmapStore(path, *frames).indexed_frame_count(), 2u);
}

TEST_F(OnlineSubmapStoreTest, FrozenPoseFileReproducesGrowingView) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  PoseUpdate update;
  update.optimized_frame_count = 2;
  for (int i = 0; i < 2; ++i) {
    RigidTransform pose;
    pose.translation.x() = 10 + i;
    update.poses.push_back({static_cast<std::uint64_t>(i), pose});
  }
  std::vector<RigidTransform> raw;
  for (int i = 0; i < 3; ++i) raw.push_back(frames->ReadRawBodyPose(i));
  const auto snapshot = MakePoseSnapshot(update, {}, raw);
  writer.WritePoseSnapshot(snapshot);
  const auto first = Bytes(path / "poses/1.yml");
  Add(4);
  OnlineSubmapStore reader(path, *frames);
  EXPECT_EQ(reader.ReadPoseSnapshot(1).frame_count, 3u);
  EXPECT_DOUBLE_EQ(reader.MaterializePoseSnapshot(1, 4)
                       .poses.back()
                       .map_from_body.translation.x(),
                   13);
  EXPECT_EQ(Bytes(path / "poses/1.yml"), first);
  EXPECT_THROW(reader.MaterializePoseSnapshot(1, 5), std::invalid_argument);
  EXPECT_THROW(writer.WritePoseSnapshot(snapshot), std::invalid_argument);
  EXPECT_THROW(frames->ReadPoses(1), std::runtime_error);
}

TEST_F(OnlineSubmapStoreTest, RejectsCorruptHistoryAndIgnoresPending) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  for (int i = 0; i < 2; ++i)
    for (const auto& revision : builder.AddKeyframe(i, i, {}))
      writer.AppendRevision(revision);
  std::ofstream(path / "records/0/.pending-4.yml") << "partial";
  EXPECT_EQ(OnlineSubmapStore(path, *frames).size(), 1u);
  fs::remove(path / "records/0/2.yml");
  EXPECT_THROW((OnlineSubmapStore{path, *frames}), std::runtime_error);
}

TEST_F(OnlineSubmapStoreTest, InvalidInputDoesNotPoisonWriter) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  auto first = builder.AddKeyframe(0, 0, {}).front();
  auto bad = first;
  bad.membership_revision = 2;
  EXPECT_THROW(writer.AppendRevision(bad), std::invalid_argument);
  writer.AppendRevision(first);
  EXPECT_THROW(writer.AppendRevision(first), std::invalid_argument);
  auto second = builder.AddKeyframe(1, 1, {});
  for (const auto& revision : second) writer.AppendRevision(revision);
  EXPECT_EQ(writer.Read(0).membership_revision, 3u);
}

TEST_F(OnlineSubmapStoreTest, CorruptRecordsAreRejectedWithoutRepair) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  writer.AppendRevision(builder.AddKeyframe(0, 0, {}).front());
  const auto record = path / "records/0/1.yml";
  const auto original = Bytes(record);
  for (int fault = 0; fault < 9; ++fault) {
    std::string text = "%YAML:1.0\n---\n";
    text += "schema_version: " + std::to_string(fault == 0 ? 99 : 1) + "\n";
    text += "id: \"" + std::string(fault == 1 ? "1099511627776" : "0") + "\"\n";
    text += "membership_revision: \"" + std::string(fault == 2 ? "2" : "1") +
            "\"\n";
    text +=
        "anchor_keyframe_id: \"" + std::string(fault == 3 ? "1" : "0") + "\"\n";
    text += "state: " + std::string(fault == 4 ? "unloaded" : "active") + "\n";
    text += fault == 5   ? "keyframe_ids: [\"0\", \"1\"]\n"
            : fault == 6 ? "keyframe_ids: [\"00\"]\n"
            : fault == 7 ? "keyframe_ids: [\"18446744073709551615\"]\n"
                         : "keyframe_ids: [\"0\"]\n";
    text += fault == 8 ? "support_keyframe_ids: [\"0\"]\n"
                       : "support_keyframe_ids: []\n";
    std::ofstream(record) << text;
    EXPECT_THROW((OnlineSubmapStore{path, *frames}), std::runtime_error);
    EXPECT_EQ(Bytes(record), text);
  }
  std::ofstream(record) << original;
  EXPECT_EQ(OnlineSubmapStore(path, *frames).size(), 1u);
}

TEST_F(OnlineSubmapStoreTest, FailedSealLeavesActiveRevision) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  OnlineSubmapBuilder builder(2, {});
  writer.AppendRevision(builder.AddKeyframe(0, 0, {}).front());
  const auto revisions = builder.AddKeyframe(1, 1, {});
  writer.AppendRevision(revisions.front());
  fs::create_directory(path / "records/0/.pending-3.yml");
  EXPECT_THROW(writer.AppendRevision(revisions.back()), std::runtime_error);
  const OnlineSubmapStore reader(path, *frames);
  EXPECT_EQ(reader.Read(0).membership_revision, 2u);
  EXPECT_EQ(reader.Read(0).state, OnlineSubmapState::kActive);
  EXPECT_EQ(reader.indexed_frame_count(), 2u);
}

TEST_F(OnlineSubmapStoreTest, FailedPoseCommitPreservesOldVersion) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  std::vector<RigidTransform> raw;
  PoseUpdate update;
  update.optimized_frame_count = 3;
  for (int i = 0; i < 3; ++i) {
    raw.push_back(frames->ReadRawBodyPose(i));
    update.poses.push_back({static_cast<std::uint64_t>(i), raw.back()});
  }
  const auto first = MakePoseSnapshot(update, {}, raw);
  writer.WritePoseSnapshot(first);
  const auto bytes = Bytes(path / "poses/1.yml");
  update.base_version = 1;
  const auto next = MakePoseSnapshot(update, first, raw);
  fs::create_directory(path / "poses/.pending-2.yml");
  EXPECT_THROW(writer.WritePoseSnapshot(next), std::runtime_error);
  EXPECT_THROW(writer.AppendRevision(
                   OnlineSubmapBuilder(2, {}).AddKeyframe(0, 0, {}).front()),
               std::runtime_error);
  const OnlineSubmapStore reader(path, *frames);
  EXPECT_EQ(reader.ReadPoseSnapshot(1).version, 1u);
  EXPECT_THROW(reader.ReadPoseSnapshot(2), std::out_of_range);
  EXPECT_EQ(Bytes(path / "poses/1.yml"), bytes);
  auto corrupt = bytes;
  const auto position = corrupt.find("last_optimized_anchor");
  ASSERT_NE(position, std::string::npos);
  corrupt.replace(position, std::string("last_optimized_anchor").size(),
                  "unknown_policy");
  std::ofstream(path / "poses/1.yml") << corrupt;
  EXPECT_THROW((OnlineSubmapStore{path, *frames}), std::runtime_error);
}

TEST_F(OnlineSubmapStoreTest, EmptyIndexAndExistingDirectoryAreProtected) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  EXPECT_EQ(OnlineSubmapStore(path, *frames).size(), 0u);
  const auto bytes = Bytes(path / "dataset.yml");
  EXPECT_THROW((OnlineSubmapStore{path, *frames, 2, {}}), std::runtime_error);
  EXPECT_EQ(Bytes(path / "dataset.yml"), bytes);
  EXPECT_THROW((OnlineSubmapStore{root / "invalid", *frames, 0, {}}),
               std::invalid_argument);
  EXPECT_FALSE(fs::exists(root / "invalid"));
}

TEST_F(OnlineSubmapStoreTest, ReadsAndExtendsProjectedTailFile) {
  { OnlineSubmapStore writer(path, *frames, 2, {}); }
  const auto pose_file = path / "poses/1.yml";
  std::ofstream(pose_file) << R"yaml(%YAML:1.0
---
schema_version: 2
version: "1"
base_version: "0"
parent_frame: map
frame_count: "3"
optimized_frame_count: "1"
tail_policy: last_optimized_anchor_projected
tail_anchor_keyframe_id: "0"
poses:
  - id: "0"
    map_from_body:
      rotation: ["1", "0", "0", "0", "1", "0", "0", "0", "1"]
      translation: ["10", "0", "0"]
  - id: "1"
    map_from_body:
      rotation: ["1", "0", "0", "0", "1", "0", "0", "0", "1"]
      translation: ["11", "0", "0"]
  - id: "2"
    map_from_body:
      rotation: ["1", "0", "0", "0", "1", "0", "0", "0", "1"]
      translation: ["12", "0", "0"]
)yaml";
  const auto bytes = Bytes(pose_file);
  OnlineSubmapStore reader(path, *frames);
  const auto snapshot = reader.ReadPoseSnapshot(1);
  EXPECT_EQ(snapshot.version, 1u);
  EXPECT_EQ(snapshot.frame_count, 3u);
  EXPECT_EQ(snapshot.tail_policy,
            PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_DOUBLE_EQ(snapshot.poses[2].map_from_body.translation.x(), 12);
  Add(4);
  const auto grown = reader.MaterializePoseSnapshot(1, 4);
  EXPECT_DOUBLE_EQ(grown.poses[3].map_from_body.translation.x(), 13);
  EXPECT_EQ(Bytes(pose_file), bytes);
}

TEST_F(OnlineSubmapStoreTest, WritesProjectedTailAndPreservesLegacyVersion) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  std::vector<RigidTransform> raw;
  for (int i = 0; i < 3; ++i) raw.push_back(frames->ReadRawBodyPose(i));
  PoseUpdate first;
  first.optimized_frame_count = 1;
  first.poses.push_back({0, {}});
  const auto legacy = MakePoseSnapshot(first, {}, raw);
  writer.WritePoseSnapshot(legacy);
  const auto legacy_bytes = Bytes(path / "poses/1.yml");

  PoseUpdate second;
  second.base_version = 1;
  second.optimized_frame_count = 2;
  second.tail_policy = PoseTailPolicy::kProjectedLastOptimizedAnchor;
  second.poses.push_back({0, {}});
  second.poses.push_back({1, raw[1]});
  const auto projected = MakePoseSnapshot(second, legacy, raw);
  writer.WritePoseSnapshot(projected);

  const OnlineSubmapStore reader(path, *frames);
  EXPECT_EQ(reader.ReadPoseSnapshot(1).tail_policy,
            PoseTailPolicy::kLegacyLastOptimizedAnchor);
  EXPECT_EQ(reader.ReadPoseSnapshot(2).tail_policy,
            PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_EQ(Bytes(path / "poses/1.yml"), legacy_bytes);
  const cv::FileStorage file((path / "poses/2.yml").string(),
                             cv::FileStorage::READ);
  ASSERT_TRUE(file.isOpened());
  EXPECT_EQ(static_cast<int>(file["schema_version"]), 2);
  EXPECT_EQ(file["tail_policy"].string(), "last_optimized_anchor_projected");
}

TEST_F(OnlineSubmapStoreTest, RejectsMismatchedPoseSchemaAndPolicy) {
  OnlineSubmapStore writer(path, *frames, 2, {});
  std::vector<RigidTransform> raw;
  for (int i = 0; i < 3; ++i) raw.push_back(frames->ReadRawBodyPose(i));
  PoseUpdate update;
  update.optimized_frame_count = 1;
  update.poses.push_back({0, {}});
  writer.WritePoseSnapshot(MakePoseSnapshot(update, {}, raw));
  const auto pose_file = path / "poses/1.yml";
  const auto legacy = Bytes(pose_file);
  const auto schema_pos = legacy.find("schema_version: 1");
  ASSERT_NE(schema_pos, std::string::npos);
  auto mismatch = legacy;
  mismatch.replace(schema_pos, std::string("schema_version: 1").size(),
                   "schema_version: 2");
  std::ofstream(pose_file) << mismatch;
  EXPECT_THROW((OnlineSubmapStore{path, *frames}), std::runtime_error);

  mismatch = legacy;
  const auto policy_pos = mismatch.find("last_optimized_anchor");
  ASSERT_NE(policy_pos, std::string::npos);
  mismatch.replace(policy_pos, std::string("last_optimized_anchor").size(),
                   "last_optimized_anchor_projected");
  std::ofstream(pose_file) << mismatch;
  EXPECT_THROW((OnlineSubmapStore{path, *frames}), std::runtime_error);
}

}  // namespace
}  // namespace livo_atlas
