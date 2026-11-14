#include "livo_atlas/io/loop_closure_store.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

#include "backend/loop_dataset_fixtures.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;

std::string Bytes(const fs::path& path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}

void Replace(const fs::path& path, const std::string& from,
             const std::string& to) {
  auto text = Bytes(path);
  const auto position = text.find(from);
  ASSERT_NE(position, std::string::npos) << from << '\n' << text;
  text.replace(position, from.size(), to);
  std::ofstream(path) << text;
}

// Replace one emitted YAML field, including its indented children.
void ReplaceBlock(const fs::path& path, const std::string& key,
                  const std::string& replacement) {
  auto text = Bytes(path);
  const auto found = text.find('\n' + key + ':');
  ASSERT_NE(found, std::string::npos) << key;
  const auto begin = found + 1;
  auto end = text.find('\n', begin) + 1;
  const auto indent = key.find_first_not_of(' ');
  while (end < text.size()) {
    const auto content = text.find_first_not_of(' ', end);
    if (content == std::string::npos || content - end <= indent) break;
    const auto newline = text.find('\n', content);
    end = newline == std::string::npos ? text.size() : newline + 1;
  }
  text.replace(begin, end - begin, replacement);
  std::ofstream(path) << text;
}

class LoopClosureStoreTest : public testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("c2b-journal-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    Calibration calibration;
    calibration.camera = {"Pinhole", 12, 8, .5, 4, 5, 6, 4, {}};
    frames_ = std::make_unique<KeyframeStore>(root_, calibration);
    OnlineSubmapOptions options;
    options.support_keyframes = 0;
    poses_ = std::make_unique<OnlineSubmapStore>(root_ / "online_submaps",
                                                 *frames_, 1, options);
    OnlineSubmapBuilder builder(1, options);
    const auto cloud = loop_test::DenseCloud();
    for (std::uint64_t id = 0; id < 4; ++id) {
      auto frame = loop_test::DenseFrame(id, cloud);
      frames_->Append(frame);
      for (const auto& group :
           builder.AddKeyframe(id, frame.timestamp, frame.odom_from_body))
        poses_->AppendRevision(group);
    }
    for (const auto& group : builder.Finish()) poses_->AppendRevision(group);
    journal_ = std::make_unique<LoopClosureStore>(frames_->CaptureReadView(),
                                                  loop_test::NearbyOptions());
  }

  void TearDown() override { fs::remove_all(root_); }

  LoopTaskResult Result(
      std::uint64_t query_id = 2,
      const LoopAdmissionOptions& options = loop_test::NearbyOptions()) {
    LoopAdmissionInput input;
    input.query_id = query_id;
    input.poses = poses_->MaterializePoseSnapshot(0, 4);
    for (const auto& pose : input.poses.poses)
      input.raw_poses.push_back(pose.map_from_body);
    for (std::uint64_t id = 0; id < 4; ++id) {
      input.submaps.push_back({poses_->Read(id), 10.0 + id, 10.0 + id,
                               input.poses.poses[id].map_from_body, 0, 4, 0});
    }
    LoopTask task{
        1, std::move(input), {10, 11, 12, 13}, frames_->CaptureReadView()};
    auto result = RunLoopTask(task, options);
    return {std::move(task), std::move(result), {}};
  }

  PoseSnapshot Candidate(const LoopTaskResult& result) {
    return MakePoseSnapshot(result.admission->proposal->update,
                            result.task.input.poses,
                            result.task.input.raw_poses);
  }

  LoopSessionStats Stats() {
    LoopSessionStats stats;
    stats.frame_count = 4;
    stats.pose_version = stats.accepted = stats.submitted = stats.completed = 1;
    stats.peak_work_payloads = 1;
    return stats;
  }

  fs::path Prepared() { return root_ / "loop_closure/prepared/1.yml"; }

  fs::path root_;
  std::unique_ptr<KeyframeStore> frames_;
  std::unique_ptr<OnlineSubmapStore> poses_;
  std::unique_ptr<LoopClosureStore> journal_;
};

TEST_F(LoopClosureStoreTest, PreparedIsNotCommit) {
  const auto result = Result();
  ASSERT_TRUE(result.admission->proposal);
  journal_->WritePrepared(result, Candidate(result));
  const auto before = Bytes(Prepared());
  const auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
      << testing::PrintToString(report.issues);
  EXPECT_EQ(report.committed_version, 0u);
  EXPECT_EQ(report.uncommitted_prepared, (std::vector<std::uint64_t>{1}));
  EXPECT_EQ(Bytes(Prepared()), before);
}

TEST_F(LoopClosureStoreTest, PublishedPoseMatchesPrepared) {
  const auto result = Result();
  const auto candidate = Candidate(result);
  journal_->WritePrepared(result, candidate);
  poses_->WritePoseSnapshot(candidate);
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  journal_->Finish(Stats());
  auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
  EXPECT_EQ(report.committed_version, 1u);
  fs::rename(Prepared(), Prepared().parent_path() / ".pending-1.yml");
  report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, ContinuousVersionAndProvenance) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  const auto original = Bytes(Prepared());
  for (const auto& change : std::vector<std::pair<std::string, std::string>>{
           {"rotation_sigma_rad: \"0.05235987755982989",
            "rotation_sigma_rad: \"0.1"},
           {"optimized_frame_count: \"4\"", "optimized_frame_count: \"3\""},
           {"membership_revision: \"2\"", "membership_revision: \"3\""},
           {"task_id: \"1\"", "task_id: \"1\"\ntask_id: \"2\""}}) {
    std::ofstream(Prepared()) << original;
    Replace(Prepared(), change.first, change.second);
    EXPECT_EQ(InspectLoopDataset(root_).status,
              LoopRecoveryStatus::kInconsistent)
        << change.first;
  }
  std::ofstream(Prepared()) << original;
  const auto frame = root_ / "frames/0/frame.yml";
  const auto raw = Bytes(frame);
  Replace(frame, "timestamp: \"10\"", "timestamp: \"10.25\"");
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
  std::ofstream(frame) << raw;
  fs::copy_file(Prepared(), Prepared().parent_path() / "3.yml");
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, AtomicSummaryAndImmutableRecords) {
  const auto result = Result();
  const auto candidate = Candidate(result);
  journal_->WritePrepared(result, candidate);
  const auto prepared = Bytes(Prepared());
  EXPECT_THROW(journal_->WritePrepared(result, candidate), std::exception);
  EXPECT_EQ(Bytes(Prepared()), prepared);
  poses_->WritePoseSnapshot(candidate);
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  EXPECT_THROW(journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4),
               std::exception);
  const auto summary = root_ / "loop_closure/summary.yml";
  const auto incomplete = Bytes(summary);
  fs::create_directory(root_ / "loop_closure/.pending-summary.yml");
  EXPECT_THROW(journal_->Finish(Stats()), std::exception);
  EXPECT_EQ(Bytes(summary), incomplete);
  EXPECT_EQ(Bytes(Prepared()), prepared);
  fs::remove(root_ / "loop_closure/.pending-summary.yml");
  journal_->Finish(Stats());
  EXPECT_NE(Bytes(summary), incomplete);
  EXPECT_EQ(InspectLoopDataset(root_).status, LoopRecoveryStatus::kConsistent);
}

TEST_F(LoopClosureStoreTest, FalseCompleteAndMissingTailAreDistinguished) {
  auto stats = Stats();
  journal_->Finish(stats);
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, UnfinishedTailPreservesCommittedPrefix) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  frames_->Append(loop_test::DenseFrame(4, loop_test::DenseCloud()));
  auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
      << testing::PrintToString(report.issues);
  EXPECT_EQ(report.committed_version, 1u);
  OnlineSubmap tail;
  tail.primary = {4, 4, {4}};
  tail.membership_revision = 1;
  poses_->AppendRevision(tail);
  report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete);
  EXPECT_EQ(report.committed_version, 1u);
}

TEST_F(LoopClosureStoreTest, ChecksRawExpectedAndSessionSourcesWithoutClouds) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  const auto original = Bytes(Prepared());
  Replace(Prepared(), "timestamp: \"10\"", "timestamp: \"10.125\"");
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
  std::ofstream(Prepared()) << original;
  auto changed = original;
  const auto expected = changed.find("expected:");
  ASSERT_NE(expected, std::string::npos);
  const auto version = changed.find("version: \"1\"", expected);
  ASSERT_NE(version, std::string::npos);
  changed.replace(version, std::string("version: \"1\"").size(),
                  "version: \"2\"");
  std::ofstream(Prepared()) << changed;
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
  std::ofstream(Prepared()) << original;
  const auto metadata = Bytes(root_ / "dataset.yml");
  std::ofstream(root_ / "dataset.yml", std::ios::app) << "\n# external edit\n";
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
  std::ofstream(root_ / "dataset.yml") << metadata;
  for (auto id : frames_->ids())
    fs::remove(root_ / "frames" / std::to_string(id) / "cloud.pcd");
  const auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
      << testing::PrintToString(report.issues);
  EXPECT_EQ(report.committed_version, 1u);
}

TEST_F(LoopClosureStoreTest, ChecksEventsAndRunsReadonlyCliWithExactExitCodes) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  const auto parent = root_.parent_path();
  const auto output_prefix = root_.filename().string() + "-inspect-";
  const auto run = [&](int expected, const std::string& suffix) {
    const auto output = parent / (output_prefix + suffix);
    const std::string command = std::string("\"") + LIVO_ATLAS_CHECK_LOOPS +
                                "\" \"" + root_.string() + "\" \"" +
                                output.string() + "\"";
    EXPECT_EQ(std::system(command.c_str()), expected * 256);
    EXPECT_TRUE(fs::is_regular_file(output / "report.yml"));
    fs::remove_all(output);
  };
  run(3, "prepared");
  poses_->WritePoseSnapshot(Candidate(result));
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  journal_->WriteCoalesced(0, 1, 2);
  journal_->WriteResourceLimit(2, 4);
  auto stats = Stats();
  stats.skipped = 2;
  stats.resource_limited = 1;
  journal_->Finish(stats);
  run(0, "complete");
  const auto session = root_ / "loop_closure/session.yml";
  std::ofstream(session, std::ios::app) << "options: {}\n";
  run(1, "duplicate");
  EXPECT_EQ(
      std::system((std::string("\"") + LIVO_ATLAS_CHECK_LOOPS + "\"").c_str()),
      2 * 256);
}

TEST_F(LoopClosureStoreTest, ConstraintMustMatchItsVerification) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  auto text = Bytes(Prepared());
  const auto measurement = text.find("from_body_from_to_body:");
  ASSERT_NE(measurement, std::string::npos);
  const auto translation = text.find("translation:", measurement);
  const auto begin = text.find('"', translation);
  const auto end = text.find('"', begin + 1);
  ASSERT_NE(end, std::string::npos);
  text.replace(begin, end - begin + 1, "\"0.125\"");
  std::ofstream(Prepared()) << text;
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, PendingDirectoriesAreNotPublishedMetadata) {
  const auto pending = root_ / "loop_closure/.pending-orphan";
  fs::create_directory(pending);
  std::ofstream(pending / "invalid.yml") << "not committed YAML";
  const auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kIncomplete)
      << testing::PrintToString(report.issues);
}

TEST_F(LoopClosureStoreTest, MissingQueryDiagnosticsCannotClaimCompletion) {
  const auto result = Result(0);
  ASSERT_EQ(result.admission->status, LoopAdmissionStatus::kNoCandidate);
  journal_->WriteQuery(result, LoopDisposition::kRejected, 0, 4);
  auto stats = Stats();
  stats.pose_version = stats.accepted = 0;
  stats.rejected = 1;
  journal_->Finish(stats);
  ASSERT_EQ(InspectLoopDataset(root_).status, LoopRecoveryStatus::kConsistent);
  const auto query = root_ / "loop_closure/queries/1.yml";
  auto text = Bytes(query);
  const auto diagnostics = text.find("diagnostics:");
  ASSERT_NE(diagnostics, std::string::npos);
  text.resize(diagnostics);
  std::ofstream(query) << text;
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, AcceptedQueryMustMatchPreparedFrontiersAndSource) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  journal_->Finish(Stats());
  const auto query = root_ / "loop_closure/queries/1.yml";
  const auto original = Bytes(query);
  for (const auto& change : std::vector<std::pair<std::string, std::string>>{
           {"query_id: \"2\"", "query_id: \"3\""},
           {"dispatch_frame_count: \"4\"", "dispatch_frame_count: \"3\""}}) {
    std::ofstream(query) << original;
    Replace(query, change.first, change.second);
    EXPECT_EQ(InspectLoopDataset(root_).status,
              LoopRecoveryStatus::kInconsistent)
        << change.first;
  }
  auto coherent = original;
  const std::string dispatch = "dispatch_frame_count: \"4\"";
  coherent.replace(coherent.find(dispatch), dispatch.size(),
                   "dispatch_frame_count: \"3\"");
  const auto diagnostics = coherent.find("diagnostics:");
  const std::string count = "frame_count: \"4\"";
  coherent.replace(coherent.find(count, diagnostics), count.size(),
                   "frame_count: \"3\"");
  std::ofstream(query) << coherent;
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, RejectsTruncatedPreparedAdmissionDiagnostics) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  journal_->Finish(Stats());
  ASSERT_EQ(InspectLoopDataset(root_).status, LoopRecoveryStatus::kConsistent);
  ReplaceBlock(Prepared(), "diagnostics",
               "diagnostics:\n   status: proposed\n   base_version: \"0\"\n"
               "   frame_count: \"4\"\n   query_id: \"2\"\n");
  EXPECT_EQ(InspectLoopDataset(root_).status,
            LoopRecoveryStatus::kInconsistent);
}

TEST_F(LoopClosureStoreTest, RejectsMalformedNestedAdmissionDiagnostics) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  journal_->Finish(Stats());
  ASSERT_EQ(InspectLoopDataset(root_).status, LoopRecoveryStatus::kConsistent);
  for (const auto& path : {Prepared(), root_ / "loop_closure/queries/1.yml"}) {
    const auto original = Bytes(path);
    for (const auto& change : std::vector<std::pair<std::string, std::string>>{
             {"   candidates", "   candidates:\n      - invalid\n"},
             {"   geometry_checks", "   geometry_checks:\n      - invalid\n"},
             {"   trial", "   trial: invalid\n"},
             {"   trial", ""},
             {"            inliers", ""},
             {"            source_coverage",
              "            source_coverage: \"nan\"\n"},
             {"            representation_revision", ""},
             {"            submap_id", "            submap_id: \"3\"\n"},
             {"      odometry_residuals",
              "      odometry_residuals:\n         - invalid\n"}}) {
      std::ofstream(path) << original;
      ReplaceBlock(path, change.first, change.second);
      const auto report = InspectLoopDataset(root_);
      EXPECT_EQ(report.status, LoopRecoveryStatus::kInconsistent)
          << path << ' ' << change.first;
    }
    std::ofstream(path) << original;
  }
}

TEST_F(LoopClosureStoreTest, CommittedTaskCannotHaveNonacceptedDisposition) {
  const auto result = Result();
  journal_->WritePrepared(result, Candidate(result));
  poses_->WritePoseSnapshot(Candidate(result));
  // An absent post-publication query remains a valid interrupted session.
  auto report = InspectLoopDataset(root_);
  ASSERT_EQ(report.status, LoopRecoveryStatus::kIncomplete);
  ASSERT_EQ(report.committed_version, 1u);
  journal_->WriteQuery(result, LoopDisposition::kAccepted, 1, 4);
  const auto query = root_ / "loop_closure/queries/1.yml";
  const auto original = Bytes(query);
  for (const auto* disposition : {"stale", "resource_limit"}) {
    std::ofstream(query) << original;
    Replace(query, "disposition: accepted",
            std::string("disposition: ") + disposition);
    report = InspectLoopDataset(root_);
    EXPECT_EQ(report.status, LoopRecoveryStatus::kInconsistent) << disposition;
    EXPECT_EQ(report.committed_version, 1u);
  }
}

TEST_F(LoopClosureStoreTest, EarlyGeometryRejectionOmitsUncomputedDiagnostics) {
  auto options = loop_test::NearbyOptions();
  options.registration.min_points = options.registration.max_points;
  const auto result = Result(2, options);
  ASSERT_EQ(result.admission->status, LoopAdmissionStatus::kGeometryRejected);
  ASSERT_FALSE(result.admission->trial);
  ASSERT_FALSE(result.admission->candidates.empty());
  const auto& verification = result.admission->candidates.front().verification;
  ASSERT_EQ(verification.status, LoopStatus::kInsufficientPoints);
  ASSERT_FALSE(verification.forward.metrics);
  ASSERT_FALSE(verification.target_from_source);
  journal_->WriteQuery(result, LoopDisposition::kRejected, 0, 4);
  auto stats = Stats();
  stats.pose_version = stats.accepted = 0;
  stats.rejected = 1;
  journal_->Finish(stats);
  const auto report = InspectLoopDataset(root_);
  EXPECT_EQ(report.status, LoopRecoveryStatus::kConsistent)
      << testing::PrintToString(report.issues);
}

}  // namespace
}  // namespace livo_atlas
