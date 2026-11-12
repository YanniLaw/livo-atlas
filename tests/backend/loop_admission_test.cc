#include "livo_atlas/backend/loop_admission.h"

#include <algorithm>
#include <map>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "backend/loop_fixtures.h"

namespace livo_atlas {
namespace {

LoopAdmissionInput Input() {
  LoopAdmissionInput input;
  input.raw_poses.resize(101);
  for (std::size_t i = 0; i < input.raw_poses.size(); ++i)
    input.raw_poses[i].translation.x() = i * 0.001;
  input.poses = MaterializePoseSnapshot({}, input.raw_poses, 101);
  for (const auto id : {0u, 5u}) {
    LoopSubmapInfo info;
    const auto anchor = id == 0 ? 0u : 100u;
    info.membership = {
        {id, anchor, {anchor}}, {}, 2, OnlineSubmapState::kSealed};
    info.first_timestamp = info.last_timestamp = anchor;
    info.parent_from_submap = input.raw_poses[anchor];
    info.frame_count = 101;
    input.submaps.push_back(info);
  }
  input.query_id = 5;
  return input;
}

GeometrySubmap ReadGeometry(std::uint64_t id, const PoseSnapshot& poses) {
  const auto anchor = id == 0 ? 0u : 100u;
  auto result = loop_test::Scene(id, anchor);
  result.pose_version = poses.version;
  result.frame_count = poses.frame_count;
  result.optimized_frame_count = poses.optimized_frame_count;
  result.parent_frame = poses.parent_frame;
  result.parent_from_submap = poses.poses[anchor].map_from_body;
  return result;
}

void RefreshInfo(LoopAdmissionInput* input) {
  for (auto& info : input->submaps) {
    info.pose_version = input->poses.version;
    info.frame_count = input->poses.frame_count;
    info.optimized_frame_count = input->poses.optimized_frame_count;
    info.parent_from_submap =
        input->poses.poses[info.membership.primary.anchor_keyframe_id]
            .map_from_body;
  }
}

LoopAdmissionInput HistoryInput() {
  auto input = Input();
  input.raw_poses.resize(201);
  for (std::size_t i = 101; i < input.raw_poses.size(); ++i)
    input.raw_poses[i].translation.x() = i * 0.001;
  input.poses = MaterializePoseSnapshot({}, input.raw_poses, 201);
  for (auto ids : {std::make_pair(3u, 50u), std::make_pair(10u, 200u)}) {
    LoopSubmapInfo info;
    info.membership = {{ids.first, ids.second, {ids.second}},
                       {},
                       2,
                       OnlineSubmapState::kSealed};
    info.first_timestamp = info.last_timestamp = ids.second;
    input.submaps.push_back(info);
  }
  RefreshInfo(&input);
  return input;
}

GeometrySubmap HistoryGeometry(std::uint64_t id, const PoseSnapshot& poses) {
  const auto anchor = id == 0 ? 0u : id == 3 ? 50u : id == 5 ? 100u : 200u;
  RigidTransform transform;
  if (id == 10) transform.translation.x() = 0.15;
  auto result = loop_test::Scene(id, anchor, transform);
  result.pose_version = poses.version;
  result.frame_count = poses.frame_count;
  result.optimized_frame_count = poses.optimized_frame_count;
  result.parent_frame = poses.parent_frame;
  result.parent_from_submap = poses.poses[anchor].map_from_body;
  // Lower coverage makes submap 3 the unambiguous quality winner at query 10.
  if (id == 0) {
    auto data = std::make_shared<GeometryData>(*result.data);
    for (int i = 0; i < 30; ++i) {
      auto voxel = data->voxels.front();
      voxel.key = {1000 + i, 1000, 1000};
      voxel.centroid = {50.0 + i, 50, 50};
      data->voxels.push_back(voxel);
    }
    result.data = data;
  }
  return result;
}

void Accept(const LoopProposal& proposal, LoopAdmissionInput* input) {
  input->poses =
      MakePoseSnapshot(proposal.update, input->poses, input->raw_poses);
  input->accepted_loops.push_back(proposal.loop);
  RefreshInfo(input);
}

TEST(LoopAdmissionTest, ProposesProjectedUpdateWithoutMutatingInput) {
  const auto input = Input();
  std::vector<std::uint64_t> requested_versions;
  const auto result = ProposeLoopUpdate(input, [&](auto id, const auto& poses) {
    requested_versions.push_back(poses.version);
    auto geometry = ReadGeometry(id, poses);
    geometry.representation_revision = 10 + requested_versions.size();
    return geometry;
  });
  ASSERT_EQ(result.status, LoopAdmissionStatus::kProposed);
  ASSERT_TRUE(result.proposal);
  ASSERT_TRUE(result.trial);
  EXPECT_FALSE(result.optimization_error);
  EXPECT_GT(result.trial->graph_build_seconds, 0);
  EXPECT_GT(result.trial->solve_seconds, 0);
  EXPECT_EQ(result.proposal->update.base_version, 0u);
  EXPECT_EQ(result.proposal->update.optimized_frame_count, 101u);
  EXPECT_EQ(result.proposal->update.tail_policy,
            PoseTailPolicy::kProjectedLastOptimizedAnchor);
  EXPECT_LT(
      result.proposal->update.poses.back().map_from_body.translation.norm(),
      0.01);
  EXPECT_EQ(result.proposal->loop.constraint.from_id, 0u);
  EXPECT_EQ(result.proposal->loop.constraint.to_id, 100u);
  ASSERT_EQ(result.geometry_checks.size(), 1u);
  ASSERT_TRUE(result.geometry_checks[0].measurement);
  ASSERT_TRUE(result.geometry_checks[0].prediction);
  EXPECT_EQ(result.geometry_checks[0].measurement->status,
            LoopStatus::kVerified);
  EXPECT_EQ(result.geometry_checks[0].prediction->status,
            LoopStatus::kVerified);
  EXPECT_EQ(requested_versions, (std::vector<std::uint64_t>{0, 0, 1, 1}));
  EXPECT_DOUBLE_EQ(input.raw_poses.back().translation.x(), 0.1);
  EXPECT_DOUBLE_EQ(input.poses.poses.back().map_from_body.translation.x(), 0.1);
  EXPECT_EQ(input.poses.version, 0u);
  EXPECT_TRUE(input.accepted_loops.empty());
}

TEST(LoopAdmissionTest, RefusesTrialsAtEachGraphGateWithoutReturningAnUpdate) {
  const auto input = Input();
  LoopAdmissionOptions options;
  options.graph.max_iterations = 1;
  auto result = ProposeLoopUpdate(input, ReadGeometry, options);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kOptimizationRejected);
  EXPECT_FALSE(result.proposal);
  ASSERT_TRUE(result.trial);
  EXPECT_TRUE(result.trial->iteration_limit_reached);
  options = {};
  options.max_factor_residual = 0.001;
  result = ProposeLoopUpdate(input, ReadGeometry, options);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kConstraintResidual);
  EXPECT_FALSE(result.proposal);
  EXPECT_TRUE(result.geometry_checks.empty());
  options = {};
  options.max_pose_correction_m = 0.01;
  result = ProposeLoopUpdate(input, ReadGeometry, options);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kPoseCorrection);
  EXPECT_FALSE(result.proposal);
  EXPECT_TRUE(result.geometry_checks.empty());
  EXPECT_EQ(ProposeLoopUpdate(input, ReadGeometry).status,
            LoopAdmissionStatus::kProposed);
}

TEST(LoopAdmissionTest, ChecksPreviewGeometryAndRejectsStaleSourcePoses) {
  const auto input = Input();
  const auto changed = [](auto id, const auto& poses) {
    auto geometry = ReadGeometry(id, poses);
    if (poses.version && id == 5) {
      auto data = std::make_shared<GeometryData>(*geometry.data);
      for (auto& voxel : data->voxels) voxel.centroid.x() += 10;
      geometry.data = data;
    }
    return geometry;
  };
  const auto result = ProposeLoopUpdate(input, changed);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kGeometryChanged);
  EXPECT_FALSE(result.proposal);
  ASSERT_EQ(result.geometry_checks.size(), 1u);
  ASSERT_TRUE(result.geometry_checks[0].measurement);
  EXPECT_EQ(result.geometry_checks[0].measurement->status,
            LoopStatus::kLowOverlap);
  EXPECT_FALSE(result.geometry_checks[0].prediction);
  const auto stale = [](auto id, const auto& poses) {
    auto geometry = ReadGeometry(id, poses);
    auto data = std::make_shared<GeometryData>(*geometry.data);
    data->sources.front().submap_from_body.translation.x() = 0.1;
    geometry.data = data;
    return geometry;
  };
  EXPECT_THROW(ProposeLoopUpdate(input, stale), std::invalid_argument);
}

TEST(LoopAdmissionTest, BudgetsAndNoCandidatesDoNotReadAssets) {
  auto input = Input();
  const auto forbidden = [](auto, const auto&) -> GeometrySubmap {
    throw std::logic_error("Unexpected geometry read");
  };
  LoopAdmissionOptions options;
  options.max_keyframes = 100;
  EXPECT_EQ(ProposeLoopUpdate(input, forbidden, options).status,
            LoopAdmissionStatus::kResourceLimit);
  options.max_keyframes = 101;
  EXPECT_EQ(ProposeLoopUpdate(input, ReadGeometry, options).status,
            LoopAdmissionStatus::kProposed);
  input.query_id = 0;
  const auto result = ProposeLoopUpdate(input, forbidden);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kNoCandidate);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_FALSE(result.trial);
  EXPECT_FALSE(result.proposal);
}

TEST(LoopAdmissionTest, RejectsInvalidViewVersionNoiseAndOptions) {
  auto input = Input();
  LoopAdmissionOptions options;
  options.graph.loop_huber_delta = 0;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry, options),
               std::invalid_argument);
  options = {};
  options.registration.max_voxels = 99;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry, options),
               std::invalid_argument);
  input.submaps.front().pose_version = 1;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry), std::invalid_argument);
  input = Input();
  input.raw_poses.pop_back();
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry), std::invalid_argument);
  input = Input();
  const auto proposed = ProposeLoopUpdate(input, ReadGeometry);
  ASSERT_TRUE(proposed.proposal);
  input.poses =
      MakePoseSnapshot(proposed.proposal->update, input.poses, input.raw_poses);
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry), std::invalid_argument);
}

TEST(LoopAdmissionTest,
     PropagatesReaderFailuresIncludingAllocationAndUnknownTypes) {
  const auto input = Input();
  EXPECT_THROW(ProposeLoopUpdate(input,
                                 [](auto, const auto&) -> GeometrySubmap {
                                   throw std::runtime_error("Unreadable asset");
                                 }),
               std::runtime_error);
  EXPECT_THROW(ProposeLoopUpdate(input,
                                 [](auto, const auto&) -> GeometrySubmap {
                                   throw std::bad_alloc();
                                 }),
               std::bad_alloc);
  EXPECT_THROW(ProposeLoopUpdate(
                   input, [](auto, const auto&) -> GeometrySubmap { throw 7; }),
               int);
}

TEST(LoopAdmissionTest,
     RechecksAcceptedMeasurementAndPredictionOnEveryProposal) {
  auto input = HistoryInput();
  const auto first = ProposeLoopUpdate(input, HistoryGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  input.query_id = 10;
  LoopAdmissionOptions options;
  options.search.min_keyframe_gap = 120;
  const auto second = ProposeLoopUpdate(input, HistoryGeometry, options);
  ASSERT_EQ(second.status, LoopAdmissionStatus::kProposed);
  ASSERT_TRUE(second.proposal);
  EXPECT_EQ(second.proposal->loop.target.primary.id, 3u);
  EXPECT_EQ(second.proposal->update.base_version, 1u);
  EXPECT_EQ(second.proposal->update.poses.size(), 201u);
  ASSERT_EQ(second.geometry_checks.size(), 2u);
  EXPECT_FALSE(second.geometry_checks[0].new_constraint);
  EXPECT_TRUE(second.geometry_checks[1].new_constraint);
  for (const auto& check : second.geometry_checks) {
    ASSERT_TRUE(check.measurement);
    ASSERT_TRUE(check.prediction);
    EXPECT_EQ(check.measurement->status, LoopStatus::kVerified);
    EXPECT_EQ(check.prediction->status, LoopStatus::kVerified);
  }
  const auto changed = [](auto id, const auto& poses) {
    auto geometry = HistoryGeometry(id, poses);
    if (poses.version == 2 && id == 0) {
      auto data = std::make_shared<GeometryData>(*geometry.data);
      for (auto& voxel : data->voxels) voxel.centroid.z() += 10;
      geometry.data = data;
    }
    return geometry;
  };
  const auto rejected = ProposeLoopUpdate(input, changed, options);
  EXPECT_EQ(rejected.status, LoopAdmissionStatus::kGeometryChanged);
  EXPECT_FALSE(rejected.proposal);
  ASSERT_EQ(rejected.geometry_checks.size(), 1u);
  EXPECT_FALSE(rejected.geometry_checks[0].new_constraint);
  EXPECT_EQ(rejected.geometry_checks[0].target_id, 0u);
  EXPECT_EQ(input.poses.version, 1u);
  EXPECT_EQ(input.accepted_loops.size(), 1u);
}

TEST(LoopAdmissionTest,
     RejectsHistoricalResidualIncreaseAndExhaustedLoopBudget) {
  auto input = HistoryInput();
  const auto first = ProposeLoopUpdate(input, HistoryGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  input.query_id = 10;
  LoopAdmissionOptions options;
  options.search.min_keyframe_gap = 120;
  options.max_accepted_residual_increase = 0;
  const auto rejected = ProposeLoopUpdate(input, HistoryGeometry, options);
  EXPECT_EQ(rejected.status, LoopAdmissionStatus::kConstraintResidual);
  ASSERT_TRUE(rejected.trial);
  const auto& historical = rejected.trial->loop_residuals.front();
  EXPECT_GT(historical.final_whitened_norm, historical.initial_whitened_norm);
  EXPECT_LT(historical.final_whitened_norm, options.max_factor_residual);
  EXPECT_FALSE(rejected.proposal);
  options.max_loops = 1;
  const auto no_read = [](auto, const auto&) -> GeometrySubmap {
    throw std::logic_error("Budget rejection must precede asset access");
  };
  EXPECT_EQ(ProposeLoopUpdate(input, no_read, options).status,
            LoopAdmissionStatus::kResourceLimit);
}

TEST(LoopAdmissionTest,
     ReportsNumericalFailureWithoutInventingTrialDiagnostics) {
  auto input = HistoryInput();
  const auto first = ProposeLoopUpdate(input, HistoryGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  input.query_id = 10;
  input.raw_poses[25].translation.x() = 1e200;
  LoopAdmissionOptions options;
  options.search.min_keyframe_gap = 120;
  const auto result = ProposeLoopUpdate(input, HistoryGeometry, options);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kOptimizationRejected);
  EXPECT_TRUE(result.optimization_error);
  EXPECT_FALSE(result.trial);
  EXPECT_FALSE(result.proposal);
}

TEST(LoopAdmissionTest, RejectsInvalidAcceptedNoiseAndDuplicateConstraints) {
  auto input = Input();
  const auto first = ProposeLoopUpdate(input, ReadGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  input.accepted_loops[0].constraint.translation_sigma_m = 0.3;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry), std::invalid_argument);
  input.accepted_loops[0] = first.proposal->loop;
  input.accepted_loops.push_back(first.proposal->loop);
  ++input.poses.version;
  ++input.poses.base_version;
  RefreshInfo(&input);
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry), std::invalid_argument);
}

TEST(LoopAdmissionTest, ValidatesRawRotationAndHistoryPolicyBeforeBudgets) {
  auto input = Input();
  input.poses.poses[50].map_from_body.rotation(0, 0) = 1.000001;
  LoopAdmissionOptions options;
  options.max_keyframes = 100;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry, options),
               std::invalid_argument);
  input = Input();
  const auto first = ProposeLoopUpdate(input, ReadGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  input.poses.tail_policy = PoseTailPolicy::kLegacyLastOptimizedAnchor;
  EXPECT_THROW(ProposeLoopUpdate(input, ReadGeometry, options),
               std::invalid_argument);
}

TEST(LoopAdmissionTest,
     ChecksPredictedTransformAfterOriginalMeasurementPasses) {
  LoopAdmissionOptions options;
  // Allow the roughly 0.1 mm fitting error, but reject the graph's several-mm
  // compromise between the odometry and loop measurement.
  options.registration.max_rmse_m = 0.001;
  options.registration.max_p95_m = 0.001;
  const auto result = ProposeLoopUpdate(Input(), ReadGeometry, options);
  ASSERT_FALSE(result.candidates.empty());
  const auto& verification = result.candidates.front().verification;
  ASSERT_TRUE(verification.forward.metrics);
  ASSERT_TRUE(verification.reverse.metrics);
  ASSERT_EQ(verification.status, LoopStatus::kVerified)
      << "forward rmse=" << verification.forward.metrics->rmse_m.value_or(-1)
      << " reverse rmse=" << verification.reverse.metrics->rmse_m.value_or(-1)
      << " forward p95=" << verification.forward.metrics->p95_m.value_or(-1)
      << " reverse p95=" << verification.reverse.metrics->p95_m.value_or(-1);
  EXPECT_EQ(result.status, LoopAdmissionStatus::kGeometryChanged);
  EXPECT_FALSE(result.proposal);
  ASSERT_EQ(result.geometry_checks.size(), 1u);
  const auto& report = result.geometry_checks.front();
  ASSERT_TRUE(report.measurement);
  ASSERT_TRUE(report.prediction);
  EXPECT_EQ(report.measurement->status, LoopStatus::kVerified);
  EXPECT_EQ(report.prediction->status, LoopStatus::kHighResidual);
}

TEST(LoopAdmissionTest, RequiresRebuiltLocalSourcesAfterNonrigidPoseChange) {
  auto input = Input();
  input.submaps[0].membership.primary.keyframe_ids.push_back(1);
  input.submaps[0].membership.membership_revision = 3;
  input.submaps[0].last_timestamp = 1;
  const auto reader = [&](auto id, const auto& poses, bool stale) {
    auto geometry = ReadGeometry(id, poses);
    if (id == 0) {
      geometry.membership = input.submaps[0].membership;
      auto data = std::make_shared<GeometryData>(*geometry.data);
      const auto& anchor = poses.poses[0].map_from_body;
      const auto& member = poses.poses[1].map_from_body;
      RigidTransform local;
      local.rotation = anchor.rotation.transpose() * member.rotation;
      local.translation = anchor.rotation.transpose() *
                          (member.translation - anchor.translation);
      if (stale) local.translation.x() = 0.001;
      data->sources.push_back({1, local});
      geometry.data = data;
    }
    return geometry;
  };
  EXPECT_THROW(ProposeLoopUpdate(input,
                                 [&](auto id, const auto& poses) {
                                   return reader(id, poses, true);
                                 }),
               std::invalid_argument);
  EXPECT_EQ(ProposeLoopUpdate(input,
                              [&](auto id, const auto& poses) {
                                return reader(id, poses, false);
                              })
                .status,
            LoopAdmissionStatus::kProposed);
}

TEST(LoopAdmissionTest, GeometryAndCorrelationRejectBeforeGraphConstruction) {
  auto input = Input();
  const auto planar = [](auto id, const auto& poses) {
    auto geometry = ReadGeometry(id, poses);
    auto data = std::make_shared<GeometryData>(*geometry.data);
    data->voxels.resize(100);
    geometry.data = data;
    return geometry;
  };
  LoopAdmissionOptions options;
  options.registration.voxel_size_m = 0.2;
  const auto rejected = ProposeLoopUpdate(input, planar, options);
  EXPECT_EQ(rejected.status, LoopAdmissionStatus::kGeometryRejected);
  EXPECT_FALSE(rejected.trial);
  EXPECT_FALSE(rejected.proposal);
  const auto first = ProposeLoopUpdate(input, ReadGeometry);
  ASSERT_TRUE(first.proposal);
  Accept(*first.proposal, &input);
  const auto repeated = ProposeLoopUpdate(input, ReadGeometry);
  EXPECT_EQ(repeated.status, LoopAdmissionStatus::kCorrelated);
  EXPECT_FALSE(repeated.trial);
  EXPECT_FALSE(repeated.proposal);
}

TEST(LoopAdmissionTest,
     UsesVoxelBudgetForQualityCountsRatherThanCoarsePointBudget) {
  LoopAdmissionOptions options;
  options.registration.max_points = 200;
  options.registration.min_inliers = 300;
  options.registration.min_normal_pairs = 300;
  EXPECT_EQ(ProposeLoopUpdate(Input(), ReadGeometry, options).status,
            LoopAdmissionStatus::kProposed);
  options.registration.max_voxels = 299;
  EXPECT_THROW(ProposeLoopUpdate(Input(), ReadGeometry, options),
               std::invalid_argument);
  options.registration.min_inliers = 100;
  EXPECT_THROW(ProposeLoopUpdate(Input(), ReadGeometry, options),
               std::invalid_argument);
}

TEST(LoopAdmissionTest, PreservesCallerOwnedGeometryAndAllPoseMatrices) {
  const auto input = Input();
  const auto before = input;
  std::map<std::uint64_t, GeometrySubmap> original;
  std::map<std::uint64_t, GeometryData> saved;
  for (auto id : {0u, 5u}) {
    original.emplace(id, ReadGeometry(id, input.poses));
    saved.emplace(id, *original.at(id).data);
  }
  const auto result = ProposeLoopUpdate(input, [&](auto id, const auto& poses) {
    return poses.version ? ReadGeometry(id, poses) : original.at(id);
  });
  ASSERT_TRUE(result.proposal);
  for (std::size_t i = 0; i < input.raw_poses.size(); ++i) {
    EXPECT_TRUE(input.raw_poses[i].rotation == before.raw_poses[i].rotation);
    EXPECT_TRUE(input.raw_poses[i].translation ==
                before.raw_poses[i].translation);
    EXPECT_TRUE(input.poses.poses[i].map_from_body.rotation ==
                before.poses.poses[i].map_from_body.rotation);
    EXPECT_TRUE(input.poses.poses[i].map_from_body.translation ==
                before.poses.poses[i].map_from_body.translation);
  }
  for (const auto& entry : original) {
    const auto& data = *entry.second.data;
    const auto& previous = saved.at(entry.first);
    EXPECT_EQ(data.input_point_count, previous.input_point_count);
    ASSERT_EQ(data.sources.size(), previous.sources.size());
    for (std::size_t i = 0; i < data.sources.size(); ++i) {
      EXPECT_EQ(data.sources[i].keyframe_id, previous.sources[i].keyframe_id);
      EXPECT_TRUE(data.sources[i].submap_from_body.rotation ==
                  previous.sources[i].submap_from_body.rotation);
      EXPECT_TRUE(data.sources[i].submap_from_body.translation ==
                  previous.sources[i].submap_from_body.translation);
    }
    ASSERT_EQ(data.voxels.size(), previous.voxels.size());
    for (std::size_t i = 0; i < data.voxels.size(); ++i) {
      const auto& a = data.voxels[i];
      const auto& b = previous.voxels[i];
      EXPECT_TRUE(a.key == b.key);
      EXPECT_EQ(a.point_count, b.point_count);
      EXPECT_TRUE(a.centroid == b.centroid);
      EXPECT_TRUE(a.spatial_covariance == b.spatial_covariance);
      EXPECT_DOUBLE_EQ(a.mean_intensity, b.mean_intensity);
      EXPECT_EQ(a.keyframe_ids, b.keyframe_ids);
      ASSERT_EQ(a.normal.has_value(), b.normal.has_value());
      if (a.normal) EXPECT_TRUE(*a.normal == *b.normal);
    }
  }
}

TEST(LoopAdmissionTest, RoundedRotationsDoNotScalePhysicalCorrectionBounds) {
  for (double scale : {0.99999, 1.00001}) {
    auto input = Input();
    std::vector<Eigen::Matrix3d> rotations;
    for (std::size_t i = 0; i < input.raw_poses.size(); ++i) {
      rotations.push_back(Eigen::AngleAxisd(0.001 * i, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix());
      input.raw_poses[i].rotation = scale * rotations.back();
    }
    input.poses =
        MaterializePoseSnapshot({}, input.raw_poses, input.raw_poses.size());
    RefreshInfo(&input);
    const auto reader = [](auto id, const auto& poses) {
      auto geometry = ReadGeometry(id, poses);
      auto data = std::make_shared<GeometryData>(*geometry.data);
      const auto& rotation = geometry.parent_from_submap.rotation;
      data->sources.front().submap_from_body.rotation =
          rotation.transpose() * rotation;
      geometry.data = data;
      return geometry;
    };
    const auto baseline = ProposeLoopUpdate(input, reader);
    ASSERT_EQ(baseline.status, LoopAdmissionStatus::kProposed);
    ASSERT_TRUE(baseline.proposal);
    double displacement = 0, angle = 0;
    for (const auto& pose : baseline.proposal->update.poses) {
      displacement =
          std::max(displacement, (pose.map_from_body.translation -
                                  input.raw_poses[pose.id].translation)
                                     .stableNorm());
      angle = std::max(angle, Eigen::AngleAxisd(rotations[pose.id].transpose() *
                                                pose.map_from_body.rotation)
                                      .angle() *
                                  (180 / 3.14159265358979323846));
    }
    // The margin is smaller than rounding-induced scaling but well above
    // floating-point roundoff. Bounds are derived from world displacement and
    // the known rigid rotations, independently of the gate implementation.
    const double margin = scale > 1 ? 1.000001 : 0.999999;
    const auto expected = scale > 1 ? LoopAdmissionStatus::kProposed
                                    : LoopAdmissionStatus::kPoseCorrection;
    LoopAdmissionOptions options;
    options.max_pose_correction_m = displacement * margin;
    EXPECT_EQ(ProposeLoopUpdate(input, reader, options).status, expected)
        << scale;
    options = {};
    options.max_pose_correction_deg = angle * margin;
    EXPECT_EQ(ProposeLoopUpdate(input, reader, options).status, expected)
        << scale;
    EXPECT_DOUBLE_EQ(input.raw_poses[0].rotation(0, 0), scale);
  }
}

}  // namespace
}  // namespace livo_atlas
