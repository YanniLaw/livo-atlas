#include "livo_atlas/backend/pose_graph.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "core/fixtures.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/core/online_submap_update.h"
#include "livo_atlas/io/keyframe_store.h"

namespace livo_atlas {
namespace {

using FramePose = std::pair<std::uint64_t, RigidTransform>;

SystemOptions OnlineGeometryOptions() {
  auto options = test::Options(FrontendMode::kLidarOnly);
  options.keyframes.enabled = options.submaps.enabled = true;
  options.keyframes.time_threshold = 0;
  options.submap_mode = SubmapMode::kOnline;
  options.submaps.max_keyframes = 3;
  options.geometry.enabled = true;
  return options;
}

TEST(PoseGraphSystemTest, DelayedUpdateRebuildsGeometryAndPreservesFrontend) {
  std::vector<std::vector<double>> reference_states;
  std::vector<RigidTransform> reference_raw;
  for (const bool optimize : {false, true}) {
    auto options = OnlineGeometryOptions();
    std::vector<std::vector<double>> states;
    std::vector<RigidTransform> raw;
    {
      LivoSystem system(options);
      PoseGraph graph({});
      std::optional<FramePose> pending;
      bool overflow = false;
      system.SetOnlineSubmapCallback([&](const OnlineSubmapUpdate& event) {
        if (!event.keyframe) return;
        if (pending) {
          overflow = true;
          return;
        }
        pending.emplace(event.keyframe->id, event.keyframe->odom_from_body);
      });

      const auto collect = [&](const FrontendResult& result, const auto&,
                               double) {
        const auto& state = result.estimate;
        std::vector<double> values(state.cov.data(),
                                   state.cov.data() + state.cov.size());
        values.insert(values.end(), state.rot_end.data(),
                      state.rot_end.data() + state.rot_end.size());
        for (const auto* vector :
             {&state.pos_end, &state.vel_end, &state.bias_g, &state.bias_a}) {
          values.insert(values.end(), vector->data(),
                        vector->data() + vector->size());
        }
        states.push_back(std::move(values));
      };

      const auto feed = [&](int frame) {
        EXPECT_TRUE(system.AddLidar(test::Scan(100 + frame * 0.25)));
        for (;;) {
          const auto status = system.ProcessNext(collect);
          if (pending) {
            graph.AddKeyframe(pending->first, pending->second);
            raw.push_back(pending->second);
            pending.reset();
          }
          if (status == SyncStatus::kWaiting) break;
        }
      };

      for (int frame = 0; frame < 5; ++frame) feed(frame);
      ASSERT_FALSE(overflow);
      ASSERT_GE(raw.size(), 3u);
      const auto original = system.GetPoseSnapshot();
      const auto original_geometry = system.GetGeometrySubmap(0);
      ASSERT_TRUE(original_geometry.data);
      ASSERT_GE(original_geometry.membership.primary.keyframe_ids.size(), 3u);

      if (optimize) {
        RigidTransform measurement;
        measurement.translation.x() = 0.5;
        const LoopConstraint loop{0, 2, measurement, 0.01, 0.01};
        graph.AddLoopConstraint(loop);
        const auto candidate = graph.Optimize(*original);
        ASSERT_FALSE(candidate.iteration_limit_reached);
        EXPECT_LT(candidate.final_error, candidate.initial_error);

        PoseGraphOptions limited_options;
        limited_options.max_iterations = 1;
        PoseGraph limited(limited_options);
        for (std::size_t i = 0; i < raw.size(); ++i) {
          limited.AddKeyframe(i, raw[i]);
        }
        limited.AddLoopConstraint(loop);
        const auto limited_candidate = limited.Optimize(*original);
        ASSERT_TRUE(limited_candidate.iteration_limit_reached);
        // The example deliberately does not commit an iteration-limited result.
        EXPECT_EQ(system.GetPoseSnapshot()->version, 0u);

        feed(5);
        ASSERT_GT(raw.size(), candidate.update.optimized_frame_count);
        system.ApplyPoseUpdate(candidate.update);
        const auto corrected = system.GetPoseSnapshot();
        EXPECT_EQ(corrected->version, 1u);
        EXPECT_EQ(corrected->optimized_frame_count,
                  candidate.update.optimized_frame_count);
        EXPECT_EQ(corrected->frame_count, raw.size());
        EXPECT_EQ(corrected->tail_policy,
                  PoseTailPolicy::kProjectedLastOptimizedAnchor);
        const auto anchor = candidate.update.optimized_frame_count - 1;
        const auto& optimized_anchor =
            candidate.update.poses[anchor].map_from_body;
        const Eigen::Matrix3d correction =
            optimized_anchor.rotation * raw[anchor].rotation.transpose();
        const Eigen::Vector3d expected_tail =
            optimized_anchor.translation +
            correction * (raw.back().translation - raw[anchor].translation);
        EXPECT_TRUE(corrected->poses.back().map_from_body.translation.isApprox(
            expected_tail, 1e-8));

        const auto rebuilt = system.GetGeometrySubmap(0);
        EXPECT_EQ(rebuilt.pose_version, 1u);
        EXPECT_EQ(rebuilt.membership.primary.keyframe_ids,
                  original_geometry.membership.primary.keyframe_ids);
        EXPECT_NE(rebuilt.data.get(), original_geometry.data.get());
        EXPECT_GT(rebuilt.representation_revision,
                  original_geometry.representation_revision);
        EXPECT_EQ(original_geometry.pose_version, 0u);
        EXPECT_EQ(original->version, 0u);
        EXPECT_THROW(system.ApplyPoseUpdate(candidate.update),
                     std::invalid_argument);

        const auto retried = graph.Optimize(*corrected);
        ASSERT_FALSE(retried.iteration_limit_reached);
        EXPECT_EQ(retried.update.base_version, 1u);
        system.ApplyPoseUpdate(retried.update);
        EXPECT_EQ(system.GetPoseSnapshot()->version, 2u);
        EXPECT_EQ(corrected->version, 1u);
      } else {
        feed(5);
      }
      feed(6);
      feed(7);
      EXPECT_FALSE(overflow);
      system.Finish();

      const KeyframeStore stored(options.exporter.directory / "keyframes");
      ASSERT_EQ(stored.ids().size(), raw.size());
      for (std::size_t i = 0; i < raw.size(); ++i) {
        const auto pose = stored.ReadRawBodyPose(i);
        EXPECT_TRUE((pose.rotation.array() == raw[i].rotation.array()).all());
        EXPECT_TRUE(
            (pose.translation.array() == raw[i].translation.array()).all());
      }
    }
    if (optimize) {
      EXPECT_EQ(states, reference_states);
      ASSERT_EQ(raw.size(), reference_raw.size());
      for (std::size_t i = 0; i < raw.size(); ++i) {
        EXPECT_TRUE(
            (raw[i].rotation.array() == reference_raw[i].rotation.array())
                .all());
        EXPECT_TRUE(
            (raw[i].translation.array() == reference_raw[i].translation.array())
                .all());
      }
    } else {
      reference_states = std::move(states);
      reference_raw = std::move(raw);
    }
    std::filesystem::remove_all(options.exporter.directory);
  }
}

}  // namespace
}  // namespace livo_atlas
