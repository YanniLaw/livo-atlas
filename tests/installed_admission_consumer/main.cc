#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

#include "livo_atlas/common/logging.h"
#include "livo_atlas/core/geometry_map_output.h"
#include "livo_atlas/core/global_output_options.h"
#include "livo_atlas/core/livo_system.h"
#include "livo_atlas/core/trajectory_output.h"
#include "livo_atlas/mapping/geometry_map_builder.h"
#include "livo_atlas/mapping/map_correction.h"
#include "livo_atlas/mapping/pose_snapshot.h"
#ifdef EXPECT_GTSAM
#include "livo_atlas/backend/pose_graph.h"
#endif
#ifdef EXPECT_LOOP
#include "livo_atlas/backend/loop_closure.h"
#endif
#ifdef EXPECT_ADMISSION
#include "livo_atlas/backend/loop_admission.h"
#include "livo_atlas/core/loop_backend.h"
#endif

namespace livo_atlas {

int RunInstalledAdmissionConsumer() {
  try {
    const std::vector<RigidTransform> raw(3);
    const auto poses = MaterializePoseSnapshot({}, raw, raw.size());
    const auto history = BuildTrajectorySnapshot(poses, {1, 2, 3});
    if (history.frame_count != 3 || history.keyframes.size() != 3 ||
        history.keyframes[2].id != 2 || history.keyframes[2].timestamp != 3)
      throw std::runtime_error(
          "Installed trajectory builder lost its frontier");
    auto geometry_poses = std::make_shared<PoseSnapshot>();
    geometry_poses->frame_count = 1;
    geometry_poses->poses.push_back({0, {}});
    OnlineSubmap group;
    group.primary.keyframe_ids = {0};
    GeometryMapBuilder map_builder({}, {}, [](std::uint64_t) {
      PointCloud::Ptr cloud(new PointCloud);
      Point point;
      point.x = 0.1f;
      point.intensity = 4;
      cloud->push_back(point);
      return cloud;
    });
    const auto map = map_builder.Build({group}, geometry_poses);
    if (map.status != GeometryMapStatus::kReady || map.points.size() != 1 ||
        map.sources.size() != 1 || map.sources[0].input_point_count != 1)
      throw std::runtime_error(
          "Installed geometry map builder lost its source");
#ifdef EXPECT_GTSAM
    PoseGraph graph({});
    for (std::uint64_t i = 0; i < raw.size(); ++i) graph.AddKeyframe(i, raw[i]);
    const auto optimized = graph.Optimize(poses);
    if (optimized.update.poses.size() != 3 || optimized.final_error != 0)
      return 1;
#endif
#ifdef EXPECT_LOOP
    GeometrySubmap target;
    target.membership = {{0, 0, {0}}, {}, 2, OnlineSubmapState::kSealed};
    target.frame_count = 3;
    target.representation_revision = 1;
    auto target_data = std::make_shared<GeometryData>();
    target_data->sources.push_back({0, {}});
    target.data = target_data;
    auto source = target;
    source.membership.primary = {3, 2, {2}};
    auto source_data = std::make_shared<GeometryData>();
    source_data->sources.push_back({2, {}});
    source.data = source_data;
    if (VerifyLoop(target, source).status != LoopStatus::kInsufficientPoints)
      return 1;
#endif
#ifdef EXPECT_ADMISSION
    LoopAdmissionInput input;
    input.raw_poses = raw;
    input.poses = poses;
    LoopSubmapInfo info;
    info.membership = target.membership;
    info.frame_count = 3;
    input.submaps.push_back(info);
    const auto result = ProposeLoopUpdate(
        input, [](std::uint64_t, const PoseSnapshot&) -> GeometrySubmap {
          throw std::logic_error("No candidate should read geometry");
        });
    if (result.status != LoopAdmissionStatus::kNoCandidate || result.proposal)
      return 1;
#endif
    const GlobalOutputOptions output{true, true, "atlas/map_2"};
    ValidateGlobalOutputOptions(output);
    const RigidTransform published{Eigen::Matrix3d::Identity(), {1, 2, 3}};
    if (!ApplyMapCorrection({}, published)
             .translation.isApprox(Eigen::Vector3d(1, 2, 3)))
      return 1;
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("livo-installed-loops-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    {
      SystemOptions options;
      options.frontend.calibration.camera = {"Pinhole", 512, 512, 1, 200,
                                             200,       256, 256, {}};
      options.frontend.imu.enabled = false;
      options.image_enabled = false;
      options.exporter.directory = directory;
      options.keyframes.enabled = options.submaps.enabled = true;
      options.submap_mode = SubmapMode::kOnline;
      LivoSystem system(options);
      const auto correction = system.GetMapCorrection();
      if (correction.pose_version != 0 || correction.frame_count != 0 ||
          correction.optimized_frame_count != 0 ||
          !ApplyMapCorrection(correction, published)
               .translation.isApprox(published.translation))
        throw std::runtime_error("Installed map correction is not identity");
#ifdef EXPECT_ADMISSION
      LoopBackend backend(system);
#endif
      // Read these values only after Finish has joined their sole writer.
      std::vector<TrajectorySnapshot> received;
      TrajectoryOutputOptions trajectory_options;
      trajectory_options.enabled = true;
      TrajectoryOutput trajectory(
          system, trajectory_options,
          [&](const auto& item) { received.push_back(item); });
      std::vector<GeometryMapSnapshot> maps;
      GeometryMapOutputOptions geometry_options;
      geometry_options.enabled = true;
      GeometryMapOutput geometry(
          system, geometry_options, options.geometry,
          [&](const auto& item) { maps.push_back(item); });
      system.SetOnlineSubmapCallback([&](const auto& update) {
#ifdef EXPECT_ADMISSION
        backend.Observe(update);
#endif
        trajectory.Observe(update);
        geometry.Observe(update);
      });
#ifdef EXPECT_ADMISSION
      backend.Finish();
      const auto report = InspectLoopDataset(directory / "keyframes");
      if (report.status != LoopRecoveryStatus::kConsistent)
        throw std::runtime_error(
            "Installed backend did not finish consistently");
#else
      system.Finish();
#endif
      trajectory.Finish();
      geometry.Finish();
      if (received.size() != 1 || received[0].revision != 1 ||
          received[0].producer_id.size() != 32 ||
          received[0].frame_count != 0 ||
          received[0].status != TrajectoryStatus::kReady)
        throw std::runtime_error(
            "Installed trajectory worker lost its final state");
      if (maps.size() != 1 || maps[0].revision != 1 ||
          maps[0].producer_id.size() != 32 || maps[0].frame_count != 0 ||
          maps[0].status != GeometryMapStatus::kReady)
        throw std::runtime_error(
            "Installed geometry worker lost its final state");
    }
    std::filesystem::remove_all(directory);
    return 0;
  } catch (const std::exception& error) {
    LOG(ERROR) << error.what();
    return 1;
  }
}

}  // namespace livo_atlas

int main(int, char** argv) {
  livo_atlas::InitializeLogging(argv[0]);
  return livo_atlas::RunInstalledAdmissionConsumer();
}
