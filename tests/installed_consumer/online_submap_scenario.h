#ifndef LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_ONLINE_SUBMAP_SCENARIO_H_
#define LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_ONLINE_SUBMAP_SCENARIO_H_

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "livo_atlas/io/colmap_exporter.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/io/submap_store.h"
namespace livo_atlas::test_data {
// Deterministic stopped datasets: one has both explicit index namespaces and
// a raw tail not yet indexed; the other only has the legacy index.
inline void WriteOnlineFixtures(const std::filesystem::path& root) {
  namespace fs = std::filesystem;
  if (!fs::create_directory(root))
    throw std::runtime_error("Fixture directory already exists");
  Calibration calibration;
  calibration.camera = {"Pinhole", 8, 6, 1, 4, 4, 3.5, 2.5, {}};
  for (const auto* name : {"online", "legacy"}) {
    const auto dataset = root / name;
    KeyframeStore frames(dataset, calibration);
    std::vector<RigidTransform> raw;
    OnlineSubmapBuilder builder(2, {});
    std::unique_ptr<OnlineSubmapStore> online;
    if (std::string(name) == "online")
      online = std::make_unique<OnlineSubmapStore>(
          dataset / "online_submaps", frames, 2, OnlineSubmapOptions{});
    for (std::uint64_t id = 0; id < 4; ++id) {
      Keyframe frame;
      frame.id = id;
      frame.timestamp = frame.cloud_timestamp = 100 + id;
      frame.image_timestamp = frame.timestamp;
      frame.odom_from_body.translation.x() = id;
      frame.camera_from_odom = RigidTransform{};
      frame.camera_from_odom->translation.x() = -static_cast<double>(id);
      frame.image = cv::Mat(6, 8, CV_8UC3, cv::Scalar(5, 17, 29));
      PointCloud::Ptr cloud(new PointCloud);
      Point point{};
      point.z = 4;
      point.intensity = 7;
      cloud->push_back(point);
      frame.cloud = cloud;
      frames.Append(frame);
      raw.push_back(frame.odom_from_body);
      if (online && id < 3)
        for (const auto& revision :
             builder.AddKeyframe(id, frame.timestamp, frame.odom_from_body))
          online->AppendRevision(revision);
      if (online && id == 2) {
        PoseUpdate update;
        update.optimized_frame_count = 2;
        for (std::uint64_t i = 0; i < 2; ++i) {
          auto pose = raw[i];
          pose.translation.x() += 10;
          update.poses.push_back({i, pose});
        }
        online->WritePoseSnapshot(MakePoseSnapshot(update, {}, raw));
      }
    }
    SubmapStore legacy(dataset / "submaps", frames, 2);
    SubmapBuilder old_builder(2);
    for (auto id : frames.ids())
      if (const auto group = old_builder.AddKeyframe(id)) legacy.Append(*group);
    if (online) {
      const auto extended = online->MaterializePoseSnapshot(1, 4);
      if (extended.poses.back().map_from_body.translation.x() != 13)
        throw std::runtime_error("Unexpected materialized tail");
      frames.WritePoses(1, extended.poses);
      bool rejected = false;
      try {
        frames.WritePoses(1, extended.poses);
      } catch (const std::invalid_argument&) {
        rejected = true;
      }
      if (!rejected)
        throw std::runtime_error("Compatibility export overwrote a version");
      const auto exported = ExportColmap(dataset, root / "colmap", {1, 1});
      if (exported.images != 4 || exported.points == 0)
        throw std::runtime_error("Online compatibility COLMAP export failed");
    }
  }
}
}  // namespace livo_atlas::test_data
#endif  // LIVO_ATLAS_TESTS_INSTALLED_CONSUMER_ONLINE_SUBMAP_SCENARIO_H_
