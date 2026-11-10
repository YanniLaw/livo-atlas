#include "livo_atlas/io/geometry_exporter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/persistence.hpp>

#include "dataset_metadata.h"
#include "livo_atlas/io/keyframe_store.h"
#include "livo_atlas/io/online_submap_store.h"
#include "livo_atlas/mapping/geometry_mapper.h"

namespace livo_atlas {
namespace {
namespace fs = std::filesystem;
namespace meta = dataset_metadata;

bool Within(const fs::path& path, const fs::path& parent) {
  auto item = path.begin();
  for (auto prefix = parent.begin(); prefix != parent.end(); ++prefix, ++item) {
    if (item == path.end() || *item != *prefix) return false;
  }
  return true;
}

void WriteOptions(cv::FileStorage& yaml, const GeometryOptions& options) {
  yaml << "options"
       << "{"
       << "voxel_size" << meta::Decimal(options.voxel_size)
       << "min_points_per_voxel" << options.min_points_per_voxel
       << "max_plane_rms" << meta::Decimal(options.max_plane_rms)
       << "max_plane_ratio" << meta::Decimal(options.max_plane_ratio)
       << "max_voxels_per_submap" << options.max_voxels_per_submap
       << "max_points_per_keyframe" << options.max_points_per_keyframe
       << "max_resident_submaps" << options.max_resident_submaps << "}";
}

void WriteCalibration(cv::FileStorage& yaml, const Calibration& calibration) {
  const auto& camera = calibration.camera;
  yaml << "calibration"
       << "{"
       << "camera"
       << "{"
       << "model" << camera.model << "width" << camera.width << "height"
       << camera.height << "scale" << meta::Decimal(camera.scale) << "fx"
       << meta::Decimal(camera.fx) << "fy" << meta::Decimal(camera.fy) << "cx"
       << meta::Decimal(camera.cx) << "cy" << meta::Decimal(camera.cy)
       << "distortion"
       << "[";
  for (double value : camera.distortion) yaml << meta::Decimal(value);
  yaml << "]"
       << "}";
  meta::WriteTransform(yaml, "imu_from_lidar", calibration.imu_from_lidar);
  meta::WriteTransform(yaml, "camera_from_lidar",
                       calibration.camera_from_lidar);
  yaml << "}";
}

void WriteMetadata(const fs::path& path, const GeometrySubmap& submap) {
  meta::WriteYaml(path, [&](auto& yaml) {
    yaml << "submap_id" << std::to_string(submap.membership.primary.id)
         << "anchor_keyframe_id"
         << std::to_string(submap.membership.primary.anchor_keyframe_id)
         << "membership_revision"
         << std::to_string(submap.membership.membership_revision) << "state"
         << (submap.membership.state == OnlineSubmapState::kActive ? "active"
                                                                   : "sealed")
         << "pose_version" << std::to_string(submap.pose_version)
         << "frame_count" << std::to_string(submap.frame_count)
         << "optimized_frame_count"
         << std::to_string(submap.optimized_frame_count) << "parent_frame"
         << (submap.parent_frame == PoseFrame::kOdom ? "odom" : "map")
         << "representation_revision"
         << std::to_string(submap.representation_revision)
         << "input_point_count"
         << std::to_string(submap.data->input_point_count)
         << "primary_keyframes"
         << "[";
    for (const auto id : submap.membership.primary.keyframe_ids)
      yaml << std::to_string(id);
    yaml << "]"
         << "support_keyframes"
         << "[";
    for (const auto id : submap.membership.support_keyframe_ids)
      yaml << std::to_string(id);
    yaml << "]";
    meta::WriteTransform(yaml, "parent_from_submap", submap.parent_from_submap);
    yaml << "sources"
         << "[";
    for (const auto& source : submap.data->sources) {
      yaml << "{"
           << "keyframe_id" << std::to_string(source.keyframe_id);
      meta::WriteTransform(yaml, "submap_from_body", source.submap_from_body);
      yaml << "}";
    }
    yaml << "]";
  });
}

void WriteVoxels(const fs::path& directory, const GeometryData& data) {
  std::ofstream voxels, contributors;
  voxels.exceptions(std::ios::badbit | std::ios::failbit);
  contributors.exceptions(std::ios::badbit | std::ios::failbit);
  voxels.imbue(std::locale::classic());
  contributors.imbue(std::locale::classic());
  voxels.open(directory / "voxels.csv", std::ios::binary);
  contributors.open(directory / "contributors.csv", std::ios::binary);
  voxels << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "kx,ky,kz,count,x,y,z,cxx,cxy,cxz,cyy,cyz,czz,intensity,plane_"
            "valid,nx,ny,nz\n";
  contributors << "kx,ky,kz,keyframe_id\n";
  for (const auto& voxel : data.voxels) {
    const auto& k = voxel.key;
    const auto& p = voxel.centroid;
    const auto& c = voxel.spatial_covariance;
    const Eigen::Vector3d normal =
        voxel.normal.value_or(Eigen::Vector3d::Zero());
    voxels << k.x << ',' << k.y << ',' << k.z << ',' << voxel.point_count << ','
           << p.x() << ',' << p.y() << ',' << p.z() << ',' << c(0, 0) << ','
           << c(0, 1) << ',' << c(0, 2) << ',' << c(1, 1) << ',' << c(1, 2)
           << ',' << c(2, 2) << ',' << voxel.mean_intensity << ','
           << (voxel.normal ? 1 : 0) << ',' << normal.x() << ',' << normal.y()
           << ',' << normal.z() << '\n';
    for (const auto id : voxel.keyframe_ids) {
      contributors << k.x << ',' << k.y << ',' << k.z << ',' << id << '\n';
    }
  }
  voxels.close();
  contributors.close();
}

}  // namespace

void ExportGeometry(const fs::path& dataset, const fs::path& output,
                    std::uint64_t pose_version,
                    const GeometryOptions& options) {
  ValidateGeometryOptions(options);
  if (dataset.empty() || output.empty())
    throw std::invalid_argument("Empty geometry path");
  const auto source = fs::canonical(dataset);
  const auto destination = fs::weakly_canonical(fs::absolute(output));
  if (Within(destination, source))
    throw std::invalid_argument("Geometry output is inside source dataset");
  if (fs::exists(fs::symlink_status(output)))
    throw std::invalid_argument("Geometry output already exists");

  KeyframeStore keyframes(source);
  OnlineSubmapStore store(source / "online_submaps", keyframes);
  const auto frozen = store.ReadPoseSnapshot(pose_version);
  auto poses = std::make_shared<const PoseSnapshot>(
      store.MaterializePoseSnapshot(pose_version, keyframes.ids().size()));
  std::vector<OnlineSubmap> groups;
  for (std::size_t i = 0; i < store.size(); ++i)
    groups.push_back(store.Read(i));
  GeometryMapper mapper(options, [&](std::uint64_t id) {
    return keyframes.ReadBodyCloud(id, options.max_points_per_keyframe);
  });
  mapper.Update(groups, poses);

  const auto parent = fs::absolute(output).parent_path();
  const auto pending = parent / (".pending-" + output.filename().string());
  if (fs::exists(fs::symlink_status(pending)))
    throw std::invalid_argument("Pending geometry output already exists");
  fs::create_directories(parent);
  fs::create_directory(pending);
  for (std::size_t id = 0; id < groups.size(); ++id) {
    const auto directory = pending / "submaps" / std::to_string(id);
    fs::create_directories(directory);
    const auto submap = mapper.GetSubmap(id);
    WriteMetadata(directory / "metadata.yml", submap);
    WriteVoxels(directory, *submap.data);
  }
  meta::WriteYaml(pending / "manifest.yml", [&](auto& yaml) {
    yaml << "schema_version" << 1 << "algorithm"
         << "voxel_moments_v1"
         << "source_dataset" << source.string() << "pose_version"
         << std::to_string(pose_version) << "stored_frame_count"
         << std::to_string(pose_version ? frozen.frame_count : 0)
         << "frame_count" << std::to_string(poses->frame_count)
         << "optimized_frame_count"
         << std::to_string(poses->optimized_frame_count) << "unindexed_frames"
         << std::to_string(keyframes.ids().size() -
                           store.indexed_frame_count());
    WriteOptions(yaml, options);
    WriteCalibration(yaml, keyframes.calibration());
    yaml << "submaps"
         << "[";
    for (std::size_t id = 0; id < groups.size(); ++id)
      yaml << std::to_string(id);
    yaml << "]";
  });
  if (fs::exists(fs::symlink_status(output)))
    throw std::invalid_argument("Geometry output appeared during export");
  fs::rename(pending, output);
}

}  // namespace livo_atlas
