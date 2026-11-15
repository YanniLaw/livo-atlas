#ifndef LIVO_ATLAS_SRC_ROS_TRAJECTORY_MESSAGES_H_
#define LIVO_ATLAS_SRC_ROS_TRAJECTORY_MESSAGES_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "livo_atlas/mapping/trajectory_snapshot.h"
#include "ros/time_conversion.h"

namespace livo_atlas::ros_detail {

inline std::int64_t TrajectoryNanoseconds(double time) {
  return RosNanoseconds(time, "Trajectory", "trajectory");
}

// Both generated schemas have the same fields. Only construction of their
// native time types differs; keep conversion and error clearing identical.
template <typename Message, typename Path, typename Stamp>
std::pair<Message, Path> MakeTrajectoryMessages(
    const TrajectorySnapshot& source, const std::string& map_frame,
    Stamp stamp) {
  if (source.producer_id.size() != 32 || !source.revision ||
      !std::all_of(source.producer_id.begin(), source.producer_id.end(),
                   [](char c) {
                     return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
                   }) ||
      (source.status != TrajectoryStatus::kReady &&
       source.status != TrajectoryStatus::kResourceLimited &&
       source.status != TrajectoryStatus::kError) ||
      (source.pose_version == 0
           ? source.parent_frame != PoseFrame::kOdom ||
                 source.optimized_frame_count != 0
           : source.parent_frame != PoseFrame::kMap ||
                 !source.optimized_frame_count ||
                 source.optimized_frame_count > source.frame_count))
    throw std::invalid_argument("Invalid trajectory output identity or header");

  Message message;
  Path path;
  message.header.frame_id =
      source.parent_frame == PoseFrame::kOdom ? "camera_init" : map_frame;
  message.producer_id = source.producer_id;
  message.revision = source.revision;
  message.pose_version = source.pose_version;
  message.frame_count = source.frame_count;
  message.optimized_frame_count = source.optimized_frame_count;
  message.status = static_cast<std::uint8_t>(source.status);
  message.error = source.error.substr(0, 1024);
  try {
    if (source.status == TrajectoryStatus::kReady) {
      if (source.frame_count != source.keyframes.size() ||
          source.frame_count > 100000)
        throw std::invalid_argument(
            "Trajectory payload does not fit its frontier");
      if (!source.keyframes.empty())
        message.header.stamp =
            stamp(TrajectoryNanoseconds(source.keyframes.back().timestamp));
      message.keyframes.resize(source.keyframes.size());
      path.poses.resize(source.keyframes.size());
      for (std::size_t i = 0; i < source.keyframes.size(); ++i) {
        const auto& input = source.keyframes[i];
        const auto& pose = input.parent_from_body;
        if (input.id != i || !pose.rotation.allFinite() ||
            !pose.translation.allFinite() ||
            (pose.rotation.transpose() * pose.rotation -
             Eigen::Matrix3d::Identity())
                    .cwiseAbs()
                    .maxCoeff() > 1e-9 ||
            std::abs(pose.rotation.determinant() - 1) > 1e-9)
          throw std::invalid_argument("Invalid trajectory keyframe ID or pose");
        auto& output = message.keyframes[i];
        output.id = input.id;
        output.timestamp = input.timestamp;
        output.pose.position.x = pose.translation.x();
        output.pose.position.y = pose.translation.y();
        output.pose.position.z = pose.translation.z();
        Eigen::Quaterniond quaternion(pose.rotation);
        quaternion.normalize();
        output.pose.orientation.x = quaternion.x();
        output.pose.orientation.y = quaternion.y();
        output.pose.orientation.z = quaternion.z();
        output.pose.orientation.w = quaternion.w();
        path.poses[i].header.frame_id = message.header.frame_id;
        path.poses[i].header.stamp =
            stamp(TrajectoryNanoseconds(input.timestamp));
        path.poses[i].pose = output.pose;
      }
    }
  } catch (const std::exception& error) {
    message.status = static_cast<std::uint8_t>(TrajectoryStatus::kError);
    message.error = std::string(error.what()).substr(0, 1024);
    message.header.stamp = stamp(0);
    message.keyframes.clear();
    path.poses.clear();
  }
  path.header = message.header;
  return {std::move(message), std::move(path)};
}

}  // namespace livo_atlas::ros_detail

#endif  // LIVO_ATLAS_SRC_ROS_TRAJECTORY_MESSAGES_H_
