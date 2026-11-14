"""Copy trajectory messages to ordinary JSON values for paired ROS validation."""


def _pose_record(pose):
    return dict(translation=[pose.position.x, pose.position.y, pose.position.z],
                rotation_xyzw=[pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w])


def trajectory_record(message, to_ns):
    return dict(stamp_ns=to_ns(message.header.stamp), frame=message.header.frame_id,
                producer_id=message.producer_id, revision=message.revision,
                pose_version=message.pose_version, frame_count=message.frame_count,
                optimized_frame_count=message.optimized_frame_count,
                status=message.status, error=message.error,
                keyframes=[dict(id=row.id, timestamp=row.timestamp,
                                **_pose_record(row.pose)) for row in message.keyframes])


def trajectory_path_record(message, to_ns):
    return dict(stamp_ns=to_ns(message.header.stamp), frame=message.header.frame_id,
                poses=[dict(stamp_ns=to_ns(row.header.stamp), frame=row.header.frame_id,
                            **_pose_record(row.pose)) for row in message.poses])
