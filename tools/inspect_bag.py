#!/usr/bin/env python3
"""Print read-only statistics for the ROS1 bag formats used by LIVO-Atlas."""

import argparse
import json
import math
import re
import sys


SUPPORTED_TYPES = {
    "sensor_msgs/Image",
    "sensor_msgs/Imu",
    "livox_ros_driver/CustomMsg",
}
NSEC_PER_SEC = 1_000_000_000


def time_to_ns(stamp):
    return int(stamp.secs) * NSEC_PER_SEC + int(stamp.nsecs)


def update_range(stats, value):
    if stats["first_ns"] is None:
        stats["first_ns"] = value
    stats["last_ns"] = value
    if stats["min_ns"] is None or value < stats["min_ns"]:
        stats["min_ns"] = value
    if stats["max_ns"] is None or value > stats["max_ns"]:
        stats["max_ns"] = value


def minimum_image_row_bytes(width, encoding):
    """Return the minimum encoded row size when the ROS encoding is known."""
    bytes_per_pixel = {
        "mono8": 1,
        "mono16": 2,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "rgb16": 6,
        "bgr16": 6,
        "rgba16": 8,
        "bgra16": 8,
        "yuv422": 2,
    }.get(encoding)
    if bytes_per_pixel is None and encoding.startswith("bayer_"):
        suffix = encoding.rsplit("_", 1)[-1]
        if suffix.endswith("8"):
            bytes_per_pixel = 1
        elif suffix.endswith("16"):
            bytes_per_pixel = 2
    if bytes_per_pixel is None:
        match = re.fullmatch(r"(8|16|32|64)[USF]C([1-9][0-9]*)", encoding)
        if match:
            bytes_per_pixel = (int(match.group(1)) // 8) * int(match.group(2))
    return None if bytes_per_pixel is None else width * bytes_per_pixel


def new_topic_stats(msg_type):
    return {
        "type": msg_type,
        "message_count": 0,
        "bag_time": {"min_ns": None, "max_ns": None, "first_ns": None, "last_ns": None},
        "header_time": {"min_ns": None, "max_ns": None, "first_ns": None, "last_ns": None},
        "frame_ids": set(),
        "previous_header_ns": None,
        "header_repeated_count": 0,
        "header_rollback_count": 0,
        "positive_interval_count": 0,
        "positive_interval_sum_ns": 0,
        "positive_interval_min_ns": None,
        "positive_interval_max_ns": None,
    }


def update_header(stats, message):
    stamp_ns = time_to_ns(message.header.stamp)
    update_range(stats["header_time"], stamp_ns)
    stats["frame_ids"].add(str(message.header.frame_id))

    previous = stats["previous_header_ns"]
    if previous is not None:
        interval_ns = stamp_ns - previous
        if interval_ns == 0:
            stats["header_repeated_count"] += 1
        elif interval_ns < 0:
            stats["header_rollback_count"] += 1
        else:
            stats["positive_interval_count"] += 1
            stats["positive_interval_sum_ns"] += interval_ns
            current_min = stats["positive_interval_min_ns"]
            current_max = stats["positive_interval_max_ns"]
            stats["positive_interval_min_ns"] = (
                interval_ns if current_min is None else min(current_min, interval_ns)
            )
            stats["positive_interval_max_ns"] = (
                interval_ns if current_max is None else max(current_max, interval_ns)
            )
    stats["previous_header_ns"] = stamp_ns


def update_image(stats, message):
    width = int(message.width)
    height = int(message.height)
    encoding = str(message.encoding)
    step = int(message.step)
    data_length = len(message.data)
    key = (width, height, encoding, step)
    row_bytes = minimum_image_row_bytes(width, encoding)
    layouts = stats.setdefault("image_layouts", {})
    layout = layouts.get(key)
    if layout is None:
        layout = {
            "width": width,
            "height": height,
            "encoding": encoding,
            "step": step,
            "minimum_row_bytes": row_bytes,
            "message_count": 0,
            "data_length_min_bytes": data_length,
            "data_length_max_bytes": data_length,
            "data_length_matches_step_height_count": 0,
            "data_length_mismatch_count": 0,
            "row_step_too_small_count": 0 if row_bytes is not None else None,
            "layout_valid_count": 0 if row_bytes is not None else None,
        }
        layouts[key] = layout

    layout["message_count"] += 1
    layout["data_length_min_bytes"] = min(layout["data_length_min_bytes"], data_length)
    layout["data_length_max_bytes"] = max(layout["data_length_max_bytes"], data_length)
    data_length_valid = data_length == step * height
    if data_length_valid:
        layout["data_length_matches_step_height_count"] += 1
    else:
        layout["data_length_mismatch_count"] += 1

    if row_bytes is not None:
        row_step_valid = step >= row_bytes
        if not row_step_valid:
            layout["row_step_too_small_count"] += 1
        if width > 0 and height > 0 and data_length_valid and row_step_valid:
            layout["layout_valid_count"] += 1


def update_custom_message(stats, message):
    point_num = int(message.point_num)
    points = message.points
    points_length = len(points)
    stats["point_num_min"] = min(stats.get("point_num_min", point_num), point_num)
    stats["point_num_max"] = max(stats.get("point_num_max", point_num), point_num)
    if points_length != point_num:
        stats["point_count_mismatch_count"] = stats.get("point_count_mismatch_count", 0) + 1

    timebase = int(message.timebase)
    if "first_timebase" not in stats:
        stats["first_timebase"] = timebase
        lines = set()
        min_offset = None
        max_offset = None
        for point in points:
            lines.add(int(point.line))
            offset_time = int(point.offset_time)
            min_offset = offset_time if min_offset is None else min(min_offset, offset_time)
            max_offset = offset_time if max_offset is None else max(max_offset, offset_time)
        stats["first_frame_points"] = {
            "line_ids": sorted(lines),
            "offset_time_min": min_offset,
            "offset_time_max": max_offset,
        }
    stats["last_timebase"] = timebase


def finalize_topic(stats):
    count = stats["positive_interval_count"]
    interval_mean = (
        stats["positive_interval_sum_ns"] / count if count else None
    )
    result = {
        "type": stats["type"],
        "message_count": stats["message_count"],
        "bag_time_ns": stats["bag_time"],
        "header_time_ns": stats["header_time"],
        "header_adjacent_non_increasing": {
            "repeated_count": stats["header_repeated_count"],
            "rollback_count": stats["header_rollback_count"],
        },
        "header_positive_interval_ns": {
            "count": count,
            "min_ns": stats["positive_interval_min_ns"],
            "max_ns": stats["positive_interval_max_ns"],
            "mean_ns": interval_mean,
        },
        "frame_ids": sorted(stats["frame_ids"]),
    }

    if stats["type"] == "sensor_msgs/Image":
        result["image_layouts"] = [
            stats["image_layouts"][key]
            for key in sorted(stats.get("image_layouts", {}))
        ]
    elif stats["type"] == "sensor_msgs/Imu":
        result["imu_non_finite_counts"] = stats["imu_non_finite_counts"]
    elif stats["type"] == "livox_ros_driver/CustomMsg":
        result["custom_msg"] = {
            "point_num_min": stats.get("point_num_min"),
            "point_num_max": stats.get("point_num_max"),
            "point_count_mismatch_count": stats.get("point_count_mismatch_count", 0),
            "first_timebase": stats.get("first_timebase"),
            "last_timebase": stats.get("last_timebase"),
            "first_frame_points": stats.get("first_frame_points"),
        }
    return result


def inspect_bag(bag_path):
    import rosbag

    topics = {}
    with rosbag.Bag(bag_path, mode="r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        for topic, info in topic_info.items():
            if info.msg_type not in SUPPORTED_TYPES:
                raise ValueError(
                    "unsupported message type on {}: {}".format(topic, info.msg_type)
                )
            topics[topic] = new_topic_stats(info.msg_type)

        for topic, message, bag_stamp in bag.read_messages():
            stats = topics.get(topic)
            if stats is None:
                raise ValueError("message topic missing from bag metadata: {}".format(topic))
            if message._type != stats["type"]:
                raise ValueError(
                    "message type changed on {}: {}".format(topic, message._type)
                )

            stats["message_count"] += 1
            update_range(stats["bag_time"], time_to_ns(bag_stamp))
            update_header(stats, message)

            if stats["type"] == "sensor_msgs/Image":
                update_image(stats, message)
            elif stats["type"] == "sensor_msgs/Imu":
                if "imu_non_finite_counts" not in stats:
                    stats["imu_non_finite_counts"] = {
                        "linear_acceleration": {axis: 0 for axis in "xyz"},
                        "angular_velocity": {axis: 0 for axis in "xyz"},
                    }
                for field_name in ("linear_acceleration", "angular_velocity"):
                    vector = getattr(message, field_name)
                    for axis in "xyz":
                        if not math.isfinite(float(getattr(vector, axis))):
                            stats["imu_non_finite_counts"][field_name][axis] += 1
            elif stats["type"] == "livox_ros_driver/CustomMsg":
                update_custom_message(stats, message)

    return {
        "input_bag": bag_path,
        "access": {"mode": "read_only", "writes_bag": False, "ros_master_required": False},
        "topics": {topic: finalize_topic(stats) for topic, stats in sorted(topics.items())},
    }


def main():
    parser = argparse.ArgumentParser(
        description="Inspect the ROS1 Image, IMU, and Livox CustomMsg formats used by LIVO-Atlas.",
        epilog="Reads the bag only, prints JSON to stdout, and does not require a ROS master.",
    )
    parser.add_argument("bag_path", help="path to a ROS1 bag file")
    args = parser.parse_args()
    try:
        report = inspect_bag(args.bag_path)
        json.dump(report, sys.stdout, indent=2, sort_keys=True, ensure_ascii=False)
        sys.stdout.write("\n")
    except Exception as error:
        print("inspect_bag.py: error: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
