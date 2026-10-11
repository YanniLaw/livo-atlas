#!/usr/bin/env python3
"""Check AVIA transform, camera, and short-window timestamp contracts."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml


NSEC_PER_SEC = 1_000_000_000
POSITION_TOLERANCE_M = 1e-4
IMU_WINDOW_NS = 300_000_000


def stamp_ns(stamp):
    return int(stamp.secs) * NSEC_PER_SEC + int(stamp.nsecs)


def split_ns(value):
    seconds, nanoseconds = divmod(int(value), NSEC_PER_SEC)
    return {"ns": int(value), "secs": seconds, "nsecs": nanoseconds}


def seconds_ns(value):
    return int(round(float(value) * NSEC_PER_SEC))


def stamp_record(stamp):
    result = split_ns(stamp_ns(stamp))
    return result


def matrix_metrics(matrix):
    determinant = float(np.linalg.det(matrix))
    orthogonality = float(np.linalg.norm(matrix.T @ matrix - np.eye(3), ord="fro"))
    return {
        "determinant": determinant,
        "determinant_error_from_one": abs(determinant - 1.0),
        "orthogonality_error_frobenius": orthogonality,
    }


def source_evidence():
    return {
        "lidar_to_world": "src/livo-atlas/src/LIVMapper.cpp:637-652",
        "imu_lidar_extrinsic_inverse": "src/livo-atlas/src/vio.cpp:29-33",
        "imu_to_camera_composition": "src/livo-atlas/src/vio.cpp:35-58",
        "world_to_camera_state": "src/livo-atlas/src/vio.cpp:1690-1696",
        "image_offset": "src/livo-atlas/src/LIVMapper.cpp:846-860",
        "imu_offset": "src/livo-atlas/src/LIVMapper.cpp:770-785",
        "livox_header_and_split": "src/livo-atlas/src/LIVMapper.cpp:726-745,953-1024",
        "livox_point_offset_and_correction": "src/livo-atlas/src/preprocess.cpp:164-186",
        "pinhole_scale": "src/rpg_vikit/vikit_common/src/pinhole_camera.cpp:19-38",
    }


def check_transform(config):
    calibration = config["extrin_calib"]
    ext_r = np.asarray(calibration["extrinsic_R"], dtype=float).reshape(3, 3)
    ext_t = np.asarray(calibration["extrinsic_T"], dtype=float).reshape(3)
    r_cl = np.asarray(calibration["Rcl"], dtype=float).reshape(3, 3)
    p_cl = np.asarray(calibration["Pcl"], dtype=float).reshape(3)

    yaw = math.radians(30.0)
    r_wi = np.asarray(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    p_wi = np.asarray([1.0, 2.0, 3.0], dtype=float)

    # Keep the configured matrices intact; transpose is the source's inverse assumption.
    r_li = ext_r.T
    p_li = -r_li @ ext_t
    r_ci = r_cl @ r_li
    p_ci = r_cl @ p_li + p_cl
    r_cw = r_ci @ r_wi.T
    p_cw = -r_ci @ r_wi.T @ p_wi + p_ci

    lidar_points = [
        np.asarray([0.5, -1.25, 2.0]),
        np.asarray([-1.1, 0.4, 3.2]),
        np.asarray([2.25, 1.75, -0.6]),
    ]
    comparisons = []
    world_vs_imu_max = 0.0
    world_vs_direct_max = 0.0
    lidar_camera_roundtrip_max = 0.0
    for point in lidar_points:
        p_i = ext_r @ point + ext_t
        p_w = r_wi @ p_i + p_wi
        p_camera_world = r_cw @ p_w + p_cw
        p_camera_imu = r_ci @ p_i + p_ci
        p_camera_direct = r_cl @ point + p_cl
        p_lidar_recovered = r_cl.T @ (p_camera_direct - p_cl)
        world_vs_imu = float(np.linalg.norm(p_camera_world - p_camera_imu))
        world_vs_direct = float(np.linalg.norm(p_camera_world - p_camera_direct))
        lidar_camera_roundtrip = float(np.linalg.norm(p_lidar_recovered - point))
        world_vs_imu_max = max(world_vs_imu_max, world_vs_imu)
        world_vs_direct_max = max(world_vs_direct_max, world_vs_direct)
        lidar_camera_roundtrip_max = max(
            lidar_camera_roundtrip_max, lidar_camera_roundtrip
        )
        comparisons.append(
            {
                "lidar_point": point.tolist(),
                "camera_via_world": p_camera_world.tolist(),
                "camera_via_imu": p_camera_imu.tolist(),
                "camera_direct_from_lidar": p_camera_direct.tolist(),
                "lidar_recovered_from_camera": p_lidar_recovered.tolist(),
                "world_vs_imu_error_m": world_vs_imu,
                "world_vs_direct_error_m": world_vs_direct,
                "lidar_camera_lidar_roundtrip_error_m": lidar_camera_roundtrip,
            }
        )

    rotations = {
        "configured_extrinsic_R": ext_r,
        "configured_Rcl": r_cl,
        "synthetic_Rwi_yaw_30deg": r_wi,
        "derived_Rli": r_li,
        "derived_Rci": r_ci,
        "derived_Rcw": r_cw,
    }
    return {
        "formulas": {
            "lidar_to_imu": "p_i = extR * p_l + extT",
            "lidar_to_world": "p_w = Rwi * (extR * p_l + extT) + Pwi",
            "imu_to_lidar_inverse_assumed": "Rli = extR.T; Pli = -extR.T * extT",
            "imu_to_camera": "Rci = Rcl * extR.T; Pci = -Rcl * extR.T * extT + Pcl",
            "world_to_camera": "Rcw = Rci * Rwi.T; Pcw = -Rci * Rwi.T * Pwi + Pci",
            "direct_lidar_to_camera": "p_c = Rcl * p_l + Pcl",
            "camera_to_lidar_inverse_assumed": "Rlc = Rcl.T; Plc = -Rcl.T * Pcl; p_l_recovered = Rcl.T * (p_c - Pcl)",
        },
        "synthetic_state": {"Rwi": r_wi.tolist(), "Pwi_m": p_wi.tolist()},
        "configured_extrinsics": {
            "extrinsic_R_used_as_loaded": ext_r.tolist(),
            "extrinsic_T_m": ext_t.tolist(),
            "Rcl_used_as_loaded": r_cl.tolist(),
            "Pcl_m": p_cl.tolist(),
            "orthogonalization_applied": False,
        },
        "rotation_quality": {
            name: matrix_metrics(matrix) for name, matrix in rotations.items()
        },
        "path_comparison": {
            "point_count": len(lidar_points),
            "max_world_vs_imu_error_m": world_vs_imu_max,
            "max_world_vs_direct_error_m": world_vs_direct_max,
            "max_path_error_m": max(world_vs_imu_max, world_vs_direct_max),
            "tolerance_m": POSITION_TOLERANCE_M,
            "points": comparisons,
        },
        "lidar_camera_lidar_roundtrip": {
            "max_error_m": lidar_camera_roundtrip_max,
            "tolerance_m": POSITION_TOLERANCE_M,
            "rotation_inverse_assumption": "Rcl.T",
            "translation_inverse": "-Rcl.T * Pcl",
        },
    }


def check_camera(camera_config):
    scale = float(camera_config["scale"])
    raw_width = int(camera_config["cam_width"])
    raw_height = int(camera_config["cam_height"])
    effective_width = int(raw_width * scale)
    effective_height = int(raw_height * scale)
    intrinsics = {
        name: float(camera_config[key]) * scale
        for name, key in (
            ("fx", "cam_fx"),
            ("fy", "cam_fy"),
            ("cx", "cam_cx"),
            ("cy", "cam_cy"),
        )
    }
    return {
        "model": str(camera_config["cam_model"]),
        "raw_size_px": {"width": raw_width, "height": raw_height},
        "scale": scale,
        "effective_size_px": {"width": effective_width, "height": effective_height},
        "effective_intrinsics_px": intrinsics,
        "principal_axis_projection": {
            "camera_point_m": [0.0, 0.0, 5.0],
            "pixel_uv": [intrinsics["cx"], intrinsics["cy"]],
        },
        "accuracy_claim": "Configuration scale and projection logic only; this does not establish calibration accuracy.",
    }


def image_summary(message, bag_stamp, index, image_offset_ns, exposure_ns):
    header = stamp_ns(message.header.stamp)
    corrected = header + image_offset_ns
    capture = corrected + exposure_ns
    return {
        "index": index,
        "header": {
            **stamp_record(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "seq": int(message.header.seq),
        },
        "bag_record_time": stamp_record(bag_stamp),
        "layout": {
            "width": int(message.width),
            "height": int(message.height),
            "encoding": str(message.encoding),
            "step": int(message.step),
            "data_bytes": len(message.data),
        },
        "header_plus_img_offset_ns": split_ns(corrected),
        "capture_time_after_exposure_ns": split_ns(capture),
    }


def lidar_summary(message, bag_stamp, index):
    header = stamp_ns(message.header.stamp)
    return {
        "index": index,
        "header": {
            **stamp_record(message.header.stamp),
            "frame_id": str(message.header.frame_id),
            "seq": int(message.header.seq),
        },
        "bag_record_time": stamp_record(bag_stamp),
        "timebase_ns": int(message.timebase),
        "timebase_minus_header_ns": int(message.timebase) - header,
        "point_num_field": int(message.point_num),
        "points_array_length": len(message.points),
    }


def read_short_bag(bag_path, topics, image_offset_ns, exposure_ns, imu_offset_ns):
    import rosbag

    image_topic = topics["img_topic"]
    lidar_topic = topics["lid_topic"]
    imu_topic = topics["imu_topic"]
    images = []
    lidars = []

    with rosbag.Bag(str(bag_path), mode="r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        expected = {
            image_topic: "sensor_msgs/Image",
            lidar_topic: "livox_ros_driver/CustomMsg",
            imu_topic: "sensor_msgs/Imu",
        }
        for topic, msg_type in expected.items():
            if topic not in topic_info:
                raise ValueError("configured topic missing from bag: {}".format(topic))
            if topic_info[topic].msg_type != msg_type:
                raise ValueError(
                    "unexpected type on {}: {}".format(topic, topic_info[topic].msg_type)
                )

        for topic, message, bag_stamp in bag.read_messages(
            topics=[image_topic, lidar_topic]
        ):
            if topic == image_topic and len(images) < 3:
                images.append((message, bag_stamp))
            elif topic == lidar_topic and len(lidars) < 3:
                lidars.append((message, bag_stamp))
            if len(images) == 3 and len(lidars) == 3:
                break

        if len(images) < 3 or len(lidars) < 3:
            raise ValueError("bag has fewer than three configured images or lidar messages")

        first_image_header_ns = stamp_ns(images[0][0].header.stamp)
        imu_limit_ns = first_image_header_ns + IMU_WINDOW_NS
        imu_messages = []
        imu_monotonic = True
        previous_imu_ns = None
        imu_stop_reason = "bag_end_before_window"
        for _, message, bag_stamp in bag.read_messages(topics=[imu_topic]):
            header_ns = stamp_ns(message.header.stamp)
            if previous_imu_ns is not None and header_ns < previous_imu_ns:
                imu_monotonic = False
            previous_imu_ns = header_ns
            if header_ns > imu_limit_ns:
                imu_stop_reason = "first_header_after_window"
                break
            imu_messages.append((message, bag_stamp))

    first_lidar = lidars[0][0]
    offsets_ns = [int(point.offset_time) for point in first_lidar.points]
    if not offsets_ns:
        raise ValueError("first CustomMsg has no points")

    first_image_header_ns = stamp_ns(images[0][0].header.stamp)
    corrected_first_image_ns = (
        first_image_header_ns + image_offset_ns + exposure_ns
    )
    first_lidar_header_ns = stamp_ns(first_lidar.header.stamp)
    threshold_offset_ns = corrected_first_image_ns - first_lidar_header_ns
    before_count = sum(offset < threshold_offset_ns for offset in offsets_ns)
    at_or_after_count = len(offsets_ns) - before_count
    before_neighbors = [
        (index, offset)
        for index, offset in enumerate(offsets_ns)
        if offset < threshold_offset_ns
    ]
    after_neighbors = [
        (index, offset)
        for index, offset in enumerate(offsets_ns)
        if offset >= threshold_offset_ns
    ]
    threshold_neighbors = {
        "nearest_before": None,
        "nearest_at_or_after": None,
    }
    if before_neighbors:
        index, offset = max(before_neighbors, key=lambda item: item[1])
        threshold_neighbors["nearest_before"] = {
            "array_index": index,
            "offset_time_ns": offset,
        }
    if after_neighbors:
        index, offset = min(after_neighbors, key=lambda item: item[1])
        threshold_neighbors["nearest_at_or_after"] = {
            "array_index": index,
            "offset_time_ns": offset,
        }
    edge_indexes = sorted(
        set(
            list(range(min(5, len(offsets_ns))))
            + list(range(max(0, len(offsets_ns) - 5), len(offsets_ns)))
        )
    )

    imu_rows = []
    for message, bag_stamp in imu_messages:
        raw_ns = stamp_ns(message.header.stamp)
        imu_rows.append(
            {
                "header": {
                    **stamp_record(message.header.stamp),
                    "frame_id": str(message.header.frame_id),
                    "seq": int(message.header.seq),
                },
                "bag_record_time": stamp_record(bag_stamp),
                "corrected_header_ns": split_ns(raw_ns - imu_offset_ns),
            }
        )
    imu_rows.sort(key=lambda row: row["corrected_header_ns"]["ns"])

    before = [row for row in imu_rows if row["corrected_header_ns"]["ns"] <= corrected_first_image_ns]
    after = [row for row in imu_rows if row["corrected_header_ns"]["ns"] >= corrected_first_image_ns]
    nearest_before = before[-1] if before else None
    nearest_after = after[0] if after else None

    first_offset = min(offsets_ns)
    last_offset = max(offsets_ns)
    first_point_by_header = first_lidar_header_ns + first_offset
    last_point_by_header = first_lidar_header_ns + last_offset
    first_timebase_ns = int(first_lidar.timebase)
    first_point_by_timebase = first_timebase_ns + first_offset
    last_point_by_timebase = first_timebase_ns + last_offset

    return {
        "topics": topics,
        "first_three_images": [
            image_summary(message, bag_stamp, index, image_offset_ns, exposure_ns)
            for index, (message, bag_stamp) in enumerate(images, start=1)
        ],
        "first_three_custom_messages": [
            lidar_summary(message, bag_stamp, index)
            for index, (message, bag_stamp) in enumerate(lidars, start=1)
        ],
        "first_scan_raw_point_offsets": {
            "unit": "nanoseconds",
            "point_count": len(offsets_ns),
            "minimum_offset_ns": first_offset,
            "maximum_offset_ns": last_offset,
            "first_array_offset_ns": offsets_ns[0],
            "last_array_offset_ns": offsets_ns[-1],
            "first_and_last_five_by_array_index": [
                {"array_index": index, "offset_time_ns": offsets_ns[index]}
                for index in edge_indexes
            ],
            "nearest_raw_offsets_to_split_threshold": threshold_neighbors,
            "estimated_point_range_using_lidar_header": {
                "start_ns": split_ns(first_point_by_header),
                "end_ns": split_ns(last_point_by_header),
                "basis": "LIVMapper uses the CustomMsg header as frame begin; preprocess converts offset_time to curvature.",
            },
            "timebase_plus_offset_range_informational_only": {
                "start_ns": split_ns(first_point_by_timebase),
                "end_ns": split_ns(last_point_by_timebase),
                "basis": "Raw CustomMsg timebase plus raw offset_time; current LIVMapper CustomMsg path does not use timebase.",
            },
        },
        "first_split": {
            "image_header_ns": split_ns(first_image_header_ns),
            "img_time_offset_ns_applied_once": int(image_offset_ns),
            "exposure_time_init_ns_applied_once": int(exposure_ns),
            "capture_time_ns": split_ns(corrected_first_image_ns),
            "lidar_header_ns": split_ns(first_lidar_header_ns),
            "point_time_threshold_relative_to_lidar_header_ns": threshold_offset_ns,
            "raw_point_count_before_threshold": before_count,
            "raw_point_count_at_or_after_threshold": at_or_after_count,
            "raw_point_count_total": len(offsets_ns),
            "partition_rule": "raw offset_time_ns < threshold is before; equality and later are at/after",
            "raw_offsets_are_not_a_full_preprocess_replay": True,
        },
        "imu": {
            "header_window_start_ns": split_ns(first_image_header_ns),
            "header_window_end_ns": split_ns(first_image_header_ns + IMU_WINDOW_NS),
            "offset_ns_subtracted_once": int(imu_offset_ns),
            "stored_sample_count": len(imu_rows),
            "stop_reason": imu_stop_reason,
            "header_monotonic_through_window": imu_monotonic,
            "first_sample": imu_rows[0] if imu_rows else None,
            "last_sample": imu_rows[-1] if imu_rows else None,
            "all_samples_apply_offset_once": all(
                row["corrected_header_ns"]["ns"]
                == row["header"]["ns"] - imu_offset_ns
                for row in imu_rows
            ),
            "nearest_before_first_split": nearest_before,
            "nearest_at_or_after_first_split": nearest_after,
            "split_bracketed_by_stored_imu": nearest_before is not None and nearest_after is not None,
        },
    }


def assertion(name, passed, observed, criterion):
    return {
        "name": name,
        "passed": bool(passed),
        "observed": observed,
        "criterion": criterion,
    }


def build_report(bag_path):
    project_root = Path(__file__).resolve().parents[1]
    avia_path = project_root / "config" / "avia.yaml"
    camera_path = project_root / "config" / "camera_pinhole.yaml"
    with avia_path.open("r", encoding="utf-8") as stream:
        avia = yaml.safe_load(stream)
    with camera_path.open("r", encoding="utf-8") as stream:
        camera_config = yaml.safe_load(stream)

    time_config = avia["time_offset"]
    image_offset_ns = seconds_ns(time_config["img_time_offset"])
    exposure_ns = seconds_ns(time_config["exposure_time_init"])
    imu_offset_ns = seconds_ns(time_config["imu_time_offset"])

    transforms = check_transform(avia)
    camera = check_camera(camera_config)
    bag = read_short_bag(
        bag_path,
        avia["common"],
        image_offset_ns,
        exposure_ns,
        imu_offset_ns,
    )
    expected_size = camera["effective_size_px"]
    measured_sizes_match = all(
        row["layout"]["width"] == expected_size["width"]
        and row["layout"]["height"] == expected_size["height"]
        for row in bag["first_three_images"]
    )
    corrected_capture = bag["first_three_images"][0]["capture_time_after_exposure_ns"]["ns"]
    formula_capture = (
        bag["first_three_images"][0]["header"]["ns"]
        + image_offset_ns
        + exposure_ns
    )
    imu_corrected_once = bag["imu"]["all_samples_apply_offset_once"]
    assertions = [
        assertion(
            "transform_paths_within_tolerance",
            transforms["path_comparison"]["max_path_error_m"] <= POSITION_TOLERANCE_M,
            transforms["path_comparison"]["max_path_error_m"],
            "max LiDAR→world→camera versus LiDAR→camera path error <= 1e-4 m",
        ),
        assertion(
            "lidar_camera_lidar_roundtrip_within_tolerance",
            transforms["lidar_camera_lidar_roundtrip"]["max_error_m"]
            <= POSITION_TOLERANCE_M,
            transforms["lidar_camera_lidar_roundtrip"]["max_error_m"],
            "max LiDAR→camera→LiDAR error using Rcl.T and inverse translation <= 1e-4 m",
        ),
        assertion(
            "camera_scale_and_bag_dimensions_match",
            measured_sizes_match,
            {
                "effective_camera_size_px": expected_size,
                "first_three_bag_sizes_px": [row["layout"] for row in bag["first_three_images"]],
            },
            "camera size after scale equals each of the first three image dimensions",
        ),
        assertion(
            "first_capture_time_formula",
            corrected_capture == formula_capture,
            {"reported_ns": corrected_capture, "header_plus_offsets_ns": formula_capture},
            "capture = first image header + img_time_offset + exposure_time_init, once each",
        ),
        assertion(
            "imu_offset_applied_once",
            imu_corrected_once,
            {"sample_count": bag["imu"]["stored_sample_count"], "offset_ns": imu_offset_ns},
            "each stored IMU corrected header = raw header - imu_time_offset",
        ),
        assertion(
            "raw_first_scan_partition_complete",
            bag["first_split"]["raw_point_count_before_threshold"]
            + bag["first_split"]["raw_point_count_at_or_after_threshold"]
            == bag["first_split"]["raw_point_count_total"],
            {
                "before": bag["first_split"]["raw_point_count_before_threshold"],
                "at_or_after": bag["first_split"]["raw_point_count_at_or_after_threshold"],
                "total": bag["first_split"]["raw_point_count_total"],
            },
            "raw first-scan point counts on both sides sum to the complete point array length",
        ),
        assertion(
            "first_custom_message_point_count_matches_array",
            bag["first_three_custom_messages"][0]["point_num_field"]
            == bag["first_three_custom_messages"][0]["points_array_length"],
            {
                "point_num": bag["first_three_custom_messages"][0]["point_num_field"],
                "array_length": bag["first_three_custom_messages"][0]["points_array_length"],
            },
            "first CustomMsg point_num equals its serialized points array length",
        ),
        assertion(
            "stored_imu_brackets_first_split",
            bag["imu"]["split_bracketed_by_stored_imu"],
            {
                "before_ns": None
                if bag["imu"]["nearest_before_first_split"] is None
                else bag["imu"]["nearest_before_first_split"]["corrected_header_ns"]["ns"],
                "after_ns": None
                if bag["imu"]["nearest_at_or_after_first_split"] is None
                else bag["imu"]["nearest_at_or_after_first_split"]["corrected_header_ns"]["ns"],
            },
            "at least one corrected IMU header is at/before and one is at/after the first split",
        ),
        assertion(
            "imu_headers_monotonic_in_read_window",
            bag["imu"]["header_monotonic_through_window"],
            bag["imu"]["header_monotonic_through_window"],
            "IMU headers do not roll back in the inspected header-time window",
        ),
    ]
    all_passed = all(item["passed"] for item in assertions)
    return {
        "schema_version": 1,
        "input": {
            "bag_path": str(bag_path.resolve()),
            "avia_config": str(avia_path),
            "camera_config": str(camera_path),
            "bag_messages_read": "first three Image and CustomMsg per configured topic; IMU header window through first image + 0.3 s",
        },
        "scope": {
            "included": [
                "Synthetic transform consistency and LiDAR→camera→LiDAR roundtrip using configured AVIA extrinsics and fixed Rwi yaw=30 degrees, Pwi=[1,2,3].",
                "Scale-derived effective pinhole size and principal-axis projection.",
                "Exact raw header and bag-record timestamps, CustomMsg timebase, first scan offsets, configured image/IMU corrections, and first split point counts.",
            ],
            "excluded": [
                "No claim about real calibration accuracy or physical coordinate correctness.",
                "No claim that image, LiDAR, and IMU are physically synchronized; nearest IMU times only describe observed coverage.",
                "Raw point-offset threshold counts do not replay preprocess.cpp anomaly correction, filtering, or the full LIO/VIO state machine.",
                "CustomMsg timebase-plus-offset is informational; the inspected source path uses header plus converted offsets and does not consume timebase.",
            ],
            "assertions": assertions,
            "overall_passed": all_passed,
            "exit_code_rule": "0 if every listed formula, corrected-time, image-size, point-partition, and local IMU-coverage assertion passes; 2 if any assertion fails; 1 for an operational/input error.",
        },
        "source_evidence": source_evidence(),
        "transform_check": transforms,
        "camera_check": camera,
        "time_check": {
            "configured_offsets_seconds": {
                "img_time_offset": float(time_config["img_time_offset"]),
                "exposure_time_init": float(time_config["exposure_time_init"]),
                "imu_time_offset": float(time_config["imu_time_offset"]),
            },
            "correction_contract": {
                "image": "header + img_time_offset, then + exposure_time_init at split; each appears once",
                "imu": "header - imu_time_offset once",
                "ros_driver_bug_fix_enabled": bool(avia["common"].get("ros_driver_bug_fix", False)),
                "livox_custommsg": "header is used as scan begin; this callback does not add lidar_time_offset or replace header with timebase",
            },
            **bag,
        },
    }


def main():
    workspace_root = Path(__file__).resolve().parents[3]
    default_bag = workspace_root / "Retail_Street.bag"
    default_output = workspace_root / "validation" / "P0" / "contracts.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        type=Path,
        default=default_bag,
        help="ROS1 bag path (default: workspace Retail_Street.bag)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output,
        help="JSON report path (default: workspace validation/P0/contracts.json)",
    )
    args = parser.parse_args()

    try:
        report = build_report(args.bag)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    except Exception as error:
        print("check_contracts: {}".format(error), file=sys.stderr)
        return 1

    print("report: {}".format(args.output.resolve()))
    print("overall_passed: {}".format(report["scope"]["overall_passed"]))
    return 0 if report["scope"]["overall_passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
