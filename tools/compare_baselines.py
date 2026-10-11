#!/usr/bin/env python3
"""Compare two TUM-format baseline trajectories without pose alignment."""

import argparse
import bisect
import json
import math
import re
import sys
from pathlib import Path


POSITION_LIMIT_M = 0.02
ROTATION_LIMIT_DEG = 0.1
ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*m")
TIMING_TITLE = re.compile(r"\|\s*(LIO Mapping Time|VIO Time)\s*\|")
CURRENT_TOTAL = re.compile(
    r"\|\s*Current Total Time\s*\|\s*"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")


def load_trajectory(path):
    poses = {}
    invalid_lines = []
    invalid_quaternions = 0
    duplicate_timestamps = 0
    out_of_order_pairs = 0
    previous_stamp = None
    parsed_rows = 0

    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            try:
                if len(fields) != 8:
                    raise ValueError("expected 8 columns: t x y z qx qy qz qw")
                values = [float(field) for field in fields]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("non-finite value")
                stamp = values[0]
                quaternion = values[4:8]
                norm = math.sqrt(sum(component * component for component in quaternion))
                if norm <= 1e-12:
                    invalid_quaternions += 1
                    raise ValueError("zero quaternion")
                normalized = tuple(component / norm for component in quaternion)
                parsed_rows += 1
                if previous_stamp is not None:
                    if stamp < previous_stamp:
                        out_of_order_pairs += 1
                previous_stamp = stamp
                if stamp in poses:
                    duplicate_timestamps += 1
                else:
                    poses[stamp] = (tuple(values[1:4]), normalized)
            except ValueError as error:
                invalid_lines.append({"line": line_number, "reason": str(error)})

    return {
        "path": str(path),
        "poses": poses,
        "parsed_rows": parsed_rows,
        "invalid_lines": invalid_lines,
        "invalid_quaternions": invalid_quaternions,
        "duplicate_timestamps": duplicate_timestamps,
        "out_of_order_pairs": out_of_order_pairs,
    }


def nearest_timestamp_deltas(stamps_a, stamps_b):
    if not stamps_a or not stamps_b:
        return None
    sorted_a = sorted(stamps_a)
    sorted_b = sorted(stamps_b)

    def nearest_distance(stamp, candidates):
        index = bisect.bisect_left(candidates, stamp)
        choices = []
        if index < len(candidates):
            choices.append(abs(candidates[index] - stamp))
        if index:
            choices.append(abs(candidates[index - 1] - stamp))
        return min(choices)

    deltas = [nearest_distance(stamp, sorted_b) for stamp in sorted_a]
    deltas.extend(nearest_distance(stamp, sorted_a) for stamp in sorted_b)
    return max(deltas) if deltas else None


def quaternion_angle_degrees(quaternion_a, quaternion_b):
    dot = abs(sum(a * b for a, b in zip(quaternion_a, quaternion_b)))
    dot = min(1.0, max(0.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def rms(values):
    if not values:
        return None
    return math.sqrt(sum(value * value for value in values) / len(values))


def summarize_trajectory(trajectory):
    poses = trajectory["poses"]
    valid = (
        trajectory["parsed_rows"] > 0
        and not trajectory["invalid_lines"]
        and trajectory["duplicate_timestamps"] == 0
        and trajectory["out_of_order_pairs"] == 0
    )
    return {
        "path": trajectory["path"],
        "parsed_rows": trajectory["parsed_rows"],
        "unique_timestamps": len(poses),
        "invalid_line_count": len(trajectory["invalid_lines"]),
        "invalid_lines": trajectory["invalid_lines"][:20],
        "invalid_quaternion_count": trajectory["invalid_quaternions"],
        "valid_quaternion_count": trajectory["parsed_rows"],
        "duplicate_timestamps": trajectory["duplicate_timestamps"],
        "out_of_order_pairs": trajectory["out_of_order_pairs"],
        "timestamp_sequence_valid": (
            trajectory["duplicate_timestamps"] == 0
            and trajectory["out_of_order_pairs"] == 0
        ),
        "valid": valid,
    }


def load_run_summary(directory):
    summary_path = directory / "summary.json"
    if not summary_path.is_file():
        return None, "summary.json is missing"
    try:
        return json.loads(summary_path.read_text(encoding="utf-8")), None
    except (OSError, ValueError) as error:
        return None, "could not read summary.json: {}".format(error)


def resource_metrics(summary_a, summary_b):
    names = {
        "peak_rss_bytes": "resource_peak_rss_bytes",
        "peak_vmhwm_bytes": "resource_peak_vmhwm_bytes",
        "playback_wall_seconds": "playback_wall_seconds",
    }
    result = {}
    for output_key, summary_key in names.items():
        value_a = None if summary_a is None else summary_a.get(summary_key)
        value_b = None if summary_b is None else summary_b.get(summary_key)
        ratio = None
        if isinstance(value_a, (int, float)) and isinstance(value_b, (int, float)) and value_b != 0:
            ratio = value_a / value_b
        result[output_key] = {"a": value_a, "b": value_b, "ratio_a_over_b": ratio}
    return result


def parse_mapper_timing(path):
    values = {"lio_mapping": [], "vio": []}
    section = None
    read_error = None
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                clean = ANSI_ESCAPE.sub("", line)
                title = TIMING_TITLE.search(clean)
                if title:
                    section = "lio_mapping" if title.group(1) == "LIO Mapping Time" else "vio"
                    continue
                if section is None:
                    continue
                current_total = CURRENT_TOTAL.search(clean)
                if current_total:
                    value = float(current_total.group(1))
                    if math.isfinite(value):
                        values[section].append(value)
    except OSError as error:
        read_error = str(error)

    result = {}
    for section_name, samples in values.items():
        ordered = sorted(samples)
        p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)] if ordered else None
        result[section_name] = {
            "count": len(samples),
            "mean_s": sum(samples) / len(samples) if samples else None,
            "p95_s": p95,
            "max_s": max(samples) if samples else None,
            "p95_method": "nearest rank (ceil(0.95*n))",
        }
    result["log_path"] = str(path)
    result["read_error"] = read_error
    return result


def timing_comparison(directory_a, directory_b):
    timing_a = parse_mapper_timing(directory_a / "mapper.log")
    timing_b = parse_mapper_timing(directory_b / "mapper.log")
    result = {}
    for section in ("lio_mapping", "vio"):
        metrics = {}
        for key in ("count", "mean_s", "p95_s", "max_s"):
            value_a = timing_a[section][key]
            value_b = timing_b[section][key]
            ratio = None
            if value_a is not None and value_b not in (None, 0):
                ratio = value_a / value_b
            metrics[key] = {"a": value_a, "b": value_b, "ratio_a_over_b": ratio}
        result[section] = metrics
    result["logs"] = {
        "a": {"path": timing_a["log_path"], "read_error": timing_a["read_error"]},
        "b": {"path": timing_b["log_path"], "read_error": timing_b["read_error"]},
    }
    return result


def compare(directory_a, directory_b):
    trajectory_a = load_trajectory(directory_a / "trajectory.txt")
    trajectory_b = load_trajectory(directory_b / "trajectory.txt")
    poses_a = trajectory_a["poses"]
    poses_b = trajectory_b["poses"]
    stamps_a = set(poses_a)
    stamps_b = set(poses_b)
    common_stamps = sorted(stamps_a & stamps_b)

    position_differences = []
    rotation_differences = []
    for stamp in common_stamps:
        position_a, quaternion_a = poses_a[stamp]
        position_b, quaternion_b = poses_b[stamp]
        position_differences.append(math.sqrt(sum(
            (a - b) * (a - b) for a, b in zip(position_a, position_b))))
        rotation_differences.append(quaternion_angle_degrees(quaternion_a, quaternion_b))

    max_position = max(position_differences) if position_differences else None
    max_rotation = max(rotation_differences) if rotation_differences else None
    run_a, summary_error_a = load_run_summary(directory_a)
    run_b, summary_error_b = load_run_summary(directory_b)
    set_equal = stamps_a == stamps_b
    trajectory_a_summary = summarize_trajectory(trajectory_a)
    trajectory_b_summary = summarize_trajectory(trajectory_b)
    mapper_codes_zero = (
        run_a is not None and run_b is not None
        and run_a.get("exit_codes", {}).get("mapper") == 0
        and run_b.get("exit_codes", {}).get("mapper") == 0
    )
    collector_codes_zero = (
        run_a is not None and run_b is not None
        and run_a.get("exit_codes", {}).get("collector") == 0
        and run_b.get("exit_codes", {}).get("collector") == 0
    )

    checks = {
        "time_sets_equal": set_equal,
        "both_trajectories_valid": trajectory_a_summary["valid"] and trajectory_b_summary["valid"],
        "max_position_difference_within_0_02m": (
            max_position is not None and max_position <= POSITION_LIMIT_M),
        "max_rotation_difference_within_0_1deg": (
            max_rotation is not None and max_rotation <= ROTATION_LIMIT_DEG),
        "runs_completed": (
            run_a is not None and run_b is not None
            and run_a.get("run_status") == "complete"
            and run_b.get("run_status") == "complete"
        ),
        "mapper_exit_codes_zero": mapper_codes_zero,
        "collector_exit_codes_zero": collector_codes_zero,
    }
    report = {
        "baseline_a": str(directory_a),
        "baseline_b": str(directory_b),
        "comparison_method": "Exact numeric timestamp intersection; no SE(3) alignment.",
        "trajectory_a": trajectory_a_summary,
        "trajectory_b": trajectory_b_summary,
        "timestamps": {
            "common_count": len(common_stamps),
            "only_in_a_count": len(stamps_a - stamps_b),
            "only_in_b_count": len(stamps_b - stamps_a),
            "time_sets_equal": set_equal,
            "max_nearest_timestamp_delta_s": nearest_timestamp_deltas(stamps_a, stamps_b),
        },
        "pose_differences": {
            "position_max_m": max_position,
            "position_rms_m": rms(position_differences),
            "rotation_max_deg": max_rotation,
            "rotation_rms_deg": rms(rotation_differences),
            "rotation_metric": "2*acos(abs(dot(q_a_normalized, q_b_normalized))) in degrees",
            "common_pose_count": len(common_stamps),
        },
        "resources": resource_metrics(run_a, run_b),
        "algorithm_timing": timing_comparison(directory_a, directory_b),
        "run_summaries": {
            "a_status": None if run_a is None else run_a.get("run_status"),
            "b_status": None if run_b is None else run_b.get("run_status"),
            "a_mapper_exit_code": None if run_a is None else run_a.get("exit_codes", {}).get("mapper"),
            "b_mapper_exit_code": None if run_b is None else run_b.get("exit_codes", {}).get("mapper"),
            "a_collector_exit_code": None if run_a is None else run_a.get("exit_codes", {}).get("collector"),
            "b_collector_exit_code": None if run_b is None else run_b.get("exit_codes", {}).get("collector"),
            "a_read_error": summary_error_a,
            "b_read_error": summary_error_b,
        },
        "acceptance": {
            "thresholds": {
                "max_position_difference_m": POSITION_LIMIT_M,
                "max_rotation_difference_deg": ROTATION_LIMIT_DEG,
                "timestamp_sets_must_match": True,
            },
            "checks": checks,
            "passed": all(checks.values()),
        },
        "accuracy_note": "This is a repeatability comparison, not ground-truth ATE/RPE.",
        "timing_note": (
            "Current Total Time is assigned to the most recent LIO/VIO title; p95 uses nearest rank."),
    }
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare two trajectory.txt files and emit a JSON repeatability report.")
    parser.add_argument("baseline_a", help="first baseline output directory")
    parser.add_argument("baseline_b", help="second baseline output directory")
    parser.add_argument("--output", help="write JSON to a new file instead of stdout")
    args = parser.parse_args(argv)
    directory_a = Path(args.baseline_a).expanduser().resolve()
    directory_b = Path(args.baseline_b).expanduser().resolve()
    for directory in (directory_a, directory_b):
        if not directory.is_dir():
            print("compare_baselines: baseline directory not found: {}".format(directory),
                  file=sys.stderr)
            return 2
        if not (directory / "trajectory.txt").is_file():
            print("compare_baselines: trajectory.txt not found in {}".format(directory),
                  file=sys.stderr)
            return 2

    report = compare(directory_a, directory_b)
    rendered = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        output_path = Path(args.output).expanduser().resolve()
        if output_path.exists():
            print("compare_baselines: output file already exists: {}".format(output_path),
                  file=sys.stderr)
            return 2
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered, encoding="utf-8")
    else:
        sys.stdout.write(rendered)
    return 0 if report["acceptance"]["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
