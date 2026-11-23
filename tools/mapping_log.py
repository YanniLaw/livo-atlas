"""Read captured LIVO logs from either ROS runner."""

import math
import re


_GLOG_PREFIX = re.compile(r'^([IWEF])\d{4}(?:\d{4})? \d{2}:\d{2}:\d{2}\.\d{6}\s+')
_FRONTEND_UPDATE = re.compile(
    r'\blivo_atlas update kind=(lio|vio) frontend_process_seconds=(\S+)')
_LEGACY_DURATION = re.compile(r'\|\s*Current Total Time\s*\|\s*([0-9.eE+-]+)')
_STAGE_TIMING = re.compile(r'\blivo_atlas timing stage=([a-z_]+) seconds=(\S+)')
_BUFFER_PEAK = re.compile(
    r'\blivo_atlas sensor_buffer_peak queue=([a-z_]+) count=(\d+) span_s=(\d+(?:\.\d+)?)')
_PATH_LIMIT = re.compile(r'\blivo_atlas frontend ready path_max_poses=([1-9]\d*)\b')


def glog_severity(line):
    match = _GLOG_PREFIX.match(line)
    return match[1] if match else None


def read_node_logs(output):
    stdout = (output / 'node.log').read_text(errors='replace')
    stderr = output / 'node-stderr.log'
    return stdout, stderr.read_text(errors='replace') if stderr.is_file() else ''


def read_mapping_log(output):
    merged = output / 'mapping.log'
    if merged.is_file():
        return merged.read_text(errors='replace')
    return '\n'.join(read_node_logs(output))


def read_path_max_poses(log):
    match = _PATH_LIMIT.search(log)
    return int(match.group(1)) if match else None


def _append_seconds(samples, value):
    try:
        seconds = float(value)
    except ValueError:
        return
    if math.isfinite(seconds) and seconds >= 0:
        samples.append(seconds)


def summarize_timings(samples, np):
    summary = {}
    for stage, values in samples.items():
        if not values:
            continue
        p50, p95, p99 = (float(value) for value in np.percentile(values, [50, 95, 99]))
        summary[stage] = dict(count=len(values), p50=p50, median=p50,
                              p95=p95, p99=p99, max=max(values))
    return summary


def summarize_frontend_updates(log, np):
    counts = {'lio': 0, 'vio': 0}
    samples = {kind: [] for kind in counts}
    updates = _FRONTEND_UPDATE.findall(log)
    if updates:
        for kind, value in updates:
            counts[kind] += 1
            _append_seconds(samples[kind], value)
    else:
        kind = None
        for line in log.splitlines():
            if 'LIO Mapping Time' in line:
                kind = 'lio'
                counts[kind] += 1
            elif 'VIO Time' in line:
                kind = 'vio'
                counts[kind] += 1
            match = _LEGACY_DURATION.search(line)
            if match and kind is not None:
                _append_seconds(samples[kind], match.group(1))
    return counts, summarize_timings(samples, np)


def summarize_stage_timings(log, np):
    samples = {}
    for stage, value in _STAGE_TIMING.findall(log):
        _append_seconds(samples.setdefault(stage, []), value)
    peaks = {queue: dict(count=int(count), span_s=float(span))
             for queue, count, span in _BUFFER_PEAK.findall(log)}
    return summarize_timings(samples, np), peaks
