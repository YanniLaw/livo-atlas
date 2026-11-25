"""Streaming summaries for the shared GLOG timing protocol; no ROS/NumPy."""

import math
import re


STAGE = re.compile(r'livo_atlas timing stage=(\w+) seconds=(\S+)')
FRONTEND = re.compile(
    r'livo_atlas update kind=(lio|vio) frontend_process_seconds=(\S+)')
CACHE_FIELDS = {'submap', 'pose_version', 'reader_reset', 'built',
                'cloud_reads', 'evictions', 'resident_submaps'}


def summarize(values):
    values = sorted(values)
    if not values:
        return {'count': 0, 'total': 0}

    def percentile(fraction):
        position = (len(values) - 1) * fraction
        lower = int(position)
        upper = min(lower + 1, len(values) - 1)
        return values[lower] + (values[upper] - values[lower]) * (position - lower)

    return dict(count=len(values), total=sum(values), p50=percentile(.5),
                p95=percentile(.95), p99=percentile(.99), max=values[-1])


def analyze_lines(lines):
    stages, frontend = {}, {}
    cache = dict(requests=0, builds=0, hits=0, cloud_reads=0, evictions=0,
                 reader_resets=0, max_resident_submaps=0)

    def append(destination, match):
        name, token = match.groups()
        value = float(token)
        if not math.isfinite(value) or value < 0:
            raise ValueError('Invalid timing sample: ' + token)
        destination.setdefault(name, []).append(value)

    for line in lines:
        match = STAGE.search(line)
        if match:
            append(stages, match)
        match = FRONTEND.search(line)
        if match:
            append(frontend, match)
        if 'livo_atlas loop_geometry_cache ' in line:
            fields = {k: int(v) for k, v in re.findall(r'(\w+)=(\d+)', line)}
            if not CACHE_FIELDS <= fields.keys():
                raise ValueError('Incomplete geometry cache record')
            if fields['built'] not in (0, 1) or fields['reader_reset'] not in (0, 1):
                raise ValueError('Invalid cache boolean')
            cache['requests'] += 1
            cache['builds'] += fields['built']
            cache['hits'] += 1 - fields['built']
            cache['reader_resets'] += fields['reader_reset']
            cache['cloud_reads'] += fields['cloud_reads']
            cache['evictions'] += fields['evictions']
            cache['max_resident_submaps'] = max(
                cache['max_resident_submaps'], fields['resident_submaps'])
    if not stages and not frontend:
        raise ValueError('No recognized timing samples')
    return dict(schema_version=1,
                stages={k: summarize(v) for k, v in stages.items()},
                frontend={k: summarize(v) for k, v in frontend.items()},
                cache=cache)
