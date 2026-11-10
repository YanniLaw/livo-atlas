"""Summarize geometry timings emitted by the shared C++ system."""

import math
import re


FIELDS = ('geometry_total_seconds', 'geometry_update_seconds',
          'geometry_cloud_read_seconds', 'geometry_full_build_seconds',
          'geometry_snapshot_seconds')


def summarize_geometry_timings(log, np):
    result = {}
    for name in FIELDS:
        values = [float(value) for value in
                  re.findall(r'\b' + name + r'=([0-9.eE+-]+)', log)]
        if any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError('Invalid geometry timing: ' + name)
        if values:
            result[name] = {'count': len(values), 'median': float(np.median(values)),
                            'p95': float(np.percentile(values, 95)), 'max': max(values)}
    return result
