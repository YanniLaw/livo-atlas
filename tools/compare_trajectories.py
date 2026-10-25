#!/usr/bin/python3
"""Compare TUM trajectories at identical sensor times without pose alignment."""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def compare_trajectories(reference, candidate):
    for poses in (reference, candidate):
        if poses.ndim != 2 or poses.shape[1] != 8 or not len(poses) or not np.isfinite(poses).all():
            raise ValueError('Expected a nonempty finite trajectory with eight columns')
        if not (np.diff(poses[:, 0]) > 0).all():
            raise ValueError('Sensor timestamps must be strictly increasing')
        if not np.allclose(np.linalg.norm(poses[:, 4:8], axis=1), 1.0, atol=1e-5):
            raise ValueError('Trajectory contains an invalid quaternion')
    if reference.shape != candidate.shape or not np.array_equal(reference[:, 0], candidate[:, 0]):
        raise ValueError('Trajectories must have the same sensor timestamp sequence')

    qa = reference[:, 4:8]
    qb = candidate[:, 4:8]
    qa = qa / np.linalg.norm(qa, axis=1)[:, None]
    qb = qb / np.linalg.norm(qb, axis=1)[:, None]
    relative_xyz = (qa[:, 3:4] * qb[:, :3] - qb[:, 3:4] * qa[:, :3]
                    - np.cross(qa[:, :3], qb[:, :3]))
    relative_w = qa[:, 3] * qb[:, 3] + np.sum(qa[:, :3] * qb[:, :3], axis=1)
    angle = np.degrees(2 * np.arctan2(np.linalg.norm(relative_xyz, axis=1), np.abs(relative_w)))
    position = np.linalg.norm(reference[:, 1:4] - candidate[:, 1:4], axis=1)
    return {
        'trajectory_rows': len(reference), 'sensor_timestamps_identical': True,
        'exported_values_identical': bool(np.array_equal(reference, candidate)),
        'position_rms_m': float(np.sqrt(np.mean(position**2))),
        'position_max_m': float(position.max()),
        'orientation_max_deg': float(angle.max()), 'pose_alignment_applied': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--max-position-m', type=float)
    parser.add_argument('--max-orientation-deg', type=float)
    args = parser.parse_args()
    for limit in (args.max_position_m, args.max_orientation_deg):
        if limit is not None and (not math.isfinite(limit) or limit < 0):
            parser.error('Difference limits must be finite and nonnegative')
    try:
        result = compare_trajectories(np.loadtxt(args.reference, ndmin=2),
                                      np.loadtxt(args.candidate, ndmin=2))
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    result.update(reference=str(args.reference), candidate=str(args.candidate))
    criteria = []
    if args.max_position_m is not None:
        criteria.append(result['position_max_m'] <= args.max_position_m)
    if args.max_orientation_deg is not None:
        criteria.append(result['orientation_max_deg'] <= args.max_orientation_deg)
    result['within_limits'] = all(criteria) if criteria else None
    result['limits'] = {'position_m': args.max_position_m,
                        'orientation_deg': args.max_orientation_deg}
    report = json.dumps(result, indent=2) + '\n'
    if args.output is not None:
        args.output.write_text(report)
    print(report, end='')
    return 1 if criteria and not all(criteria) else 0


if __name__ == '__main__':
    raise SystemExit(main())
