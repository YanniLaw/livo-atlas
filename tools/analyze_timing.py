#!/usr/bin/env python3
"""Summarize an existing node log without changing it."""

import argparse
import json
from pathlib import Path

try:
    from .timing_analysis import analyze_lines
except ImportError:
    from timing_analysis import analyze_lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.log.open(errors='replace') as stream:
        report = analyze_lines(stream)
    report['log'] = str(args.log.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
