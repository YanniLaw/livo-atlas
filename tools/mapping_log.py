"""Read captured LIVO logs from either ROS runner."""

import re


_GLOG_PREFIX = re.compile(r'^([IWEF])\d{4}(?:\d{4})? \d{2}:\d{2}:\d{2}\.\d{6}\s+')


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
