#!/usr/bin/env python3
"""Convert a ROS1 bag using rosbags==0.10.9 and official Livox ROS2 .msg files.

This optional migration tool is independent of the estimator and ROS runtime.
"""
import argparse
from dataclasses import fields, is_dataclass
import hashlib
import json
from collections import Counter, defaultdict
from importlib.metadata import version
from pathlib import Path
import struct

import numpy as np


def converted_type(name):
    return name.replace('livox_ros_driver/', 'livox_ros_driver2/', 1)


def convert_message(message, target_store):
    """Small reference conversion used to check the binary conversion path."""
    if is_dataclass(message):
        target = target_store.types[converted_type(message.__msgtype__)]
        return target(**{field.name: convert_message(getattr(message, field.name), target_store)
                         for field in fields(target) if field.name != '__msgtype__'})
    if isinstance(message, list):
        return [convert_message(item, target_store) for item in message]
    return message


def message_digest(message):
    """Hash logical fields, ignoring the ROS1-only Header.seq and type rename."""
    digest = hashlib.sha256()

    def add(value):
        if is_dataclass(value):
            digest.update(converted_type(value.__msgtype__).encode())
            for field in fields(value):
                if field.name == '__msgtype__' or (value.__msgtype__ == 'std_msgs/msg/Header' and field.name == 'seq'):
                    continue
                digest.update(field.name.encode() + b'\0')
                add(getattr(value, field.name))
        elif isinstance(value, np.ndarray):
            digest.update(str((value.dtype.str, value.shape)).encode() + value.tobytes())
        elif isinstance(value, list):
            digest.update(struct.pack('<Q', len(value)))
            for item in value:
                add(item)
        elif isinstance(value, float):
            digest.update(b'f' + struct.pack('<d', value))
        else:
            digest.update(repr(value).encode() + b'\0')
    add(message)
    return digest.hexdigest()


def file_digest(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            digest.update(block)
    return digest.hexdigest()


def convert_bag(source, destination, livox_msg_dir):
    from rosbags.rosbag1 import Reader
    from rosbags.rosbag2 import Reader as Reader2, Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    if version('rosbags') != '0.10.9':
        raise RuntimeError('Use an isolated environment with rosbags==0.10.9')
    src = get_typestore(Stores.ROS1_NOETIC)
    dst = get_typestore(Stores.ROS2_HUMBLE)
    definitions = {}
    for name in ('CustomPoint', 'CustomMsg'):
        path = Path(livox_msg_dir) / (name + '.msg')
        dst.register(get_types_from_msg(path.read_text(), 'livox_ros_driver2/msg/' + name))
        definitions[name] = file_digest(path)
    counts = Counter()
    before = defaultdict(hashlib.sha256)
    after = defaultdict(hashlib.sha256)
    expected = defaultdict(list)
    mappings = {}
    with Reader(source) as reader, Writer(destination, version=8) as writer:
        connections = {}
        for connection in reader.connections:
            if connection.msgtype not in ('sensor_msgs/msg/Imu', 'sensor_msgs/msg/Image', 'livox_ros_driver/msg/CustomMsg'):
                raise ValueError('Unsupported sensor type: ' + connection.msgtype)
            src.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
            target = converted_type(connection.msgtype)
            connections[connection.id] = writer.add_connection(connection.topic, target, typestore=dst)
            mappings[connection.msgtype] = target
        for connection, timestamp, raw in reader.messages():
            # All three supported types begin with Header. Header.seq is the
            # only field removed; reject seconds outside ROS2's signed range.
            if struct.unpack_from('<I', raw, 4)[0] > 2147483647:
                raise ValueError('Sensor timestamp exceeds ROS2 signed seconds')
            target = mappings[connection.msgtype]
            cdr = dst.ros1_to_cdr(raw, target)
            restored = dst.cdr_to_ros1(cdr, target)
            if raw[4:] != restored[4:]:
                raise ValueError('Binary payload changed on ' + connection.topic)
            if counts[connection.topic] == 0:
                reference = convert_message(src.deserialize_ros1(raw, connection.msgtype), dst)
                actual = dst.deserialize_cdr(cdr, target)
                if message_digest(reference) != message_digest(actual):
                    raise ValueError('Reference conversion differs on ' + connection.topic)
            payload = struct.pack('<q', timestamp) + raw[4:]
            before[connection.topic].update(payload)
            expected[connection.topic].append(hashlib.sha256(payload).digest())
            counts[connection.topic] += 1
            writer.write(connections[connection.id], timestamp, cdr)
    observed = Counter()
    with Reader2(destination) as reader:
        for connection, timestamp, cdr in reader.messages():
            raw = dst.cdr_to_ros1(cdr, connection.msgtype)
            payload = struct.pack('<q', timestamp) + raw[4:]
            index = observed[connection.topic]
            if hashlib.sha256(payload).digest() != expected[connection.topic][index]:
                raise ValueError('Written bag payload/time differs on ' + connection.topic)
            observed[connection.topic] += 1
            after[connection.topic].update(payload)
    if observed != counts:
        raise ValueError('Written bag message counts differ')
    summary = dict(passed=True, rosbags_version=version('rosbags'),
                   source_sha256=file_digest(source), type_mapping=mappings,
                   message_counts=dict(counts), total_messages=sum(counts.values()),
                   livox_msg_sha256=definitions, header_seq_removed=True,
                   payload_sha256={topic: dict(source=value.hexdigest(), converted=after[topic].hexdigest())
                                   for topic, value in before.items()},
                   files={path.name: file_digest(path) for path in Path(destination).iterdir() if path.is_file()})
    (Path(destination) / 'conversion-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--livox-msg-dir', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(convert_bag(args.source, args.destination, args.livox_msg_dir), indent=2))


if __name__ == '__main__':
    main()
