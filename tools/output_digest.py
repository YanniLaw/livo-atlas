"""Semantic ROS output digests, excluding transport clocks and padding."""

import hashlib
import json

import numpy as np


def _hash(metadata, parts):
    digest = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode())
    for part in parts:
        digest.update(len(part).to_bytes(8, 'little'))
        digest.update(part)
    return digest.hexdigest()


def cloud_digest(message):
    fields = [(f.name, f.offset, f.datatype, f.count) for f in message.fields]
    metadata = [message.header.frame_id, message.width, message.height,
                message.point_step, message.is_bigendian, message.is_dense, fields]
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4',
             7: 'f4', 8: 'f8'}
    parts = []
    if message.width and message.height:
        for field in message.fields:
            dtype = np.dtype(('>' if message.is_bigendian else '<') + types[field.datatype])
            values = np.ndarray((message.height, message.width, field.count),
                                dtype=dtype, buffer=message.data, offset=field.offset,
                                strides=(message.row_step, message.point_step, dtype.itemsize))
            parts.append(values.tobytes())
    return _hash(metadata, parts)


def image_digest(message):
    bytes_per_pixel = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, '8UC1': 1,
                       '8UC3': 3, 'bgra8': 4, 'rgba8': 4, 'mono16': 2}
    row_bytes = message.width * bytes_per_pixel[message.encoding]
    parts = [message.data[row * message.step:row * message.step + row_bytes]
             for row in range(message.height)]
    return _hash([message.header.frame_id, message.width, message.height,
                  message.encoding, message.is_bigendian], parts)
