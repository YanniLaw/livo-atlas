import copy
import struct
from types import SimpleNamespace as Message
import unittest

from tools.output_digest import cloud_digest, image_digest


class OutputDigestTest(unittest.TestCase):
    def cloud(self, padding):
        return Message(header=Message(frame_id='camera_init', stamp=1, seq=1),
                       width=2, height=1, point_step=12, row_step=28,
                       is_bigendian=False, is_dense=True,
                       fields=[Message(name='x', offset=0, datatype=7, count=1),
                               Message(name='rgb', offset=4, datatype=6, count=1)],
                       data=struct.pack('<fI', 1.25, 0xFF3210) + padding * 4 +
                            struct.pack('<fI', -2.5, 0xABCDEF) + padding * 8)

    def test_cloud_ignores_clocks_and_padding_but_detects_point_changes(self):
        original = self.cloud(b'\x00')
        padded = self.cloud(b'\xff')
        padded.header.stamp = 42
        padded.header.seq = 19
        self.assertEqual(cloud_digest(original), cloud_digest(padded))
        for offset, value in [(0, struct.pack('<f', 1.5)),
                              (4, struct.pack('<I', 0xFF3211))]:
            changed = copy.deepcopy(original)
            changed.data = changed.data[:offset] + value + changed.data[offset + 4:]
            self.assertNotEqual(cloud_digest(original), cloud_digest(changed))

    def test_cloud_detects_frame_and_field_changes(self):
        original = self.cloud(b'\x00')
        changed = copy.deepcopy(original)
        changed.header.frame_id = 'other'
        self.assertNotEqual(cloud_digest(original), cloud_digest(changed))
        changed = copy.deepcopy(original)
        changed.fields[1].name = 'intensity'
        self.assertNotEqual(cloud_digest(original), cloud_digest(changed))

    def test_image_ignores_row_padding_and_clock_but_detects_pixels(self):
        original = Message(header=Message(frame_id='', stamp=1, seq=2),
                           width=1, height=2, step=4, encoding='bgr8',
                           is_bigendian=False, data=bytes([1, 2, 3, 8, 4, 5, 6, 9]))
        padded = copy.deepcopy(original)
        padded.step = 3
        padded.data = bytes([1, 2, 3, 4, 5, 6])
        padded.header.stamp = 20
        self.assertEqual(image_digest(original), image_digest(padded))
        padded.data = bytes([1, 2, 3, 4, 5, 7])
        self.assertNotEqual(image_digest(original), image_digest(padded))
