"""Compare the actual C++ envelope with pinned gsplat projection and pixels."""
import json
import math
import os
from pathlib import Path
import subprocess
import unittest

import numpy as np
import torch
from gsplat.cuda._wrapper import fully_fused_projection

from joint_renderer import infer_blocks, transform_parameters
from online_model import checked_cuda_device
from rasterizer import to_gsplat_camera
from state_fixtures import pose


class SpatialProjectionCudaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = os.environ.get('LIVO_PROJECTION_PROBE')
        if not cls.probe or not Path(cls.probe).is_file():
            raise AssertionError('The actual C++ Gaussian projection probe is required')
        cls.device = checked_cuda_device()
        torch.set_num_threads(1)

    def test_seeded_and_adversarial_gpu_support_fits_cpp_envelope(self):
        rng = np.random.default_rng(0xC3C2A)
        K = np.array([[100., 0., 49.5], [0., 100., 39.5], [0., 0., 1.]])
        cases = [(np.asarray([point], dtype='<f4'), pose()) for point in
                 ([0, 0, 4], [2, 0, 4], [-2, 0, 4], [0, 1.6, 4], [0, -1.6, 4],
                  [0, 0, .010001], [0, 0, .00999], [0, 0, 1e10], [0, 0, 1e11], [0, 0, -4])]
        cases.append((np.array([[0, 0, -1], [0, 0, 2]], dtype='<f4'), pose()))
        cases.append((np.array([[1e8, 0, 4]], dtype='<f4'), pose(-1e8 + .125)))
        for i in range(96):
            if i < 48:
                point = [rng.uniform(-.5, .5), rng.uniform(-.3, .3), rng.uniform(3, 6)]
                angle = rng.uniform(-math.pi, math.pi)
                transform = pose()
                transform[:3, :3] = [[math.cos(angle), -math.sin(angle), 0],
                                     [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
            else:
                point = rng.uniform([-8, -6, -2], [8, 6, 10])
                rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
                rotation[:, 2] *= np.linalg.det(rotation)
                transform = pose(*rng.uniform(-1, 1, 3))
                transform[:3, :3] = rotation
            cases.append((np.asarray([point], dtype='<f4'), transform))
        inputs, parameters = [], []
        for means, transform in cases:
            count = len(means)
            quaternion = rng.normal(size=4)
            quaternion = (quaternion / np.linalg.norm(quaternion)).astype('<f4')
            scales = np.exp(rng.uniform(np.log(.001), np.log(.2), 3))
            values = dict(means=means, quats=np.tile(quaternion, (count, 1)),
                log_scales=np.tile(np.log(scales).astype('<f4'), (count, 1)),
                opacity_logits=np.full(count, 2., dtype='<f4'),
                color_logits=np.full((count, 3), .3, dtype='<f4'))
            parameters.append(values)
            metadata = dict(content=dict(session_id='a' * 32, submap_id='0',
                source_generation='1', content_job_id='1'),
                extents=[dict(origin_keyframe_id='0', min=means.min(axis=0).tolist(),
                              max=means.max(axis=0).tolist(),
                              max_scale=math.exp(float(values['log_scales'].max())))],
                camera=dict(keyframe_id='0', calibration_id='0', width=100, height=80,
                    K=K.ravel().tolist(), camera_from_body=transform.ravel().tolist(), capture_time=0.))
            inputs.append(dict(metadata=metadata, coordinate_abs_upper=float(np.abs(transform[:3, 3]).sum())))
        process = subprocess.run([self.probe], input='\n'.join(json.dumps(item) for item in inputs) + '\n',
                                 text=True, capture_output=True, check=True)
        envelopes = [json.loads(line) for line in process.stdout.splitlines()]
        self.assertEqual(len(envelopes), len(cases))
        visible = projected = 0
        for index, ((_, transform), values, envelope) in enumerate(zip(cases, parameters, envelopes)):
            with self.subTest(index=index), torch.no_grad():
                packed = transform_parameters(values, transform)
                tensors = {key: torch.as_tensor(value, device=self.device) for key, value in packed.items()}
                intrinsic, view = to_gsplat_camera(K, np.eye(4))
                outputs = fully_fused_projection(tensors['means'], None, tensors['quats'],
                    tensors['log_scales'].exp(),
                    torch.as_tensor(view, dtype=torch.float32, device=self.device)[None],
                    torch.as_tensor(intrinsic, dtype=torch.float32, device=self.device)[None],
                    100, 80, packed=True, opacities=tensors['opacity_logits'].sigmoid())
                self.assertEqual(len(outputs), 8)
                radii, centers = outputs[3].cpu().numpy(), outputs[4].cpu().numpy()
                left, top, right, bottom = envelope['rectangles'][0]
                for radius, center in zip(radii, centers):
                    projected += 1
                    self.assertLessEqual(left, max(0., center[0] - radius[0]) + 1e-4)
                    self.assertLessEqual(top, max(0., center[1] - radius[1]) + 1e-4)
                    self.assertGreaterEqual(right, min(100., center[0] + radius[0]) - 1e-4)
                    self.assertGreaterEqual(bottom, min(80., center[1] + radius[1]) - 1e-4)
                result = infer_blocks([(0, values, transform)], K, 100, 80, self.device)
                ys, xs = np.nonzero(result['alpha'] > 0)
                if len(xs):
                    visible += 1
                    self.assertGreater(envelope['candidate_pixels'], 0)
                    self.assertGreaterEqual(xs.min(), left)
                    self.assertLess(xs.max(), right)
                    self.assertGreaterEqual(ys.min(), top)
                    self.assertLess(ys.max(), bottom)
                del tensors, outputs
        self.assertGreaterEqual(visible, 40)
        self.assertGreaterEqual(projected, visible)
        print(f'GPU envelope seed0xC3C2A: cases={len(cases)} visible={visible} projected={projected}; no false culls')


if __name__ == '__main__':
    unittest.main()
