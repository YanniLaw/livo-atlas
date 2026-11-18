import os
import unittest
from unittest import mock

import numpy as np

from joint_renderer import transform_parameters, render_blocks
from state_fixtures import pose


def parameters(means, colors=None):
    means = np.asarray(means,dtype='<f4').reshape(-1,3)
    n=len(means)
    colors=np.full((n,3),.3,dtype='<f4') if colors is None else np.asarray(colors,dtype='<f4')
    return dict(means=means,quats=np.tile(np.array([1,0,0,0],dtype='<f4'),(n,1)),
                log_scales=np.full((n,3),np.log(.1),dtype='<f4'),
                color_logits=np.log(np.clip(colors,1e-4,1-1e-4)/(1-np.clip(colors,1e-4,1-1e-4))),
                opacity_logits=np.full(n,np.log(.6/.4),dtype='<f4'))


def rotation(q):
    w,x,y,z=np.asarray(q,dtype=np.float64)
    return np.array([[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],
                     [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],
                     [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]])


class JointGeometryTest(unittest.TestCase):
    def test_rotates_translates_and_owns_all_arrays(self):
        raw=parameters([[1,2,3]])
        saved={k:v.copy() for k,v in raw.items()}
        out=transform_parameters(raw,pose(10,20,30,True))
        np.testing.assert_allclose(out['means'],[[8,21,33]],atol=1e-6)
        np.testing.assert_allclose(out['quats'],[[2**-.5,0,0,2**-.5]],atol=1e-6)
        for key in raw:
            np.testing.assert_array_equal(raw[key],saved[key])
            self.assertFalse(np.shares_memory(raw[key],out[key]))

    def test_noncommuting_rotation_preserves_anisotropic_covariance(self):
        raw=parameters([[0,0,4]])
        raw['quats'][0]=[2**-.5,2**-.5,0,0]
        raw['log_scales'][0]=np.log([.02,.04,.08])
        out=transform_parameters(raw,pose(rotated=True))
        expected=np.array([.5,.5,.5,.5])
        self.assertAlmostEqual(abs(np.dot(out['quats'][0],expected)),1,places=6)
        r=rotation(out['quats'][0])
        covariance=r@np.diag(np.exp(2*out['log_scales'][0]))@r.T
        np.testing.assert_allclose(covariance,np.diag([.08**2,.02**2,.04**2]),atol=1e-8)
        raw['quats']*=-1
        negative=transform_parameters(raw,pose(rotated=True))
        np.testing.assert_allclose(rotation(negative['quats'][0]),r,atol=1e-6)

    def test_common_world_change_and_large_origin_cancel_before_float32(self):
        raw=parameters([[1,0,4]])
        parent_body=pose(5,10);parent_submap=pose(2,10);camera_body=pose(1,2,3,True)
        for change in (pose(),pose(1e6,-1e6,1e6,True)):
            relative=camera_body@np.linalg.inv(change@parent_body)@(change@parent_submap)
            out=transform_parameters(raw,relative)
            np.testing.assert_allclose(out['means'],[[1,0,7]],atol=1e-6)

    def test_bad_rigid_and_parameters_rejected(self):
        raw=parameters([[0,0,4]])
        bad=pose();bad[0,0]=2
        with self.assertRaises(ValueError):transform_parameters(raw,bad)
        for key in raw:
            invalid={k:v.copy() for k,v in raw.items()}
            invalid[key].flat[0]=np.nan
            with self.subTest(key=key),self.assertRaises(ValueError):transform_parameters(invalid,pose())

    def test_shape_count_normalization_and_scale_budgets(self):
        cases = [parameters([]), parameters(np.zeros((10001, 3)))]
        for key, replacement in [('means', np.zeros((1, 2), dtype='<f4')),
                                 ('quats', np.zeros((1, 4), dtype='<f4')),
                                 ('log_scales', np.ones((1, 3), dtype='<f4'))]:
            invalid = parameters([[0, 0, 4]])
            invalid[key] = replacement
            cases.append(invalid)
        for invalid in cases:
            with self.subTest(shape=invalid['means'].shape), self.assertRaises(ValueError):
                transform_parameters(invalid, pose())

    def test_near_rigid_rotation_is_projected_for_both_center_and_shape(self):
        raw = parameters([[1, 2, 3]])
        approximate = pose(rotated=True)
        approximate[:3, :3] *= 1 + 1e-8
        out = transform_parameters(raw, approximate)
        np.testing.assert_allclose(out['means'], [[-2, 1, 3]], atol=1e-7)
        np.testing.assert_allclose(rotation(out['quats'][0]),
                                   [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-6)
        for diagonal, want in [([1, -1, -1], [0, 1, 0, 0]),
                               ([-1, 1, -1], [0, 0, 1, 0]),
                               ([-1, -1, 1], [0, 0, 0, 1])]:
            half_turn = pose()
            half_turn[:3, :3] = np.diag(diagonal)
            result = transform_parameters(raw, half_turn)
            self.assertAlmostEqual(abs(np.dot(result['quats'][0], want)), 1, places=6)

    def test_render_request_budgets_fail_before_cuda(self):
        raw = parameters([[0, 0, 4]])
        block = (0, raw, pose())
        for blocks, width, height in [([], 8, 6), ([block] * 3, 8, 6),
                                      ([block] * 2, 8, 6), ([block], True, 6),
                                      ([block], 1001, 1000), ([block], 8, 0)]:
            with self.subTest(width=width), self.assertRaises(ValueError):
                render_blocks(blocks, np.eye(3), width, height, 'cpu')


class JointCudaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        from online_model import checked_cuda_device
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU')=='1':raise RuntimeError('CUDA required')
            raise unittest.SkipTest('CUDA required')
        cls.device=checked_cuda_device()
        torch.set_num_threads(1)

    def direct(self,raw,camera,K,width,height):
        import torch
        from rasterizer import render,to_gsplat_camera
        tensors={k:torch.as_tensor(v,device=self.device) for k,v in raw.items()}
        intrinsic,view=to_gsplat_camera(K,camera)
        with torch.no_grad():
            values=render(tensors['means'],tensors['quats'],tensors['log_scales'].exp(),
                          tensors['opacity_logits'].sigmoid(),tensors['color_logits'].sigmoid(),
                          view,intrinsic,width,height)
            return {k:v.cpu().numpy().copy() for k,v in zip(('rgb','alpha','depth_m'),values)}

    def test_single_block_agrees_with_existing_camera_render(self):
        raw=parameters([[-1.5,.5,1]])
        K=np.array([[4,0,3.5],[0,4,2.5],[0,0,1]],dtype=np.float64)
        camera=pose(1,2,3,True)
        expected=self.direct(raw,camera,K,8,6)
        actual=render_blocks([(4,raw,camera)],K,8,6,self.device)
        for key,tolerance in [('rgb',1e-5),('alpha',1e-5),('depth_m',1e-4)]:
            np.testing.assert_allclose(actual[key],expected[key],atol=tolerance,rtol=0)
        self.assertGreater(actual['alpha'][3,4],.05)
        self.assertAlmostEqual(actual['depth_m'][3,4],4,places=4)

    def test_interleaved_blocks_match_independent_front_to_back_composite(self):
        import torch
        import rasterizer
        K=np.array([[40,0,31.5],[0,40,23.5],[0,0,1]],dtype=np.float64)
        colors=[[.9,.1,.1],[.1,.9,.1],[.1,.1,.9],[.9,.9,.9]]
        layers=[parameters([[0,0,z]],[color]) for z,color in zip((2,4,6,8),colors)]
        a={k:np.concatenate([layers[0][k],layers[3][k]]) for k in layers[0]}
        b={k:np.concatenate([layers[1][k],layers[2][k]]) for k in layers[0]}
        singles=[self.direct(raw,pose(),K,64,48) for raw in layers]
        transmittance=np.ones((48,64));rgb=np.zeros((48,64,3));numerator=np.zeros((48,64))
        for image,z in zip(singles,(2,4,6,8)):
            rgb+=transmittance[...,None]*image['rgb']
            numerator+=transmittance*image['alpha']*z
            transmittance*=1-image['alpha']
        alpha=1-transmittance
        depth=np.divide(numerator,alpha,out=np.zeros_like(alpha),where=alpha>1e-8)
        real=rasterizer.render
        def checked(*args,**kwargs):
            self.assertFalse(torch.is_grad_enabled())
            self.assertTrue(all(not tensor.requires_grad for tensor in args[:5]))
            self.assertEqual(args[0].shape[0],4)
            return real(*args,**kwargs)
        with mock.patch.object(rasterizer,'render',side_effect=checked):
            actual=render_blocks([(1,b,pose()),(0,a,pose())],K,64,48,self.device)
        np.testing.assert_allclose(actual['rgb'],rgb,atol=1e-6,rtol=0)
        np.testing.assert_allclose(actual['alpha'],alpha,atol=1e-6,rtol=0)
        mask=alpha>=.05
        np.testing.assert_allclose(actual['depth_m'][mask],depth[mask],atol=1e-4,rtol=0)
        reordered=render_blocks([(0,a,pose()),(1,b,pose())],K,64,48,self.device)
        np.testing.assert_array_equal(actual['rgb'],reordered['rgb'])
        swapped=[parameters([[0,0,z]],[color]) for z,color in zip((8,6,4,2),colors)]
        changed=render_blocks([(0,{k:np.concatenate([p[k] for p in swapped]) for k in a},pose())],
                              K,64,48,self.device)
        self.assertGreater(np.max(np.abs(changed['rgb']-actual['rgb'])),.1)

    def test_invisible_content_has_finite_background(self):
        K = np.array([[4, 0, 3.5], [0, 4, 2.5], [0, 0, 1]], dtype=np.float64)
        result = render_blocks([(0, parameters([[0, 0, -4]]), pose())],
                               K, 8, 6, self.device)
        for key in ('rgb', 'alpha', 'depth_m'):
            self.assertTrue(np.isfinite(result[key]).all())
            np.testing.assert_array_equal(result[key], np.zeros_like(result[key]))


if __name__=='__main__':unittest.main()
