import copy
import os
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

from gaussian_coverage import coverage_summary, projected_support


class ProjectionSupportTest(unittest.TestCase):
    def query(self, height=3, width=5):
        return dict(image_mask=np.ones((height,width),bool), depth_mask=np.ones((height,width),bool),
                    depth_m=np.full((height,width),2.))

    def test_rotated_camera_covers_native_border_pixels(self):
        q=self.query();camera=np.eye(4)
        angle=.37;camera[:3,:3]=[[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]]
        camera[:3,3]=[1.7,-1.3,.8]
        k=np.array([[381.,0,2],[0,379.,1],[0,0,1]])
        yy,xx=np.indices((3,5));z=np.full(15,2.)
        local=np.column_stack(((xx.ravel()-2)*z/381,(yy.ravel()-1)*z/379,z))
        points=(local-camera[:3,3])@camera[:3,:3]
        result=projected_support(points,camera,k,q,0)
        np.testing.assert_array_equal(result['any'],np.ones((3,5)))
        np.testing.assert_array_equal(result['consistent'],np.ones((3,5)))

    def test_half_pixel_borders_and_outside_neighborhood(self):
        q=self.query();k=np.eye(3)
        points=np.array([[-1.,2.,2.],[9.,2.,2.],[-2.2,2.,2.],[0,0,-2.]])
        result=projected_support(points,np.eye(4),k,q,0)
        self.assertEqual(int(result['any'].sum()),1)
        self.assertEqual(result['any'][1,0],1)
        result=projected_support(points[2:3],np.eye(4),k,q,2)
        self.assertGreater(result['any'][1,0],0)

    def test_circular_radius_depth_is_tested_at_each_target_pixel(self):
        q=self.query(7,7);q['depth_m'][3,4]=1.;q['depth_m'][3,2]=3.
        q['depth_mask'][2,3]=False
        k=np.array([[1.,0,3],[0,1.,3],[0,0,1.]])
        result=projected_support(np.array([[0.,0.,2.]]),np.eye(4),k,q,2)
        self.assertEqual(int(result['any'].sum()),13)
        self.assertEqual(int(result['consistent'].sum()),10)
        self.assertEqual(result['behind'][3,4],1)
        self.assertEqual(result['front'][3,2],1)
        self.assertEqual(result['any'][2,3],1)
        self.assertEqual(result['consistent'][2,3],0)
        self.assertEqual(result['any'][1,1],0)

    def test_invalid_inputs_are_rejected(self):
        q=self.query();points=np.array([[0.,0.,2.]])
        cases=[(points,np.eye(4),np.eye(3),q,1),
               (points*np.nan,np.eye(4),np.eye(3),q,0),
               (points,np.eye(4)*2,np.eye(3),q,0),
               (points,np.eye(4),np.zeros((3,3)),q,0)]
        bad=copy.deepcopy(q);bad['image_mask'][0,0]=False
        cases.append((points,np.eye(4),np.eye(3),bad,0))
        for values in cases:
            with self.subTest(values=values),self.assertRaises(ValueError):projected_support(*values)

    def test_coverage_and_depth_classes_partition_only_known_holes(self):
        q=self.query(1,4);q['depth_mask'][0,3]=False
        initial=dict(alpha=np.array([[.1,0,.1,0.]]))
        trained=dict(alpha=np.array([[.1,.1,0.,0.]]))
        maps={key:np.zeros((1,4),dtype='i4') for key in ('any','consistent','front','behind')}
        maps['any'][0,2]=2;maps['front'][0,2]=1;maps['behind'][0,2]=1
        supports={name:{r:copy.deepcopy(maps) for r in (0,2,4)}
                  for name in ('all_initial','retained_initial','retained_trained')}
        result=coverage_summary(q,initial,trained,supports)
        for key,value in [('image_pixels',4),('common_covered',1),('gained_coverage',1),('lost_coverage',1),
                          ('persistent_holes',1),('hole_depth_known',1),('hole_depth_unknown',1)]:
            self.assertEqual(result[key],value)
        classes=result['support']['retained_trained']['2']['labelled_holes']
        self.assertEqual(classes,dict(consistent=0,front_only=0,behind_only=0,mixed=1,none=0))


class CoverageGpuTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import torch
        if not torch.cuda.is_available():
            if os.environ.get('LIVO_REQUIRE_GPU')=='1':raise RuntimeError('CUDA required')
            raise unittest.SkipTest('CUDA unavailable')

    def test_observation_matches_original_training_through_eviction(self):
        import torch
        from dataclasses import replace
        from diagnose_gaussian_coverage import train_observed_group
        from evaluate_gaussian_submaps import train_group
        from online_model import checked_cuda_device
        from test_online_model import make_view
        from worker_config import DEFAULT_WORKER_OPTIONS
        options=replace(DEFAULT_WORKER_OPTIONS,min_free_gpu_memory_mib=128,gpu_memory_fraction=.2,
                        coverage_sampling='replace',coverage_uncovered_ratio=.25)
        device=checked_cuda_device(options);torch.set_num_threads(1)
        views=[dict(make_view(key,dense=True),parent_from_body=np.eye(4)) for key in range(12)]
        group=dict(submap_id=0,anchor_id=0,train_ids=list(range(12)),held_out_ids=[99])
        poses=np.tile(np.eye(4),(12,1,1))
        torch.manual_seed(7)
        block,evidence=train_group(group,views,poses,device,seed_policy='coverage',options=options)
        random_state=torch.cuda.get_rng_state()
        torch.manual_seed(7)
        actual=train_observed_group(group,views,poses,device,seed_policy='coverage',options=options)
        for name,value in block.parameters.items():
            np.testing.assert_allclose(actual['block'].parameters[name],value,atol=2e-5,rtol=0)
        self.assertTrue(torch.equal(random_state,torch.cuda.get_rng_state()))
        self.assertEqual(actual['evidence']['retained_frame_ids'],list(range(2,12)))
        for before,after in zip(evidence['training'],actual['evidence']['training']):
            for key in ('keyframe_id','training_frame_ids','steps','frame_ids','gaussian_count'):
                self.assertEqual(before[key],after[key])
        self.assertEqual(len(actual['all_initial_means']),12*48)
        self.assertEqual(len(actual['initial']['means']),10*48)
        np.testing.assert_array_equal(actual['all_initial_means'][actual['all_origin_ids']>=2],actual['initial']['means'])

    def fixture(self, blocks=2):
        from test_diagnose_gaussian_seeds import cli_fixture
        args,prepared=cli_fixture(self,blocks=blocks)
        args.keyframe_id=1
        args.selector_executable=Path(os.environ['LIVO_SELECTOR_BIN'])
        return args,prepared

    def test_captured_initializers_include_real_coverage_replacements(self):
        from dataclasses import replace
        from diagnose_gaussian_coverage import train_observed_group
        from online_model import checked_cuda_device
        from test_online_model import make_view
        from worker_config import DEFAULT_WORKER_OPTIONS, LearningRates
        options=replace(DEFAULT_WORKER_OPTIONS,min_free_gpu_memory_mib=128,gpu_memory_fraction=.2,
                        coverage_sampling='replace',coverage_uncovered_ratio=.25,
                        learning_rates=LearningRates(0.,0.,0.,0.))
        views=[dict(make_view(key,width=80,height=80,dense=True),parent_from_body=np.eye(4)) for key in range(2)]
        group=dict(submap_id=0,anchor_id=0,train_ids=[0,1],held_out_ids=[99])
        result=train_observed_group(group,views,np.tile(np.eye(4),(2,1,1)),checked_cuda_device(options),
                                    seed_policy='coverage',options=options)
        self.assertGreater(result['evidence']['training'][1]['seed_selection']['replaced_seeds'],0)
        self.assertEqual(len(result['all_initial_means']),2000)
        self.assertIsNone(result['prepasses'][0])
        self.assertEqual(len(result['prepasses'][1]),800)
        np.testing.assert_array_equal(result['initial']['means'],result['block'].parameters['means'])
        for name,value in result['initial'].items():
            np.testing.assert_array_equal(value,result['block'].parameters[name])

    def test_cli_binds_heldout_and_owned_arrays(self):
        import json
        import diagnose_gaussian_coverage as diagnostic
        args,prepared=self.fixture()
        with mock.patch.object(diagnostic,'prepare_group',side_effect=prepared):report=diagnostic.run(args)
        self.assertEqual(report['status'],'succeeded')
        self.assertEqual(report['keyframe_id'],1)
        self.assertEqual(report['selected_ids'],[0])
        self.assertEqual(report['groups'][0]['training'][0]['training_frame_ids'],[0]*20)
        self.assertIsNone(report['summary']['joint'])
        self.assertEqual(json.loads((args.output/'report.json').read_text())['split_id'],report['split_id'])
        with np.load(args.output/'samples.npz',allow_pickle=False) as data:
            self.assertIn('block_0_initial_means',data)
            self.assertIn('single_trained_alpha',data)
        args.output=args.output.parent/'bad-role';args.keyframe_id=0
        with self.assertRaises(ValueError):diagnostic.run(args)
        self.assertFalse(args.output.exists())

    def test_cli_rejects_small_budget_and_failed_publication(self):
        import yaml
        import diagnose_gaussian_coverage as diagnostic
        args,prepared=self.fixture();document=yaml.safe_load(args.config.read_text())
        document['gaussian']['evaluation']['max_output_mib']=1
        args.config.write_text(yaml.safe_dump(document))
        with self.assertRaisesRegex(ValueError,'budget'):diagnostic.run(args)
        self.assertFalse(args.output.exists())
        document['gaussian']['evaluation']['max_output_mib']=64
        args.config.write_text(yaml.safe_dump(document))
        write=diagnostic._exclusive_write
        def partial(path,payload):
            if path.name=='.report.pending.json':
                write(path,payload[:64]);raise OSError('partial report')
            write(path,payload)
        with mock.patch.object(diagnostic,'prepare_group',side_effect=prepared),mock.patch.object(diagnostic,'_exclusive_write',side_effect=partial):
            with self.assertRaisesRegex(OSError,'partial report'):diagnostic.run(args)
        self.assertFalse((args.output/'report.json').exists())
        self.assertTrue((args.output/'failure.json').is_file())

    def test_joint_query_uses_actual_two_block_selection(self):
        import diagnose_gaussian_coverage as diagnostic
        args,prepared=self.fixture(blocks=5);args.keyframe_id=9
        with mock.patch.object(diagnostic,'prepare_group',side_effect=prepared):report=diagnostic.run(args)
        self.assertEqual(len(report['selected_ids']),2)
        self.assertEqual(report['selected_ids'][-1],4)
        self.assertIsNotNone(report['summary']['joint'])
        for summary in report['summary'].values():
            self.assertEqual(sum(summary[k] for k in ('common_covered','gained_coverage','lost_coverage','persistent_holes')),
                             summary['image_pixels'])

    def test_changed_config_and_post_sample_disk_drop_prevent_publication(self):
        import diagnose_gaussian_coverage as diagnostic
        args,prepared=self.fixture()
        original=args.config.read_text()
        def changed(*values):
            result=prepared(*values);args.config.write_text(original+'\n');return result
        with mock.patch.object(diagnostic,'prepare_group',side_effect=changed),self.assertRaisesRegex(ValueError,'Input changed'):
            diagnostic.run(args)
        self.assertFalse((args.output/'report.json').exists())
        args.config.write_text(original);args.output=args.output.parent/'disk-drop'
        save=diagnostic.np.savez_compressed;free=10*1024**3
        def saved(*values,**kwargs):
            nonlocal free
            save(*values,**kwargs);free=0
        with mock.patch.object(diagnostic,'prepare_group',side_effect=prepared),\
             mock.patch.object(diagnostic.np,'savez_compressed',side_effect=saved),\
             mock.patch.object(diagnostic.shutil,'disk_usage',side_effect=lambda _:mock.Mock(free=free)):
            with self.assertRaisesRegex(ValueError,'reserve'):diagnostic.run(args)
        self.assertFalse((args.output/'report.json').exists())


if __name__=='__main__':unittest.main()
