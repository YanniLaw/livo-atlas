import json
from contextlib import contextmanager
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import weakref

import numpy as np
import yaml

import gaussian_map
import joint_renderer
from joint_fixtures import make_snapshot, pose, rigid_update
from joint_renderer import load_snapshot, load_block, verify_snapshot


class JointSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory, self.keyframes = make_snapshot(Path(self.temp.name))

    def load(self, key=5, ids=None):
        return load_snapshot(self.directory,self.keyframes,key,[1,0] if ids is None else ids)

    def edit(self, filename, change):
        path = self.directory / filename
        data = json.loads(path.read_text())
        change(data)
        path.write_text(json.dumps(data))

    def test_third_block_query_is_owned_and_selection_is_canonical(self):
        snapshot = self.load()
        self.assertEqual(snapshot.selected_ids,(0,1))
        self.assertEqual(snapshot.query_submap_id,2)
        self.assertEqual(snapshot.query['keyframe_id'],5)
        self.assertEqual(snapshot.query['rgb'].shape,(6,8,3))
        self.assertNotIn('camera_from_parent',snapshot.query)
        np.testing.assert_allclose(snapshot.camera_from_parent,pose(1,-3,3,True),atol=1e-12)
        self.assertEqual(len(snapshot.poses),6)
        verify_snapshot(snapshot)

    def test_rigid_old_capture_and_projected_tail_use_final_poses(self):
        rigid_update(self.directory,self.keyframes)
        snapshot = self.load()
        self.assertEqual(snapshot.document['optimized_frame_count'],'4')
        np.testing.assert_allclose(snapshot.poses[5],pose(5,10),atol=1e-12)
        np.testing.assert_allclose(snapshot.camera_from_parent,pose(11,-3,3,True),atol=1e-12)
        block = load_block(snapshot,0)
        self.assertEqual(block[0],0)
        self.assertEqual(set(block[1]),{'means','quats','log_scales','color_logits','opacity_logits'})
        np.testing.assert_allclose(block[2],pose(1,-3,3,True),atol=1e-12)

    def test_only_one_full_state_survives_each_validation_read(self):
        prior=[]
        real=gaussian_map.read_state
        def read(path):
            for ref in prior:self.assertIsNone(ref())
            state=real(path)
            prior.append(weakref.ref(state))
            return state
        with mock.patch.object(gaussian_map,'read_state',side_effect=read):
            snapshot=self.load()
        self.assertTrue(all(ref() is None for ref in prior))
        self.assertEqual(snapshot.query['rgb'][0,0,0],np.float32(.1))

    def test_invalid_ids_and_block_counts_are_rejected(self):
        for ids in ([],[0,0],[0,1,2],[True],[-1],[2**64],[3],[0.5]):
            with self.subTest(ids=ids),self.assertRaises(ValueError):self.load(ids=ids)
        for key in (True,-1,2**64,6,1.5):
            with self.subTest(key=key),self.assertRaises(ValueError):self.load(key=key)

    def test_unretained_frame_is_not_substituted(self):
        with tempfile.TemporaryDirectory() as tmp:
            d,k=make_snapshot(Path(tmp),retained=1)
            with self.assertRaisesRegex(ValueError,'retained'):
                load_snapshot(d,k,0,[0,1])

    def test_history_dirty_bad_digest_and_failed_report_rejected(self):
        path=self.directory/'map.json'
        original=path.read_text()
        for change in (lambda d:d['entries'][0].update(availability='needs_rebuild'),
                       lambda d:d['entries'][0].update(volatile=1),
                       lambda d:d['entries'][0]['asset'].update(manifest_sha256='0'*64),
                       lambda d:d.update(frame_count='7'),
                       lambda d:d.update(session_id='b'*32)):
            path.write_text(original)
            self.edit('map.json',change)
            with self.assertRaises((ValueError, OSError)):self.load()
        path.write_text(original)
        report=self.directory/'finalization.json';original=report.read_text()
        for key,value in [('status','timed_out'),('worker_stopped',0),('eligible_set_frozen',0),
                          ('map_sequence','2'),('session_id','b'*32)]:
            report.write_text(original)
            self.edit('finalization.json',lambda d:d.update({key:value}))
            with self.subTest(key=key),self.assertRaises(ValueError):self.load()

    def test_unselected_missing_is_reported_but_selected_missing_is_rejected(self):
        import shutil
        document = json.loads((self.directory / 'map.json').read_text())
        row = document['entries'][2]
        for key in ('content', 'content_source', 'trained_frame_ids', 'gaussian_count',
                    'asset', 'asset_source', 'asset_trained_frame_ids', 'asset_gaussian_count',
                    'asset_current'):
            row.pop(key)
        row.update(availability='missing', resident=0)
        (self.directory / 'map.json').write_text(json.dumps(document))
        shutil.rmtree(self.directory / 'states/3')
        self.edit('finalization.json', lambda r:r.update(eligible_submap_ids=['0','1'],
            asset_ids=['1','2'],eligible=2,current=2,missing=1))
        snapshot = self.load(key=1,ids=[0,1])
        self.assertEqual(snapshot.map_report['missing'],1)
        with self.assertRaisesRegex(ValueError,'missing'):
            self.load(key=1,ids=[2])

    def test_visible_source_changes_invalidate_snapshot_and_block_reload(self):
        snapshot=self.load()
        path=self.keyframes/'frames/0/frame.yml'
        path.write_text(path.read_text()+'\n')
        with self.assertRaisesRegex(ValueError,'changed'):verify_snapshot(snapshot)
        with self.assertRaisesRegex(ValueError,'changed'):load_block(snapshot,0)

    def test_manifest_changed_after_validation_cannot_be_reloaded(self):
        snapshot=self.load()
        path=self.directory/'states/1/manifest.json'
        path.write_text(path.read_text()+'\n')
        with self.assertRaises(ValueError):load_block(snapshot,0)

    def test_new_group_revision_invalidates_snapshot(self):
        snapshot=self.load()
        (self.keyframes/'online_submaps/records/0/4.yml').write_text('{}')
        with self.assertRaisesRegex(ValueError,'changed'):verify_snapshot(snapshot)

    def test_newer_pose_version_is_rejected_before_opening_the_snapshot(self):
        rigid_update(self.directory, self.keyframes)
        path = self.keyframes / 'online_submaps/poses/1.yml'
        newer = yaml.safe_load(path.read_text())
        newer.update(version='2', base_version='1')
        for row in newer['poses']:
            row['map_from_body']['translation'][1] += 10
        path.with_name('2.yml').write_text(yaml.safe_dump(newer))
        # The old inspector only checks the explicitly requested prefix/version.
        gaussian_map.inspect_gaussian_map(self.directory, self.keyframes, 1, 6)
        with self.assertRaises(ValueError):
            self.load()

    def test_later_raw_frame_frontier_is_not_silently_ignored(self):
        path = self.keyframes / 'frames/6'
        path.mkdir()
        frame = yaml.safe_load((self.keyframes / 'frames/5/frame.yml').read_text())
        frame['id'] = '6'
        frame['odom_from_body']['translation'][0] = 6
        (path / 'frame.yml').write_text(yaml.safe_dump(frame))
        gaussian_map.inspect_gaussian_map(self.directory, self.keyframes, 0, 6)
        with self.assertRaises(ValueError):
            self.load()

    def test_authoritative_group_root_is_required_for_rendering_only(self):
        shutil.rmtree(self.keyframes / 'online_submaps/records')
        gaussian_map.inspect_gaussian_map(self.directory, self.keyframes, 0, 6)
        with self.assertRaises(ValueError):
            self.load()

    def test_group_root_and_group_directory_symlinks_are_rejected(self):
        for link_root in (False, True):
            with self.subTest(link_root=link_root), tempfile.TemporaryDirectory() as tmp:
                directory, keyframes = make_snapshot(Path(tmp))
                path = keyframes / 'online_submaps/records'
                if not link_root:
                    path = path / '0'
                target = Path(tmp) / 'group-target'
                path.rename(target)
                path.symlink_to(target, target_is_directory=True)
                gaussian_map.inspect_gaussian_map(directory, keyframes, 0, 6)
                with self.assertRaises(ValueError):
                    load_snapshot(directory, keyframes, 5, [0, 1])

    def test_map_switch_before_identity_capture_is_rejected(self):
        real_open = gaussian_map._open_file
        map_path = self.directory / 'map.json'
        switched = False

        @contextmanager
        def switch_after_read(path, limit):
            nonlocal switched
            with real_open(path, limit) as stream:
                yield stream
            if Path(path) == map_path and not switched:
                switched = True
                document = json.loads(map_path.read_text())
                document['map_sequence'] = '2'
                for row in document['entries']:
                    old = row['asset']['asset_id']
                    new = str(int(old) + 3)
                    (self.directory / 'states' / old).rename(self.directory / 'states' / new)
                    row['asset']['asset_id'] = new
                map_path.write_text(json.dumps(document))
                self.edit('finalization.json', lambda r: r.update(map_sequence='2', asset_ids=['4', '5', '6']))

        rejected = False
        with mock.patch.object(gaussian_map, '_open_file', side_effect=switch_after_read), \
             mock.patch.object(joint_renderer, '_open_file', side_effect=switch_after_read):
            try:
                self.load()
            except (ValueError, OSError):
                rejected = True
        # Both on-disk maps are valid; rejecting the change is the behavior tested.
        summary = gaussian_map.inspect_gaussian_map(self.directory, self.keyframes, 0, 6)
        gaussian_map.inspect_gaussian_finalization(self.directory, summary)
        self.assertTrue(switched)
        self.assertTrue(rejected, 'Mixed map references and monitored paths were accepted')

    def test_cpu_preflight_never_imports_torch(self):
        script='''
import sys
class BlockTorch:
 def find_spec(self,fullname,path=None,target=None):
  if fullname.split('.')[0]=='torch':raise AssertionError('GPU import during CPU preflight')
sys.meta_path.insert(0,BlockTorch())
from joint_renderer import load_snapshot
snapshot=load_snapshot(sys.argv[1],sys.argv[2],5,[0,1])
print(snapshot.selected_ids)
'''
        result=subprocess.run([sys.executable,'-c',script,str(self.directory),str(self.keyframes)],
                              capture_output=True,text=True,timeout=20)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('(0, 1)',result.stdout)


if __name__=='__main__':unittest.main()
