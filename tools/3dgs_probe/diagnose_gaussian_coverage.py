"""Observe bounded initial/retained Gaussian support on one frozen heldout camera."""
import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path
import random
import shutil
import sys
from tempfile import TemporaryDirectory
import time

import numpy as np
import torch

from evaluate_gaussian_sampling import admit_training_view
from evaluate_gaussian_submaps import TrainedBlock, camera_from_final_parent, prepare_group
from evaluate_heldout_snapshot import initialize_cuda, load_settings
from evaluation_metrics import measure
from gaussian_coverage import COHORTS, RADII, coverage_summary, projected_support
from gaussian_map import _require
from gaussian_state import _open_file
from heldout_query import canonical_query, load_evaluation_snapshot, verify_evaluation_snapshot
from heldout_selection import EvaluationSelector
from holdout_split import HoldoutSplit, MAX_SPLIT_BYTES
from online_model import OnlineGaussianModel
from prepare_holdout_split import file_sha
from render_gaussian_map import _exclusive_write, _json_bytes, _new_output, _sources_report
from joint_renderer import infer_blocks
from train import create_parameters
from worker_config import load_worker_config


class CaptureModel(OnlineGaussianModel):
    """Copy actual initializer inputs; no extra renders or RNG during training."""
    def reset(self, generation):
        super().reset(generation)
        self.initial = {}
        self.prepasses = {}

    def _coverage_prepass(self, view):
        mask, evidence = super()._coverage_prepass(view)
        self.prepasses[view['keyframe_id']] = None if mask is None else np.packbits(mask.ravel(),bitorder='little')
        return mask, evidence

    def _append(self, view, initial):
        self.initial[view['keyframe_id']] = {key:value.copy() for key,value in initial.items()}
        super()._append(view, initial)


def train_observed_group(group, views, poses, device, *, seed_policy, options):
    _require([v['keyframe_id'] for v in views] == group['train_ids'] and
             all(v['split']=='train' for v in views) and not set(group['train_ids']) & set(group['held_out_ids']),
             'Diagnostic training roles differ')
    model = CaptureModel(device, options=options)
    model.reset(1)
    anchor = poses[group['anchor_id']]
    started = time.perf_counter()
    torch.cuda.reset_peak_memory_stats(device)
    training = []
    for index, view in enumerate(views):
        camera = camera_from_final_parent(view, poses[view['keyframe_id']])
        _, metrics = model.train_candidate(dict(view,camera_from_parent=camera @ anchor,generation='1'),seed_policy=seed_policy)
        active = group['train_ids'][max(0,index+1-10):index+1]
        others = active[:-1]
        schedule = [view['keyframe_id']]*10 + ([others[j % len(others)] for j in range(10)] if others else [view['keyframe_id']]*10)
        _require(metrics['frame_ids']==active and metrics['steps']==20 and metrics['training_frame_ids']==schedule,
                 'Diagnostic training lineage differs')
        training.append(dict(keyframe_id=view['keyframe_id'],**metrics))
        model.decide(True)
    cpu = lambda values:{key:value.detach().cpu().numpy().copy() for key,value in values.items()}
    retained = {key:np.concatenate([model.initial[frame][key] for frame in model.frame_ids]) for key in model.initial[model.frame_ids[0]]}
    initial = cpu(create_parameters(retained,device))
    parameters = cpu(model.parameters)
    origins = model.origin_ids.cpu().numpy().astype('i8')
    ids, counts = np.unique(origins,return_counts=True)
    _require(ids.tolist()==model.frame_ids, 'Retained initialization origin mismatch')
    evidence = dict(submap_id=group['submap_id'],anchor_id=group['anchor_id'],parent_from_submap=anchor.tolist(),
        training=training,retained_frame_ids=model.frame_ids,gaussian_origins={str(k):int(n) for k,n in zip(ids,counts)},
        training_seconds=time.perf_counter()-started,gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(device),
        gpu_peak_reserved_bytes=torch.cuda.max_memory_reserved(device))
    return dict(block=TrainedBlock(group['submap_id'],anchor,parameters),evidence=evidence,initial=initial,origin_ids=origins,
        prepasses={key:model.prepasses.get(key) for key in group['train_ids']},
        all_initial_means=np.concatenate([model.initial[key]['means'] for key in group['train_ids']]),
        all_origin_ids=np.concatenate([np.full(len(model.initial[key]['means']),key,dtype='i8') for key in group['train_ids']]))


def analyze_mode(ids, arrays, query):
    camera = arrays['camera_from_parent']
    support = {}
    for cohort in COHORTS:
        points = []
        for key in ids:
            prefix = f'block_{key}_'
            name = 'all_initial_means' if cohort=='all_initial' else ('initial_means' if cohort=='retained_initial' else 'trained_means')
            pose = camera @ arrays[prefix+'anchor']
            points.append(arrays[prefix+name].astype('f8') @ pose[:3,:3].T + pose[:3,3])
        points = np.concatenate(points)
        support[cohort] = {radius:projected_support(points,np.eye(4),arrays['K'],query,radius) for radius in RADII}
    return support


def _diagnostic(path, arrays, mode, support):
    from PIL import Image, ImageDraw
    image = arrays['image_mask'].astype(bool)
    labels = arrays['depth_mask'].astype(bool)
    rgb = arrays[mode+'_trained_rgb']
    holes = image & (arrays[mode+'_trained_alpha'] < .05)
    marked = rgb.copy();marked[holes] = [1,0,1]
    classes = rgb.copy()*.25
    maps = support['retained_trained'][4]
    c,f,b = (maps[key] > 0 for key in ('consistent','front','behind'))
    classes[holes & ~labels] = [.5,.5,.5]
    classes[holes & labels] = [1,0,0]
    for mask,color in ((c,[0,1,0]),(~c & f & ~b,[1,.5,0]),(~c & b & ~f,[0,.3,1]),(~c & f & b,[.7,0,1])):
        classes[holes & labels & mask] = color
    density = np.log1p(maps['any']).astype('f4')
    density /= max(1.,float(density.max()))
    panels = [('reference',arrays['reference_rgb']),('retained initial',arrays[mode+'_initial_rgb']),
              ('trained',rgb),('holes magenta',marked),('r4 depth support',classes),('r4 log center count',np.repeat(density[...,None],3,axis=2))]
    sheet = Image.new('RGB',(6*320,280),'white')
    for index,(title,value) in enumerate(panels):
        value=value.copy();value[~image]=0
        tile=Image.fromarray(np.rint(np.clip(value,0,1)*255).astype('u1'));tile.thumbnail((320,256))
        sheet.paste(tile,(index*320,24));ImageDraw.Draw(sheet).text((index*320+3,5),mode+': '+title,fill='black')
    sheet.save(path)


def run(args):
    output = _new_output(args.map_directory,args.keyframes,args.output)
    _require(type(args.random_seed) is int and 0 <= args.random_seed < 2**32,'Invalid random seed')
    settings = load_settings(args.config)
    raw_config = Path(args.config).read_bytes()
    _require(hashlib.sha256(raw_config).hexdigest()==settings.config_sha256,'Config changed')
    snapshot = load_evaluation_snapshot(args.map_directory,args.keyframes)
    if settings.split_file:
        with _open_file(Path(settings.split_file),MAX_SPLIT_BYTES) as stream:
            _require(HoldoutSplit(stream.read()).split_id==snapshot.split.split_id,'Configured split differs')
    _require(type(args.keyframe_id) is int and args.keyframe_id in snapshot.split.frames,'Unknown query')
    frame = snapshot.split.frames[args.keyframe_id]
    _require(frame['role']=='heldout' and frame['query'],'Query must be a frozen heldout query')
    owner = int(frame['submap_id'])
    preparer = args.prepare_executable.resolve(strict=True)
    preparer_hash = file_sha(preparer)
    selector = EvaluationSelector(snapshot,settings.policy,args.selector_executable or preparer.with_name('livo_atlas_select_gaussian_neighbor'))
    selection = selector.select(frame)
    ids = selection['selected_ids']
    _require(ids==sorted(set(ids)) and owner in ids and ids[-1]==owner and 1 <= len(ids) <= 2,'Invalid bounded selection')
    if selection['selection'] is not None:selection['selection'].pop('selection_seconds',None)
    groups = []
    for key in ids:
        primary = [(k,f) for k,f in snapshot.split.frames.items() if int(f['submap_id'])==key]
        train = [k for k,f in primary if f['role']=='train']
        held = [k for k,f in primary if f['role']=='heldout']
        _require(train and 0 < len(primary) <= 32,'Selected diagnostic group exceeds bounds')
        groups.append(dict(submap_id=key,anchor_id=int(snapshot.document['entries'][key]['anchor_keyframe_id']),
                           train_ids=train,held_out_ids=held or [args.keyframe_id],frame_count=len(snapshot.poses)))
    maximum = min(64*1024**2,settings.max_output_bytes)
    reserve = max(1024**3,settings.min_free_disk_bytes)
    modes = ['single'] + (['joint'] if len(ids)==2 else [])
    pixels = frame['width']*frame['height']
    prepass_bytes=sum((snapshot.split.frames[key]['width']*snapshot.split.frames[key]['height']+7)//8
                      for group in groups for key in group['train_ids'])
    header_bytes=len(snapshot.split.payload)+len(snapshot.poses)*128+len(raw_config)+2*1024**2+65536
    predicted = pixels*(18+40*len(modes)) + len(ids)*2*1024**2 + len(modes)*2*1024**2 + prepass_bytes + header_bytes
    _require(maximum >= 2*1024**2 and predicted <= maximum,'Diagnostic output budget is insufficient')
    scratch = max(sum(snapshot.split.frames[k]['width']*snapshot.split.frames[k]['height']*32+128*1024
                      for k in group['train_ids']+group['held_out_ids']) for group in groups)
    _require(shutil.disk_usage(output.parent).free >= scratch+maximum+reserve,'Insufficient diagnostic disk reserve')
    output.mkdir(mode=0o700)
    try:
        _exclusive_write(output/'config.yaml',raw_config)
        config=load_worker_config(output/'config.yaml')
        _exclusive_write(output/'split.json',snapshot.split.payload)
        _exclusive_write(output/'poses.f64le',np.asarray(snapshot.poses,dtype='<f8').tobytes())
        random.seed(args.random_seed);np.random.seed(args.random_seed)
        device,software = initialize_cuda(settings.worker)
        torch.manual_seed(args.random_seed);torch.cuda.manual_seed_all(args.random_seed)
        arrays, evidence, query = {}, [], None
        for group in groups:
            _require(shutil.disk_usage(output).free >= scratch+maximum+reserve,'Insufficient preparation reserve')
            with TemporaryDirectory(prefix='.prepare-',dir=output) as temporary:
                views=prepare_group(preparer,snapshot.keyframes,Path(temporary)/'bundle',group,snapshot.split.source['camera_sha256'])
            if args.keyframe_id in views:query=canonical_query(snapshot.split,views[args.keyframe_id])
            training=[admit_training_view(snapshot.split,views[key]) for key in group['train_ids']]
            value=train_observed_group(group,training,snapshot.poses,device,seed_policy=config.seed_policy,options=settings.worker)
            key=group['submap_id'];prefix=f'block_{key}_'
            arrays[prefix+'anchor']=value['block'].parent_from_submap
            for name in ('all_initial_means','all_origin_ids','origin_ids'):arrays[prefix+name]=value[name]
            for frame_id,mask in value['prepasses'].items():
                arrays[prefix+f'prepass_{frame_id}']=np.array([],dtype='u1') if mask is None else mask
            for state,parameters in (('initial',value['initial']),('trained',value['block'].parameters)):
                arrays.update({prefix+state+'_'+name:data for name,data in parameters.items()})
            evidence.append(value['evidence'])
        _require(query is not None,'Query was not prepared')
        arrays.update(reference_rgb=query['rgb'],reference_depth_m=query['depth_m'],image_mask=query['image_mask'],
            depth_mask=query['depth_mask'],K=query['K'],camera_from_parent=query['camera_from_body'] @ np.linalg.inv(snapshot.poses[args.keyframe_id]))
        summaries={'single':None,'joint':None};metrics={};pictures=[]
        for mode in modes:
            selected=[owner] if mode=='single' else ids
            for state in ('initial','trained'):
                blocks=[]
                for key in selected:
                    prefix=f'block_{key}_'
                    parameters={name:arrays[prefix+state+'_'+name] for name in ('means','quats','log_scales','color_logits','opacity_logits')}
                    blocks.append((key,parameters,arrays['camera_from_parent'] @ arrays[prefix+'anchor']))
                rendered=infer_blocks(blocks,arrays['K'],frame['width'],frame['height'],device)
                arrays.update({mode+'_'+state+'_'+name:rendered[name] for name in ('rgb','alpha','depth_m')})
                metrics[mode+'_'+state]=measure(rendered,query)
            support=analyze_mode(selected,arrays,query)
            summaries[mode]=coverage_summary(query,{'alpha':arrays[mode+'_initial_alpha']},{'alpha':arrays[mode+'_trained_alpha']},support)
            path=output/(mode+'.png');_diagnostic(path,arrays,mode,support);pictures.append(path.name)
        _require(sum(v.nbytes for v in arrays.values())+1024**2 <= maximum,'Diagnostic raw array budget exceeded')
        _require(shutil.disk_usage(output).free >= maximum+reserve,'Insufficient sample reserve')
        np.savez_compressed(output/'samples.npz',**arrays)
        paths=['samples.npz','config.yaml','split.json','poses.f64le',*pictures]
        artifacts={name:dict(bytes=(output/name).stat().st_size,sha256=file_sha(output/name)) for name in paths}
        report=dict(schema=1,kind='livo_atlas.gaussian_coverage_diagnostic',status='succeeded',keyframe_id=args.keyframe_id,
            owner_submap_id=owner,selected_ids=ids,selection=selection,selection_policy=settings.policy,
            selector_runtime_sha256=selector.runtime_sha256,random_seed=args.random_seed,seed_policy=config.seed_policy,
            training_options=asdict(settings.worker),software=software,split_id=snapshot.split.split_id,
            sources=_sources_report(snapshot),source_sha256=snapshot.source_digests,config_sha256=settings.config_sha256,
            preparer_sha256=preparer_hash,final_poses_sha256=file_sha(output/'poses.f64le'),groups=evidence,
            summary=summaries,metrics=metrics,artifacts=artifacts)
        verify_evaluation_snapshot(snapshot)
        _require(file_sha(args.config)==file_sha(output/'config.yaml')==settings.config_sha256 and file_sha(preparer)==preparer_hash,'Input changed')
        _require((selector.executable is None or file_sha(selector.executable)==selector.sha256) and
                 all(file_sha(Path(path))==digest for path,digest in selector.libraries.items()),'Selector changed')
        payload=_json_bytes(report,2*1024**2)
        _require(sum(row['bytes'] for row in artifacts.values())+len(payload) <= maximum,'Diagnostic output budget exceeded')
        _require(shutil.disk_usage(output).free >= len(payload)+65536+reserve,'Insufficient report reserve')
        pending=output/'.report.pending.json';_exclusive_write(pending,payload)
        _require(shutil.disk_usage(output).free >= reserve,'Insufficient reserve after report write')
        pending.replace(output/'report.json')
        return report
    except BaseException as error:
        if not (output/'report.json').exists():
            _exclusive_write(output/'failure.json',_json_bytes(dict(status='failed',error=str(error)),65536))
        raise


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('map-directory','keyframes','prepare-executable','config','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--selector-executable',type=Path)
    parser.add_argument('--keyframe-id',type=int,required=True)
    parser.add_argument('--random-seed',type=int,default=0)
    try:run(parser.parse_args(argv))
    except Exception as error:
        print(f'{type(error).__name__}: {error}',file=sys.stderr);return 1
    return 0


if __name__=='__main__':sys.exit(main())
