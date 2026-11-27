"""Bounded receipt verification for the frozen four-condition sampling study."""

import hashlib
import json
from pathlib import Path
import re

from compare_gaussian_holdout import _check_training
from compare_gaussian_sampling import _CONTROLS, _load, _same_configuration
from evaluate_gaussian_sampling import normalized_training_options, verify_configuration
from gaussian_map import _require
from gaussian_state import _open_file, _reject_constant, _unique_object
from heldout_query import load_evaluation_snapshot, verify_evaluation_snapshot
from joint_renderer import _real_directory, _stamp
from render_gaussian_map import _sources_report


MAX_JSON = 4*1024**2
MAX_ARTIFACT = 64*1024**2
CONDITIONS = {f'{distribution}-{policy}': dict(distribution=distribution, policy=policy)
              for distribution in ('flat', 'stratified') for policy in ('uniform', 'coverage')}


def _path(root, relative):
    relative = Path(relative)
    _require(not relative.is_absolute() and relative.parts and all(part not in ('.', '..') for part in relative.parts),
             'Unsafe study artifact path')
    parent = root
    for part in relative.parts[:-1]:
        parent /= part
        _real_directory(parent)
    return root/relative


def capture_file(path, receipts, maximum=MAX_ARTIFACT, expected=None, *, content=False):
    """Hash the same bounded bytes we parse; stream large artifacts without retaining them."""
    path = Path(path).absolute()
    digest, chunks, size = hashlib.sha256(), [], 0
    with _open_file(path, maximum) as stream:
        while True:
            raw = stream.read(min(1024**2, maximum+1-size))
            if not raw:
                break
            size += len(raw)
            _require(size <= maximum, 'Study artifact grew beyond its byte budget')
            digest.update(raw)
            if content:
                chunks.append(raw)
    receipt = dict(bytes=size, sha256=digest.hexdigest())
    if expected is not None:
        _require(isinstance(expected, dict) and expected.keys() == receipt.keys()
                 and type(expected['bytes']) is int and expected == receipt, 'Study artifact differs: '+str(path))
    key = str(path)
    _require(key not in receipts or receipts[key] == receipt, 'Study input changed while loading: '+key)
    _require(key in receipts or len(receipts) < 8192, 'Too many study receipts')
    receipts[key] = receipt
    return b''.join(chunks) if content else receipt


def read_json(path, receipts, expected_sha=None):
    raw = capture_file(path, receipts, MAX_JSON, content=True)
    if expected_sha is not None:
        _require(hashlib.sha256(raw).hexdigest() == expected_sha, 'Study report digest differs: '+str(path))
    value = json.loads(raw, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
    _require(isinstance(value, dict), 'Expected a study object')
    return value


def _digest_file(path, digest, receipts, maximum=MAX_ARTIFACT):
    actual = capture_file(path, receipts, maximum)
    _require(actual['sha256'] == digest, 'Frozen digest differs: '+str(path))
    return actual


def _protocol(study, receipts):
    protocol = read_json(study/'protocol.json', receipts)
    _require(protocol.get('conditions') == CONDITIONS and protocol.get('seeds') == [0, 1],
             'Expected four fixed conditions and seeds 0/1')
    scenes = protocol.get('scenes')
    _require(isinstance(scenes, dict) and 1 <= len(scenes) <= 2 and all(
        isinstance(name, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,31}', name) for name in scenes),
        'Invalid study scene set')
    _require(set(protocol['selection_plans']) == set(scenes) and isinstance(protocol['software'], dict)
             and protocol['software'], 'Missing frozen plans or software')
    for scene, setting in scenes.items():
        _require(isinstance(protocol['selection_plans'][scene], list)
                 and len(protocol['selection_plans'][scene]) == setting['queries'], 'Frozen selection plan differs from query count')
        _require(type(setting['queries']) is int and 1 <= setting['queries'] <= 32
                 and type(setting['calls']) is int and setting['calls'] > 0, 'Study exceeds query/training bounds')
    return protocol


def verify_splits_and_static(study, coverage, candidates):
    """Add the split and static ancestor checks missing from the historical local verifier."""
    study, coverage, candidates = (Path(root).resolve(strict=True) for root in (study, coverage, candidates))
    for root in (study, coverage, candidates):
        _real_directory(root)
    receipts = {}
    protocol = _protocol(study, receipts)
    static_audit = read_json(study/'static-audit.json', receipts)
    _require(static_audit['status'] == 'succeeded', 'Incomplete static audit')
    support = read_json(study/'static-support.json', receipts, static_audit['static_report_sha256'])
    _require(support['kind'] == 'frozen_feedback_support_proxy' and support['training_calls'] == support['render_calls'] == 0,
             'Expected a frozen static support study')
    _digest_file(study/'static-selections.npz', support['selections_sha256'], receipts)
    _digest_file(study/'reference_sampler.py', static_audit['reference_sampler_sha256'], receipts, MAX_JSON)
    _digest_file(coverage/'audit_study.py', static_audit['independent_counter_sha256'], receipts, MAX_JSON)
    depth_audit = read_json(candidates/'audit.json', receipts, support['depth_audit_sha256'])
    _require(depth_audit['status'] == 'succeeded', 'Incomplete depth audit')
    cases = {f'{scene}-{seed}' for scene in protocol['scenes'] for seed in (0, 1)}
    _require(set(support['runs']) == set(static_audit['runs']) == set(depth_audit['runs']) == cases,
             'Incomplete static case set')
    for scene, setting in protocol['scenes'].items():
        source = Path(setting['sources']['map_directory'])/'split.json'
        frozen = capture_file(source, receipts, MAX_JSON, content=True)
        for condition in CONDITIONS:
            for seed in (0, 1):
                root = study/f'{scene}-{condition}-{seed}'
                _real_directory(root)
                _require(capture_file(root/'split.json', receipts, MAX_JSON, content=True) == frozen,
                         'Per-run split bytes differ from frozen reference')
        for seed in (0, 1):
            case = f'{scene}-{seed}'
            a, b = coverage/case, candidates/case
            _real_directory(a); _real_directory(b)
            expected = support['runs'][case]
            capture = read_json(a/'report.json', receipts, expected['capture_report_sha256'])
            depth = read_json(b/'report.json', receipts, expected['depth_report_sha256'])
            _require(expected['depth_report_sha256'] == depth_audit['runs'][case]['report_sha256'],
                     'Depth audit case differs')
            for report, kind in ((capture, 'gaussian_coverage_diagnostic'), (depth, 'depth_candidate_diagnostic')):
                _require(report.get('schema') == 1 and report.get('kind') == 'livo_atlas.'+kind
                         and report.get('status') == 'succeeded' and report['random_seed'] == seed
                         and report['keyframe_id'] == setting['static_query'] and report['split_id'] == setting['split_id']
                         and all(report[key] == setting[key] for key in
                             ('sources', 'source_sha256', 'preparer_sha256', 'selector_runtime_sha256')),
                         'Static case differs from frozen scene')
            _require(depth['selected_ids'] == capture['selected_ids'] and
                     1 <= len(capture['selected_ids']) <= 2, 'Static selections differ')
            _require(depth['diagnostic_report'] == receipts[str(a/'report.json')]
                     and depth['diagnostic_artifacts'] == capture['artifacts'], 'Static ancestry differs')
            expected_names = {'samples.npz', 'single.png', 'config.yaml', 'split.json', 'poses.f64le'}
            if len(capture['selected_ids']) == 2:
                expected_names.add('joint.png')
            _require(set(capture['artifacts']) == expected_names and set(depth['artifacts']) == {'counts.npz'},
                     'Unexpected static artifact set')
            for root, report in ((a, capture), (b, depth)):
                _require(sum(item['bytes'] for item in report['artifacts'].values()) <= MAX_ARTIFACT,
                         'Static case exceeds total artifact budget')
                for name, item in report['artifacts'].items():
                    capture_file(_path(root, name), receipts, expected=item)
            _require(capture_file(a/'split.json', receipts, MAX_JSON, content=True) == frozen
                     and receipts[str(a/'poses.f64le')]['sha256'] == capture['final_poses_sha256'] == setting['final_poses_sha256']
                     and receipts[str(a/'config.yaml')]['sha256'] == capture['config_sha256'], 'Static frozen bytes differ')
    return dict(protocol=protocol, receipts=receipts, roots=(study, coverage, candidates))


def load_study_inputs(study, coverage, candidates):
    inputs = verify_splits_and_static(study, coverage, candidates)
    study = inputs['roots'][0]
    receipts, protocol = inputs['receipts'], inputs['protocol']
    evidence = read_json(study/'final-evidence.json', receipts)
    _require(evidence['status'] == 'succeeded', 'Incomplete source study')
    documents = {name: read_json(study/(name+'.json'), receipts, evidence[name.replace('-', '_')+'_sha256'])
                 for name in ('protocol', 'implementation', 'audit', 'static-audit', 'comparison-audit', 'summary', 'review-verification')}
    for name in ('audit', 'static-audit', 'comparison-audit', 'review-verification'):
        _require(documents[name].get('status') == 'succeeded', 'Incomplete source audit: '+name)
    audit = documents['audit']
    completed = read_json(study/'resume-complete.json', receipts)
    _require(audit['status'] == completed['status'] == 'succeeded' and documents['protocol'] == protocol,
             'Incomplete or changed study audit')
    for name in ('protocol', 'implementation', 'review-verification'):
        _require(audit[name.replace('-', '_')+'_sha256'] == receipts[str(study/(name+'.json'))]['sha256'],
                 'Study audit binding differs')
    names = {f'{scene}-{condition}-{seed}' for scene in protocol['scenes'] for condition in CONDITIONS for seed in (0, 1)}
    calls = 8*sum(setting['calls'] for setting in protocol['scenes'].values())
    _require(type(protocol['expected_calls']) is int and protocol['expected_calls'] == calls
             and type(protocol['expected_steps']) is int and protocol['expected_steps'] == 20*calls, 'Frozen training budget differs')
    _require(set(audit['receipts']) == set(completed['runs']) == names and evidence['successful_runs'] == len(names)
             and evidence['successful_training_calls'] == audit['calls'] == calls
             and evidence['successful_training_steps'] == audit['steps'] == 20*calls, 'Study budget/count differs')
    inputs.update(runs={}, snapshots={})
    for scene, setting in protocol['scenes'].items():
        sources = setting['sources']
        reference = Path(setting['reference']).resolve(strict=True)
        _require(all(Path(sources[key]).resolve(strict=True).is_relative_to(reference)
                     for key in ('map_directory', 'keyframes')), 'Frozen snapshot lies outside its reference')
        snapshot = load_evaluation_snapshot(sources['map_directory'], sources['keyframes'])
        _require(_sources_report(snapshot) == sources and snapshot.source_digests == setting['source_sha256']
                 and snapshot.split.split_id == setting['split_id'], 'Live snapshot differs from frozen scene')
        inputs['snapshots'][scene] = snapshot
        for distribution in ('flat', 'stratified'):
            config = protocol['configs'][scene+'-'+distribution]
            _digest_file(Path(config['path']), config['sha256'], receipts, 256*1024)
        for condition, control in CONDITIONS.items():
            for seed in (0, 1):
                name = f'{scene}-{condition}-{seed}'
                root = study/name
                expected = audit['receipts'][name]['report_sha256']
                _require(expected == completed['runs'][name]['report_sha256'], 'Resume report receipt differs')
                captured = read_json(root/'report.json', receipts, expected)
                report, split, digest = _load(root)
                _require(report == captured and digest == expected and split.payload == snapshot.split.payload,
                         'Run changed during verification')
                _require(len(report['queries']) == setting['queries'] and report['training_calls'] == setting['calls']
                         and report['seed_policy'] == control['policy'] and report['random_seed'] == seed
                         and report['training_options']['seed_distribution'] == control['distribution']
                         and all(report[key] == setting[key] for key in
                             ('sources', 'source_sha256', 'final_poses_sha256', 'preparer_sha256', 'selector_runtime_sha256')),
                         'Run differs from frozen controls')
                config = protocol['configs'][scene+'-'+control['distribution']]
                _require(normalized_training_options(report) == config['options'] and report['config_sha256'] == config['sha256'],
                         'Run configuration differs from protocol')
                _require(_same_configuration(report['selection_plan'], protocol['selection_plans'][scene])
                         and _same_configuration(report['software'], protocol['software'])
                         and report['selection_policy'] == 'spatial'
                         and report['selection_semantics'] == 'frozen_reference_online_models',
                         'Run selection plan or software differs from protocol')
                for relative, item in report['artifacts'].items():
                    capture_file(_path(root, relative), receipts, expected=item)
                for filename, key in (('config.yaml', 'config_sha256'), ('poses.f64le', 'final_poses_sha256')):
                    _digest_file(root/filename, report[key], receipts)
                inputs['runs'][name] = dict(root=root, scene=scene, condition=condition, seed=seed, report=report, split=split)
        # Selection/calibration/optimizer budgets stay fixed across all eight results.
        anchor = inputs['runs'][f'{scene}-flat-uniform-0']['report']
        controls = tuple(key for key in _CONTROLS if key not in ('random_seed', 'config_sha256', 'training_options'))
        for run in (run for run in inputs['runs'].values() if run['scene'] == scene):
            report = run['report']
            _check_training(report['groups'], anchor, report)
            _require(all(report[key] == anchor[key] for key in controls), 'Uncontrolled scene comparison')
            options = normalized_training_options(report)
            options.pop('seed_distribution')
            baseline = normalized_training_options(anchor)
            baseline.pop('seed_distribution')
            _require(options == baseline, 'Other optimizer options differ')
        flat = verify_configuration(study/f'{scene}-flat-uniform-0', anchor)
        stratified = verify_configuration(study/f'{scene}-stratified-uniform-0', inputs['runs'][f'{scene}-stratified-uniform-0']['report'])
        flat['gaussian']['worker'].pop('seed_distribution', None)
        stratified['gaussian']['worker'].pop('seed_distribution', None)
        _require(_same_configuration(flat, stratified), 'Other YAML controls differ')
    inputs['stamps'] = {name: _stamp(Path(name)) for name in receipts}
    for snapshot in inputs['snapshots'].values():
        inputs['stamps'].update(snapshot.fingerprint['stamps'])
    verify_input_stamps(inputs)
    return inputs


def verify_input_stamps(inputs, outputs=None):
    """Fast identity pass after long hashing; not a filesystem transaction."""
    for stamps in (inputs['stamps'], outputs or {}):
        for name, expected in stamps.items():
            _require(_stamp(Path(name)) == expected, 'Study file identity changed: '+name)


def verify_inputs(inputs):
    for snapshot in inputs.get('snapshots', {}).values():
        verify_evaluation_snapshot(snapshot)
    for name, receipt in inputs['receipts'].items():
        capture_file(Path(name), {}, max(receipt['bytes'], 1), expected=receipt)
    if 'stamps' in inputs:
        verify_input_stamps(inputs)
