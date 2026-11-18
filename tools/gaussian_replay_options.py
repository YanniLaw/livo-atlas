"""Shared Gaussian replay options for the Noetic and Humble runners."""
import argparse
import os
from pathlib import Path


def nonempty_path(value):
    if not value.strip():
        raise argparse.ArgumentTypeError('path must not be empty')
    return Path(value)


def add_gaussian_arguments(parser):
    parser.add_argument('--gaussian-socket', type=nonempty_path,
                        help='Socket of an already started online Gaussian worker')
    parser.add_argument('--gaussian-output', type=nonempty_path,
                        help='New directory for online Gaussian previews')
    parser.add_argument('--gaussian-persist-submaps', action='store_true',
                        help='Persist online Gaussian submaps; requires socket/output and uses a 30 s shutdown')
    parser.add_argument('--gaussian-finalize-on-shutdown', action='store_true',
                        help='Repair confirmed submaps after the backend finishes; requires persistence')
    parser.add_argument('--gaussian-finalize-timeout-ms', type=int, default=60000,
                        help='Final repair budget in milliseconds (1..600000)')
    parser.add_argument('--gaussian-joint-render', action='store_true',
                        help='Render confirmed adjacent Gaussian blocks during playback')
    parser.add_argument('--gaussian-joint-render-interval-ms', type=int, default=1000)
    parser.add_argument('--gaussian-joint-render-timeout-ms', type=int, default=5000)


def validate_gaussian_arguments(args, parser):
    if args.gaussian_joint_render and not args.gaussian_persist_submaps:
        parser.error('--gaussian-joint-render requires --gaussian-persist-submaps')
    if not 100 <= args.gaussian_joint_render_interval_ms <= 60000:
        parser.error('--gaussian-joint-render-interval-ms must be in 100..60000')
    if not 1 <= args.gaussian_joint_render_timeout_ms <= 60000:
        parser.error('--gaussian-joint-render-timeout-ms must be in 1..60000')
    if args.gaussian_finalize_on_shutdown and not args.gaussian_persist_submaps:
        parser.error('--gaussian-finalize-on-shutdown requires --gaussian-persist-submaps')
    if not 1 <= args.gaussian_finalize_timeout_ms <= 600000:
        parser.error('--gaussian-finalize-timeout-ms must be in 1..600000')
    if args.gaussian_persist_submaps and not args.gaussian_socket:
        parser.error('--gaussian-persist-submaps requires --gaussian-socket and --gaussian-output')
    if bool(args.gaussian_socket) != bool(args.gaussian_output):
        parser.error('--gaussian-socket and --gaussian-output must be provided together')
    if args.gaussian_socket:
        if args.submap_mode == 'keyframe_groups':
            parser.error('--gaussian-socket requires online submaps')
        if len(os.fsencode(args.gaussian_socket.absolute())) > 107:
            parser.error('--gaussian-socket path is too long')
        if os.path.lexists(args.gaussian_output):
            parser.error('--gaussian-output must not already exist')
        gaussian_output = args.gaussian_output.resolve()
        baseline_output = args.output.resolve()
        if gaussian_output == baseline_output or gaussian_output in baseline_output.parents:
            parser.error('--gaussian-output must not contain the baseline output')
        dataset = baseline_output / 'node_output/keyframes'
        if gaussian_output == dataset or dataset in gaussian_output.parents:
            parser.error('--gaussian-output must be separate from the keyframe dataset')


def gaussian_parameters(args):
    if not args.gaussian_socket:
        return {}
    parameters = {'gaussian.enabled': True,
        'gaussian.socket_path': str(args.gaussian_socket.resolve()),
        'gaussian.output_directory': str(args.gaussian_output.resolve()),
        'gaussian.persist_submaps': args.gaussian_persist_submaps,
        'gaussian.finalize_on_shutdown': args.gaussian_finalize_on_shutdown,
        'gaussian.finalize_timeout_ms': args.gaussian_finalize_timeout_ms,
        'gaussian.joint_render_enabled': args.gaussian_joint_render,
        'gaussian.joint_render_interval_ms': args.gaussian_joint_render_interval_ms,
        'gaussian.joint_render_timeout_ms': args.gaussian_joint_render_timeout_ms}
    if args.gaussian_persist_submaps:
        parameters['gaussian.shutdown_timeout_ms'] = 30000
    return parameters
