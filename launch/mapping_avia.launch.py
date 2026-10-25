"""Humble entry point sharing the existing Avia and camera calibration YAML."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_parameters(path):
    """Flatten existing ROS1 YAML; calibration arrays accept mixed YAML numbers."""
    result = {}

    def flatten(value, prefix=''):
        for name, item in value.items():
            key = f'{prefix}.{name}' if prefix else name
            if isinstance(item, dict):
                flatten(item, key)
            else:
                if key.startswith('extrin_calib.') and isinstance(item, list):
                    item = [float(number) for number in item]
                result[key] = item

    with Path(path).open(encoding='utf-8') as stream:
        flatten(yaml.safe_load(stream))
    return result


def launch_node(context):
    parameters = load_parameters(LaunchConfiguration('config').perform(context))
    parameters.update(load_parameters(LaunchConfiguration('camera').perform(context)))
    parameters['qos.reliability'] = LaunchConfiguration('reliability').perform(context)
    parameters['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    directory = LaunchConfiguration('output_directory').perform(context)
    if directory:
        parameters['output.directory'] = directory
    return [Node(package='livo_atlas', executable='fastlivo_mapping',
                 name='laserMapping', output='screen', parameters=[parameters])]


def generate_launch_description():
    share = Path(get_package_share_directory('livo_atlas'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/avia.yaml')),
        DeclareLaunchArgument('camera', default_value=str(share / 'config/camera_pinhole.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('reliability', default_value='best_effort',
                              choices=['best_effort', 'reliable']),
        OpaqueFunction(function=launch_node),
    ])
