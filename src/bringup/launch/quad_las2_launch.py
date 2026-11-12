"""Packed-camera VB input with four independent GPU/EP stereo pipelines."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_config = str(Path(get_package_share_directory('camera_io')) /
                        'config' / 'seeker_omni_depth.json')
    declarations = [
        DeclareLaunchArgument(
            'calibration_file',
            description='External Kalibr four-camera calibration; the stereo 0.yaml is not compatible'),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('stereo_matching_library'),
        DeclareLaunchArgument('las2_library_dir'),
        DeclareLaunchArgument('camera_config_file', default_value=camera_config),
        DeclareLaunchArgument('input_file', default_value=''),
        DeclareLaunchArgument('worker_cpus', default_value='0-7'),
        DeclareLaunchArgument('overlap', default_value='true'),
        DeclareLaunchArgument('prefer_latest', default_value='true'),
        DeclareLaunchArgument('horizontal_fov_degrees', default_value='100.0'),
        DeclareLaunchArgument('preview', default_value='true'),
        DeclareLaunchArgument('preview_directory', default_value='/dev/shm/quad_preview'),
        DeclareLaunchArgument('preview_bind', default_value='0.0.0.0'),
        DeclareLaunchArgument('preview_port', default_value='8088'),
        DeclareLaunchArgument('preview_fps', default_value='10.0'),
        DeclareLaunchArgument('preview_max_depth', default_value='10.0'),
    ]
    names = ('front', 'right', 'rear', 'left')
    declarations += [DeclareLaunchArgument('crop_' + name + '_ratio', default_value='0.0')
                     for name in names]
    ready_topics = ['/las2/' + name + '/ready' for name in names]
    nodes = [Node(
        package='las2', executable='las2_readiness_barrier',
        parameters=[{'input_topics': ready_topics}], output='screen', sigterm_timeout='3', sigkill_timeout='2'), Node(
        package='camera_io', executable='camera_mpp_publisher',
        name='quad_camera_bridge', output='screen', sigterm_timeout='3', sigkill_timeout='2',
        prefix=['taskset -c ', LaunchConfiguration('worker_cpus')],
        additional_env={'LD_PRELOAD': 'libmpp.so'},
        parameters=[{
            'config_file': LaunchConfiguration('camera_config_file'),
            'input_file': LaunchConfiguration('input_file'),
            'autostart': True, 'file_start_ready_topic': '/las2/quad/ready',
        }])]
    for index, name in enumerate(names):
        nodes.append(Node(
            package='las2', executable='las2_pipeline_node',
            name='las2_' + name, output='screen', sigterm_timeout='3', sigkill_timeout='2',
            prefix=['taskset -c ', LaunchConfiguration('worker_cpus')],
            additional_env={
                'LD_PRELOAD': LaunchConfiguration('stereo_matching_library'),
                'FASTRTPS_DEFAULT_PROFILES_FILE': str(
                    Path(get_package_share_directory('las2')) /
                    'config' / 'quad_descriptor_transport.xml'),
                'LD_LIBRARY_PATH': [LaunchConfiguration('las2_library_dir'), ':',
                                    EnvironmentVariable('LD_LIBRARY_PATH', default_value='')],
            },
            parameters=[{
                'crop_bottom_ratio': ParameterValue(
                    LaunchConfiguration('crop_' + name + '_ratio'), value_type=float),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'model_path': LaunchConfiguration('model_path'),
                'left_camera': index, 'right_camera': (index + 1) % 4,
                'core_affinity': str(8 + 2 * index) + ';' + str(9 + 2 * index),
                'threads_per_instance': 2,
                'output_topic': '/las2/' + name + '/depth_desc',
                'ready_topic': ready_topics[index],
                'overlap': ParameterValue(LaunchConfiguration('overlap'), value_type=bool),
                'prefer_latest': ParameterValue(LaunchConfiguration('prefer_latest'), value_type=bool),
                'horizontal_fov_degrees': ParameterValue(
                    LaunchConfiguration('horizontal_fov_degrees'), value_type=float),
            }]))
    nodes += [Node(
        package='las2', executable='quad_preview_node',
        condition=IfCondition(LaunchConfiguration('preview')),
        prefix=['taskset -c ', LaunchConfiguration('worker_cpus')],
        parameters=[{
            'directory': LaunchConfiguration('preview_directory'),
            'fps': ParameterValue(LaunchConfiguration('preview_fps'), value_type=float),
            'max_depth': ParameterValue(LaunchConfiguration('preview_max_depth'), value_type=float),
        }], output='screen'), Node(
        package='las2', executable='quad_preview_web.py',
        condition=IfCondition(LaunchConfiguration('preview')),
        prefix=['taskset -c ', LaunchConfiguration('worker_cpus')],
        arguments=[
            '--directory', LaunchConfiguration('preview_directory'),
            '--bind', LaunchConfiguration('preview_bind'),
            '--port', LaunchConfiguration('preview_port'),
            '--fps', LaunchConfiguration('preview_fps'),
            '--max-depth', LaunchConfiguration('preview_max_depth'),
            '--crop-ratios',
        ] + [LaunchConfiguration('crop_' + name + '_ratio') for name in names],
        ros_arguments=[], output='screen')]
    return LaunchDescription(declarations + nodes)
