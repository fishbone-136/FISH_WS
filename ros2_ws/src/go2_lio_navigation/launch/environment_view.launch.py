"""Attach the persistent map/view to an already running LIO session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('go2_lio_navigation')
    mapper = Node(
        package='go2_lio_navigation', executable='environment_mapper.py',
        name='environment_mapper', output='screen',
        parameters=[os.path.join(share, 'config', 'environment_mapping.yaml'), {
            'parent_launch_pid': ParameterValue(
                LaunchConfiguration('parent_launch_pid'), value_type=int),
        }],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
    )
    view = Node(
        package='rviz2', executable='rviz2', name='environment_rviz', output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )
    terrain_display = Node(
        package='go2_lio_navigation', executable='environment_terrain_display.py',
        name='environment_terrain_display', output='screen',
        parameters=[{'display_resolution': .175}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('parent_launch_pid', default_value='0'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'config', 'lio_environment.rviz')),
        RegisterEventHandler(OnProcessExit(
            target_action=mapper,
            on_exit=[EmitEvent(event=Shutdown(reason='Environment mapper stopped'))],
        )),
        mapper, terrain_display, view,
    ])
