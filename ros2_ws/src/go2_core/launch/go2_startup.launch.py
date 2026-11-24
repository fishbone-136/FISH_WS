"""Backward-compatible entry point: Go2 online mapping + Nav2 on Humble."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    terrain_mode = LaunchConfiguration("terrain_mode")
    mapping_launch = os.path.join(
        get_package_share_directory("go2_navigation"),
        "launch",
        "go2_mapping.launch.py",
    )
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "video_enable": LaunchConfiguration("video_enable"),
            "lidar_source": LaunchConfiguration("lidar_source"),
            "start_hap_driver": LaunchConfiguration("start_hap_driver"),
            "use_best_effort": LaunchConfiguration("use_best_effort"),
            "accumulation_frames": LaunchConfiguration("accumulation_frames"),
            "cloud_min_height": LaunchConfiguration("cloud_min_height"),
            "fixed_cloud_frame": LaunchConfiguration("fixed_cloud_frame"),
            "terrain_mode": terrain_mode,
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("video_enable", default_value="false"),
        DeclareLaunchArgument("lidar_source", default_value="unitree", choices=["unitree", "hap"]),
        DeclareLaunchArgument("start_hap_driver", default_value="true"),
        DeclareLaunchArgument("use_best_effort", default_value="true"),
        DeclareLaunchArgument(
            "terrain_mode", default_value="indoor",
            choices=["indoor", "outdoor", "gravel"]),
        DeclareLaunchArgument(
            "accumulation_frames",
            default_value="1",
            description="Latest cloud only prevents rotational scan smear",
        ),
        DeclareLaunchArgument(
            "cloud_min_height",
            default_value="-0.33",
            description="Minimum HAP-frame z included in the 2-D scan",
        ),
        DeclareLaunchArgument("fixed_cloud_frame", default_value="livox_frame"),
        include,
    ])
