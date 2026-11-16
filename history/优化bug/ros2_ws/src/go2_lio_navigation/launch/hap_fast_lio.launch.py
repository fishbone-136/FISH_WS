"""Low-load HAP + FAST-LIO launch used for front-end validation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    main_launch = os.path.join(
        get_package_share_directory("go2_lio_navigation"),
        "launch",
        "go2_lio_navigation.launch.py",
    )
    front_end = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_hap_driver": LaunchConfiguration("start_hap_driver"),
            "hap_config_path": LaunchConfiguration("hap_config_path"),
            "hap_publish_freq": LaunchConfiguration("hap_publish_freq"),
            "hap_x": LaunchConfiguration("hap_x"),
            "hap_y": LaunchConfiguration("hap_y"),
            "hap_z": LaunchConfiguration("hap_z"),
            "hap_roll": LaunchConfiguration("hap_roll"),
            "hap_pitch": LaunchConfiguration("hap_pitch"),
            "hap_yaw": LaunchConfiguration("hap_yaw"),
            "enable_loop_closure": "false",
            "enable_go2_base": "false",
            "enable_perception": "false",
            "enable_terrain": "false",
            "enable_nav2": "false",
            "video_enable": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("start_hap_driver", default_value="true"),
        DeclareLaunchArgument("hap_publish_freq", default_value="15.0"),
        DeclareLaunchArgument(
            "hap_config_path",
            default_value=os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "config",
                "HAP_config.json",
            ),
        ),
        DeclareLaunchArgument("hap_x", default_value="0.14"),
        DeclareLaunchArgument("hap_y", default_value="0.0"),
        DeclareLaunchArgument("hap_z", default_value="0.13"),
        DeclareLaunchArgument("hap_roll", default_value="0.0"),
        DeclareLaunchArgument("hap_pitch", default_value="0.0"),
        DeclareLaunchArgument("hap_yaw", default_value="0.0"),
        front_end,
    ])
