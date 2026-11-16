"""Go2 已有地图定位 + Nav2（ROS 2 Humble）。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from go2_navigation.terrain_profiles import (
    outdoor_nav2_rewrites,
    uses_rough_terrain_navigation,
)


def _include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def _include_bringup(context, nav2_share, nav_to_pose_bt):
    rewrites = {"default_nav_to_pose_bt_xml": nav_to_pose_bt}
    if uses_rough_terrain_navigation(
        LaunchConfiguration("terrain_mode").perform(context)
    ):
        rewrites.update(outdoor_nav2_rewrites())

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites=rewrites,
        convert_types=True,
    )
    return [TimerAction(period=2.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": configured_params,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )])]


def _include_perception(context):
    del context
    perception = _include("go2_perception", "go2_pointcloud_process.launch.py", {
        "cloud_topic": LaunchConfiguration("cloud_topic"),
        "target_frame": LaunchConfiguration("base_frame"),
        "use_best_effort": LaunchConfiguration("use_best_effort"),
        "accumulation_frames": LaunchConfiguration("accumulation_frames"),
        "cloud_min_height": LaunchConfiguration("cloud_min_height"),
        "cloud_max_height": LaunchConfiguration("cloud_max_height"),
        "fixed_cloud_frame": LaunchConfiguration("fixed_cloud_frame"),
        "max_history_age": LaunchConfiguration("max_history_age"),
    })
    return [TimerAction(period=1.0, actions=[perception])]


def generate_launch_description():
    core_share = get_package_share_directory("go2_core")
    nav_share = get_package_share_directory("go2_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_rviz = LaunchConfiguration("use_rviz")
    nav_to_pose_bt = os.path.join(
        nav_share, "config", "navigate_w_replanning_and_recovery.xml")
    terrain_mode = LaunchConfiguration("terrain_mode")

    hap = _include("go2_perception", "livox_hap.launch.py", {
        "start_driver": LaunchConfiguration("start_hap_driver"),
        "config_path": LaunchConfiguration("hap_config_path"),
        "publish_freq": LaunchConfiguration("hap_publish_freq"),
        "base_frame": LaunchConfiguration("base_frame"),
        "lidar_frame": LaunchConfiguration("lidar_frame"),
        "x": LaunchConfiguration("hap_x"),
        "y": LaunchConfiguration("hap_y"),
        "z": LaunchConfiguration("hap_z"),
        "roll": LaunchConfiguration("hap_roll"),
        "pitch": LaunchConfiguration("hap_pitch"),
        "yaw": LaunchConfiguration("hap_yaw"),
    })

    base = _include("go2_core", "go2_base.launch.py", {
        "video_enable": LaunchConfiguration("video_enable"),
        "cmd_vel_timeout": LaunchConfiguration("cmd_vel_timeout"),
        "max_angular_command": "1.2",
        "odom_source": "sport_mode_state",
        "sport_state_topic": LaunchConfiguration("sport_state_topic"),
        "base_frame": LaunchConfiguration("base_frame"),
    })
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(core_share, "config", "default.rviz")],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="", description="Absolute map YAML path"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("video_enable", default_value="false"),
        DeclareLaunchArgument("start_hap_driver", default_value="true"),
        DeclareLaunchArgument(
            "hap_config_path",
            default_value=os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "config", "HAP_config.json"),
        ),
        DeclareLaunchArgument("hap_publish_freq", default_value="10.0"),
        DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("sport_state_topic", default_value="/lf/sportmodestate"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
        DeclareLaunchArgument("hap_x", default_value="0.14"),
        DeclareLaunchArgument("hap_y", default_value="0.0"),
        DeclareLaunchArgument("hap_z", default_value="0.13"),
        DeclareLaunchArgument("hap_roll", default_value="0.0"),
        DeclareLaunchArgument("hap_pitch", default_value="0.0"),
        DeclareLaunchArgument("hap_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_best_effort", default_value="true"),
        DeclareLaunchArgument(
            "terrain_mode", default_value="indoor",
            choices=["indoor", "outdoor", "gravel"],
            description="indoor, outdoor hard surface, or gravel/grass rough terrain",
        ),
        DeclareLaunchArgument(
            "accumulation_frames",
            default_value="1",
        ),
        DeclareLaunchArgument(
            "cloud_min_height",
            default_value="-0.33",
            description="Minimum HAP-frame z included in the 2-D scan",
        ),
        DeclareLaunchArgument("cloud_max_height", default_value="0.37"),
        DeclareLaunchArgument("fixed_cloud_frame", default_value="livox_frame"),
        DeclareLaunchArgument("max_history_age", default_value="0.12"),
        DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
        hap,
        base,
        OpaqueFunction(function=_include_perception),
        OpaqueFunction(function=_include_bringup, args=[nav2_share, nav_to_pose_bt]),
        rviz,
    ])
