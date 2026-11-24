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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from go2_navigation.planar_perception import launch_planar_perception, perception_arguments

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


def generate_launch_description():
    core_share = get_package_share_directory("go2_core")
    nav_share = get_package_share_directory("go2_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_rviz = LaunchConfiguration("use_rviz")
    nav_to_pose_bt = os.path.join(
        nav_share, "config", "navigate_w_replanning_and_recovery.xml")
    terrain_mode = LaunchConfiguration("terrain_mode")

    base = _include("go2_core", "go2_base.launch.py", {
        "video_enable": LaunchConfiguration("video_enable"),
        "cmd_vel_timeout": LaunchConfiguration("cmd_vel_timeout"),
        "max_angular_command": "1.2",
        "odom_source": "sport_mode_state",
        "localization_guard_enabled": PythonExpression([
            "'", LaunchConfiguration("lidar_source"), "' == 'unitree'"]),
        "localization_valid_topic": "/unitree/navigation_source/valid",
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
        *perception_arguments(),
        DeclareLaunchArgument("sport_state_topic", default_value="/lf/sportmodestate"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "terrain_mode", default_value="indoor",
            choices=["indoor", "outdoor", "gravel"],
            description="indoor, outdoor hard surface, or gravel/grass rough terrain",
        ),
        DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
        base,
        OpaqueFunction(function=launch_planar_perception),
        OpaqueFunction(function=_include_bringup, args=[nav2_share, nav_to_pose_bt]),
        rviz,
    ])
