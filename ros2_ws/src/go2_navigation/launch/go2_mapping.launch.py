"""Go2 在线建图 + Nav2（ROS 2 Humble）。"""

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
    outdoor_slam_rewrites,
    uses_rough_terrain_navigation,
)


def _include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def _include_slam(context, use_sim_time, slam_params):
    params = slam_params
    if uses_rough_terrain_navigation(
        LaunchConfiguration("terrain_mode").perform(context)
    ):
        params = RewrittenYaml(
            source_file=slam_params,
            root_key="",
            param_rewrites=outdoor_slam_rewrites(),
            convert_types=True,
        )
    return [TimerAction(period=2.0, actions=[_include("go2_slam", "go2_slamtoolbox.launch.py", {
        "use_sim_time": use_sim_time,
        "params_file": params,
    })])]


def generate_launch_description():
    core_share = get_package_share_directory("go2_core")
    nav_share = get_package_share_directory("go2_navigation")
    slam_share = get_package_share_directory("go2_slam")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    nav_params = LaunchConfiguration("nav_params")
    slam_params = LaunchConfiguration("slam_params")
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
    nav2 = _include("go2_navigation", "go2_nav2.launch.py", {
        "use_sim_time": use_sim_time,
        "params_file": nav_params,
        "autostart": "true",
        "mapping_mode": "true",
        "terrain_mode": terrain_mode,
    })

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[nav_params, {"use_sim_time": use_sim_time}],
    )
    map_saver_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_saver"],
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(core_share, "config", "default.rviz")],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
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
        DeclareLaunchArgument(
            "nav_params",
            default_value=os.path.join(nav_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "slam_params",
            default_value=os.path.join(slam_share, "config", "mapper_params_online_async.yaml"),
        ),
        base,
        OpaqueFunction(function=launch_planar_perception),
        OpaqueFunction(function=_include_slam, args=[use_sim_time, slam_params]),
        TimerAction(period=3.0, actions=[nav2]),
        TimerAction(period=3.0, actions=[map_saver, map_saver_lifecycle]),
        rviz,
    ])
