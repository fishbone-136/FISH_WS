"""Shared LiDAR setup for online SLAM and saved-map localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def perception_arguments():
    return [
        DeclareLaunchArgument("lidar_source", default_value="unitree", choices=["unitree", "hap"]),
        DeclareLaunchArgument("start_hap_driver", default_value="true"),
        DeclareLaunchArgument("hap_config_path", default_value=""),
        DeclareLaunchArgument("hap_publish_freq", default_value="10.0"),
        DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
        DeclareLaunchArgument("hap_x", default_value="0.14"),
        DeclareLaunchArgument("hap_y", default_value="0.0"),
        DeclareLaunchArgument("hap_z", default_value="0.13"),
        DeclareLaunchArgument("hap_roll", default_value="0.0"),
        DeclareLaunchArgument("hap_pitch", default_value="0.0"),
        DeclareLaunchArgument("hap_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_best_effort", default_value="true"),
        DeclareLaunchArgument("accumulation_frames", default_value="1"),
        DeclareLaunchArgument("cloud_min_height", default_value="-0.33"),
        DeclareLaunchArgument("cloud_max_height", default_value="0.37"),
        DeclareLaunchArgument("fixed_cloud_frame", default_value="livox_frame"),
        DeclareLaunchArgument("max_history_age", default_value="0.12"),
    ]


def launch_planar_perception(context):
    config = LaunchConfiguration
    share = Path(get_package_share_directory("go2_perception")) / "launch"
    use_sim_time = config("use_sim_time")
    base_frame = config("base_frame")
    projection = {
        "use_sim_time": use_sim_time,
        "target_frame": base_frame,
        "use_best_effort": config("use_best_effort"),
    }
    if config("lidar_source").perform(context) == "unitree":
        sensor = Node(
            package="go2_perception", executable="unitree_navigation_source.py",
            name="unitree_navigation_source", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "planar_frame": base_frame}],
            additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
        )
        projection.update({
            "projection_cloud_topic": "/unitree/navigation_cloud_planar",
            "enable_accumulation": "false",
            "ground_filter_enabled": "true",
            "require_ground_for_scan": "true",
            "observed_free_space_only": "true",
        })
    else:
        hap = {
            "start_driver": config("start_hap_driver"),
            "publish_freq": config("hap_publish_freq"),
            "base_frame": base_frame,
            "lidar_frame": config("lidar_frame"),
        }
        for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
            hap[axis] = config(f"hap_{axis}")
        config_path = config("hap_config_path").perform(context)
        if config_path:
            hap["config_path"] = config_path
        sensor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "livox_hap.launch.py")),
            launch_arguments=hap.items(),
        )
        for name in ("cloud_topic", "accumulation_frames", "cloud_min_height",
                     "cloud_max_height", "fixed_cloud_frame", "max_history_age"):
            projection[name] = config(name)
    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "go2_pointcloud_process.launch.py")),
        launch_arguments=projection.items(),
    )
    return [sensor, TimerAction(period=1.0, actions=[scan])]
