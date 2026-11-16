"""F-version point-cloud conversion with a short TF wait queue."""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument


def parameter(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("accumulated_cloud_topic", default_value="/trans_cloud_f"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("use_best_effort", default_value="true"),
        DeclareLaunchArgument("accumulation_frames", default_value="1"),
        DeclareLaunchArgument("cloud_min_height", default_value="-0.33"),
        DeclareLaunchArgument("cloud_max_height", default_value="0.37"),
        DeclareLaunchArgument("scan_min_height", default_value="-2.00"),
        DeclareLaunchArgument("scan_max_height", default_value="2.00"),
        DeclareLaunchArgument("fixed_cloud_frame", default_value="livox_frame"),
        DeclareLaunchArgument("max_history_age", default_value="0.12"),
        DeclareLaunchArgument("tf_queue_size", default_value="3"),
        Node(
            package="go2_perception",
            executable="cloud_accumulation",
            name="cloud_accumulation_f",
            output="screen",
            remappings=[
                ("cloud_in", LaunchConfiguration("cloud_topic")),
                ("cloud", LaunchConfiguration("accumulated_cloud_topic")),
            ],
            parameters=[{
                "accumulation_frames": parameter("accumulation_frames", int),
                "min_height": parameter("cloud_min_height", float),
                "max_height": parameter("cloud_max_height", float),
                "fixed_frame": LaunchConfiguration("fixed_cloud_frame"),
                "max_history_age": parameter("max_history_age", float),
                "use_best_effort": parameter("use_best_effort", bool),
            }],
        ),
        Node(
            package="go2_perception",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_node_f",
            output="screen",
            remappings=[
                ("cloud_in", LaunchConfiguration("accumulated_cloud_topic")),
                ("scan", LaunchConfiguration("scan_topic")),
            ],
            parameters=[{
                "target_frame": LaunchConfiguration("target_frame"),
                # Three frames cover the observed occasional TF lag without
                # accumulating an unbounded stale-data backlog.
                "transform_tolerance": 0.0,
                "queue_size": parameter("tf_queue_size", int),
                "tf_buffer_duration": 5.0,
                "use_best_effort": parameter("use_best_effort", bool),
                "min_height": parameter("scan_min_height", float),
                "max_height": parameter("scan_max_height", float),
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.00698,
                "scan_time": 0.10,
                "range_min": 0.25,
                # Costmaps retain the complete obstacle range. SLAM applies its
                # shorter 6.5 m range independently in mapper_params_f.yaml.
                "range_max": 12.0,
                "point_spread_radius": 0.04,
                "max_spread_bins": 3,
                "max_fill_gap_bins": 3,
                "max_fill_range_difference": 0.20,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }],
        ),
    ])
