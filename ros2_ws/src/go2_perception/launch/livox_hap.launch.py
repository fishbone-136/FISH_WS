"""Livox HAP driver and the calibrated Go2-to-HAP static transform."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def typed(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_driver", default_value="true"),
        DeclareLaunchArgument(
            "config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("livox_ros_driver2"),
                "config",
                "HAP_config.json",
            ]),
        ),
        DeclareLaunchArgument("publish_freq", default_value="20.0"),
        DeclareLaunchArgument(
            "xfer_format",
            default_value="0",
            description=(
                "0=PointCloud2 (used by this workspace's timed HAP parser), "
                "1=Livox CustomMsg"
            ),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
        # Calibrated/measured mounting pose from the validated HAP navigation
        # workspace on this robot.  Keep these overridable after remounting.
        DeclareLaunchArgument("x", default_value="0.14"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.13"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_driver")),
            parameters=[{
                "xfer_format": typed("xfer_format", int),
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": typed("publish_freq", float),
                "output_data_type": 0,
                "frame_id": LaunchConfiguration("lidar_frame"),
                "lvx_file_path": "/tmp/go2_unused.lvx",
                "user_config_path": LaunchConfiguration("config_path"),
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_livox_tf",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("x"),
                "--y", LaunchConfiguration("y"),
                "--z", LaunchConfiguration("z"),
                "--roll", LaunchConfiguration("roll"),
                "--pitch", LaunchConfiguration("pitch"),
                "--yaw", LaunchConfiguration("yaw"),
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", LaunchConfiguration("lidar_frame"),
            ],
        ),
    ])
