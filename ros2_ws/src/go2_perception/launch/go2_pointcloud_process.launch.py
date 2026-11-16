from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('accumulated_cloud_topic', default_value='/trans_cloud'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('obstacle_points_topic', default_value='/obstacle_points'),
        DeclareLaunchArgument('ground_plane_topic', default_value='/perception/ground_plane'),
        DeclareLaunchArgument('ground_pose_topic',default_value=''),
        DeclareLaunchArgument('projection_node_name', default_value='pointcloud_to_laserscan_node'),
        DeclareLaunchArgument('scan_period', default_value='0.0666666667'),
        DeclareLaunchArgument('target_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_frame', default_value=''),
        DeclareLaunchArgument('level_scan', default_value='false'),
        DeclareLaunchArgument('use_best_effort', default_value='true'),
        # HAP 单帧点数足够用于 2-D 扫描。旋转时叠加传感器坐标系中的历史帧
        # 会产生拖影，因此默认只使用最新帧。
        DeclareLaunchArgument('accumulation_frames', default_value='1'),
        # 经过实机验证的 HAP 扫描切片，数值位于 livox_frame。该范围保留
        # 矮障碍并排除大部分地面与高处回波。
        DeclareLaunchArgument('cloud_min_height', default_value='-0.33'),
        DeclareLaunchArgument('cloud_max_height', default_value='0.37'),
        DeclareLaunchArgument('cloud_min_range', default_value='0.25'),
        DeclareLaunchArgument('cloud_max_range', default_value='5.50'),
        # cloud_accumulation 已做 HAP 高度切片；转换到 base_link 后不重复裁剪。
        DeclareLaunchArgument('scan_min_height', default_value='-2.00'),
        DeclareLaunchArgument('scan_max_height', default_value='2.00'),
        DeclareLaunchArgument('fixed_cloud_frame', default_value='livox_frame'),
        DeclareLaunchArgument('max_history_age', default_value='0.12'),
        DeclareLaunchArgument('enable_accumulation', default_value='true'),
        DeclareLaunchArgument(
            'projection_cloud_topic',
            default_value=LaunchConfiguration('accumulated_cloud_topic')),
        DeclareLaunchArgument('ground_filter_enabled', default_value='false'),
        DeclareLaunchArgument('observed_free_space_only', default_value='false'),
        Node(
            package='go2_perception',
            executable='cloud_accumulation',
            condition=IfCondition(LaunchConfiguration('enable_accumulation')),
            remappings=[
                ('cloud_in', LaunchConfiguration('cloud_topic')),
                ('cloud', LaunchConfiguration('accumulated_cloud_topic')),
            ],
            parameters=[{
                'accumulation_frames': LaunchConfiguration('accumulation_frames'),
                'min_height': LaunchConfiguration('cloud_min_height'),
                'max_height': LaunchConfiguration('cloud_max_height'),
                'min_range': LaunchConfiguration('cloud_min_range'),
                'max_range': LaunchConfiguration('cloud_max_range'),
                'fixed_frame': LaunchConfiguration('fixed_cloud_frame'),
                'max_history_age': LaunchConfiguration('max_history_age'),
                'use_best_effort': LaunchConfiguration('use_best_effort'),
            }],
            name='cloud_accumulation',
            output='screen',
        ),
        Node(
            package='go2_perception',
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', LaunchConfiguration('projection_cloud_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
                ('obstacle_points', LaunchConfiguration('obstacle_points_topic')),
                ('ground_plane',LaunchConfiguration('ground_plane_topic')),
            ],
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),
                'scan_frame': LaunchConfiguration('scan_frame'),
                'level_scan': LaunchConfiguration('level_scan'),
                # MessageFilter tolerance requires a transform in the future;
                # 0.5 s therefore delayed every scan. Exact sensor timestamps
                # now make a future allowance unnecessary.
                'transform_tolerance': 0.0,
                # 只等待最新扫描，CPU 短时繁忙时不追赶已经过期的点云。
                'queue_size': 1,
                'tf_buffer_duration': 5.0,
                'use_best_effort': LaunchConfiguration('use_best_effort'),
                'min_height': LaunchConfiguration('scan_min_height'),
                'max_height': LaunchConfiguration('scan_max_height'),
                'ground_filter_enabled': LaunchConfiguration('ground_filter_enabled'),
                'ground_pose_topic': LaunchConfiguration('ground_pose_topic'),
                'observed_free_space_only': LaunchConfiguration('observed_free_space_only'),
                'ground_clearance': 0.10,
                'obstacle_height_above_ground': 1.3,
                'angle_min': -3.14,
                'angle_max': 3.14,
                'angle_increment': 0.00698,
                'scan_time': LaunchConfiguration('scan_period'),
                'range_min': 0.25,
                # Match the Nav2 raytrace envelope. Longer points only clutter
                # RViz and cannot enter the 10 m rolling local costmap.
                'range_max': LaunchConfiguration('cloud_max_range'),
                # 单帧内把稀疏 3-D 回波恢复为连续障碍面；不使用历史帧，
                # 因此不会在旋转时制造拖影。
                'point_spread_radius': 0.025,
                'max_spread_bins': 2,
                'max_fill_gap_bins': 2,
                'max_fill_range_difference': 0.12,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name=LaunchConfiguration('projection_node_name'),
            output='screen',
        )
    ])
