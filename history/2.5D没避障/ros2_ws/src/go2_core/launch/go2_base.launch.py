import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'video_enable',
        default_value='false',
        description='是否启用视频功能'
    ))

    ld.add_action(DeclareLaunchArgument(
        'tcp_enable',
        default_value='false',
        description='是否启用TCP转发功能'
    ))

    ld.add_action(DeclareLaunchArgument(
        'tcp_host',
        default_value='127.0.0.1',
        description='TCP服务器主机地址'
    ))

    ld.add_action(DeclareLaunchArgument(
        'tcp_port',
        default_value='5432',
        description='TCP服务器端口'
    ))

    ld.add_action(DeclareLaunchArgument(
        'target_fps',
        default_value='30',
        description='目标帧率'
    ))

    ld.add_action(DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='图像发布话题'
    ))

    ld.add_action(DeclareLaunchArgument(
        'multicast_iface',
        default_value=os.environ.get('GO2_NETWORK_INTERFACE', 'eth0'),
        description='Go2 multicast video network interface'
    ))

    for name, default, description in (
        ('odom_source', 'sport_mode_state', 'sport_mode_state, pose, or none'),
        ('pose_topic', '/utlidar/robot_pose', 'Legacy pose fallback; unused by default'),
        ('sport_state_topic', '/lf/sportmodestate', 'Go2 leg/IMU estimator state topic'),
        ('cmd_vel_topic', '/cmd_vel', 'Velocity command topic'),
        ('interactive_command_topic', '/go2/interactive_command', 'Voice/gesture command topic'),
        ('interactive_status_topic', '/go2/interactive_status', 'Voice/gesture command ACK topic'),
        ('odom_topic', '/odom', 'Odometry output topic'),
        ('odom_frame', 'odom', 'Odometry frame'),
        ('base_frame', 'base_link', 'Robot base frame'),
        ('cmd_vel_timeout', '0.5', 'Stop watchdog timeout in seconds'),
        ('localization_guard_enabled', 'false', 'Block motion while external localization is invalid'),
        ('localization_valid_topic', '/lio/localization_valid', 'External localization health topic'),
        ('cmd_vel_deadband', '0.005', 'Velocity values below this magnitude are zero'),
        ('max_linear_command', '0.70', 'Maximum absolute forward velocity sent to Go2'),
        ('max_lateral_command', '0.30', 'Maximum absolute lateral velocity sent to Go2'),
        ('max_angular_command', '1.00', 'Maximum absolute yaw velocity sent to Go2'),
        ('min_pure_rotation_command', '0.0',
         'Legacy yaw boost; zero preserves collision-checked controller ramps'),
        ('planar_odometry', 'true', 'Remove gait roll/pitch/z from the 2D Nav2 TF'),
        ('max_pose_speed', '1.8', 'Reject short odometry translation spikes'),
        ('max_pose_angular_speed', '2.5', 'Reject short odometry yaw spikes'),
        ('pose_jump_check_window', '1.0', 'Maximum gap for pose discontinuity checks'),
        ('twist_filter_alpha', '0.35', 'Low-pass factor for measured odometry twist'),
        ('state_stationary_linear_threshold', '0.04', 'Sport state stillness linear speed'),
        ('state_stationary_angular_threshold', '0.06', 'Sport state stillness yaw rate'),
        ('state_stationary_position_delta', '0.0035', 'Sport state stillness position delta'),
        ('state_stationary_yaw_delta', '0.0045', 'Sport state stillness yaw delta'),
        ('motion_feedback_topic', '/go2/motion_execution_valid',
         'Whether the Go2 state confirms execution of active motion commands'),
        ('motion_feedback_timeout', '3.0',
         'Seconds of meaningful command without measured motion before reporting failure'),
        ('motion_feedback_stale_timeout', '1.0', 'Sport state freshness limit'),
        ('motion_feedback_command_linear', '0.15', 'Linear command checked for execution'),
        ('motion_feedback_command_angular', '0.35', 'Yaw command checked for execution'),
        ('motion_feedback_min_linear', '0.03', 'Measured linear motion threshold'),
        ('motion_feedback_min_angular', '0.05', 'Measured yaw motion threshold'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    go2_base_node = Node(
        package='go2_core',
        executable='go2_base',
        name='go2_base',
        output='screen',
        parameters=[{
            'odom_source': LaunchConfiguration('odom_source'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'sport_state_topic': LaunchConfiguration('sport_state_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'interactive_command_topic': LaunchConfiguration('interactive_command_topic'),
            'interactive_status_topic': LaunchConfiguration('interactive_status_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            'localization_guard_enabled': LaunchConfiguration('localization_guard_enabled'),
            'localization_valid_topic': LaunchConfiguration('localization_valid_topic'),
            'cmd_vel_deadband': LaunchConfiguration('cmd_vel_deadband'),
            'max_linear_command': LaunchConfiguration('max_linear_command'),
            'max_lateral_command': LaunchConfiguration('max_lateral_command'),
            'max_angular_command': LaunchConfiguration('max_angular_command'),
            'min_pure_rotation_command': LaunchConfiguration(
                'min_pure_rotation_command'),
            'planar_odometry': LaunchConfiguration('planar_odometry'),
            'max_pose_speed': LaunchConfiguration('max_pose_speed'),
            'max_pose_angular_speed': LaunchConfiguration('max_pose_angular_speed'),
            'pose_jump_check_window': LaunchConfiguration('pose_jump_check_window'),
            'twist_filter_alpha': LaunchConfiguration('twist_filter_alpha'),
            'state_stationary_linear_threshold': LaunchConfiguration(
                'state_stationary_linear_threshold'),
            'state_stationary_angular_threshold': LaunchConfiguration(
                'state_stationary_angular_threshold'),
            'state_stationary_position_delta': LaunchConfiguration(
                'state_stationary_position_delta'),
            'state_stationary_yaw_delta': LaunchConfiguration(
                'state_stationary_yaw_delta'),
            'motion_feedback_topic': LaunchConfiguration('motion_feedback_topic'),
            'motion_feedback_timeout': LaunchConfiguration('motion_feedback_timeout'),
            'motion_feedback_stale_timeout': LaunchConfiguration(
                'motion_feedback_stale_timeout'),
            'motion_feedback_command_linear': LaunchConfiguration(
                'motion_feedback_command_linear'),
            'motion_feedback_command_angular': LaunchConfiguration(
                'motion_feedback_command_angular'),
            'motion_feedback_min_linear': LaunchConfiguration('motion_feedback_min_linear'),
            'motion_feedback_min_angular': LaunchConfiguration('motion_feedback_min_angular'),
        }]
    )

    video_stream_node = Node(
        package='go2_core',
        executable='video_stream_node.py',
        name='video_stream_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('video_enable')),
        parameters=[{
            'tcp_enable': LaunchConfiguration('tcp_enable'),
            'tcp_host': LaunchConfiguration('tcp_host'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'target_fps': LaunchConfiguration('target_fps'),
            'image_topic': LaunchConfiguration('image_topic'),
            'multicast_iface': LaunchConfiguration('multicast_iface'),
        }]
    )

    ld.add_action(go2_base_node)
    ld.add_action(video_stream_node)

    return ld
