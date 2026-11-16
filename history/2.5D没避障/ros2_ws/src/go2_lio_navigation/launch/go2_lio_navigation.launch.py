"""HAP FAST-LIO2 online mapping, loop closure, 2.5-D terrain, and Nav2."""

import fcntl
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from go2_navigation.runtime_guard import conflicting_navigation_processes
from go2_navigation.safety_geometry import collision_monitor_parameters


_instance_lock = None


def acquire_single_instance_lock():
    """Reject a second full stack before duplicate sensor/TF publishers start."""
    global _instance_lock

    # Included launch descriptions may be visited more than once while ROS 2
    # resolves substitutions. Re-entering in the same Python launch process is
    # not a second stack and must reuse the lock already held by this module.
    if _instance_lock is not None:
        return

    # Also catches the currently running instance that predates this lock.
    signature = [
        "ros2", "launch", "go2_lio_navigation",
        "go2_lio_navigation.launch.py",
    ]
    for cmdline_path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            pid = int(cmdline_path.parent.name)
            if pid == os.getpid():
                continue
            argv = [
                part.decode(errors="replace")
                for part in cmdline_path.read_bytes().split(b"\0") if part
            ]
        except (OSError, ValueError):
            continue
        argv_names = [os.path.basename(argument) for argument in argv]
        if all(token in argv_names for token in signature):
            raise RuntimeError(
                f"go2_lio_navigation is already running (PID {pid}). "
                "Stop the existing launch with Ctrl+C before starting another one."
            )

    conflicts = conflicting_navigation_processes()
    if conflicts:
        raise RuntimeError(
            f"Navigation/gate processes still running (PIDs {conflicts}). "
            "Stop that session before starting a second stack.")

    lock_path = f"/tmp/go2_lio_navigation_{os.getuid()}.lock"
    try:
        if Path(lock_path).read_text(encoding="utf-8").strip() == str(os.getpid()):
            return
    except OSError:
        pass
    lock_handle = open(lock_path, "a+", encoding="utf-8")
    try:
        fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_handle.close()
        raise RuntimeError(
            "go2_lio_navigation is already starting or running. "
            "Stop the existing launch with Ctrl+C before starting another one."
        ) from exc
    lock_handle.seek(0)
    lock_handle.truncate()
    lock_handle.write(f"{os.getpid()}\n")
    lock_handle.flush()
    _instance_lock = lock_handle


def include(package, launch_file, arguments=None, condition=None):
    path = os.path.join(get_package_share_directory(package), "launch", launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def typed_float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    acquire_single_instance_lock()

    share = get_package_share_directory("go2_lio_navigation")
    nav_share = get_package_share_directory("go2_navigation")
    fast_lio_config = os.path.join(share, "config", "fast_lio_hap.yaml")
    loop_config = os.path.join(share, "config", "loop_closure.yaml")
    terrain_config = os.path.join(share, "config", "traversability.yaml")
    environment_config = os.path.join(share, "config", "environment_mapping.yaml")
    collision_config = os.path.join(share, "config", "collision_monitor.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_loop = LaunchConfiguration("enable_loop_closure")

    hap = include("go2_perception", "livox_hap.launch.py", {
        "start_driver": LaunchConfiguration("start_hap_driver"),
        "config_path": LaunchConfiguration("hap_config_path"),
        "publish_freq": LaunchConfiguration("hap_publish_freq"),
        # PointCloud2 uses best-effort/latest-frame QoS in the local Livox
        # driver and still carries HAP's timestamp/tag/line fields.
        "xfer_format": "0",
        "base_frame": "base_link_3d",
        "lidar_frame": "livox_frame",
        "x": LaunchConfiguration("hap_x"),
        "y": LaunchConfiguration("hap_y"),
        "z": LaunchConfiguration("hap_z"),
        "roll": LaunchConfiguration("hap_roll"),
        "pitch": LaunchConfiguration("hap_pitch"),
        "yaw": LaunchConfiguration("hap_yaw"),
    })

    # go2_base remains the command/watchdog bridge only. FAST-LIO supplies /odom and TF.
    base = include("go2_core", "go2_base.launch.py", {
        "video_enable": LaunchConfiguration("video_enable"),
        "odom_source": "none",
        "cmd_vel_timeout": LaunchConfiguration("cmd_vel_timeout"),
        "cmd_vel_topic": "/cmd_vel_guarded",
        # Match the checked controller/smoother yaw limit end to end.
        "max_angular_command": "1.4",
        "localization_guard_enabled": "true",
        "localization_valid_topic": "/lio/localization_valid",
    }, condition=IfCondition(LaunchConfiguration("enable_go2_base")))

    idle_stand = Node(
        package="go2_lio_navigation", executable="idle_stand_guard.py",
        name="idle_stand_guard", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_go2_base")),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[fast_lio_config, {
            "use_sim_time": use_sim_time,
            "preprocess.scan_rate": ParameterValue(PythonExpression([
                "int(float('", LaunchConfiguration("hap_publish_freq"), "'))"
            ]), value_type=int),
        }],
    )
    bridge = Node(
        package="go2_lio_navigation",
        executable="lio_base_bridge",
        name="lio_base_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/Odometry",
            "output_topic": "/odom",
            "validated_raw_topic": "/Odometry_validated",
            "valid_topic": "/lio/localization_valid",
            "odom_frame": "odom_lio",
            "base_frame": "base_link",
            "full_body_frame": "base_link_3d",
            # Nav2 consumes a 2.5-D map.  Remove gait-induced roll, pitch and
            # height oscillation from its base pose while FAST-LIO and loop
            # closure keep using the full 6-DoF sensor odometry internally.
            "planar_output": True,
            # Match the source-side envelope. The previous 3 m/s vertical
            # limit allowed nearly a metre of false height drift before the
            # bridge closed the motion gate.
            "max_linear_speed": 1.5,
            "max_vertical_speed": 2.0,
            "max_angular_speed": 3.0,
            "max_position_radius": 30.0,
            "max_absolute_z": 0.60,
            # Measured HAP/FAST-LIO scheduling gaps occasionally reach about
            # 1.3 s under full Nav2 + RViz load.  Keep a safety margin without
            # masking a genuine sensor loss for more than two seconds.
            "input_timeout": 1.8,
            # FAST-LIO makes a one-off height correction for several seconds
            # after the initial ikd-tree is built. Keep the motion gate closed
            # until that startup transient has settled; subsequent transient
            # gaps can recover faster once the estimator has already settled.
            "startup_samples": 30,
            "recovery_samples": 10,
            # Reject an isolated gait/deskew derivative spike, but latch and
            # stop before two consecutive invalid poses can reach Nav2.
            "jump_confirmation_samples": 2,
            "cancel_navigation_on_invalid": True,
            "cancel_navigation_on_motion_failure": True,
            "motion_execution_valid_topic": "/go2/motion_execution_valid",
            "clear_costmaps_on_invalid": True,
            "base_to_sensor_x": typed_float("hap_x"),
            "base_to_sensor_y": typed_float("hap_y"),
            "base_to_sensor_z": typed_float("hap_z"),
            "base_to_sensor_roll": typed_float("hap_roll"),
            "base_to_sensor_pitch": typed_float("hap_pitch"),
            "base_to_sensor_yaw": typed_float("hap_yaw"),
        }],
    )
    loop = Node(
        package="go2_lio_navigation",
        executable="scan_context_loop_closure",
        name="scan_context_loop_closure",
        output="screen",
        condition=IfCondition(enable_loop),
        parameters=[loop_config, {"use_sim_time": use_sim_time}],
    )
    identity_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="identity_map_to_odom_lio",
        condition=UnlessCondition(enable_loop),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom_lio",
        ],
    )
    terrain = Node(
        package="go2_lio_navigation",
        executable="traversability_mapper",
        name="traversability_mapper",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_terrain")),
        parameters=[terrain_config, {"use_sim_time": use_sim_time}],
    )
    environment = Node(
        package="go2_lio_navigation",
        executable="environment_mapper.py",
        name="environment_mapper",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_environment_mapping")),
        parameters=[environment_config, {"use_sim_time": use_sim_time}],
        additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
    )

    # Use FAST-LIO's deskewed body-frame scan for 2-D obstacle extraction;
    # the raw HAP PointCloud2 remains dedicated to the LIO front end.
    perception = include("go2_perception", "go2_pointcloud_process.launch.py", {
        "cloud_topic": "/cloud_registered_body",
        "obstacle_points_topic": "/obstacles/deskewed",
        "ground_plane_topic": "/perception/deskewed_ground_plane",
        "ground_pose_topic": "/Odometry_validated",
        "target_frame": "base_link_3d",
        "scan_frame": "base_link",
        "level_scan": "true",
        "fixed_cloud_frame": "livox_frame",
        # FAST-LIO publishes reliable clouds. Use one direct projection hop,
        # not two best-effort large-cloud transfers that dropped whole frames.
        "use_best_effort": "false",
        "enable_accumulation": "false",
        "projection_cloud_topic": "/cloud_registered_body",
        "accumulation_frames": "1",
        "ground_filter_enabled": "true",
        "observed_free_space_only": "true",
        "cloud_min_height": LaunchConfiguration("cloud_min_height"),
        "cloud_max_height": LaunchConfiguration("cloud_max_height"),
        "scan_period": PythonExpression([
            "1.0 / float('", LaunchConfiguration("hap_publish_freq"), "')"]),
    }, condition=IfCondition(LaunchConfiguration("enable_perception")))
    # A current raw frame reaches braking without waiting for ICP/map insertion.
    # Keep deskewed /scan for global geometry; local avoidance uses this lane.
    fast_perception = include("go2_perception", "go2_pointcloud_process.launch.py", {
        "cloud_topic": "/livox/lidar",
        "projection_cloud_topic": "/livox/lidar",
        "scan_topic": "/scan_safety",
        "obstacle_points_topic": "/obstacles/raw",
        "ground_plane_topic": "/perception/raw_ground_plane",
        "ground_pose_topic": "/Odometry_validated",
        "projection_node_name": "safety_pointcloud_to_laserscan",
        "target_frame": "base_link_3d",
        "scan_frame": "base_link",
        "level_scan": "true",
        "enable_accumulation": "false",
        "use_best_effort": "true",
        "ground_filter_enabled": "true",
        "observed_free_space_only": "true",
        "scan_period": PythonExpression([
            "1.0 / float('", LaunchConfiguration("hap_publish_freq"), "')"]),
    }, condition=IfCondition(LaunchConfiguration("enable_perception")))
    nav2 = include("go2_navigation", "go2_nav2.launch.py", {
        "use_sim_time": use_sim_time,
        "params_file": LaunchConfiguration("nav_params"),
        "autostart": "true",
        "mapping_mode": "true",
        "terrain_mode": LaunchConfiguration("terrain_mode"),
        "lio_mode": "true",
        "controller_mode": LaunchConfiguration("controller_mode"),
        "use_composition": LaunchConfiguration("nav2_use_composition"),
        "log_level": LaunchConfiguration("nav2_log_level"),
    }, condition=IfCondition(LaunchConfiguration("enable_nav2")))
    scan_gate = Node(
        package="go2_lio_navigation", executable="scan_safety_gate.py",
        name="scan_safety_gate", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{"use_sim_time": use_sim_time}],
    )
    obstacle_memory = Node(
        package="go2_lio_navigation", executable="obstacle_memory.py",
        name="obstacle_memory", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{"use_sim_time": use_sim_time,"body_frame":"base_link_3d"}],
    )
    collision_monitor = Node(
        package="nav2_collision_monitor", executable="collision_monitor",
        name="collision_monitor", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[collision_config, {"use_sim_time": use_sim_time,
            **collision_monitor_parameters(os.path.join(nav_share, "config", "nav2_params.yaml"))}],
    )
    collision_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="collision_monitor_manager", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_nav2")),
        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                     "node_names": ["collision_monitor"]}],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )
    terrain_display = Node(
        package="go2_lio_navigation", executable="environment_terrain_display.py",
        name="environment_terrain_display", output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[{"use_sim_time": use_sim_time, "display_resolution": .175}],
    )
    def localization_exited(event,context):
        if context.is_shutdown:return []
        return [LogInfo(msg='LOCALIZATION FAULT: FAST-LIO stopped. RViz remains open; '
                         'check the LiDAR/USB connection and restart navigation.'),
                Node(package='go2_lio_navigation',executable='navigation_fault_guard.py',
                     name='navigation_fault_guard',output='screen',
                     parameters=[{'use_sim_time':use_sim_time,'lio_exit_code':event.returncode}])]

    stop_on_fast_lio_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=fast_lio,
            on_exit=localization_exited,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config", default_value=os.path.join(share, "config", "lio_environment.rviz"),
            description="Environment reference + measured 2.5D live terrain, robot, plan and footprint"),
        DeclareLaunchArgument("video_enable", default_value="false"),
        DeclareLaunchArgument("start_hap_driver", default_value="true"),
        DeclareLaunchArgument("enable_loop_closure", default_value="true"),
        DeclareLaunchArgument("enable_go2_base", default_value="true"),
        DeclareLaunchArgument("enable_perception", default_value="true"),
        DeclareLaunchArgument("enable_terrain", default_value="true"),
        DeclareLaunchArgument("enable_environment_mapping", default_value="true"),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        DeclareLaunchArgument(
            "controller_mode", default_value="dwb", choices=["dwb", "mppi"]),
        DeclareLaunchArgument("nav2_log_level", default_value="warn"),
        DeclareLaunchArgument(
            "nav2_use_composition", default_value="true",
            description="Compose Nav2 to reduce DDS participants and scheduling load; false uses standalone servers"),
        DeclareLaunchArgument(
            "terrain_mode", default_value="indoor",
            choices=["indoor", "outdoor", "gravel"]),
        DeclareLaunchArgument("cmd_vel_timeout", default_value="0.5"),
        DeclareLaunchArgument("hap_publish_freq", default_value="15.0"),
        DeclareLaunchArgument(
            "hap_config_path",
            default_value=os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "config", "HAP_config.json"),
        ),
        DeclareLaunchArgument("hap_x", default_value="0.14"),
        DeclareLaunchArgument("hap_y", default_value="0.0"),
        DeclareLaunchArgument("hap_z", default_value="0.13"),
        DeclareLaunchArgument("hap_roll", default_value="0.0"),
        DeclareLaunchArgument("hap_pitch", default_value="0.0"),
        DeclareLaunchArgument("hap_yaw", default_value="0.0"),
        # Keep floor observations for adaptive ground removal and honest
        # clearing rays. A fixed sensor-height slice missed low obstacles.
        DeclareLaunchArgument("cloud_min_height", default_value="-1.5"),
        DeclareLaunchArgument("cloud_max_height", default_value="1.5"),
        DeclareLaunchArgument(
            "nav_params",
            default_value=os.path.join(nav_share, "config", "nav2_params.yaml"),
        ),
        stop_on_fast_lio_exit,
        hap,
        base,
        idle_stand,
        TimerAction(period=1.0, actions=[fast_lio, bridge, loop, identity_map_to_odom]),
        TimerAction(period=3.0, actions=[perception, fast_perception, terrain, environment]),
        TimerAction(period=6.0, actions=[nav2]),
        collision_monitor, collision_manager, obstacle_memory, scan_gate, terrain_display, rviz,
    ])
