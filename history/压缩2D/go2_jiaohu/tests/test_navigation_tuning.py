import math
import os
from pathlib import Path
from xml.etree import ElementTree

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]


def _yaml(relative_path: str):
    with (WORKSPACE / relative_path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_navigation_reacts_to_fresh_obstacles_and_ignores_goal_heading():
    params = _yaml("ros2_ws/src/go2_navigation/config/nav2_params.yaml")
    goal = params["controller_server"]["ros__parameters"]["goal_checker"]
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_map = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert 0.16 <= goal["xy_goal_tolerance"] <= 0.20
    assert goal["yaw_goal_tolerance"] > math.pi
    assert goal["stateful"] is False
    assert local["update_frequency"] >= 10.0
    # This is a dropout watchdog, not a costmap sampling period.  It must allow
    # rare >0.5 s scheduling spikes without marking the layer stale.
    assert local["obstacle_layer"]["scan"]["expected_update_rate"] == 2.0
    assert 0.1 <= local["obstacle_layer"]["scan"]["observation_persistence"] <= 0.3
    assert local["width"] >= 10
    assert local["height"] >= 10
    assert local["denoise_layer"]["minimal_group_size"] == 1
    assert local["inflation_layer"]["inflation_radius"] >= 0.7
    assert global_map["update_frequency"] >= 3.0
    assert global_map["obstacle_layer"]["scan"]["expected_update_rate"] == 2.0
    assert 0.1 <= global_map["obstacle_layer"]["scan"]["observation_persistence"] <= 0.3
    assert global_map["denoise_layer"]["minimal_group_size"] == 1
    assert global_map["inflation_layer"]["inflation_radius"] >= 0.7


def test_rotation_shim_and_dwb_are_stable_lightweight_and_quadruped_aware():
    params = _yaml("ros2_ws/src/go2_navigation/config/nav2_params.yaml")
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert follow_path["plugin"] == (
        "nav2_rotation_shim_controller::RotationShimController"
    )
    assert follow_path["primary_controller"] == "dwb_core::DWBLocalPlanner"
    assert 0.85 <= follow_path["rotate_to_heading_angular_vel"] <= 0.95
    assert follow_path["angular_disengage_threshold"] < (
        follow_path["angular_dist_threshold"]
    )
    assert follow_path["closed_loop"] is False
    assert follow_path["rotate_to_goal_heading"] is False
    assert follow_path["debug_trajectory_details"] is False
    assert follow_path["vx_samples"] * follow_path["vtheta_samples"] <= 240
    assert follow_path["vy_samples"] == 1
    assert follow_path["min_speed_xy"] >= 0.12
    assert follow_path["min_speed_theta"] >= 0.45
    assert 0.45 <= follow_path["max_vel_x"] <= 0.55
    assert 0.80 <= follow_path["max_vel_theta"] <= 0.90
    assert 0.16 <= follow_path["xy_goal_tolerance"] <= 0.20
    assert follow_path["GoalDist.scale"] > follow_path["PathAlign.scale"]
    assert follow_path["Twirling.scale"] >= 5.0
    assert "BaseObstacle" in follow_path["critics"]
    assert "Twirling" in follow_path["critics"]
    assert "RotateToGoal" not in follow_path["critics"]
    assert smoother["scale_velocities"] is True
    assert 20.0 <= smoother["smoothing_frequency"] <= 25.0
    assert smoother["max_accel"][0] <= 0.7
    assert smoother["max_accel"][2] <= 1.3
    assert smoother["max_decel"][2] >= -1.3
    assert smoother["deadband_velocity"][0] >= 0.12
    assert 0.25 <= smoother["deadband_velocity"][2] <= 0.30
    # DWB may combine a valid linear command with 0.4 rad/s yaw; an
    # axis-wise deadband above it silently removes steering.
    assert smoother["deadband_velocity"][2] < follow_path["min_speed_theta"]


def test_slam_keeps_precision_while_publishing_maps_at_four_hz():
    params = _yaml("ros2_ws/src/go2_slam/config/mapper_params_online_async.yaml")
    slam = params["slam_toolbox"]["ros__parameters"]

    assert slam["resolution"] == 0.04
    assert slam["map_update_interval"] <= 0.25
    assert slam["minimum_time_interval"] <= 0.05
    assert slam["scan_queue_size"] == 1
    assert slam["use_scan_matching"] is True
    assert slam["ceres_loss_function"] == "HuberLoss"
    assert slam["max_laser_range"] <= 8.0
    assert slam["min_pass_through"] == 2
    assert slam["occupancy_threshold"] >= 0.2


def test_global_path_is_replaced_only_when_the_goal_changes():
    tree = ElementTree.parse(
        WORKSPACE
        / "ros2_ws/src/go2_navigation/config/navigate_w_replanning_and_recovery.xml"
    )
    # Replacing a path interrupts the local controller and is visible on a
    # quadruped as a gait pause. Dynamic obstacles stay in the 10 Hz local loop.
    assert tree.findall(".//GoalUpdatedController")
    assert not tree.findall(".//RateController")
    assert not tree.findall(".//IsPathValid")
    assert not tree.findall(".//SmoothPath")
    assert len(tree.findall(".//ComputePathToPose")) == 1


def test_navfn_accepts_a_nearby_safe_goal_when_live_map_marks_the_exact_cell():
    params = _yaml("ros2_ws/src/go2_navigation/config/nav2_params.yaml")
    planner = params["planner_server"]["ros__parameters"]["GridBased"]

    assert planner["plugin"] == "nav2_navfn_planner/NavfnPlanner"
    assert 0.18 <= planner["tolerance"] <= 0.22
    assert planner["use_astar"] is True
    assert planner["allow_unknown"] is True


def test_pointcloud_defaults_use_latest_frame_without_rotational_smear():
    launch = (
        WORKSPACE
        / "ros2_ws/src/go2_perception/launch/go2_pointcloud_process.launch.py"
    ).read_text(encoding="utf-8")

    assert "DeclareLaunchArgument('accumulation_frames', default_value='1')" in launch
    assert "DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar')" in launch
    assert "DeclareLaunchArgument('cloud_min_height', default_value='-0.33')" in launch
    assert "DeclareLaunchArgument('cloud_max_height', default_value='0.37')" in launch
    assert "DeclareLaunchArgument('fixed_cloud_frame', default_value='livox_frame')" in launch
    assert "DeclareLaunchArgument('scan_min_height', default_value='-2.00')" in launch
    assert "'point_spread_radius': 0.04" in launch
    assert "'max_fill_gap_bins': 3" in launch
    assert "DeclareLaunchArgument('max_history_age', default_value='0.12')" in launch
    assert "'queue_size': 1" in launch


def test_terrain_profiles_remain_available_for_navigation_tuning():
    import sys

    package_root = WORKSPACE / "ros2_ws/src/go2_navigation"
    sys.path.insert(0, str(package_root))
    try:
        from go2_navigation.terrain_profiles import resolve_cloud_min_height

        assert resolve_cloud_min_height("indoor") == "0.02"
        assert resolve_cloud_min_height("outdoor") == "0.02"
        assert resolve_cloud_min_height("gravel") == "0.10"
        assert resolve_cloud_min_height("gravel", "0.05") == "0.05"
    finally:
        sys.path.remove(str(package_root))

    for relative_path in (
        "ros2_ws/src/go2_navigation/launch/go2_mapping.launch.py",
        "ros2_ws/src/go2_navigation/launch/go2_localization.launch.py",
    ):
        launch = (WORKSPACE / relative_path).read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar")' in launch
        assert 'default_value="-0.33"' in launch
        assert 'DeclareLaunchArgument("fixed_cloud_frame", default_value="livox_frame")' in launch
        assert '"livox_hap.launch.py"' in launch
        assert '"odom_source": "sport_mode_state"' in launch


def test_run_mapping_uses_the_validated_hap_height_slice():
    script = (WORKSPACE / "scripts/run_mapping.sh").read_text(encoding="utf-8")

    assert "cloud_min_height:=-0.33" in script
    assert "cloud_max_height:=0.37" in script


def test_go2_base_defaults_to_non_utlidar_odometry():
    launch = (
        WORKSPACE / "ros2_ws/src/go2_core/launch/go2_base.launch.py"
    ).read_text(encoding="utf-8")
    source = (
        WORKSPACE / "ros2_ws/src/go2_core/src/go2_base.cpp"
    ).read_text(encoding="utf-8")

    assert "('odom_source', 'sport_mode_state'" in launch
    assert "('sport_state_topic', '/lf/sportmodestate'" in launch
    assert 'odom_source_ == "sport_mode_state"' in source


def test_map_canvas_keeps_only_latest_map_and_builds_scale_cache():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication

    from go2_ui.navigation.navigation_page import MapCanvas

    app = QApplication.instance() or QApplication([])
    canvas = MapCanvas()
    canvas.set_active(False)

    first = {
        "width": 2,
        "height": 2,
        "resolution": 0.04,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "origin_yaw": 0.0,
        "data": bytes([0, 0, 0, 0]),
    }
    latest = {**first, "data": bytes([100, 255, 50, 0])}
    canvas.set_map(first)
    canvas.set_map(latest)

    assert canvas._image is None
    assert canvas._pending_map is latest

    canvas.resize(640, 400)
    canvas.show()
    canvas.set_active(True)
    app.processEvents()
    canvas.repaint()

    assert canvas._meta is latest
    assert canvas._pending_map is None
    assert canvas._image.depth() == 8
    assert canvas._scaled_map is not None
    assert canvas._scaled_map_revision == canvas._map_revision

    canvas.close()
    canvas.deleteLater()
    app.processEvents()
