"""Parameter overlays for terrain-specific Go2 navigation behavior."""


_AUTO_CLOUD_MIN_HEIGHTS = {
    # Flat floors/roads: retain small objects while dropping the floor itself.
    "indoor": 0.02,
    "outdoor": 0.02,
    # Grass and gravel create dense near-ground returns; use the safer rough-
    # terrain threshold there so harmless surface texture does not block paths.
    "gravel": 0.10,
}


def uses_rough_terrain_navigation(terrain_mode):
    """Whether a mode should use the conservative outdoor motion/SLAM profile."""

    return terrain_mode in {"outdoor", "gravel"}


def resolve_cloud_min_height(terrain_mode, requested="auto"):
    """Resolve ``auto`` to a terrain-specific point-cloud height in metres.

    A numeric launch override remains available for unusual sensors or sites.
    Returning a normalized string makes it safe to pass directly to an included
    ROS launch description.
    """

    value = str(requested).strip().lower()
    if value == "auto":
        try:
            return f"{_AUTO_CLOUD_MIN_HEIGHTS[terrain_mode]:.2f}"
        except KeyError as error:
            raise ValueError(f"unsupported terrain_mode: {terrain_mode}") from error

    try:
        height = float(value)
    except ValueError as error:
        raise ValueError("cloud_min_height must be 'auto' or a number") from error
    if not 0.0 <= height < 1.0:
        raise ValueError("cloud_min_height must be in [0.0, 1.0) metres")
    return f"{height:g}"


def outdoor_nav2_rewrites():
    """Return the conservative, low-latency outdoor Nav2 overlay.

    The indoor profile is the unmodified ``nav2_params.yaml`` baseline.  Keeping
    this as a leaf-parameter overlay means map resolution, height filtering and
    goal tolerances cannot accidentally drift between profiles.
    """

    return {
        # Rough odometry should not abort while the dog is making small but real
        # translational or angular progress.
        "controller_server.ros__parameters.failure_tolerance": "0.5",
        "controller_server.ros__parameters.progress_checker.required_movement_radius": "0.08",
        "controller_server.ros__parameters.progress_checker.required_movement_angle": "0.12",
        "controller_server.ros__parameters.progress_checker.movement_time_allowance": "20.0",

        # Reduce the deterministic DWB speed/acceleration envelope on loose
        # terrain so the quadruped keeps a stable gait and limits odometry slip.
        "controller_server.ros__parameters.FollowPath.max_vel_x": "0.35",
        "controller_server.ros__parameters.FollowPath.max_speed_xy": "0.35",
        "controller_server.ros__parameters.FollowPath.max_vel_theta": "0.70",
        "controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel": "0.75",
        "controller_server.ros__parameters.FollowPath.max_angular_accel": "0.90",
        "controller_server.ros__parameters.FollowPath.acc_lim_x": "0.55",
        "controller_server.ros__parameters.FollowPath.decel_lim_x": "-0.70",
        "controller_server.ros__parameters.FollowPath.acc_lim_theta": "0.90",
        "controller_server.ros__parameters.FollowPath.decel_lim_theta": "-0.80",
        "controller_server.ros__parameters.FollowPath.BaseObstacle.scale": "0.25",

        # Keep the full 5 m obstacle radius visible to the local controller.
        "local_costmap.local_costmap.ros__parameters.width": "10",
        "local_costmap.local_costmap.ros__parameters.height": "10",
        "local_costmap.local_costmap.ros__parameters.transform_tolerance": "0.25",
        "local_costmap.local_costmap.ros__parameters.denoise_layer.minimal_group_size": "1",
        "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius": "0.85",
        "local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "2.5",

        # Preserve thin real obstacles at the full 5 m sensor range and keep
        # a wider cost gradient around them on rough terrain.
        "global_costmap.global_costmap.ros__parameters.transform_tolerance": "0.25",
        "global_costmap.global_costmap.ros__parameters.denoise_layer.minimal_group_size": "1",
        "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": "0.90",
        "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor": "2.0",

        # Existing-map localization benefits from more frequent updates outdoors.
        "amcl.ros__parameters.update_min_a": "0.05",
        "amcl.ros__parameters.update_min_d": "0.05",
        "amcl.ros__parameters.max_beams": "160",
        "amcl.ros__parameters.sigma_hit": "0.25",
    }


def outdoor_slam_rewrites():
    """Return the outdoor slam_toolbox overlay.

    Nav2 costmaps still consume every raw scan for immediate obstacle avoidance.
    The stricter pass-through/occupancy values only prevent transient vegetation
    and gravel returns from becoming persistent walls in the SLAM map.
    """

    return {
        # Outdoor structures are sparser, so retain more range than the 8 m
        # indoor baseline while still rejecting very distant angular outliers.
        "slam_toolbox.ros__parameters.max_laser_range": "12.0",
        "slam_toolbox.ros__parameters.scan_buffer_size": "8",
        "slam_toolbox.ros__parameters.link_match_minimum_response_fine": "0.48",
        "slam_toolbox.ros__parameters.min_pass_through": "2",
        "slam_toolbox.ros__parameters.occupancy_threshold": "0.20",
        "slam_toolbox.ros__parameters.distance_variance_penalty": "0.40",
        "slam_toolbox.ros__parameters.angle_variance_penalty": "0.80",
    }
