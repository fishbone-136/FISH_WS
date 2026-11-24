from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from go2_navigation.terrain_profiles import (
    outdoor_nav2_rewrites,
    uses_rough_terrain_navigation,
)
from go2_navigation.runtime_guard import acquire_navigation_lock
from go2_navigation.safety_geometry import configured_protection
import os
import tempfile
import yaml
import xml.etree.ElementTree as ET


def _include_nav2(context, nav_to_pose_bt, nav_to_pose_mppi_bt):
    acquire_navigation_lock()
    terrain_mode = LaunchConfiguration('terrain_mode').perform(context)
    controller_mode = LaunchConfiguration('controller_mode').perform(context).lower()
    lio_mode = LaunchConfiguration('lio_mode').perform(context).lower() == 'true'
    selected_bt = nav_to_pose_mppi_bt if controller_mode == 'mppi' else nav_to_pose_bt
    selected_controller = 'FollowPathMPPI' if controller_mode == 'mppi' else 'FollowPath'
    if lio_mode:
        tree = ET.parse(selected_bt)
        for item in tree.iter('LatestGoalPose'):
            item.set('anchor_frame', 'odom_lio')
        tree.find('.//RateController').set('hz', '4.0')
        recovery = tree.find(".//RecoveryNode[@name='NavigateRecovery']")
        # Check the rebuilt map sooner while keeping the tree's retry limit.
        fallback = recovery.find('ReactiveFallback')
        fallback.remove(fallback.find('Wait'))
        delay = ET.SubElement(fallback, 'Delay', delay_msec='400')
        ET.SubElement(delay, 'AlwaysSuccess')
        with tempfile.NamedTemporaryFile(mode='wb', prefix='go2_anchored_goal_',
                                         suffix='.xml', delete=False) as stream:
            tree.write(stream, encoding='utf-8', xml_declaration=True)
            selected_bt = stream.name
    # Loading the unused MPPI optimizer also constructs its critics, buffers and
    # trajectory generator during a DWB startup. Keep the actual ROS list type.
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        selected_params = yaml.safe_load(stream)
    selected_params['controller_server']['ros__parameters']['controller_plugins'] = [
        selected_controller]
    # Candidate checks and global clearance share the configured physical body.
    # Local trajectory scoring still uses points plus exact execution arcs.
    if lio_mode:
        front, rear, half_width = configured_protection(
            LaunchConfiguration('params_file').perform(context))
        # Smac2D uses the inscribed radius for 2D collision costs. A zero
        # global radius can route directly beside a person where the final
        # rectangular gate must stop. Only the existing physical width becomes
        # a hard band; the 18 cm outer inflation remains a soft preference.
        global_map = selected_params['global_costmap']['global_costmap']['ros__parameters']
        global_map['footprint'] = str([
            [front, half_width], [front, -half_width],
            [-rear, -half_width], [-rear, half_width]])
        global_map['footprint_padding'] = 0.0
        # Preserve a usable centreline between 50 cm walls after the 35 cm
        # hard clearance band is rasterized, even when the walls straddle cells.
        global_map['resolution'] = 0.04
        controller = selected_params['controller_server']['ros__parameters'][selected_controller]
        if controller_mode == 'dwb':
            # The immediate path can turn away from the final goal to pass a
            # person. Prefer tracking that detour to closing the goal distance.
            controller.update({
                'angular_dist_threshold': 0.55,
                'corridor_alignment': True,
                'corridor_max_width': 0.70,
                'PathAlign.forward_point_distance': 0.45,
                'PathAlign.scale': 16.0,
                'PathDist.scale': 24.0,
                'GoalDist.scale': 12.0,
            })
        # Keep action polling from consuming most of a 250 ms replan period.
        selected_params['bt_navigator']['ros__parameters']['bt_loop_duration'] = 50
        selected_params['planner_server']['ros__parameters']['expected_planner_frequency'] = 4.0
        controller.update({
            'execution_safety.enabled': True,
            'execution_safety.point_model': (front, rear, half_width) == (0., 0., 0.),
            'execution_safety.body_front': front,
            'execution_safety.body_rear': rear,
            'execution_safety.body_half_width': half_width,
            'execution_safety.minimum_translation': controller.get('gait_min_translation', .22),
            'execution_safety.raw_sensor_x': float(LaunchConfiguration('sensor_x').perform(context)),
        })
    with tempfile.NamedTemporaryFile(mode='w', prefix='go2_selected_controller_',
                                     suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(selected_params, stream, sort_keys=False)
        selected_params_file = stream.name
    rewrites = {'default_nav_to_pose_bt_xml': selected_bt}
    if LaunchConfiguration('mapping_mode').perform(context).lower() == 'true':
        # During online mapping slam_toolbox initially publishes only the small
        # observed rectangle.  A map-sized global costmap rejects a nearby goal
        # just beyond that rectangle before the next map expansion.  Keep the
        # static SLAM layer, but use a robot-centred window while mapping.
        rewrites.update({
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'true',
            # 20 m rolling window covers local online mapping goals while using
            # less than half the cells of the old 30 m window at 4 cm resolution.
            'global_costmap.global_costmap.ros__parameters.width': '20',
            'global_costmap.global_costmap.ros__parameters.height': '20',
        })
    if uses_rough_terrain_navigation(terrain_mode):
        rewrites.update(outdoor_nav2_rewrites())

    if lio_mode:
        # FAST-LIO owns the continuous local frame. Loop closure only changes
        # map->odom_lio, so the local controller never receives a pose jump.
        rewrites.update({
            'local_costmap.local_costmap.ros__parameters.global_frame': 'odom_lio',
            # Historical obstacle cells must share the continuous LIO frame.
            # Keeping them in map leaves ghosts after a loop correction; fresh
            # rays in the corrected frame cannot reliably clear those cells.
            'global_costmap.global_costmap.ros__parameters.global_frame': 'odom_lio',
            'bt_navigator.ros__parameters.global_frame': 'odom_lio',
            'global_costmap.global_costmap.ros__parameters.rolling_window': 'true',
            # Both planners see the current raw scan, without a deskew/map delay.
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic':
                '/scan_safety',
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic':
                '/scan_safety',
            'local_costmap.local_costmap.ros__parameters.traversability_layer.enabled': 'true',
            'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
                '/traversability/global',
            'global_costmap.global_costmap.ros__parameters.static_layer.subscribe_to_updates':
                'false',
            'planner_server.ros__parameters.GridBased.plugin':
                'go2_navigation::ExactGoalSmac2D',
            'behavior_server.ros__parameters.global_frame': 'odom_lio',
        })
        if LaunchConfiguration('use_obstacle_memory').perform(context).lower() == 'true':
            rewrites.update({
                # Retain blind obstacles once, in the evidence-based memory.
                # The scan layer must not keep an independently uncleared copy.
                'local_costmap.local_costmap.ros__parameters.obstacle_layer.plugin':
                    'go2_navigation::CurrentObstacleLayer',
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.plugin':
                    'go2_navigation::CurrentObstacleLayer',
                'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.clearing': 'false',
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.clearing': 'false',
                # Nav2 treats zero as "keep the last scan indefinitely". Match
                # the LIO safety scan timeout so a stalled source stops adding
                # old marks. Persistent geometry remains in obstacle_memory.
                'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.observation_persistence': '0.25',
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.observation_persistence': '0.25',
                'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.expected_update_rate': '0.25',
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.expected_update_rate': '0.25',
                'local_costmap.local_costmap.ros__parameters.traversability_layer.map_topic':
                    '/safety/navigation_local',
                'global_costmap.global_costmap.ros__parameters.static_layer.map_topic':
                    '/safety/navigation_global',
            })

    configured_params = RewrittenYaml(
        source_file=selected_params_file,
        root_key='',
        param_rewrites=rewrites,
        convert_types=True,
    )

    # Humble's navigation_launch.py embeds this value in a PythonExpression.
    # Canonical Python booleans avoid its `not true` lowercase parsing failure.
    use_composition = (
        'True'
        if LaunchConfiguration('use_composition').perform(context).lower() == 'true'
        else 'False'
    )
    container_name = 'nav2_container'
    container = Node(
        condition=IfCondition(use_composition),
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[configured_params, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(
                'nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': 'False',
            # Dynamic traversability grids make StaticLayer report a resize at
            # every update. Keep the default terminal useful while allowing
            # callers to restore INFO during Nav2 debugging.
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )
    # Behaviors and the controller must enter the SAME smoother input. Otherwise
    # idle smoother zeros can alternate with direct behavior cmd_vel commands.
    return [GroupAction([SetRemap(src='behavior_server:cmd_vel', dst='cmd_vel_nav'), container, navigation])]


def generate_launch_description():
    # 创建LaunchDescription对象
    ld = LaunchDescription()

    # 设置配置文件路径
    navigation_share = get_package_share_directory('go2_navigation')
    nav2_config = os.path.join(navigation_share, 'config', 'nav2_params.yaml')
    nav_to_pose_bt = os.path.join(
        navigation_share, 'config', 'navigate_w_replanning_and_recovery.xml')
    nav_to_pose_mppi_bt = os.path.join(
        navigation_share, 'config', 'navigate_w_replanning_and_recovery_mppi.xml')

    # 添加launch动作
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=nav2_config))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_obstacle_memory', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'use_composition', default_value='true',
        description='Run Nav2 servers in one isolated component container'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='warn'))
    ld.add_action(DeclareLaunchArgument(
        'mapping_mode', default_value='false',
        description='Use a rolling global costmap while the SLAM map is growing'))
    ld.add_action(DeclareLaunchArgument(
        'terrain_mode', default_value='indoor',
        choices=['indoor', 'outdoor', 'gravel'],
        description='Indoor, outdoor hard surface, or gravel/grass rough terrain'))
    ld.add_action(DeclareLaunchArgument(
        'lio_mode', default_value='false',
        description='Use odom_lio, traversability grids, and SmacPlanner2D'))
    ld.add_action(DeclareLaunchArgument(
        'hap_x', default_value='0.14', description='HAP x position in base_link (metres)'))
    ld.add_action(DeclareLaunchArgument(
        'sensor_x', default_value=LaunchConfiguration('hap_x'),
        description='Active LiDAR x position in base_link (metres)'))
    ld.add_action(DeclareLaunchArgument(
        'controller_mode', default_value='dwb', choices=['dwb', 'mppi'],
        description='Primary local controller behind RotationShim'))
    # Humble 的 navigation_launch.py 包含 controller/smoother/planner/
    # behavior/bt/waypoint/velocity_smoother 及生命周期管理器。
    ld.add_action(OpaqueFunction(
        function=_include_nav2, args=[nav_to_pose_bt, nav_to_pose_mppi_bt]))

    return ld
