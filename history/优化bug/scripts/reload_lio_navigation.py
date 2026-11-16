#!/usr/bin/env python3
"""Reload Nav2 and the final gate while retaining an identified live LIO session.

Run with scripts/env.sh sourced. It refuses active goals or motion, saves the
map, and follows the owning launch PID until it exits. --detach keeps this
supervisor independent of the terminal that requested the reload.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from process_supervisor import ManagedChildren, process_identity

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-pid', type=int, required=True)
    parser.add_argument('--container-pid', type=int, required=True)
    parser.add_argument('--gate-pid', type=int, required=True)
    parser.add_argument('--base-pid', type=int,
                        help='Also reload this idle command bridge with the configured yaw limit')
    parser.add_argument('--terrain-pid', type=int,
                        help='Also reload this recent-observation mapper with its configured rate')
    parser.add_argument('--previous-supervisor-pid', type=int,
                        help='Retire an earlier reload supervisor before replacing its managed children')
    parser.add_argument('--projection-pid',type=int,action='append',default=[],
                        help='Also reload an identified obstacle projector (repeat for both scan lanes)')
    parser.add_argument('--rviz-pid',type=int,
                        help='Reopen the identified RViz with its current configuration file')
    parser.add_argument('--report-prefix', default='navigation_reload_'+time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--detach', action='store_true')
    parser.add_argument('--recover-stopped', action='store_true',
                        help='Recover an identified failed reload with no remaining command nodes')
    parser.add_argument('--gate-params-file', type=Path,
                        help='Original gate parameter file, required if its old PID already exited')
    args = parser.parse_args()
    if Path(args.report_prefix).name != args.report_prefix:
        parser.error('--report-prefix must be a filename without a directory')
    if args.detach:
        argv = [v for v in sys.argv[1:] if v != '--detach']
        log = ROOT/'reports'/f'{args.report_prefix}_supervisor.log'
        with log.open('w') as stream:
            p = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), *argv],
                                 cwd=ROOT, stdin=subprocess.DEVNULL, stdout=stream,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        print(json.dumps({'supervisor_pid': p.pid, 'log': str(log)}))
        return

    # Import ROS only in the persistent supervisor, after environment setup.
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from nav_msgs.msg import Path as NavPath
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_srvs.srv import Trigger
    from unitree_go.msg import SportModeState
    from go2_navigation.runtime_guard import conflicting_navigation_processes
    from go2_navigation.safety_geometry import configured_protection, collision_monitor_parameters

    def identity(pid, signature):
        cmd = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        if signature not in cmd:
            raise RuntimeError(f'Unexpected process identity for PID {pid}')
        return cmd

    identity(args.main_pid, 'go2_lio_navigation.launch.py')
    if args.previous_supervisor_pid is not None:
        identity(args.previous_supervisor_pid,'reload_lio_navigation.py')
        if args.previous_supervisor_pid==os.getpid():
            raise RuntimeError('Cannot retire the current supervisor')
    for pid, signature in ((args.container_pid, 'component_container_isolated'),
                           (args.gate_pid, 'scan_safety_gate.py')):
        if process_identity(pid) is not None:
            identity(pid, signature)
        elif not args.recover_stopped:
            raise RuntimeError(f'PID {pid} already exited; use explicit stopped-session recovery')
    optional_args = {}
    projection_args=[]
    for pid in args.projection_pid:
        identity(pid,'/go2_perception/pointcloud_to_laserscan_node')
        command=[v.decode() for v in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if v]
        lane='raw' if any('scan:=/scan_safety' in v for v in command) else 'deskewed'
        projection_args.append(command+['-r',f'ground_plane:=/perception/{lane}_ground_plane',
                                        '-p','ground_pose_topic:=/Odometry_validated'])
    for label, pid, signature in (('base', args.base_pid, '/go2_core/go2_base'),
                                  ('rviz',args.rviz_pid,'/rviz2/rviz2'),
                                  ('terrain', args.terrain_pid, '/go2_lio_navigation/traversability_mapper')):
        if pid is not None:
            identity(pid, signature)
            optional_args[label] = [v.decode() for v in
                Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if v]
    # Derive limits from the same installed configuration used by Nav2.
    import yaml
    from ament_index_python.packages import get_package_share_directory
    config_path = Path(get_package_share_directory('go2_navigation'))/'config/nav2_params.yaml'
    nav_config = yaml.safe_load(config_path.read_text())
    yaw_limit = nav_config['controller_server']['ros__parameters']['FollowPath']['max_vel_theta']
    expected_speed = nav_config['controller_server']['ros__parameters']['FollowPath']['max_vel_x']
    expected_tolerance=nav_config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']
    conflicts = set(conflicting_navigation_processes())-{args.container_pid, args.gate_pid}
    if conflicts:
        raise RuntimeError(f'Other navigation/gate processes still running: {sorted(conflicts)}')
    managed = ManagedChildren(args.main_pid)
    old_env = dict(v.split('=', 1) for v in
                   Path(f'/proc/{args.main_pid}/environ').read_text().split('\0') if '=' in v)
    env = os.environ.copy()
    env['GO2_NAV2_RELOAD_OWNER'] = f'{args.main_pid}:{managed.owner_identity}'
    for key in ('DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', 'ROS_DOMAIN_ID'):
        if key in old_env:
            env[key] = old_env[key]
    if process_identity(args.gate_pid) is not None:
        gate_args = [v.decode() for v in
                     Path(f'/proc/{args.gate_pid}/cmdline').read_bytes().split(b'\0') if v]
    elif args.recover_stopped and args.gate_params_file and args.gate_params_file.is_file():
        gate_args = ['/usr/bin/python3', str(ROOT/'ros2_ws/install/go2_lio_navigation/lib/'
                     'go2_lio_navigation/scan_safety_gate.py'), '--ros-args', '-r',
                     '__node:=scan_safety_gate', '--params-file', str(args.gate_params_file.resolve())]
    else:
        raise RuntimeError('Original gate parameter file required for stopped-session recovery')
    gate_args[0] = '/usr/bin/python3'
    rclpy.init()
    node = rclpy.create_node('navigation_reload_supervisor')
    state = {'statuses': [], 'velocity_samples': 0}
    report = {'owner_pid': args.main_pid, 'supervisor_pid': os.getpid(), 'applied': False}
    logs = []
    report_path = ROOT/'reports'/f'{args.report_prefix}.json'

    def write():
        report_path.write_text(json.dumps(report, indent=2))

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if not managed.owner_alive():
                raise RuntimeError('Owning navigation launch exited')
            rclpy.spin_once(node, timeout_sec=.02)

    def result(future, timeout=20):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:
            spin(.02)
        if not future.done():
            raise RuntimeError('ROS service timed out')
        return future.result()

    def service(kind, topic, request, timeout=20):
        client = node.create_client(kind, topic)
        try:
            if not client.wait_for_service(timeout_sec=4):
                raise RuntimeError('Service unavailable: '+topic)
            return result(client.call_async(request), timeout)
        finally:
            node.destroy_client(client)

    def parameters(name, keys):
        return service(GetParameters, '/'+name+'/get_parameters',
                       GetParameters.Request(names=keys)).values

    def idle():
        measured_idle_recovery = (args.recover_stopped and state['velocity_samples'] == 0 and
            time.monotonic()-state.get('sport_time', -1.e30) < .5 and
            state.get('measured_stationary', False) and
            node.count_publishers('/cmd_vel_guarded') == 0 and
            not conflicting_navigation_processes())
        commanded_idle = state['velocity_samples'] >= 10 and not any(
            abs(v)>.001 for v in state.get('velocity', [1.]))
        measured_idle = (time.monotonic()-state.get('sport_time', -1.e30) < .5 and
                         state.get('measured_stationary', False))
        if any(v in (1, 2, 3) for v in state['statuses']) or not (
                (commanded_idle and measured_idle) or measured_idle_recovery):
            raise RuntimeError('Active goal or motion: reload refused')

    def on_velocity(message):
        state['velocity'] = [message.linear.x, message.linear.y, message.angular.z]
        state['velocity_samples'] += 1

    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                             lambda m: state.update(statuses=[s.status for s in m.status_list]),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel_guarded', on_velocity, qos_profile_sensor_data)
    def on_sport(m):
        import math
        values = (float(m.velocity[0]), float(m.velocity[1]), float(m.yaw_speed))
        state.update(sport_time=time.monotonic(), measured_stationary=
                     all(math.isfinite(v) for v in values) and
                     math.hypot(values[0], values[1]) < .04 and abs(values[2]) < .10)
    node.create_subscription(SportModeState, '/lf/sportmodestate', on_sport, qos_profile_sensor_data)

    def stop(pid):
        token = process_identity(pid)
        if token is None:
            return
        # Only these identified idle-session child PIDs may be terminated;
        # never signal their process group, which belongs to the full mapper.
        first_wait=18 if pid==args.previous_supervisor_pid else 8
        for sig, timeout in ((signal.SIGINT, first_wait), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            if process_identity(pid) != token:
                break
            os.kill(pid, sig)
            report.setdefault('stop_signals', []).append({'pid': pid, 'signal': int(sig)})
            deadline = time.monotonic()+timeout
            while process_identity(pid) == token and time.monotonic()<deadline:
                spin(.1)
        if process_identity(pid) == token:
            raise RuntimeError(f'PID {pid} did not exit; refuse duplicate replacement')

    def start(command, suffix):
        stream = (ROOT/'reports'/f'{args.report_prefix}_{suffix}.log').open('w')
        logs.append(stream)
        return managed.start(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)

    def shutdown(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        spin(2)
        idle()
        report['before'] = dict(state)
        report['gate_args'] = gate_args
        saved = service(Trigger, '/mapping/save', Trigger.Request(), 30)
        if not saved.success:
            raise RuntimeError('Map save failed: '+saved.message)
        report['saved_map'] = saved.message
        idle()
        if args.previous_supervisor_pid is not None:
            # Its ownership includes the base, terrain, gate and Nav2. Stopping
            # just one child would make its monitor close the whole old set.
            stop(args.previous_supervisor_pid)
            report['retired_supervisor_pid']=args.previous_supervisor_pid
        stop(args.container_pid)
        stop(args.gate_pid)
        projections=[]
        for index,(pid,command) in enumerate(zip(args.projection_pid,projection_args)):
            stop(pid)
            projections.append(start(command,f'projection_{index}'))
        if projections:report['projection_pids']=[p.pid for p in projections]
        if args.terrain_pid is not None:
            stop(args.terrain_pid)
            terrain = start(optional_args['terrain'], 'terrain')
            report['terrain_pid'] = terrain.pid
        if args.base_pid is not None:
            stop(args.base_pid)
            base = start(optional_args['base'] + ['-p', f'max_angular_command:={yaw_limit}'], 'base')
            report['base_pid'] = base.pid
        manager = '/collision_monitor_manager/manage_nodes'
        reset = service(ManageLifecycleNodes, manager, ManageLifecycleNodes.Request(command=3))
        if not reset.success:
            raise RuntimeError('Collision-monitor reset failed')
        geometry = configured_protection()
        report['protection_rectangle'] = list(geometry)
        settings = [Parameter(name, value=value)
                    for name, value in collision_monitor_parameters().items()]
        settings += [Parameter('FootprintApproach.footprint_topic', value='/safety/body_footprint'),
                    Parameter('FootprintApproach.time_before_collision', value=1.0)]
        updated = service(SetParameters, '/collision_monitor/set_parameters',
                          SetParameters.Request(parameters=[p.to_parameter_msg() for p in settings]))
        if not all(v.successful for v in updated.results):
            raise RuntimeError('Collision-monitor parameter update rejected')
        if not service(ManageLifecycleNodes, manager, ManageLifecycleNodes.Request(command=0)).success:
            raise RuntimeError('Collision-monitor startup failed')
        gate = start(gate_args, 'gate')
        report['gate_pid'] = gate.pid
        started = time.monotonic()
        nav = start(['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'go2_navigation',
                     'go2_nav2.launch.py', 'mapping_mode:=true', 'lio_mode:=true',
                     'terrain_mode:=indoor', 'controller_mode:=dwb', 'use_composition:=true',
                     'log_level:=warn'], 'nav2')
        report['nav2_launch_pid'] = nav.pid
        write()
        deadline = time.monotonic()+45
        while time.monotonic()<deadline:
            if nav.poll() is not None or gate.poll() is not None:
                raise RuntimeError('Replacement navigation or gate exited')
            try:
                active = service(GetState, '/bt_navigator/get_state', GetState.Request(), 3)
                if active.current_state.label == 'active':
                    break
            except RuntimeError:
                pass
            spin(.2)
        else:
            raise RuntimeError('Navigation did not become active')
        report['active_seconds'] = round(time.monotonic()-started, 3)
        p = parameters('controller_server', ['goal_checker.xy_goal_tolerance',
                       'goal_checker.check_yaw', 'FollowPath.rotate_to_goal_heading',
                       'FollowPath.max_vel_x', 'controller_plugins'])
        if p[0].double_value != expected_tolerance or p[1].bool_value or p[2].bool_value or p[3].double_value != expected_speed:
            raise RuntimeError('Arrival or speed parameters did not apply')
        report['goal_tolerance'] = p[0].double_value
        report['check_yaw'] = p[1].bool_value
        report['maximum_linear_speed'] = p[3].double_value
        report['controllers'] = list(p[4].string_array_value)
        report['planner'] = parameters('planner_server', ['GridBased.plugin'])[0].string_value
        report['yaw_limit'] = parameters('controller_server', ['FollowPath.max_vel_theta'])[0].double_value
        report['map_rates'] = {
            name: [v.double_value for v in parameters(name, ['update_frequency', 'publish_frequency'])]
            for name in ('local_costmap/local_costmap', 'global_costmap/global_costmap')}
        report['costmap_geometry'] = {}
        for name in ('local_costmap/local_costmap', 'global_costmap/global_costmap'):
            actual = parameters(name, ['footprint_padding', 'inflation_layer.inflation_radius',
                                       'footprint'])
            expected = nav_config[name.split('/')[0]][name.split('/')[1]]['ros__parameters']
            if (actual[0].double_value != expected['footprint_padding'] or
                    actual[1].double_value != expected['inflation_layer']['inflation_radius'] or
                    yaml.safe_load(actual[2].string_value) != yaml.safe_load(expected['footprint'])):
                raise RuntimeError('Costmap protection parameters did not apply: '+name)
            report['costmap_geometry'][name] = [v.double_value for v in actual[:2]]
        body_topic = parameters('collision_monitor', ['FootprintApproach.footprint_topic'])[0].string_value
        if body_topic != '/safety/body_footprint':
            raise RuntimeError('Collision monitor must use the separate physical footprint')
        report['physical_footprint_topic'] = body_topic
        actual = parameters('scan_safety_gate', ['stop_front', 'stop_rear',
                            'stop_half_width', 'turn_half_length', 'turn_half_width'])
        gate_geometry = [v.double_value for v in actual]
        if any(abs(a-b) > 1.e-8 for a,b in zip(gate_geometry,
                [*geometry, max(geometry[0], geometry[1]), geometry[2]])):
            raise RuntimeError('Final scan gate protection geometry did not apply')
        report['gate_geometry'] = gate_geometry
        if args.base_pid is not None:
            actual = parameters('go2_base', ['max_angular_command'])[0].double_value
            if actual != yaw_limit:
                raise RuntimeError('Command bridge yaw limit did not match the checked controller')
            report['base_yaw_limit'] = actual
        if args.terrain_pid is not None:
            report['terrain_update_rate'] = parameters('traversability_mapper', ['update_rate'])[0].double_value
        # Remove the previous failed goal's visual path; this does not send motion.
        path_pub = node.create_publisher(NavPath, '/plan', 1)
        spin(.4)
        empty = NavPath()
        empty.header.frame_id = 'odom_lio'
        empty.header.stamp = node.get_clock().now().to_msg()
        path_pub.publish(empty)
        spin(.3)
        node.destroy_publisher(path_pub)
        report['applied'] = True
        if args.rviz_pid is not None:
            stop(args.rviz_pid)
            rviz=start(optional_args['rviz'],'rviz')
            report['rviz_pid']=rviz.pid
        write()
        print(json.dumps(report, indent=2), flush=True)
        while managed.owner_alive():
            rclpy.spin_once(node, timeout_sec=.2)
            if nav.poll() is not None or gate.poll() is not None:
                raise RuntimeError('Managed navigation child exited')
            if args.base_pid is not None and base.poll() is not None:
                raise RuntimeError('Managed base command bridge exited')
            if args.terrain_pid is not None and terrain.poll() is not None:
                raise RuntimeError('Managed terrain mapper exited')
            if any(p.poll() is not None for p in projections):
                raise RuntimeError('Managed obstacle projector exited')
    except KeyboardInterrupt:
        report['shutdown'] = 'Supervisor interrupted'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        managed.close()
        report['children_stopped'] = True
        write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for stream in logs:
            stream.close()


if __name__ == '__main__':
    main()
