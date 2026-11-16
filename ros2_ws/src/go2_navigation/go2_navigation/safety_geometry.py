"""Share execution geometry; point execution requires an explicit opt-in."""
import math
from pathlib import Path

import yaml


def protected_rectangle(parameters, *, allow_point=False):
    points = parameters['footprint']
    if isinstance(points, str):
        points = yaml.safe_load(points)
    padding = float(parameters.get('footprint_padding', 0.))
    if (not isinstance(points, list) or len(points) < 3 or
            not math.isfinite(padding) or padding < 0.):
        raise ValueError('Invalid footprint or padding')
    xy = [(float(p[0]), float(p[1])) for p in points]
    if not all(math.isfinite(v) for p in xy for v in p):
        raise ValueError('Footprint contains non-finite coordinates')
    if allow_point and padding == 0. and all(x == 0. and y == 0. for x, y in xy):
        return (0., 0., 0.)
    front = max(x for x, y in xy)+padding
    rear = -min(x for x, y in xy)+padding
    half_width = max(abs(y) for x, y in xy)+padding
    if min(front, rear, half_width) <= 0.:
        raise ValueError('Footprint must contain the robot reference point')
    # Bounding a more complex footprint remains conservative for the scan gate.
    return tuple(round(v, 9) for v in (front, rear, half_width))


def configured_protection(config_file=None):
    if config_file is None:
        from ament_index_python.packages import get_package_share_directory
        config_file = Path(get_package_share_directory('go2_navigation'))/'config/nav2_params.yaml'
    config = yaml.safe_load(Path(config_file).read_text())
    local = protected_rectangle(
        config['local_costmap']['local_costmap']['ros__parameters'], allow_point=True)
    global_ = protected_rectangle(
        config['global_costmap']['global_costmap']['ros__parameters'], allow_point=True)
    if local != global_:
        raise ValueError('Local and global planning geometry must agree')
    if 'physical_safety' in config:
        parameters = config['physical_safety']['ros__parameters']
        rectangle = protected_rectangle(parameters)
        model = parameters.get('collision_model', 'rectangle')
        if model not in ('rectangle', 'point'):
            raise ValueError('collision_model must be rectangle or point')
        # Keep the real dimensions available for restoring rectangle mode.
        return (0., 0., 0.) if model == 'point' else rectangle
    # Legacy rectangular profiles share their geometry with the final gate.
    # A point planner MUST supply a separate nonzero physical safety envelope.
    if min(local) <= 0.:
        raise ValueError('Point planning requires an explicit physical_safety footprint')
    return local


def rectangle_points(front, rear, half_width):
    return [front, half_width, front, -half_width,
            -rear, -half_width, -rear, half_width]


def collision_monitor_parameters(config_file=None):
    front, rear, half_width = configured_protection(config_file)
    point = (front, rear, half_width) == (0., 0., 0.)
    parameters = {'EmergencyStop.enabled': not point,
                  'FootprintApproach.enabled': not point}
    # Nav2 polygon actions require an area; a degenerate polygon is not a
    # valid point collision checker. Point mode uses the planner and final
    # point gate, while retaining the monitor's velocity topic chain.
    if not point:
        parameters['EmergencyStop.points'] = rectangle_points(front, rear, half_width)
    return parameters
