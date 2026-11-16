"""Derive the axis-aligned safety rectangle from the configured Nav2 footprint."""
import math
from pathlib import Path

import yaml


def protected_rectangle(parameters):
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
    local = protected_rectangle(config['local_costmap']['local_costmap']['ros__parameters'])
    global_ = protected_rectangle(config['global_costmap']['global_costmap']['ros__parameters'])
    if local != global_:
        raise ValueError('Local and global protection geometry must agree')
    return local


def rectangle_points(front, rear, half_width):
    return [front, half_width, front, -half_width,
            -rear, -half_width, -rear, half_width]
