"""Geometry and fail-closed guards for short, straight visual approaches."""
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from voice import config
from voice.camera_calibration import CameraCalibration, load_camera_calibration
from voice.visual_navigation import calibrated_mount_point, measure_target


class NearUnavailable(ValueError):
    """May select navigation, only before the first SDK velocity command."""


def body_projection():
    if config.VISUAL_CAMERA_MOUNT:
        point = lambda p: calibrated_mount_point(p, config.VISUAL_CAMERA_MOUNT)
    else:
        calibration = load_camera_calibration(config.VISUAL_CAMERA_CALIBRATION, config.VISUAL_CAMERA_FRAME)
        data = json.loads(Path(config.VISUAL_CAMERA_CALIBRATION).read_text())
        mount = data.get('parent_in_body', {})
        if mount.get('frame') != 'base_link':
            raise NearUnavailable('缺少 SDK 所需的固定机身安装标定')
        try:
            translation = tuple(float(v) for v in mount['translation_xyz'])
            rotation = tuple(float(v) for v in mount['quaternion_xyzw'])
            if (len(translation) != 3 or len(rotation) != 4 or
                    not all(math.isfinite(v) for v in (*translation, *rotation)) or
                    np.linalg.norm(translation) > 2.):
                raise ValueError('安装参数无效')
            parent = CameraCalibration('base_link', translation, rotation)
            point = lambda p: parent.point_in_parent(calibration.point_in_parent(p))
            point((0., 0., 0.))  # Validate quaternion before creating the projection.
        except (KeyError, TypeError, ValueError) as error:
            raise NearUnavailable(f'机身安装标定无效：{error}') from error
    origin = np.array(point((0., 0., 0.)))
    matrix = np.column_stack([np.array(point(axis))-origin for axis in np.eye(3)])
    return lambda points: np.asarray(points) @ matrix.T + origin


def valid_state(state, stationary=False):
    values = (*state['rpy'], state['height'], *state['position'], *state['velocity'], state['yaw_speed'])
    if (not all(math.isfinite(v) for v in values) or
            not 0. <= time.monotonic()-state['observed_at'] <= .3):
        raise ValueError('机器狗姿态反馈无效或已过期')
    if state['height'] < .18:
        raise NearUnavailable(f'机身高度仅 {state["height"]:.2f} 米，请先站稳（至少 0.18 米）')
    if max(abs(v) for v in state['rpy'][:2]) > math.radians(8):
        raise NearUnavailable('机身倾斜超过近距控制范围，请先站稳')
    if stationary and (math.hypot(*state['velocity'][:2]) > .06 or abs(state['yaw_speed']) > .1):
        raise NearUnavailable('请先让机器狗保持静止')


def box_iou(a, b):
    intersection = max(0., min(a[2], b[2])-max(a[0], b[0]))*max(0., min(a[3], b[3])-max(a[1], b[1]))
    return intersection / max(1.e-9, (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection)


def appearance(frame, box):
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = box
    roi = frame[int((y1+(y2-y1)*.2)*h):int((y2-(y2-y1)*.2)*h),
                int((x1+(x2-x1)*.2)*w):int((x2-(x2-x1)*.2)*w)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def track_person(boxes, previous, frame, original_appearance):
    candidates = [tuple(box) for box in boxes if box_iou(box, previous) >= .4]
    if len(candidates) != 1:
        raise ValueError('人物丢失、遮挡或交叉，已停止靠近')
    box = candidates[0]
    if any(box_iou(box, other) > .08 for other in boxes if tuple(other) != box):
        raise ValueError('多人相互遮挡，已停止靠近')
    if cv2.compareHist(appearance(frame, box), original_appearance, cv2.HISTCMP_CORREL) < .65:
        raise ValueError('人物外观发生明显变化，已停止靠近')
    return box


def target_in_body(snapshot, box, project):
    if snapshot is None or not 0. <= time.monotonic()-snapshot.captured_monotonic <= .20:
        raise ValueError('RGB-D 画面过期或检测延迟过高，已停止靠近')
    return project(measure_target(snapshot, box))


def near_limits(point, stand_off):
    x, y, _ = point
    distance = math.hypot(x, y)
    if (not 1. <= stand_off <= 2. or x <= 0. or distance > 4.
            or distance-stand_off > 3. or abs(math.atan2(y, x)) > math.radians(12)):
        raise NearUnavailable('目标不在近距直行范围内，需使用导航规划')
    return distance


def corridor_clear(snapshot, state, project, point, stand_off):
    """Check a body-width corridor, separating measured floor using live attitude."""
    depth = snapshot.depth_mm
    h, w = depth.shape
    center = depth[h//3:5*h//6, w//4:3*w//4]
    if np.mean((center >= 250) & (center <= 8000)) < .75:
        raise NearUnavailable('前方深度空洞过多，无法确认直线路径')
    # Deproject the actual color intrinsics; never infer focal length from FOV.
    pixels = [(u, v) for v in range(h//4, h-4, 12) for u in range(4, w-4, 12)
              if 250 <= depth[v, u] <= 8000]
    optical = np.array([snapshot.deproject(u, v, float(depth[v, u])/1000.) for u, v in pixels])
    if optical.shape[0] < 100 or not np.isfinite(optical).all():
        raise NearUnavailable('前方点云不足')
    body = project(optical)
    roll, pitch = state['rpy'][:2]
    # Height above a locally level floor, using measured body height and IMU.
    level_z = (-math.sin(pitch)*body[:, 0] + math.cos(pitch)*math.sin(roll)*body[:, 1]
               + math.cos(pitch)*math.cos(roll)*body[:, 2]) + state['height']
    forward = body[:, 0]
    heading = math.atan2(point[1], point[0])
    lateral = body[:, 1]-forward*math.tan(heading)
    corridor = (forward > .20) & (np.abs(lateral) < .40)
    floor = corridor & (forward < 2.5) & (np.abs(level_z) < .065)
    if np.count_nonzero(floor) < 15:
        raise NearUnavailable('无法确认近处平整地面，需使用导航')
    travel = max(0., math.hypot(*point[:2])-stand_off)
    obstacle = corridor & (forward < travel+.35) & (level_z > .08) & (level_z < 1.6)
    if np.count_nonzero(obstacle) >= 3:
        raise NearUnavailable('直线路径存在障碍，需使用导航绕行')
    return True


def approach_velocity(point, stand_off):
    distance = math.hypot(*point[:2])
    error = distance-stand_off
    bearing = math.atan2(point[1], point[0])
    if error <= .08:
        return 0., 0.
    speed = min(.20, .4*max(0., error-.04))
    if abs(bearing) > math.radians(8):
        speed = min(speed, .08)
    return speed, max(-.25, min(.25, bearing*.9))
