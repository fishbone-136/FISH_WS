import json
from pathlib import Path

import pytest

from voice import config
from voice.camera_calibration import load_camera_calibration
from voice.visual_navigation import calibrated_mount_point


CALIBRATION = Path(__file__).resolve().parents[1] / 'config' / 'visual_camera_calibration.json'


@pytest.mark.parametrize('frame', ['camera_color_optical_frame', 'd435i_color_optical_frame'])
@pytest.mark.parametrize('point', [(0., 0., 3.), (1., .5, 2.), (-.7, -.3, 4.)])
def test_measured_transform_matches_previous_sensor_urdf(frame, point):
    calibration = load_camera_calibration(str(CALIBRATION), frame)
    # Independent previous sensor_frames.yaml RPY, converted from radians to degrees.
    import math
    angles = tuple(math.degrees(v) for v in (.0117421720, -.0131470996, -.0024432009))
    expected = calibrated_mount_point(point, ','.join(map(str, (.0167837923, .0334362214, .0536769314, *angles))))
    assert calibration.parent_frame == 'livox_frame'
    assert calibration.point_in_parent(point) == pytest.approx(expected, abs=1.e-8)


@pytest.mark.parametrize('changes', [
    {'transform': 'parent_to_optical'}, {'version': 2}, {'parent_frame': ''},
    {'optical_frames': ['camera_depth_optical_frame']},
    {'translation_xyz': [0, 0]}, {'translation_xyz': [0, 0, float('nan')]},
    {'translation_xyz': [0, 0, 3]}, {'quaternion_xyzw': [0, 0, 0, 0]},
    {'quaternion_xyzw': [0, 0, 0, float('inf')]},
])
def test_invalid_calibration_is_rejected(tmp_path, changes):
    data = json.loads(CALIBRATION.read_text())
    data.update(changes)
    path = tmp_path / 'camera.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='标定文件不可用'):
        load_camera_calibration(str(path), 'camera_color_optical_frame')


def test_missing_calibration_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='标定'):
        load_camera_calibration(str(tmp_path / 'missing.json'), config.VISUAL_CAMERA_FRAME)
