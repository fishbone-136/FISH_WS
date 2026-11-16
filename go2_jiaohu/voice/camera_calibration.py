"""Load measured optical-to-lidar extrinsics; body pose still comes from live TF."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import SimpleNamespace

from voice.visual_navigation import transform_point


@dataclass(frozen=True)
class CameraCalibration:
    parent_frame: str
    translation_xyz: tuple
    quaternion_xyzw: tuple

    def point_in_parent(self, point):
        transform = SimpleNamespace(
            translation=SimpleNamespace(**dict(zip(('x', 'y', 'z'), self.translation_xyz))),
            rotation=SimpleNamespace(**dict(zip(('x', 'y', 'z', 'w'), self.quaternion_xyzw))))
        return transform_point(point, transform)


def load_camera_calibration(path: str, optical_frame: str) -> CameraCalibration:
    """Only use a calibration explicitly matching this optical frame and direction."""
    if not path:
        raise ValueError('缺少 D435i 安装标定：未配置相机 TF、实测安装参数或联合标定文件。')
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(data, dict) or data.get('version') != 1 or data.get('transform') != 'optical_to_parent':
            raise ValueError('需要 version=1、optical_to_parent 方向的标定')
        parent, frames = data['parent_frame'], data['optical_frames']
        if (not isinstance(parent, str) or not parent or parent.startswith('/')
                or any(c.isspace() for c in parent) or parent == optical_frame):
            raise ValueError('父坐标系无效')
        if not isinstance(frames, list) or optical_frame not in frames:
            raise ValueError(f'标定不适用于光学坐标系 {optical_frame}')
        translation = tuple(float(v) for v in data['translation_xyz'])
        quaternion = tuple(float(v) for v in data['quaternion_xyzw'])
        if (len(translation) != 3 or len(quaternion) != 4
                or not all(math.isfinite(v) for v in (*translation, *quaternion))
                or math.sqrt(sum(v*v for v in translation)) > 2.
                or not .99 <= math.sqrt(sum(v*v for v in quaternion)) <= 1.01):
            raise ValueError('平移或旋转标定无效')
        return CameraCalibration(parent, translation, quaternion)
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        raise ValueError(f'D435i 联合标定文件不可用：{error}。未执行移动。') from error
