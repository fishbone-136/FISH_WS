"""Camera lifecycle and real error reporting, without opening USB devices."""
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from deploy.person_follow import FollowCamera


def camera_sdk(monkeypatch):
    pipeline = SimpleNamespace(start=Mock(), stop=Mock())
    intrinsics = SimpleNamespace(width=640,height=480)
    reference = SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intrinsics),
        get_extrinsics_to=lambda p:SimpleNamespace(rotation=np.eye(3).flatten().tolist(),translation=[0.,0.,0.]))
    color_profile = SimpleNamespace(stream_type=lambda:'color')
    profile = SimpleNamespace(get_stream=lambda stream:reference, get_device=lambda:SimpleNamespace(
        first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:.001),
        query_sensors=lambda:[SimpleNamespace(get_stream_profiles=lambda:[color_profile])]))
    pipeline.start.return_value = profile
    settings = SimpleNamespace(enable_stream=Mock())
    align = SimpleNamespace(process=lambda frame:frame)
    sdk = SimpleNamespace(pipeline=lambda:pipeline, config=lambda:settings,
        align=lambda stream:align, stream=SimpleNamespace(color='color',depth='depth',infrared='infrared'),
        format=SimpleNamespace(bgr8='bgr8',z16='z16',y8='y8'), rs2_fov=lambda intrinsics:[78.7,63.1],
        rs2_deproject_pixel_to_point=lambda intrinsics,pixel,z:[pixel[0],pixel[1],z])
    monkeypatch.setitem(sys.modules, 'pyrealsense2', sdk)
    return pipeline, settings, profile


def test_usb2_compatible_profile_and_callback_lifecycle(monkeypatch):
    pipeline, settings, _ = camera_sdk(monkeypatch)
    camera = FollowCamera()
    camera.start()
    assert [call.args for call in settings.enable_stream.call_args_list] == [
        ('infrared',1,640,480,'y8',30), ('depth',640,480,'z16',30)]
    assert camera._align is None
    assert pipeline.start.call_args.args[1] == camera._receive
    camera.close()
    camera.close()
    pipeline.stop.assert_called_once()


def test_failed_start_preserves_root_error_and_never_stops_unstarted_pipeline(monkeypatch, caplog):
    pipeline, _, _ = camera_sdk(monkeypatch)
    pipeline.start.side_effect = RuntimeError("Couldn't resolve requests")
    camera = FollowCamera()
    camera.start()
    camera.close()
    with pytest.raises(ValueError, match="Couldn't resolve requests"):
        camera.snapshot()
    pipeline.stop.assert_not_called()
    assert "Couldn't resolve requests" in caplog.text
    assert '关闭跟随相机失败' not in caplog.text


def test_failure_after_successful_start_releases_camera(monkeypatch):
    pipeline, _, profile = camera_sdk(monkeypatch)
    profile.get_device = Mock(side_effect=RuntimeError('scale unavailable'))
    camera = FollowCamera()
    camera.start()
    with pytest.raises(ValueError, match='scale unavailable'):
        camera.snapshot()
    pipeline.stop.assert_called_once()


def test_callback_pairs_frames_converts_units_and_keeps_acquisition_time(monkeypatch):
    camera_sdk(monkeypatch)
    camera = FollowCamera(view='color')
    camera.start()
    camera._scale = .0005
    color_data = np.zeros((480,640,3), np.uint8)
    depth_data = np.full((480,640), 3000, np.uint16)
    color = SimpleNamespace(get_data=lambda:color_data,get_timestamp=lambda:100.)
    depth = SimpleNamespace(get_data=lambda:depth_data, profile=SimpleNamespace(
        as_video_stream_profile=lambda:SimpleNamespace(intrinsics='intrinsics')),get_timestamp=lambda:100.)
    frames = SimpleNamespace(get_color_frame=lambda:color, get_depth_frame=lambda:depth)
    camera._receive(SimpleNamespace(as_frameset=lambda:frames))
    sample = camera.snapshot()
    assert sample.color.shape[:2] == sample.depth_mm.shape == (480,640)
    assert sample.depth_mm[0,0] == 1500
    assert 0 <= time.monotonic()-sample.captured_monotonic < 1.
    assert sample.deproject(1.,2.,1.5) == (1.,2.,1.5)
    color_data[:] = 255
    depth_data[:] = 0
    assert sample.color[0,0,0] == 0
    assert sample.depth_mm[0,0] == 1500
    camera.close()


def test_infrared_uses_full_native_image_without_rgb_alignment(monkeypatch):
    camera_sdk(monkeypatch)
    camera = FollowCamera()
    camera.start()
    raw = np.tile(np.arange(640,dtype=np.uint8),(480,1))
    ir = SimpleNamespace(get_data=lambda:raw,get_timestamp=lambda:100.)
    depth = SimpleNamespace(get_data=lambda:np.full((480,640),2000,np.uint16),get_timestamp=lambda:100.)
    frames = SimpleNamespace(get_infrared_frame=lambda index:ir,get_depth_frame=lambda:depth)
    camera._receive(SimpleNamespace(as_frameset=lambda:frames))
    sample = camera.snapshot()
    assert sample.color.shape == (480,640,3)
    assert np.array_equal(sample.color[:,:,0],raw)
    assert np.array_equal(sample.color[:,:,2],raw)
    assert sample.deproject_many is not None
    camera.close()


def test_callback_error_is_visible_to_preview(monkeypatch):
    pipeline, _, _ = camera_sdk(monkeypatch)
    camera = FollowCamera()
    camera.start()
    camera._receive(SimpleNamespace(as_frameset=Mock(side_effect=RuntimeError('USB disconnected'))))
    with pytest.raises(ValueError, match='USB disconnected'):
        camera.snapshot()
    camera.close()
    pipeline.stop.assert_called_once()


def test_missing_frames_report_timeout(monkeypatch):
    camera_sdk(monkeypatch)
    camera = FollowCamera()
    camera.start()
    camera._started_at -= 6.
    with pytest.raises(ValueError, match='5 秒'):
        camera.snapshot()
    camera.close()
