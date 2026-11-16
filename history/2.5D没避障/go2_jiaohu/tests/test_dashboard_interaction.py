"""Exercise real Qt button wiring and layout without opening robot devices."""

import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PyQt5")
from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from go2_ui.ui import main_control_panel as controls
from go2_ui.ui.QStackedWidget import Dashboard
from go2_ui.ui.right_panel import DepthWidget


@pytest.fixture
def dashboard(monkeypatch):
    app = QApplication.instance() or QApplication([])
    stylesheet = Path(__file__).parents[1] / "go2_ui/resources/qss/cyber_blue.qss"
    app.setStyleSheet(stylesheet.read_text())
    monkeypatch.setattr(controls.VideoWidget, "_start_camera", lambda self: None)
    monkeypatch.setattr(DepthWidget, "_start_realsense", lambda self: None)
    window = Dashboard()
    window.resize(1280, 720)
    window.show()
    app.processEvents()
    yield app, window
    window.close()
    app.processEvents()


def test_click_starts_once_and_switch_stops_previous(dashboard, monkeypatch):
    app, window = dashboard
    panel = window.main_panel
    events = []

    class Speech:
        is_running = False

        def start(self):
            events.append("start voice")
            self.is_running = True
            return True

        def stop(self):
            events.append("stop voice")
            self.is_running = False

    speech = Speech()
    monkeypatch.setattr(controls, "SpeechController", lambda **kw: speech)
    panel.hand_detector = SimpleNamespace(available=True)
    gesture = SimpleNamespace(
        reset=lambda: events.append("reset gesture"),
        stop_motion=lambda: events.append("stop gesture"),
    )
    monkeypatch.setattr(controls, "GestureController", lambda: gesture)

    QTest.mouseClick(panel.voice_btn, Qt.LeftButton)
    QTest.mouseClick(panel.voice_btn, Qt.LeftButton)
    assert events == ["start voice"]
    assert panel.voice_btn.property("active") is True

    QTest.mouseClick(panel.gesture_btn, Qt.LeftButton)
    assert events == ["start voice", "stop voice", "reset gesture"]
    assert gesture.is_system_active is True
    assert panel.video_widget._hand_detection_enabled is True
    assert panel.voice_btn.property("active") is False

    panel.stop_btn.click()
    assert events[-2:] == ["stop gesture", "reset gesture"]
    assert gesture.is_system_active is False
    assert panel._mode_manager.active is None
    assert panel.video_widget._hand_detection_enabled is False
    assert panel._mode_manager.active is None


def test_failed_start_reports_error_and_returns_to_idle(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel

    class Speech:
        is_running = False

        def start(self):
            self.on_error("测试麦克风不可用")
            return False

    monkeypatch.setattr(controls, "SpeechController", lambda **kw: Speech())
    QTest.mouseClick(panel.voice_btn, Qt.LeftButton)
    assert "测试麦克风不可用" in panel.log_top.toPlainText()
    assert panel._mode_manager.active is None
    assert panel.voice_btn.property("active") is False
    assert panel.stop_btn.isEnabled() is False


def test_stop_button_and_cancelled_click(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel
    events = []
    monkeypatch.setattr(panel, "_vlm_analyze_scene", lambda: events.append("start vision"))
    monkeypatch.setattr(panel, "_stop_vlm", lambda: events.append("stop vision"))
    QTest.mouseClick(panel.vlm_btn, Qt.RightButton)
    QTest.mousePress(panel.vlm_btn, Qt.LeftButton)
    QTest.mouseRelease(panel.vlm_btn, Qt.LeftButton, pos=panel.vlm_btn.rect().bottomRight() * 2)
    assert events == []
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    assert events == ["start vision"]
    panel.stop_btn.click()
    assert events == ["start vision", "stop vision"]
    assert panel.mode_label.text() == "待机"


@pytest.mark.parametrize("width,height", [(1920, 1080), (1280, 720), (1024, 768), (800, 600)])
def test_screen_size_keeps_controls_reachable(dashboard, width, height):
    app, window = dashboard
    window.resize(width, height)
    app.processEvents()
    assert window.width() == width
    assert window.height() == height
    assert hasattr(window, "bottom_nav")
    assert len(window.bottom_nav._btns) == 2
    assert not hasattr(window, "navigation_page")
    scroll = window.home_scroll
    panel = window.main_panel
    for button in (panel.voice_btn, panel.gesture_btn, panel.eye_btn,
                   panel.vlm_btn, panel.stop_btn):
        assert panel.top_left.rect().contains(button.geometry())
    if width >= 1024:
        assert scroll.horizontalScrollBar().maximum() == 0
        assert scroll.verticalScrollBar().maximum() == 0
    else:
        scroll.ensureWidgetVisible(panel.stop_btn)
        app.processEvents()
        assert scroll.viewport().rect().contains(
            panel.stop_btn.mapTo(scroll.viewport(), panel.stop_btn.rect().center())
        )


def test_internal_window_controls(dashboard):
    app, window = dashboard
    assert window.windowFlags() & Qt.FramelessWindowHint
    window.top_bar.maximize_btn.click()
    app.processEvents()
    assert window.isMaximized()
    assert window.top_bar.maximize_btn.toolTip() == "还原"
    window.top_bar.maximize_btn.click()
    app.processEvents()
    assert not window.isMaximized()
    window.top_bar.minimize_btn.click()
    app.processEvents()
    assert window.isMinimized()
    window.showNormal()
    window.top_bar.close_btn.click()
    assert not window.isVisible()


def test_scene_uses_d435i_and_text_follows_current_frame(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel
    frames = [object(), object()]
    callbacks = []
    seen = []
    monkeypatch.setattr(window.right_panel.depth_widget, "get_latest_color_frame", lambda: frames.pop(0))
    monkeypatch.setattr(panel.video_widget, "get_frame", lambda: pytest.fail("must use D435i"))

    class Vision:
        is_ready = True

        def analyze_scene_async(self, frame, on_result):
            seen.append(frame)
            callbacks.append(on_result)
            return True

        def query_scene_async(self, frame, text, on_result):
            seen.append(frame)
            callbacks.append(on_result)
            assert text == "左边是什么？"
            return True

        def cancel_pending(self):
            pass

    monkeypatch.setattr(controls, "VLMController", Vision)
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    assert not panel.send_btn.isEnabled()
    callbacks.pop(0)("桌子上有一只红色杯子")
    assert panel.send_btn.isEnabled()
    panel.log_bottom.setPlainText("左边是什么？")
    panel.send_btn.click()
    assert len(seen) == 2 and seen[0] is not seen[1]
    assert panel.log_bottom.toPlainText() == ""
    callbacks.pop(0)("左边是一把椅子")
    assert "你：左边是什么？" in panel.log_top.toPlainText()
    assert "邦邦：左边是一把椅子" in panel.log_top.toPlainText()


def test_no_d435i_does_not_use_preview(dashboard, monkeypatch):
    _, window = dashboard
    panel = window.main_panel
    monkeypatch.setattr(window.right_panel.depth_widget, "get_latest_color_frame", lambda: None)
    monkeypatch.setattr(panel.video_widget, "get_frame", lambda: pytest.fail("unexpected fallback"))
    QTest.mouseClick(panel.vlm_btn, Qt.LeftButton)
    assert panel._mode_manager.active is None
    assert "当前彩色画面不可用" in panel.log_top.toPlainText()


def test_stale_or_disconnected_d435i_frame_is_rejected(dashboard):
    import numpy as np

    _, window = dashboard
    camera = window.right_panel.depth_widget
    camera._buf_color_bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    camera._running = True
    camera._color_frame_time = time.monotonic() - 3
    assert camera.get_latest_color_frame() is None
    camera._color_frame_time = time.monotonic()
    frame = camera.get_latest_color_frame()
    assert frame is not None and frame is not camera._buf_color_bgr
    camera._running = False
    assert camera.get_latest_color_frame() is None


def test_preview_skips_repeated_samples_and_keeps_snapshot_independent(dashboard):
    import numpy as np

    _, window = dashboard
    camera = window.right_panel.depth_widget
    preview = window.main_panel.video_widget
    camera._running = True
    camera._color_frame_time = time.monotonic()
    camera._buf_color_bgr = np.full((8, 8, 3), (10, 20, 30), dtype=np.uint8)
    camera._color_frame_count = 1
    preview.set_external_color_source(camera.get_latest_color_sample)
    preview._update_frame()
    source_id = preview._scaled_source_id
    preview._update_frame()
    assert preview._scaled_source_id == source_id
    snapshot = preview.get_frame()
    snapshot[:] = 0
    assert camera._buf_color_bgr[0, 0].tolist() == [10, 20, 30]
    assert preview._pixmap.toImage().pixelColor(0, 0).getRgb()[:3] == (30, 20, 10)
    camera._buf_color_bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    camera._color_frame_count += 1
    preview._last_update_time = 0
    preview._update_frame()
    assert preview._scaled_source_id == source_id + 1


def test_depth_callback_ignores_duplicates_and_preserves_raw_data(dashboard):
    import numpy as np

    _, window = dashboard
    camera = window.right_panel.depth_widget
    camera._running = True
    raw = np.array([[0, 1000, 4000, 65535]], dtype=np.uint16)
    color = np.zeros((1, 4, 3), dtype=np.uint8)
    depth_frame = SimpleNamespace(get_frame_number=lambda: 1, get_data=lambda: raw)
    color_frame = SimpleNamespace(get_frame_number=lambda: 1, get_data=lambda: color)
    frames = SimpleNamespace(get_depth_frame=lambda: depth_frame, get_color_frame=lambda: color_frame)
    frame = SimpleNamespace(as_frameset=lambda: frames)
    camera._on_frameset(frame)
    np.testing.assert_array_equal(camera.get_latest_depth_frame(), raw)
    rgb = np.frombuffer(camera._buf_depth_bytes, dtype=np.uint8).reshape(1, 4, 3)
    assert rgb[0, 0].tolist() == rgb[0, 3].tolist() == [24, 12, 6]
    camera._on_frameset(frame)
    assert camera._frame_count == camera._color_frame_count == 1
