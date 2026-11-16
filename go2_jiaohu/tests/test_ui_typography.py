"""Check actual rendered text geometry in supported window sizes."""
import pytest
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QLabel, QStyle, QStyleOptionComboBox

from tests.test_dashboard_interaction import dashboard
from tests.test_current_navigation_ui import DisplayWorker
from go2_ui.navigation import navigation_page as nav


def assert_labels_fit(window):
    for label in window.findChildren(QLabel):
        if not label.isVisible() or not label.text() or label.textFormat() == Qt.RichText:
            continue
        if '<span' in label.text() or (label.pixmap() and not label.pixmap().isNull()):
            continue
        flags = int(label.alignment()) | (int(Qt.TextWordWrap) if label.wordWrap() else 0)
        bounds = label.fontMetrics().boundingRect(label.contentsRect(), flags, label.text())
        assert bounds.width() <= label.contentsRect().width() + 2, label.text()
        assert bounds.height() <= label.contentsRect().height() + 2, label.text()


@pytest.mark.parametrize('width,height', [(800,600), (1024,768), (1280,720), (1920,1080)])
def test_home_and_navigation_text_fit_at_window_sizes(dashboard, monkeypatch, width, height):
    app, window = dashboard
    monkeypatch.setattr(nav, 'RosNavigationWorker', DisplayWorker)
    monkeypatch.setattr(nav.RvizView, 'start', lambda self: None)
    monkeypatch.setattr(nav.RvizView, 'shutdown', lambda self: None)
    window.resize(width, height)
    app.processEvents()
    assert_labels_fit(window)
    window.switch_page(1)
    page = window.navigation_page
    page._autostart_timer.stop()
    page.worker.pose_updated.emit(-12.345, 6.789, -2.56)
    page.worker.status_updated.emit('error', '导航未启动：机器狗尚未站稳，请保持静止后点击启动 / 重试。')
    app.processEvents()
    assert_labels_fit(window)
    combo = page.mode_combo
    option = QStyleOptionComboBox()
    combo.initStyleOption(option)
    edit = combo.style().subControlRect(QStyle.CC_ComboBox, option, QStyle.SC_ComboBoxEditField, combo)
    assert combo.fontMetrics().horizontalAdvance(combo.currentText()) <= edit.width()
    assert page.goal_x.width() > 2 * page.goal_x.fontMetrics().horizontalAdvance('-1000.00 m') / 3
    assert page.emergency_stop_button.isVisible()
    assert page.worker.commands == []
