"""Embed an owned RViz2 window; never reparent or stop an existing RViz."""
from __future__ import annotations

import ctypes
import ctypes.util
from copy import deepcopy
import logging
import os
from pathlib import Path
import time
import yaml

from PyQt5.QtCore import QProcess, QProcessEnvironment, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


PROJECT_ROOT = Path(__file__).resolve().parents[3]
# Qt 5 QMainWindow.saveState(): Tools toolbar visible, Navigation 2 dock hidden.
# Use an explicit hidden state: RViz's collapse flag alone cannot hide a dock
# during startup, before its parent window becomes visible.
EMBEDDED_WINDOW_STATE = (
    '000000ff00000000fd00000001000000000000000000000000fc0200000001fb00000018'
    '004e0061007600690067006100740069006f006e002000320000000000ffffffff00000017'
    '00ffffff000000000000000000000004000000040000000800000008fc0000000100000002'
    '000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000'
)


def rviz_config_path() -> Path:
    source = PROJECT_ROOT / 'ros2_ws/install/go2_lio_navigation/share/go2_lio_navigation/config/lio_environment.rviz'
    profile = build_ui_rviz_profile(yaml.safe_load(source.read_text()))
    target = PROJECT_ROOT / 'log/ui_navigation/terrain_view.rviz'
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(yaml.safe_dump(profile, sort_keys=False, allow_unicode=True))
    temporary.replace(target)
    return target


def build_ui_rviz_profile(source: dict) -> dict:
    """Adapt presentation without editing the navigation profile or its topics."""
    profile = deepcopy(source)
    # GoalTool dispatches through the Nav2 panel's in-process signal handler.
    # Keep that panel loaded behind the hidden dock so map goals still work.
    profile['Panels'] = [panel for panel in profile.get('Panels', [])
                         if panel.get('Class') == 'nav2_rviz_plugins/Navigation 2']
    manager = profile['Visualization Manager']
    manager['Global Options']['Background Color'] = '8; 12; 20'
    view = manager['Views']['Current']
    view.update({'Distance': 14.0, 'Target Frame': 'base_link',
                 'Focal Point': {'X': 0.0, 'Y': 0.0, 'Z': -0.2},
                 'Pitch': 1.18, 'Yaw': 1.57})
    profile['Window Geometry'] = {
        'Height': 720, 'Width': 1100, 'Hide Left Dock': True,
        'Hide Right Dock': True, 'X': 0, 'Y': 0,
        'QMainWindow State': EMBEDDED_WINDOW_STATE,
        'Navigation 2': {'collapsed': False},
    }
    return profile


def rviz_style_path() -> Path:
    return PROJECT_ROOT / 'go2_jiaohu/go2_ui/resources/qss/rviz_navigation.qss'


class X11Windows:
    """Minimal Xlib access for the PID property, without extra packages."""
    def __init__(self):
        self.lib = ctypes.CDLL(ctypes.util.find_library('X11'))
        self.lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.lib.XOpenDisplay.restype = ctypes.c_void_p
        self.lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.lib.XDefaultRootWindow.restype = ctypes.c_ulong
        self.lib.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        self.lib.XInternAtom.restype = ctypes.c_ulong
        self.lib.XGetWindowProperty.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
            ctypes.c_long, ctypes.c_long, ctypes.c_int, ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte))]
        self.lib.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint)]
        self.lib.XFree.argtypes = [ctypes.c_void_p]
        self.lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.display = self.lib.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('无法连接 X11 显示服务')
        self.root = self.lib.XDefaultRootWindow(self.display)
        # RViz may close between discovery and a property request. Suppress
        # errors only for our connection; never let a stale window kill UI.
        self.lib.XSetErrorHandler.argtypes = [ctypes.c_void_p]
        self.lib.XSetErrorHandler.restype = ctypes.c_void_p
        self._handler_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)
        def handle(display, event):
            if display == self.display:
                return 0
            return self._previous_handler(display, event) if self._previous_handler else 0
        self._handler = self._handler_type(handle)
        previous = self.lib.XSetErrorHandler(ctypes.cast(self._handler, ctypes.c_void_p))
        self._previous_pointer = previous
        self._previous_handler = self._handler_type(previous) if previous else None

    def property(self, window, name):
        atom = self.lib.XInternAtom(self.display, name.encode(), 0)
        actual, fmt = ctypes.c_ulong(), ctypes.c_int()
        count, remaining = ctypes.c_ulong(), ctypes.c_ulong()
        data = ctypes.POINTER(ctypes.c_ubyte)()
        result = self.lib.XGetWindowProperty(self.display, window, atom, 0, 4096, 0, 0,
            ctypes.byref(actual), ctypes.byref(fmt), ctypes.byref(count),
            ctypes.byref(remaining), ctypes.byref(data))
        try:
            if result != 0 or fmt.value != 32 or not data:
                return []
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))
            return [int(values[i]) for i in range(count.value)]
        finally:
            if data:
                self.lib.XFree(data)

    def children(self, window):
        root, parent, count = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_uint()
        children = ctypes.POINTER(ctypes.c_ulong)()
        ok = self.lib.XQueryTree(self.display, window, ctypes.byref(root), ctypes.byref(parent),
                                ctypes.byref(children), ctypes.byref(count))
        try:
            return [int(children[i]) for i in range(count.value)] if ok else []
        finally:
            if children:
                self.lib.XFree(children)

    def find(self, pid):
        windows = self.property(self.root, '_NET_CLIENT_LIST') or self.children(self.root)
        normal = self.lib.XInternAtom(self.display, b'_NET_WM_WINDOW_TYPE_NORMAL', 0)
        # Main-window PID is enough under a desktop WM; one child level also
        # supports a plain X server without EWMH.
        for window in windows:
            if self.property(window, '_NET_WM_PID') == [pid]:
                kind = self.property(window, '_NET_WM_WINDOW_TYPE')
                if not kind or normal in kind:
                    return window
                continue  # Do not attach a transient splash screen/dialog.
            for child in self.children(window):
                if self.property(child, '_NET_WM_PID') == [pid]:
                    return child
        return None

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display)
            self.lib.XSetErrorHandler(self._previous_pointer)
            self.display = None

    def embed(self, window, parent):
        # Bypass WM reparenting of a foreign Qt top-level window. A plain
        # fromWinId container can be pulled back out by GNOME/XWayland.
        class Attributes(ctypes.Structure):
            _fields_ = [('background_pixmap',ctypes.c_ulong),('background_pixel',ctypes.c_ulong),
                ('border_pixmap',ctypes.c_ulong),('border_pixel',ctypes.c_ulong),
                ('bit_gravity',ctypes.c_int),('win_gravity',ctypes.c_int),
                ('backing_store',ctypes.c_int),('backing_planes',ctypes.c_ulong),
                ('backing_pixel',ctypes.c_ulong),('save_under',ctypes.c_int),
                ('event_mask',ctypes.c_long),('do_not_propagate_mask',ctypes.c_long),
                ('override_redirect',ctypes.c_int),('colormap',ctypes.c_ulong),('cursor',ctypes.c_ulong)]
        attributes=Attributes();attributes.override_redirect=1
        self.lib.XChangeWindowAttributes.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.POINTER(Attributes)]
        self.lib.XReparentWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_int,ctypes.c_int]
        self.lib.XMapWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
        self.lib.XUnmapWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
        self.lib.XUnmapWindow(self.display,window)
        self.lib.XChangeWindowAttributes(self.display,window,1<<9,ctypes.byref(attributes))
        self.lib.XReparentWindow(self.display,window,parent,0,0)
        self.lib.XMapWindow(self.display,window)

    def fit(self, window, width, height, parent):
        root,actual_parent,count=ctypes.c_ulong(),ctypes.c_ulong(),ctypes.c_uint()
        children=ctypes.POINTER(ctypes.c_ulong)()
        ok=self.lib.XQueryTree(self.display,window,ctypes.byref(root),ctypes.byref(actual_parent),
            ctypes.byref(children),ctypes.byref(count))
        if children:self.lib.XFree(children)
        if not ok:return
        # The WM may finish unmanaging the original top-level asynchronously
        # and move it back to root once. Reassert the native container parent.
        if actual_parent.value!=parent:self.embed(window,parent)
        self.lib.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
        self.lib.XFlush.argtypes=[ctypes.c_void_p]
        self.lib.XMoveResizeWindow(self.display,window,0,0,max(1,width),max(1,height))
        self.lib.XFlush(self.display)


class RvizView(QWidget):
    log_message = pyqtSignal(str)
    expand_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._foreign = self._container = None
        self._window_id = self._candidate_window = None
        self._candidate_since = 0.
        self._started_at = 0.
        self._x11 = None
        self._closing = False
        self._restart_requested = False
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._read_output)
        self._process.errorOccurred.connect(self._process_error)
        self._process.finished.connect(self._finished)
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._attach)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        toolbar = QHBoxLayout()
        self.toolbar = toolbar
        badge = QLabel('实时地形 · 跟随机器狗')
        badge.setStyleSheet('color:#7AA8C8; font-size:13px; border:none;')
        toolbar.addWidget(badge)
        toolbar.addStretch()
        reset = QPushButton('重置视角')
        reset.setToolTip('恢复以机器狗为中心的默认视角，不影响导航任务')
        reset.clicked.connect(self.reload_view)
        self.expand_button = QPushButton('展开地图')
        self.expand_button.clicked.connect(lambda _checked=False: self.expand_requested.emit())
        for button in (reset, self.expand_button):
            button.setStyleSheet('QPushButton {padding:3px 12px; min-height:26px; font-size:13px;}')
            toolbar.addWidget(button)
        self._layout.addLayout(toolbar)
        self.message = QLabel('正在准备 2.5D 地形导航视图…')
        self.message.setAlignment(Qt.AlignCenter)
        self.message.setWordWrap(True)
        self._layout.addWidget(self.message)
        self.retry = QPushButton('重新加载 2.5D 地形视图')
        self.retry.clicked.connect(self.start)
        self._layout.addWidget(self.retry)
        legend = QLabel('<span style="color:#5ABE91">● 可通行</span>　'
                        '<span style="color:#EB6455">● 障碍</span>　'
                        '<span style="color:#FFCD5A">━ 路径</span>　'
                        '<span style="color:#41AAFF">▲ 机器狗</span>')
        legend.setAlignment(Qt.AlignCenter)
        legend.setStyleSheet('font-size:13px; border:none; padding:4px;')
        self._layout.addWidget(legend)

    def set_expanded(self, expanded):
        self.expand_button.setText('恢复侧栏' if expanded else '展开地图')

    def reload_view(self):
        if self._process.state() == QProcess.NotRunning:
            self.start()
            return
        self._restart_requested = True
        self._closing = True
        pid = self._process.processId()
        self._process.terminate()
        def finish_stop():
            if self._process.state() != QProcess.NotRunning and self._process.processId() == pid:
                self._process.kill()
        QTimer.singleShot(3000, finish_stop)

    def _show_error(self, text):
        if self._process.state() == QProcess.NotRunning:
            self._timer.stop()
            if self._x11 is not None:
                self._x11.close()
                self._x11 = None
        self.message.setText(text)
        self.message.show()
        self.retry.show()
        self.log_message.emit(text)

    def _process_error(self, _error):
        if not self._closing:
            self._show_error(self._process.errorString())

    def start(self):
        if self._process.state() != QProcess.NotRunning:
            return
        if QGuiApplication.platformName() != 'xcb':
            self._show_error('RViz 嵌入需要 X11。请用 QT_QPA_PLATFORM=xcb bash scripts/run_ui.sh 启动。')
            return
        config = rviz_config_path()
        if not config.is_file():
            self._show_error(f'当前 RViz 配置不存在：{config}')
            return
        try:
            self._x11 = X11Windows()
        except Exception as error:
            self._show_error(str(error))
            return
        env = QProcessEnvironment.systemEnvironment()
        for name in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR', 'QT_PLUGIN_PATH'):
            env.remove(name)
        env.insert('QT_QPA_PLATFORM', 'xcb')
        self._process.setProcessEnvironment(env)
        self._closing = False
        self._started_at = time.monotonic()
        self._candidate_window = None
        self.message.setText('正在加载当前 2.5D 地形、障碍和导航路径…')
        self.retry.hide()
        self._process.start('/opt/ros/humble/lib/rviz2/rviz2',
                            ['-d', str(config), '-style', 'Fusion', '-stylesheet', str(rviz_style_path()),
                             '--ros-args', '-r', '__node:=fishbone_ui_rviz'])
        self._timer.start()

    def _attach(self):
        if self._process.state() == QProcess.NotRunning:
            return
        window = self._x11.find(int(self._process.processId()))
        if window is not None:
            if window == self._window_id:
                self._fit_window()
                return
            now = time.monotonic()
            if window != self._candidate_window:
                self._candidate_window = window
                self._candidate_since = now
                return
            # RViz recreates its main native window while loading a profile.
            # Wait for it to settle, and keep watching for later replacements.
            if now - self._started_at < 2. or now - self._candidate_since < 1.:
                return
            if self._container is not None:
                self._layout.removeWidget(self._container)
                self._container.hide()
                self._container.deleteLater()
            self._container = QWidget(self)
            self._container.setAttribute(Qt.WA_NativeWindow)
            self._container.setMinimumSize(480, 320)
            self._container.setFocusPolicy(Qt.StrongFocus)
            self.message.hide()
            self._layout.insertWidget(1, self._container, 1)
            self._container.show()
            self._layout.activate()
            self._window_id = window
            self._x11.embed(window,int(self._container.winId()))
            self._fit_window()
            self.log_message.emit('2.5D 地形视图已连接，可使用 Nav2 Goal 选择目标点')
        elif self._container is not None:
            self._fit_window()
        elif time.monotonic() - self._started_at > 45.:
            self._timer.stop()
            self._show_error('RViz 窗口加载超时；请查看日志，关闭视图后重新加载。')
            self._process.terminate()

    def _fit_window(self):
        if self._window_id is not None and self._container is not None:
            self._x11.fit(self._window_id,self._container.width(),self._container.height(),
                          int(self._container.winId()))

    def _read_output(self):
        lines = bytes(self._process.readAllStandardOutput()).decode('utf-8', errors='replace')
        for line in lines.splitlines():
            logging.getLogger(__name__).info('[RViz] %s', line)
            if 'error' in line.lower() or 'fatal' in line.lower():
                self.log_message.emit('[RViz] ' + line)

    def _finished(self, code, _status):
        self._timer.stop()
        self._window_id = self._candidate_window = None
        if self._container is not None:
            # Remove the native host after our RViz process has stopped.
            self._container.deleteLater()
            self._container = self._foreign = None
        if self._x11 is not None:
            self._x11.close()
            self._x11 = None
        if self._restart_requested:
            self._restart_requested = False
            QTimer.singleShot(0, self.start)
        elif not self._closing:
            self._show_error(f'RViz 显示进程已退出（{code}），可重新加载')

    def shutdown(self):
        self._restart_requested = False
        self._closing = True
        self._timer.stop()
        if self._process.state() != QProcess.NotRunning:
            self._process.terminate()
            if not self._process.waitForFinished(3000):
                self._process.kill()
                self._process.waitForFinished(1000)
        elif self._x11 is not None:
            self._x11.close()
            self._x11 = None
