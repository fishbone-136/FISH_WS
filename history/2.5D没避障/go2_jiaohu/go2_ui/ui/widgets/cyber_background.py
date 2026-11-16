import math
import random
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPixmap, QLinearGradient, QRadialGradient, QColor, QPen, QBrush, QPainterPath
from PyQt5.QtCore import QTimer, QPointF, Qt


class CyberBackground(QWidget):
    """深空霓虹背景 — 渐变底 + 点阵网格 + 浮动粒子 + 扫描线 + 边缘暗角"""

    SCAN_COUNT = 4
    SCAN_MIN_SPEED = 0.4
    SCAN_MAX_SPEED = 1.0
    GRID_SPACING = 42
    ACCENT_COUNT = 35
    PARTICLE_COUNT = 50

    def __init__(self, parent=None, animated=False):
        super().__init__(parent)
        self._animated = animated
        self._background_cache = None
        self._scan_lines = []
        self._grid_dots = []
        self._accent_dots = []
        self._particles = []

        self._build_grid()
        self._init_scan_lines()
        self._init_particles()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._animate)
        # A full-window animation also repaints transparent child panels.
        # Keep the dashboard decoration static so camera frames get priority.
        if animated:
            self._timer.start(50)

    # ------------------------------------------------------------------
    # 初始化数据
    # ------------------------------------------------------------------

    def _build_grid(self):
        w = max(self.width(), 2600)
        h = max(self.height(), 1500)
        spacing = self.GRID_SPACING

        self._grid_dots.clear()
        for x in range(0, w + spacing, spacing):
            for y in range(0, h + spacing, spacing):
                ox = x + random.uniform(-2.5, 2.5)
                oy = y + random.uniform(-2.5, 2.5)
                self._grid_dots.append((ox, oy))

        self._accent_dots.clear()
        for _ in range(self.ACCENT_COUNT):
            self._accent_dots.append((
                random.uniform(0, w),
                random.uniform(0, h),
                random.uniform(1.2, 2.8),
            ))

    def _init_scan_lines(self):
        self._scan_lines.clear()
        for _ in range(self.SCAN_COUNT):
            self._scan_lines.append({
                'y': random.uniform(0, max(self.height(), 600)),
                'speed': random.uniform(self.SCAN_MIN_SPEED, self.SCAN_MAX_SPEED),
                'alpha': random.randint(12, 30),
                'width_ratio': random.uniform(0.55, 0.85),
            })

    def _init_particles(self):
        self._particles.clear()
        for _ in range(self.PARTICLE_COUNT):
            self._particles.append(self._new_particle(random.uniform(0, max(self.height(), 900))))

    def _new_particle(self, start_y=None):
        w = max(self.width(), 400)
        h = max(self.height(), 400)
        return {
            'x': random.uniform(0, w),
            'y': start_y if start_y is not None else random.uniform(0, h),
            'speed': random.uniform(0.25, 0.9),
            'size': random.uniform(1.2, 3.0),
            'alpha': random.randint(18, 55),
            'wobble_phase': random.uniform(0, math.pi * 2),
            'wobble_amp': random.uniform(0.15, 0.7),
        }

    # ------------------------------------------------------------------
    # 动画
    # ------------------------------------------------------------------

    def _animate(self):
        h = self.height()
        for s in self._scan_lines:
            s['y'] += s['speed']
            if s['y'] > h + 20:
                s['y'] = -20
                s['speed'] = random.uniform(self.SCAN_MIN_SPEED, self.SCAN_MAX_SPEED)
                s['width_ratio'] = random.uniform(0.55, 0.85)

        for p in self._particles:
            p['y'] -= p['speed']
            p['x'] += math.sin(self._anim_frame() * 0.025 + p['wobble_phase']) * p['wobble_amp']
            if p['y'] < -20:
                new = self._new_particle(h + random.uniform(10, 60))
                p.update(new)

        self.update()

    _frame = 0

    @classmethod
    def _anim_frame(cls):
        cls._frame += 1
        return cls._frame

    # ------------------------------------------------------------------
    # 绘制
    # ------------------------------------------------------------------

    def paintEvent(self, event):
        if not self._animated:
            if self._background_cache is None:
                self._background_cache = QPixmap(self.size())
                cached_painter = QPainter(self._background_cache)
                self._paint_background(cached_painter)
            painter = QPainter(self)
            painter.drawPixmap(0, 0, self._background_cache)
            painter.end()
            return
        painter = QPainter(self)
        self._paint_background(painter)

    def _paint_background(self, painter):
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # ---- 1. 渐变底色 ----
        gradient = QLinearGradient(0, 0, 0, h)
        gradient.setColorAt(0.0, QColor('#080C16'))
        gradient.setColorAt(0.4, QColor('#060A12'))
        gradient.setColorAt(1.0, QColor('#04080C'))
        painter.fillRect(self.rect(), gradient)

        # ---- 2. 点阵网格 ----
        painter.setPen(Qt.NoPen)
        grid_color = QColor(0, 220, 255, 10)
        for x, y in self._grid_dots:
            if 0 <= x <= w and 0 <= y <= h:
                painter.setBrush(grid_color)
                painter.drawEllipse(QPointF(x, y), 1.0, 1.0)

        accent_color = QColor(0, 235, 255, 30)
        for x, y, size in self._accent_dots:
            if 0 <= x <= w and 0 <= y <= h:
                painter.setBrush(accent_color)
                painter.drawEllipse(QPointF(x, y), size, size)

        # ---- 3. 浮动粒子 ----
        for p in self._particles:
            px, py = p['x'], p['y']
            if -10 <= px <= w + 10 and -10 <= py <= h + 10:
                # 辉光
                glow = QRadialGradient(px, py, p['size'] * 2.5)
                glow.setColorAt(0.0, QColor(0, 240, 255, p['alpha']))
                glow.setColorAt(1.0, QColor(0, 240, 255, 0))
                painter.setBrush(glow)
                painter.drawEllipse(QPointF(px, py), p['size'] * 2.5, p['size'] * 2.5)
                # 核心
                painter.setBrush(QColor(0, 240, 255, min(255, p['alpha'] + 30)))
                painter.drawEllipse(QPointF(px, py), p['size'] * 0.7, p['size'] * 0.7)

        # ---- 4. 扫描线 ----
        for s in self._scan_lines:
            sy = s['y']
            a = s['alpha']
            wr = s['width_ratio']
            margin = int(w * (1.0 - wr) / 2)

            glow = QLinearGradient(0, sy - 8, 0, sy + 8)
            glow.setColorAt(0.0, QColor(0, 240, 255, 0))
            glow.setColorAt(0.25, QColor(0, 240, 255, max(0, a // 4)))
            glow.setColorAt(0.5, QColor(0, 240, 255, a))
            glow.setColorAt(0.75, QColor(0, 240, 255, max(0, a // 4)))
            glow.setColorAt(1.0, QColor(0, 240, 255, 0))
            painter.setPen(Qt.NoPen)
            painter.setBrush(glow)
            painter.drawRect(margin, int(sy - 8), w - 2 * margin, 16)

        # ---- 5. 边缘暗角 ----
        vignette = QRadialGradient(w / 2, h / 2, max(w, h) * 0.62)
        vignette.setColorAt(0.0, QColor(0, 0, 0, 0))
        vignette.setColorAt(0.65, QColor(0, 0, 0, 20))
        vignette.setColorAt(1.0, QColor(0, 0, 0, 100))
        painter.setBrush(vignette)
        painter.drawRect(0, 0, w, h)

        # ---- 6. 四角 HUD 装饰线 ----
        gap, arm = 32, 50
        painter.setPen(QPen(QColor(0, 200, 255, 25), 1, Qt.SolidLine, Qt.RoundCap))
        # 左上
        painter.drawLine(gap, gap + arm, gap, gap)
        painter.drawLine(gap, gap, gap + arm, gap)
        # 右上
        painter.drawLine(w - gap - arm, gap, w - gap, gap)
        painter.drawLine(w - gap, gap, w - gap, gap + arm)
        # 左下
        painter.drawLine(gap, h - gap - arm, gap, h - gap)
        painter.drawLine(gap, h - gap, gap + arm, h - gap)
        # 右下
        painter.drawLine(w - gap - arm, h - gap, w - gap, h - gap)
        painter.drawLine(w - gap, h - gap - arm, w - gap, h - gap)

        painter.end()

    # ------------------------------------------------------------------
    # resize
    # ------------------------------------------------------------------

    def resizeEvent(self, event):
        self._background_cache = None
        self._build_grid()
        self._init_particles()
        super().resizeEvent(event)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)
