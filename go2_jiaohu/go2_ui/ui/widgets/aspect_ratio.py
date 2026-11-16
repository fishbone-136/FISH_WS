"""Fit a camera viewport to its source without stretching or cropping."""
from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import QWidget, QSizePolicy


class AspectRatioContainer(QWidget):
    def __init__(self, view, width=4, height=3, parent=None):
        super().__init__(parent)
        self.view = view
        self._source_size = QSize(width, height)
        view.setParent(self)
        view.setMinimumSize(0, 0)
        view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet('background: transparent; border: none;')

    def set_aspect_ratio(self, width, height):
        if width > 0 and height > 0 and self._source_size != QSize(width, height):
            self._source_size = QSize(width, height)
            self._fit_view()

    def _fit_view(self):
        area = self.contentsRect()
        from PyQt5.QtCore import Qt
        size = self._source_size.scaled(area.size(), Qt.KeepAspectRatio)
        self.view.setGeometry(area.x() + (area.width() - size.width()) // 2,
                              area.y() + (area.height() - size.height()) // 2,
                              size.width(), size.height())

    def sizeHint(self):
        return QSize(320, 240)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._fit_view()
