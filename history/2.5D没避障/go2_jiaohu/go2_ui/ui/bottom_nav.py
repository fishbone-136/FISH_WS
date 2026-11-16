import os
from PyQt5.QtWidgets import QWidget, QFrame, QLabel, QHBoxLayout, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap

_ICON_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "icons")

_BTN_STYLE = (
    "QFrame {"
    "  background-color: #0C1E3A;"
    "  border: 1px solid #1A3454;"
    "  border-radius: 0px;"
    "}"
    "QFrame[hover=\"true\"] {"
    "  background-color: #163358;"
    "  border: 1px solid #1A5080;"
    "}"
    "QFrame[pressed=\"true\"] {"
    "  background-color: #081428;"
    "  border: 1px solid #0F3F5F;"
    "}"
    "QFrame[active=\"true\"] {"
    "  background-color: #12345C;"
    "  border: 2px solid #00F0FF;"
    "}"
)

_TXT_STYLE = (
    "color: #00F0FF; font-size: 18px; font-weight: bold;"
    "background: transparent; border: none;"
)


class _NavButton(QFrame):
    clicked = pyqtSignal()

    def __init__(self, text: str, icon_path: str, parent=None):
        super().__init__(parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(_BTN_STYLE)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addStretch()

        icon_label = QLabel()
        icon_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        icon_label.setFixedSize(24, 24)
        icon_label.setStyleSheet("background: transparent; border: none;")
        if os.path.exists(icon_path):
            pix = QPixmap(icon_path).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            icon_label.setPixmap(pix)
        layout.addWidget(icon_label)
        layout.addSpacing(16)

        text_label = QLabel(text)
        text_label.setAttribute(Qt.WA_TransparentForMouseEvents)
        text_label.setStyleSheet(_TXT_STYLE)
        layout.addWidget(text_label)
        layout.addStretch()

    def _refresh(self):
        self.style().unpolish(self)
        self.style().polish(self)

    def mousePressEvent(self, event):
        self.setProperty("pressed", event.button() == Qt.LeftButton)
        self._refresh()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        clicked = (event.button() == Qt.LeftButton
                   and self.property("pressed") and self.rect().contains(event.pos()))
        self.setProperty("pressed", False)
        self._refresh()
        if clicked:
            self.clicked.emit()
        super().mouseReleaseEvent(event)

    def enterEvent(self, event):
        self.setProperty("hover", True)
        self._refresh()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.setProperty("hover", False)
        self.setProperty("pressed", False)
        self._refresh()
        super().leaveEvent(event)


class BottomNav(QWidget):
    switch_page = pyqtSignal(int)

    def __init__(self, parent=None, items=None):
        super().__init__(parent)
        self.setObjectName("BottomNav")
        self._btns: list[_NavButton] = []
        self._items = items
        self.init_ui()

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(8)

        items = self._items or [
            ("首页", 0, "home.svg"),
            ("导航", 1, "daoh.svg"),
            ("协同", 2, "xiet.svg"),
            ("任务", 3, "renw.svg"),
        ]

        for text, page_idx, icon_name in items:
            icon_path = os.path.join(_ICON_DIR, icon_name)
            btn = _NavButton(text, icon_path)
            btn.clicked.connect(lambda idx=page_idx: self.switch_page.emit(idx))
            self._btns.append(btn)
            layout.addWidget(btn, 1)

    def set_icon(self, index: int, icon_path: str):
        if 0 <= index < len(self._btns):
            btn = self._btns[index]
            btn_layout = btn.layout()
            icon_label = btn_layout.itemAt(1).widget()
            if isinstance(icon_label, QLabel) and os.path.exists(icon_path):
                pix = QPixmap(icon_path).scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                icon_label.setPixmap(pix)

    def set_active(self, index: int):
        for button_index, button in enumerate(self._btns):
            button.setProperty("active", button_index == index)
            button._refresh()
