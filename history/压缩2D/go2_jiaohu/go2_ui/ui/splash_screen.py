from PyQt5.QtWidgets import *
from PyQt5.QtCore import *


class SplashScreen(QSplashScreen):
    def __init__(self, pix):
        super().__init__(pix)
        self.showMessage("FISHBONE Loading...", Qt.AlignBottom)
