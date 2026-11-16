"""Timestamped, bounded logs shared by the terminal and desktop UI."""

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path


def configure_logging(log_path=None):
    path = Path(log_path) if log_path else Path(__file__).resolve().parents[2] / "log" / "ui_runtime.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    if not root.handlers:
        root.addHandler(logging.StreamHandler())
    for handler in root.handlers:
        handler.setFormatter(formatter)
    if not any(getattr(handler, "baseFilename", None) == str(path.resolve()) for handler in root.handlers):
        handler = RotatingFileHandler(path, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8")
        handler.setFormatter(formatter)
        root.addHandler(handler)
    logging.info("界面与语音运行日志：%s", path)
    return path
