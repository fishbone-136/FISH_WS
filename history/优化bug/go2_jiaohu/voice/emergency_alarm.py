"""本地一键报警：审计日志、ROS 2 事件和可选现场报警程序。"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
import re
import shlex
import shutil
import subprocess

from . import config


@dataclass(frozen=True)
class AlarmResult:
    success: bool
    channels: tuple[str, ...]
    errors: tuple[str, ...]


class EmergencyAlarm:
    """触发现场报警链路；不内置也不访问任何云端 API。"""

    def trigger(self, source_text: str = "voice") -> AlarmResult:
        event = {
            "event": "emergency_alert",
            "robot": config.ROBOT_NAME,
            "source": "voice",
            "recognized_text": source_text,
            "time": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        }
        channels: list[str] = []
        errors: list[str] = []

        try:
            log_dir = os.path.dirname(config.ALARM_LOG_PATH)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            with open(config.ALARM_LOG_PATH, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
            channels.append("local_log")
        except OSError as exc:
            errors.append(f"报警日志写入失败: {exc}")

        ros2 = shutil.which("ros2")
        if ros2 and config.ALARM_ROS_TOPIC:
            try:
                # `ros2 topic pub --once` 默认等待一个订阅者。先做短时发现，现场
                # 没有报警订阅者时安静跳过，避免每次本地报警都阻塞五秒并报超时。
                probe = subprocess.run(
                    [ros2, "topic", "info", config.ALARM_ROS_TOPIC,
                     "--no-daemon", "--spin-time", "0.3"],
                    check=False,
                    timeout=2.0,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                probe_output = f"{probe.stdout}\n{probe.stderr}"
                match = re.search(r"Subscription count:\s*(\d+)", probe_output)
                if probe.returncode == 0 and match and int(match.group(1)) > 0:
                    data = json.dumps(event, ensure_ascii=False).replace("'", "")
                    subprocess.run(
                        [ros2, "topic", "pub", "--once",
                         "--wait-matching-subscriptions", "1",
                         config.ALARM_ROS_TOPIC, "std_msgs/msg/String",
                         f"{{data: '{data}'}}"],
                        check=True,
                        timeout=4.0,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.PIPE,
                        text=True,
                    )
                    channels.append("ros2")
                elif probe.returncode != 0 and "Unknown topic" not in probe_output:
                    detail = (probe.stderr or probe.stdout).strip()
                    errors.append(
                        f"ROS 2 报警通道检查失败: {detail or f'退出码 {probe.returncode}'}"
                    )
            except (OSError, subprocess.SubprocessError) as exc:
                errors.append(f"ROS 2 报警发布失败: {exc}")

        if config.ALARM_COMMAND:
            try:
                command = shlex.split(config.ALARM_COMMAND)
                if not command:
                    raise ValueError("BANGBANG_ALARM_COMMAND 为空")
                env = os.environ.copy()
                env["BANGBANG_ALARM_EVENT"] = json.dumps(event, ensure_ascii=False)
                subprocess.run(
                    command,
                    check=True,
                    timeout=10.0,
                    env=env,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                channels.append("site_command")
            except (ValueError, OSError, subprocess.SubprocessError) as exc:
                errors.append(f"现场报警程序失败: {exc}")

        return AlarmResult(bool(channels), tuple(channels), tuple(errors))
