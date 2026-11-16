import json
import os
import tempfile
import unittest
from subprocess import CompletedProcess
from unittest.mock import patch

from voice import config
from voice.emergency_alarm import EmergencyAlarm


class EmergencyAlarmTests(unittest.TestCase):
    def test_local_alarm_is_recorded_without_claiming_external_channel(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "alarm.jsonl")
            with (
                patch.object(config, "ALARM_LOG_PATH", path),
                patch.object(config, "ALARM_COMMAND", ""),
                patch("voice.emergency_alarm.shutil.which", return_value=None),
            ):
                result = EmergencyAlarm().trigger("一键报警")
            self.assertTrue(result.success)
            self.assertEqual(result.channels, ("local_log",))
            with open(path, "r", encoding="utf-8") as stream:
                event = json.loads(stream.readline())
            self.assertEqual(event["robot"], "邦邦")
            self.assertEqual(event["event"], "emergency_alert")

    def test_missing_ros_subscriber_is_skipped_without_timeout_error(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "alarm.jsonl")
            probe = CompletedProcess(
                args=[], returncode=1, stdout="", stderr="Unknown topic"
            )
            with (
                patch.object(config, "ALARM_LOG_PATH", path),
                patch.object(config, "ALARM_COMMAND", ""),
                patch("voice.emergency_alarm.shutil.which", return_value="/opt/ros2"),
                patch("voice.emergency_alarm.subprocess.run", return_value=probe) as run,
            ):
                result = EmergencyAlarm().trigger("一键报警")
            self.assertEqual(result.channels, ("local_log",))
            self.assertEqual(result.errors, ())
            self.assertEqual(run.call_count, 1)

    def test_ros_alarm_publishes_when_a_subscriber_exists(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "alarm.jsonl")
            probe = CompletedProcess(
                args=[], returncode=0,
                stdout="Publisher count: 0\nSubscription count: 1\n", stderr="",
            )
            published = CompletedProcess(args=[], returncode=0, stdout="", stderr="")
            with (
                patch.object(config, "ALARM_LOG_PATH", path),
                patch.object(config, "ALARM_COMMAND", ""),
                patch("voice.emergency_alarm.shutil.which", return_value="/opt/ros2"),
                patch(
                    "voice.emergency_alarm.subprocess.run",
                    side_effect=(probe, published),
                ) as run,
            ):
                result = EmergencyAlarm().trigger("一键报警")
            self.assertEqual(result.channels, ("local_log", "ros2"))
            self.assertEqual(result.errors, ())
            self.assertEqual(run.call_count, 2)


if __name__ == "__main__":
    unittest.main()
