"""隔离的 rclpy 动作桥；stdin/stdout 使用 JSON Lines。"""

from __future__ import annotations

import json
import os
import select
import sys
import time

from . import config


def _send(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def main() -> int:
    try:
        import rclpy
        from std_msgs.msg import String

        rclpy.init(args=None)
        node = rclpy.create_node(f"bangbang_interactive_bridge_{os.getpid()}")
        command_pub = node.create_publisher(String, config.ROBOT_COMMAND_TOPIC, 10)

        def on_status(message: String) -> None:
            try:
                payload = json.loads(message.data)
                if isinstance(payload, dict):
                    _send(payload)
            except (json.JSONDecodeError, TypeError):
                pass

        node.create_subscription(String, config.ROBOT_COMMAND_STATUS_TOPIC, on_status, 10)
        _send({
            "status": "ready",
            "command_topic": config.ROBOT_COMMAND_TOPIC,
            "status_topic": config.ROBOT_COMMAND_STATUS_TOPIC,
        })
    except Exception as exc:
        _send({"status": "error", "error": f"rclpy 运动桥初始化失败: {exc}"})
        return 1

    input_fd = sys.stdin.fileno()
    input_buffer = ""
    should_exit = False
    try:
        while not should_exit and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([input_fd], [], [], 0.03)
            if readable:
                chunk = os.read(input_fd, 4096)
                if not chunk:
                    break
                input_buffer += chunk.decode("utf-8", errors="replace")

            while "\n" in input_buffer and not should_exit:
                line, input_buffer = input_buffer.split("\n", 1)
                if not line.strip():
                    continue
                payload = {}
                try:
                    payload = json.loads(line)
                    command = str(payload.get("command", ""))
                    if command in {"shutdown", "quit"}:
                        if command == "shutdown" and command_pub.get_subscription_count() > 0:
                            stop = String()
                            stop.data = json.dumps({
                                "request_id": f"shutdown-{os.getpid()}-{time.monotonic_ns()}",
                                "action": "stop",
                                "speed": 0.0,
                                "duration": 0.0,
                                "angle": 0.0,
                            })
                            command_pub.publish(stop)
                            rclpy.spin_once(node, timeout_sec=0.05)
                        should_exit = True
                        continue

                    request_id = str(payload.get("request_id", ""))
                    if not request_id:
                        raise ValueError("缺少 request_id")
                    command_matches = command_pub.get_subscription_count()
                    status_matches = node.count_publishers(config.ROBOT_COMMAND_STATUS_TOPIC)
                    if command_matches == 0 or status_matches == 0:
                        _send({
                            "status": "rejected",
                            "request_id": request_id,
                            "error": (
                                "go2_base 命令/回执话题尚未完整匹配；"
                                "请先启动包含 go2_base 的建图或导航 launch"
                            ),
                        })
                        continue
                    message = String()
                    message.data = json.dumps(payload, ensure_ascii=False)
                    command_pub.publish(message)
                except Exception as exc:
                    _send({
                        "status": "rejected",
                        "request_id": str(payload.get("request_id", "")),
                        "error": str(exc),
                    })
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
