"""隔离的 Unitree SDK2 动作进程；stdin/stdout 使用 JSON Lines 协议。"""

from __future__ import annotations

import json
import math
import os
import select
import sys
import time
from typing import Any

from . import config


def _send_status(
    status: str, error: str = "", request_id: str = "", **details: Any
) -> None:
    payload = {"status": status, "error": error, **details}
    if request_id:
        payload["request_id"] = request_id
    print(json.dumps(payload, ensure_ascii=False), flush=True)


_MOTION_ACTIONS = {"forward", "backward", "left", "right", "turn_left", "turn_right"}
_INSTANT_ACTIONS = {"stop", "sit", "stand", "recovery"}


def _visual_velocity_spec(payload, now):
    """Short, timestamped leases for depth-feedback control; never timed travel."""
    vx, wz, issued, lease = (float(payload[key]) for key in ('vx', 'wz', 'issued_at', 'lease'))
    if (not all(math.isfinite(v) for v in (vx, wz, issued, lease))
            or not 0. <= vx <= min(.20, config.MAX_LINEAR_SPEED)
            or abs(wz) > min(.30, config.MAX_TURN_SPEED)
            or not .10 <= lease <= .30 or not 0. <= now-issued <= .15 or now >= issued+lease):
        raise ValueError('视觉闭环速度或租约无效/已过期')
    return (vx, 0., wz), issued+lease


def _motion_spec(
    action: str, speed: float, duration: float, angle: float
) -> tuple[tuple[float, float, float], float]:
    """把已校验动作映射到 SDK 速度；不得再次放大或重算租约时间。"""
    del angle  # 角度已经在 validate_actions 中换算成最终 duration。
    max_speed = (
        config.MAX_TURN_SPEED if action.startswith("turn_") else config.MAX_LINEAR_SPEED
    )
    speed = min(max(abs(float(speed)), 0.10), max_speed)
    duration = min(
        max(float(duration), 0.10),
        min(config.MAX_ACTION_DURATION, config.MAX_TOTAL_DURATION),
    )
    values = {
        "forward": (speed, 0.0, 0.0),
        "backward": (-speed, 0.0, 0.0),
        "left": (0.0, speed, 0.0),
        "right": (0.0, -speed, 0.0),
        "turn_left": (0.0, 0.0, speed),
        "turn_right": (0.0, 0.0, -speed),
    }
    return values[action], duration


def _result_code(result: Any) -> int:
    if isinstance(result, bool):
        return 0 if result else -1
    if isinstance(result, int):
        return result
    raise RuntimeError(f"SDK 返回了未知结果: {result!r}")


def _execute_instant(client, action: str) -> int:
    methods = {
        "stop": client.StopMove,
        "sit": client.StandDown,
        "stand": client.StandUp,
        "recovery": client.RecoveryStand,
    }
    if action not in methods:
        raise ValueError(f"不支持的 SDK 动作: {action or '?'}")
    return _result_code(methods[action]())


def _stop_active_motion(client) -> int:
    """先发零速度再调用有响应的 StopMove，降低单个 UDP 包丢失的影响。"""
    for _ in range(3):
        try:
            _result_code(client.Move(0.0, 0.0, 0.0))
        except Exception:
            pass
        time.sleep(config.ROBOT_SDK_MOVE_INTERVAL)
    # 即便零速度帧失败，也必须继续尝试有响应的 StopMove。
    stop_code = _result_code(client.StopMove())
    return stop_code if stop_code != 0 else 0


def _run_command_loop(client, input_stream) -> None:
    """运行可抢占动作循环；每条请求都返回首个 SDK API 的结果。"""
    active_motion: tuple[float, float, float] | None = None
    motion_deadline = 0.0
    next_move_at = 0.0
    input_buffer = ""
    input_fd = input_stream.fileno()
    should_exit = False

    while not should_exit:
        now = time.monotonic()
        if active_motion is not None:
            next_event = min(motion_deadline, next_move_at)
            timeout = min(0.10, max(0.0, next_event - now))
        else:
            timeout = 0.25
        readable, _, _ = select.select([input_fd], [], [], timeout)

        if readable:
            chunk = os.read(input_fd, 4096)
            if not chunk:
                break
            input_buffer += chunk.decode("utf-8", errors="replace")

        while "\n" in input_buffer and not should_exit:
            line, input_buffer = input_buffer.split("\n", 1)
            if not line.strip():
                continue
            request_id = ""
            control = ""
            try:
                payload = json.loads(line)
                request_id = str(payload.get("request_id", ""))
                control = str(payload.get("command", ""))
                if control in {"shutdown", "quit"}:
                    if control == "shutdown" or active_motion is not None:
                        _stop_active_motion(client)
                    active_motion = None
                    should_exit = True
                    continue

                if control == 'visual_velocity':
                    motion, deadline = _visual_velocity_spec(payload, time.monotonic())
                    code = _result_code(client.Move(*motion))
                    if code != 0:
                        raise RuntimeError(f'视觉闭环 Move 失败，SDK code={code}')
                    active_motion = motion
                    motion_deadline = deadline
                    next_move_at = time.monotonic()+config.ROBOT_SDK_MOVE_INTERVAL
                    _send_status('accepted', request_id=request_id)
                    continue

                action = str(payload.get("action", "")).strip().lower()
                speed = float(payload.get("speed", 0.3))
                duration = float(payload.get("duration", 2.0))
                angle = float(payload.get("angle", 0.0))

                if action not in _MOTION_ACTIONS | _INSTANT_ACTIONS:
                    raise ValueError(f"动作不在 SDK 白名单中: {action or '?'}")

                # A live command with the same velocity is a lease renewal.
                # Never insert zero velocity / StopMove between its heartbeats.
                # Expired leases and actual direction changes still stop first.
                if action in _MOTION_ACTIONS:
                    motion, lease = _motion_spec(action, speed, duration, angle)
                    if active_motion == motion and time.monotonic() < motion_deadline:
                        code = _result_code(client.Move(*motion))
                        if code != 0:
                            _stop_active_motion(client)
                            active_motion = None
                            raise RuntimeError(f"Move 续期失败，SDK code={code}")
                        motion_deadline = time.monotonic() + lease
                        next_move_at = time.monotonic() + config.ROBOT_SDK_MOVE_INTERVAL
                        _send_status("accepted", request_id=request_id, action=action)
                        continue

                previous_stop_code: int | None = None
                if active_motion is not None:
                    stop_code = _stop_active_motion(client)
                    active_motion = None
                    if stop_code != 0:
                        raise RuntimeError(f"停止上一动作失败，SDK code={stop_code}")
                    previous_stop_code = stop_code

                if action in _MOTION_ACTIONS:
                    motion, lease = _motion_spec(action, speed, duration, angle)
                    code = _result_code(client.Move(*motion))
                    if code != 0:
                        raise RuntimeError(f"Move 返回错误，SDK code={code}")
                    active_motion = motion
                    motion_deadline = time.monotonic() + lease
                    next_move_at = time.monotonic() + config.ROBOT_SDK_MOVE_INTERVAL
                elif action == "stop":
                    code = (
                        previous_stop_code
                        if previous_stop_code is not None
                        else _stop_active_motion(client)
                    )
                    if code != 0:
                        raise RuntimeError(f"stop 返回错误，SDK code={code}")
                else:
                    code = _execute_instant(client, action)
                    if code != 0:
                        raise RuntimeError(f"{action} 返回错误，SDK code={code}")
                _send_status("accepted", request_id=request_id, action=action)
            except Exception as exc:
                if control == 'visual_velocity' and active_motion is not None:
                    try:
                        _stop_active_motion(client)
                    finally:
                        active_motion = None
                _send_status("rejected", str(exc), request_id=request_id)

        if active_motion is not None and not should_exit:
            now = time.monotonic()
            if now >= motion_deadline:
                code = _stop_active_motion(client)
                if code != 0:
                    print(
                        f"[Unitree SDK worker] 自动停车失败，SDK code={code}",
                        file=sys.stderr,
                        flush=True,
                    )
                active_motion = None
            elif now >= next_move_at:
                code = _result_code(client.Move(*active_motion))
                if code != 0:
                    print(
                        f"[Unitree SDK worker] Move 心跳失败，SDK code={code}",
                        file=sys.stderr,
                        flush=True,
                    )
                    try:
                        _stop_active_motion(client)
                    except Exception as exc:
                        print(
                            f"[Unitree SDK worker] Move 失败后停车异常: {exc}",
                            file=sys.stderr,
                            flush=True,
                        )
                    active_motion = None
                next_move_at = now + config.ROBOT_SDK_MOVE_INTERVAL

    if active_motion is not None:
        _stop_active_motion(client)


def main() -> int:
    interface = sys.argv[1] if len(sys.argv) > 1 else config.ROBOT_INTERFACE
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        ChannelFactoryInitialize(0, interface)
        client = SportClient()
        client.SetTimeout(config.ROBOT_SDK_RPC_TIMEOUT)
        client.Init()
        api_version = "not-verified"
        if config.ROBOT_SDK_VERIFY_SERVER:
            code, version = client.GetServerApiVersion()
            if _result_code(code) != 0:
                raise RuntimeError(f"Go2 Sport 服务探测失败，SDK code={code}")
            api_version = str(version or "unknown")
    except Exception as exc:
        _send_status("error", str(exc))
        return 1

    _send_status("ready", api_version=api_version, interface=interface)
    _run_command_loop(client, sys.stdin)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
