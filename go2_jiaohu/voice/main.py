"""邦邦文本语音桥服务端。

机器 A 可把本地 ASR 识别后的 UTF-8 文本按行发到 9999 端口；机器 B 复用与
主 UI 完全相同的 Ollama、安全口令、动作、视觉引导和报警逻辑。
"""

from __future__ import annotations

import logging
import os
import socket
import sys
import time


if __package__ in {None, ""}:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from deploy.speech_controller import SpeechController


logging.basicConfig(level=logging.INFO, format="%(asctime)s [BangBang Voice Server] %(message)s")


def flush_socket_buffer(connection: socket.socket) -> None:
    """播报后清除远端麦克风采集到的自身回声。"""
    time.sleep(0.5)
    try:
        connection.setblocking(False)
        while connection.recv(4096):
            pass
    except (BlockingIOError, OSError):
        pass
    finally:
        connection.setblocking(True)


def main() -> int:
    print("=" * 56)
    print("  邦邦（Unitree Go2）本地 Ollama 中英文语音系统已启动")
    print("=" * 56)
    controller = SpeechController()
    controller.on_error = lambda message: logging.warning("%s", message)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", 9999))
    server.listen(1)
    try:
        while True:
            logging.info("等待文本语音前端连接 9999 端口…")
            connection, address = server.accept()
            logging.info("语音前端 %s 已连接", address)
            buffer = ""
            try:
                flush_socket_buffer(connection)
                while True:
                    data = connection.recv(4096)
                    if not data:
                        break
                    buffer += data.decode("utf-8", errors="replace")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        text = line.strip()
                        if not text:
                            continue
                        logging.info("收到语音文本: %s", text)
                        controller.simulate_text(text)
                        flush_socket_buffer(connection)
            except (ConnectionResetError, BrokenPipeError, OSError) as exc:
                logging.warning("语音前端断开: %s", exc)
            finally:
                connection.close()
    except KeyboardInterrupt:
        logging.info("语音服务停止")
    finally:
        controller.stop()
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
