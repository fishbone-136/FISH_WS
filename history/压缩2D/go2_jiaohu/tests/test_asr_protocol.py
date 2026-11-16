import subprocess
import sys

import pytest

from voice.local_asr import LocalWhisperASR, LocalAsrError


def _worker(source):
    bridge = LocalWhisperASR()
    bridge._process = subprocess.Popen([sys.executable, "-u", "-c", source],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
    return bridge


def test_progress_and_final_result_in_one_pipe_read():
    bridge = _worker('import os,time; os.write(1, b\'{"status":"ready"}\\n{"status":"progress","stage":"rescue_load"}\\n{"status":"ok","text":"hello"}\\n\'); time.sleep(2)')
    progress = []
    bridge.on_progress = progress.append
    try:
        assert bridge._read_response(1)["status"] == "ready"
        assert bridge._read_response(1) == {"status": "ok", "text": "hello"}
        assert progress == [{"status": "progress", "stage": "rescue_load"}]
    finally:
        bridge.close(force=True)


def test_partial_json_does_not_block_past_request_timeout():
    bridge = _worker('import os,time; os.write(1, b\'{"status":\'); time.sleep(5)')
    try:
        with pytest.raises(LocalAsrError, match="超时"):
            bridge._read_response(.15)
    finally:
        bridge.close(force=True)


def test_error_message_is_read_even_when_worker_exits():
    bridge = _worker('import os; os.write(1, b\'{"status":"error","error":"model unavailable"}\\n\')')
    try:
        bridge._process.wait(1)
        assert bridge._read_response(1) == {"status": "error", "error": "model unavailable"}
    finally:
        bridge.close(force=True)
