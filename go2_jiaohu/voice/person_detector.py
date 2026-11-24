"""A shared, isolated CUDA detector, with conservative target selection."""
import atexit
import base64
import json
import logging
import math
import os
from pathlib import Path
import re
import select
import subprocess
import threading
import time

import cv2

from voice import config
from voice.visual_navigation import GroundedTarget, measure_target


def person_side(target):
    if re.fullmatch(r'(?:(?:前面|前方|眼前)的?)?(?:人|人员|一个人|那个人)', target):
        return 'any'
    match = re.fullmatch(r'(左|右)(?:边|侧)(?:的)?(?:人|那个人)', target)
    return {'左': 'left', '右': 'right'}.get(match[1]) if match else None


class PersonNotFound(ValueError):
    pass


def select_person(boxes, side):
    candidates = [tuple(box) for box in boxes if side == 'any' or
                  (side == 'left' and (box[0]+box[2])/2 < .45) or
                  (side == 'right' and (box[0]+box[2])/2 > .55)]
    if not candidates:
        raise PersonNotFound('当前画面没有可靠的人物检测结果')
    if len(candidates) != 1:
        raise ValueError('当前人物目标不唯一或未检测到，请明确左侧/右侧并确保目标清晰可见。')
    return candidates[0]


class PersonDetector:
    def __init__(self, confidence=.55):
        if not math.isfinite(confidence) or not .25 <= confidence <= .90:
            raise ValueError('人物检测阈值应在 0.25–0.90 之间')
        self.confidence = confidence
        self._lock = threading.Lock()
        self._process = None

    def _read(self, timeout):
        process = self._process
        if process is None or process.poll() is not None:
            raise RuntimeError('人物检测进程未就绪')
        ready, _, _ = select.select([process.stdout], [], [], timeout)
        if not ready:
            raise TimeoutError('人物检测超时')
        line = process.stdout.readline()
        if not line:
            raise RuntimeError('人物检测进程已退出')
        data = json.loads(line)
        if 'error' in data:
            raise RuntimeError(data['error'])
        return data

    def _ensure(self):
        if self._process is not None and self._process.poll() is None:
            return
        project = Path(config.PROJECT_DIR)
        env = os.environ.copy()
        env.pop('PYTHONNOUSERSITE', None)
        torch_root = project/'third_party/kokoro-python'
        env['PYTHONPATH'] = os.pathsep.join([str(project/'third_party/person_detector_deps'), str(torch_root), str(project)])
        sysroot = project/'third_party/cuda-sysroot'
        libraries = [torch_root/'torch/lib', sysroot/'usr/lib/aarch64-linux-gnu',
                     sysroot/'usr/local/cuda-12.6/lib64',
                     Path.home()/'.local/lib/python3.10/site-packages/torch/lib']
        env['LD_LIBRARY_PATH'] = os.pathsep.join([str(path) for path in libraries if path.is_dir()]
            + ([env['LD_LIBRARY_PATH']] if env.get('LD_LIBRARY_PATH') else []))
        env['YOLO_CONFIG_DIR'] = str(project/'third_party/person_detector_settings')
        env['YOLO_AUTOINSTALL'] = 'false'
        self._process = subprocess.Popen(['/usr/bin/python3', '-u', '-m', 'voice.person_detector_worker',
            config.PERSON_DETECTOR_MODEL], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=None, text=True, bufsize=1, env=env, cwd=project)
        try:
            ready = self._read(30.)
            if ready.get('ready') is not True:
                raise RuntimeError('人物检测模型加载失败')
            logging.info('人物快速检测已就绪：%s', ready.get('backend'))
        except Exception:
            self._close()
            raise

    def detect(self, frame):
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError('人物检测图像编码失败')
        with self._lock:
            self._ensure()
            try:
                self._process.stdin.write(json.dumps({'image': base64.b64encode(encoded).decode('ascii'),
                    'confidence': self.confidence})+'\n')
                self._process.stdin.flush()
                data = self._read(2.)
                return data['boxes']
            except Exception:
                self._close()
                raise

    def warmup(self):
        def run():
            with self._lock:
                try:
                    self._ensure()
                except Exception as error:
                    logging.warning('人物快速检测预热失败：%s', error)
        threading.Thread(target=run, daemon=True, name='person-detector-warmup').start()

    def _close(self):
        process, self._process = self._process, None
        if process is not None:
            if process.stdin:
                process.stdin.close()
            try:
                process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

    def close(self):
        with self._lock:
            self._close()


detector = PersonDetector()
atexit.register(detector.close)


def locate_person(snapshot, intent, request_id):
    started = time.monotonic()
    box = select_person(detector.detect(snapshot.color), person_side(intent.target))
    result = GroundedTarget(request_id, intent, snapshot, box, measure_target(snapshot, box))
    logging.info('人物快速定位耗时 %.3fs', time.monotonic()-started)
    return result
