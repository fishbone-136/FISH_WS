"""RGB-D preview and explicit keyboard start for basic SDK person following."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import queue
import threading
import time

import cv2
import numpy as np

from deploy.person_follow_controller import PersonFollowController
from voice import config
from voice.near_approach import body_projection
from voice.person_detector import PersonDetector, select_person
from voice.person_follow import FollowFilter, depth_cloud, measure_person, camera_projection
from voice.visual_navigation import RgbdSnapshot


# Partial-body/IR detections need a lower threshold than semantic approach.
# Keep the existing shared approach detector and its 0.55 threshold unchanged.
detector = PersonDetector(confidence=.35)


class FollowCamera:
    """One camera owner, with SDK callbacks independent of inference/SDK waits."""
    def __init__(self, view='infrared'):
        if view not in ('infrared','color'):
            raise ValueError('相机模式应为 infrared 或 color')
        self.view = view
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._error = ''
        self._pipeline = None
        self._align = None
        self._rs = None
        self._scale = None
        self._started = False
        self._started_at = None
        self._deproject = self._deproject_many = None

    def start(self):
        # IR1 and depth share the wide optical view. RGB 640x480 is a narrower
        # sensor crop; keep it as an explicit alternative, never pretend to zoom out.
        try:
            import pyrealsense2 as rs
            self._rs = rs
            self._pipeline = rs.pipeline()
            settings = rs.config()
            if self.view == 'infrared':
                settings.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            else:
                settings.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            settings.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self._align = rs.align(rs.stream.color) if self.view == 'color' else None
            profile = self._pipeline.start(settings, self._receive)
            self._started = True
            self._started_at = time.monotonic()
            device = profile.get_device()
            reference = profile.get_stream(rs.stream.color if self.view == 'color' else rs.stream.depth)
            intrinsics = reference.as_video_stream_profile().get_intrinsics()
            extrinsics = None
            if self.view == 'infrared':
                color_profiles = [p for sensor in device.query_sensors() for p in sensor.get_stream_profiles()
                                  if p.stream_type() == rs.stream.color]
                if not color_profiles:
                    raise ValueError('缺少用于机身安装标定的彩色光学参考')
                extrinsics = reference.get_extrinsics_to(color_profiles[0])
            self._deproject, self._deproject_many = camera_projection(rs,intrinsics,extrinsics)
            self._scale = device.first_depth_sensor().get_depth_scale()
            fov = rs.rs2_fov(intrinsics)
            logging.info('跟随相机已启动：%s 640x480 @ 30 FPS，完整视野 %.1f° × %.1f°',
                         self.view, fov[0], fov[1])
        except Exception as error:
            self._record_error('启动失败：'+str(error))
            self.close()

    def _record_error(self, message):
        with self._lock:
            if self._error:
                return
            self._error = message
        logging.error('D435i %s', message)

    def _receive(self, frame):
        if self._stop.is_set() or self._scale is None:
            return
        observed = time.monotonic()
        try:
            frames = frame.as_frameset()
            if self._align is not None:
                frames = self._align.process(frames)
            color = frames.get_color_frame() if self.view == 'color' else frames.get_infrared_frame(1)
            depth = frames.get_depth_frame()
            if not color or not depth:
                return
            if abs(color.get_timestamp()-depth.get_timestamp()) > 50.:
                return
            image = np.asanyarray(color.get_data()).copy()
            if self.view == 'infrared':
                image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
            sample = RgbdSnapshot(image,
                np.clip(np.rint(np.asanyarray(depth.get_data()).astype(float)*self._scale*1000),
                        0, 65535).astype(np.uint16), time.time(), observed,
                self._deproject, self._deproject_many)
            with self._lock:
                self._latest = sample
        except Exception as error:
            self._record_error('采集失败：'+str(error))

    def snapshot(self):
        with self._lock:
            error, sample = self._error, self._latest
        if error:
            raise ValueError('D435i '+error)
        now = time.monotonic()
        if self._started and sample is None and now-self._started_at > 5.:
            raise ValueError('D435i 启动后 5 秒内没有同步 RGB-D 画面，请检查相机占用和 USB 连接')
        if sample is not None and now-sample.captured_monotonic > 1.:
            raise ValueError('D435i 同步 RGB-D 画面已中断，请检查 USB 连接')
        return sample

    def close(self):
        self._stop.set()
        if self._started:
            self._started = False
            try:
                self._pipeline.stop()
            except Exception:
                logging.exception('关闭跟随相机失败')
        self._pipeline = None


def draw_result(sample, boxes=(), measurement=None, caption='PREVIEW', show_controls=True):
    frame = sample.color.copy()
    h, w = frame.shape[:2]
    cv2.line(frame,(w//2,32),(w//2,h-30),(255,200,0),1)
    for box in boxes:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (int(x1*w), int(y1*h)), (int(x2*w), int(y2*h)), (0, 220, 255), 2)
        cx,cy = int((x1+x2)*w*.5),int((y1+y2)*h*.5)
        cv2.circle(frame,(cx,cy),4,(255,0,255),-1)
        cv2.line(frame,(w//2,cy),(cx,cy),(255,0,255),1)
    if measurement is not None:
        for u, v in measurement.pixels:
            cv2.circle(frame, (int(u), int(v)), 2, (0, 255, 0), -1)
        caption += f'  range={measurement.distance:.2f}m'
    cv2.putText(frame, caption, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 0), 1)
    if show_controls:
        cv2.putText(frame, 'SPACE: follow | S/ESC: stop | Q: quit', (8, h-12),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
    return frame


def analyze_preview(sample, preview_state, project, people=None):
    """Slow model startup/inference must never own the GUI event loop."""
    boxes = (people if people is not None else detector).detect(sample.color)
    measurement = None
    try:
        box = select_person(boxes, 'any')
        measurement = measure_person(sample, box, depth_cloud(sample, preview_state, project),project=project)
        message = '预览使用估计机身高度；绿色点为测距取样。'
    except ValueError as error:
        message = str(error)
    return sample, boxes, measurement, message


def preview_window_closed(title):
    """GTK reports VISIBLE=-1 for an open window: that means unsupported."""
    try:
        visible = cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible == 0
        # The GTK backend supports image rectangles, even when VISIBLE is
        # unavailable. Negative dimensions mean the window was destroyed.
        _, _, width, height = cv2.getWindowImageRect(title)
        return width < 0 or height < 0
    except cv2.error:
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description='D435i 人体跟随：默认预览，空格启动，S/ESC 停止，Q 退出；仅使用 SDK。')
    parser.add_argument('--distance', type=float, default=1.5, help='到腿部表面的机身中心水平距离，1–2 米')
    parser.add_argument('--speed', type=float, default=.35, help='最大前进速度，默认 0.35，可设 0.05–0.40 米/秒')
    parser.add_argument('--camera-view', choices=('infrared','color'), default='infrared',
                        help='默认广角红外；color 使用原来的窄视野彩色流')
    parser.add_argument('--duration', type=float, default=120., help='单次跟随上限，1–600 秒')
    parser.add_argument('--preview-height', type=float, default=.25, help='预览的估计机身高度；启动后使用 SDK 实测姿态')
    parser.add_argument('--preview-only', action='store_true', help='仅预览相机和人物检测，禁用空格启动运动')
    args = parser.parse_args(argv)
    try:
        FollowFilter(args.distance, args.speed)
        if not np.isfinite(args.duration) or not 1 <= args.duration <= 600:
            raise ValueError('duration 应在 1–600 秒之间')
        if not np.isfinite(args.preview_height) or not .18 <= args.preview_height <= .5:
            raise ValueError('preview-height 应在 0.18–0.5 米之间')
    except ValueError as error:
        parser.error(str(error))
    logging.basicConfig(level=logging.INFO)
    log_path = Path(config.WORKSPACE_DIR)/'log'/'person_follow.log'
    try:
        log_path.parent.mkdir(parents=True,exist_ok=True)
        root = logging.getLogger()
        if not any(getattr(handler,'baseFilename',None) == str(log_path) for handler in root.handlers):
            handler = RotatingFileHandler(log_path,maxBytes=2*1024*1024,backupCount=2,encoding='utf-8')
            handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
            root.addHandler(handler)
    except OSError as error:
        logging.warning('无法写入跟随日志 %s：%s',log_path,error)
    # Share the existing UI lock; neither program may own the camera/motors twice.
    lock_path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))/f'bangbang-ui-{os.getuid()}.lock'
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('交互 UI 或人体跟随已在运行，请先关闭')
        return run_preview(args)


def run_preview(args):
    project = body_projection()
    events = queue.Queue(maxsize=1)
    def update(message, **details):
        if not details.get('sample'):
            logging.info('%s', message)
        try:
            events.get_nowait()
        except queue.Empty:
            pass
        events.put_nowait((message, details))
    camera = FollowCamera(view=getattr(args,'camera_view','infrared'))
    controller = PersonFollowController(camera.snapshot, update, project=project, people=detector)
    title = 'Go2 SDK Person Follow'
    last_frame = -1.
    last_follow_at = -float('inf')
    preview_state = dict(height=args.preview_height, rpy=(0., 0., 0.))
    frame = np.zeros((480, 640, 3), np.uint8)
    preview_message = ''
    status_since = time.monotonic()
    pending_message = ''
    preview_only = getattr(args, 'preview_only', False)
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='follow-preview')
    pending = None
    latest_result = None
    detector_ready = False
    first_frame = True
    result = 0
    try:
        cv2.setNumThreads(1)
        logging.info('正在创建预览窗口：DISPLAY=%s', os.environ.get('DISPLAY', '(未设置)'))
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.putText(frame, 'STARTING CAMERA...', (16, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    .7, (0,255,255), 2)
        cv2.imshow(title, frame)
        cv2.waitKey(1)
        logging.info('预览窗口已创建，正在启动相机…')
        camera.start()
        while True:
            if controller.active:
                try:
                    _, details = events.get_nowait()
                    if 'sample' in details:
                        last_follow_at = details['sample'].captured_monotonic
                        frame = draw_result(details['sample'], [details['box']], details['measurement'],
                            f"FOLLOW vx={details['vx']:.2f} wz={details['wz']:.2f}")
                except queue.Empty:
                    pass
                live = camera.snapshot()
                if live is not None and time.monotonic()-live.captured_monotonic <= .20:
                    # A dropout/slow inference must not leave the preview frozen.
                    if time.monotonic()-last_follow_at > .20:
                        frame = draw_result(live,caption='FOLLOW: waiting fresh feedback')
            else:
                if pending is not None and pending.done():
                    latest_result = pending.result()
                    pending = None
                    if not detector_ready:
                        detector_ready = True
                        logging.info('人物检测已就绪；%s', '当前为仅预览模式，运动已禁用。' if preview_only
                                     else '按空格启动跟随，S/Esc 停止，Q 退出。')
                    message = latest_result[3]
                    if message != pending_message:
                        pending_message, status_since = message, time.monotonic()
                    if message != preview_message and time.monotonic()-status_since >= .5:
                        logging.info('%s', message)
                        preview_message = message
                sample = camera.snapshot()
                if sample is not None and sample.captured_monotonic > last_frame:
                    last_frame = sample.captured_monotonic
                    if first_frame:
                        logging.info('已收到首帧 RGB-D；正在后台加载人物检测模型，画面保持刷新…')
                        first_frame = False
                    # Never draw stale boxes on a newer live image, including
                    # the old image used for the first model warmup request.
                    if latest_result and time.monotonic()-latest_result[0].captured_monotonic <= .20:
                        frame = draw_result(*latest_result[:3], caption='PREVIEW (estimated pose)')
                    else:
                        caption = 'PREVIEW: detecting...' if detector_ready else 'LOADING DETECTOR (motion disabled)'
                        frame = draw_result(sample, caption=caption)
                    if pending is None:
                        pending = pool.submit(analyze_preview, sample, preview_state, project)
            cv2.imshow(title, frame)
            key = cv2.waitKey(10) & 0xff
            if key in (ord('q'), ord('Q')) or preview_window_closed(title):
                break
            if key in (ord('s'), ord('S'), 27):
                controller.cancel()
            elif key == ord(' '):
                if preview_only:
                    logging.info('仅预览模式，运动已禁用。')
                elif not detector_ready:
                    logging.info('人物检测仍在加载，暂不启动跟随。')
                else:
                    controller.start(args.distance, args.speed, args.duration)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError, OSError, cv2.error) as error:
        logging.error('人体跟随预览退出：%s', error)
        result = 1
    finally:
        if not controller.close():
            logging.error('跟随线程仍在释放资源；SDK 速度租约不会继续续期')
        camera.close()
        cv2.destroyAllWindows()
        pool.shutdown(wait=True, cancel_futures=True)
        detector.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
