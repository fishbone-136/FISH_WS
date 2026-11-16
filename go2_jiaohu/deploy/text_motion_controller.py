"""Execute explicit UI text actions off the GUI thread, using the SDK bridge."""
import logging
import threading
import time

from voice import config
from voice.robot_interface import Go2RobotController
from voice.text_motion import describe_action


def navigation_is_idle():
    try:
        from go2_navigation.runtime_guard import conflicting_navigation_processes
        return not conflicting_navigation_processes(include_launch=True)
    except ImportError:
        return False  # Deploy control must fail closed when arbitration is absent.


class TextMotionController:
    def __init__(self,on_update,*,robot=None,guard=None):
        self._robot=robot if robot is not None else Go2RobotController(auto_connect=False)
        self._guard=guard or navigation_is_idle
        self._on_update=on_update
        self._lock=threading.Lock()
        self._stop=threading.Event()
        self._thread=None

    @property
    def is_busy(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self,actions,request_id):
        with self._lock:
            if self.is_busy:
                return False
            self._stop.clear()
            self._thread=threading.Thread(target=self._run,args=(tuple(actions),request_id),
                daemon=True,name='go2-visual-text-actions')
            self._thread.start()
        return True

    def _notify(self,request_id,state,message):
        logging.info('视觉文字动作 [%s] %s: %s',request_id,state,message)
        try:
            self._on_update(request_id,state,message)
        except Exception:
            logging.exception('文字动作状态回调失败')

    def _run(self,actions,request_id):
        state,message='finished','动作定时控制已结束，已请求停止。'
        try:
            if not self._guard():
                raise RuntimeError('导航仍在运行，请在导航页关闭导航后再发送前进、后退等直接动作。')
            self._notify(request_id,'connecting','正在连接机器狗运动接口…')
            if self._stop.is_set():
                return
            if self._robot.client is None and not self._robot.reconnect():
                raise RuntimeError('运动接口连接失败：'+self._robot.last_error)
            for index,action in enumerate(actions):
                if self._stop.is_set():
                    message='已取消文字动作。'
                    break
                if not self._guard():
                    raise RuntimeError('检测到导航控制源启动，已中止直接动作。')
                if not self._robot.execute_action(action.action,speed=action.speed,
                        duration=action.duration,angle=action.angle):
                    raise RuntimeError(f'动作 {action.action} 未被接受：{self._robot.last_error}')
                self._notify(request_id,'accepted',f'已下发第 {index+1}/{len(actions)} 个动作：{describe_action(action)}。')
                deadline=time.monotonic()+action.duration+.1
                while time.monotonic()<deadline:
                    if self._stop.wait(min(.1,max(0.,deadline-time.monotonic()))):
                        break
                    if not self._guard():
                        raise RuntimeError('检测到导航控制源启动，已中止直接动作。')
        except Exception as error:
            state,message='error',str(error)
        finally:
            try:
                if self._robot.client is not None and not self._robot.execute_action('stop',speed=0.,duration=0.):
                    state,message='error','停止未收到运动接口确认：'+self._robot.last_error
            except Exception as error:
                state,message='error',f'停止动作失败：{error}'
            finally:
                try:
                    self._robot.shutdown()
                except Exception as error:
                    state,message='error',f'释放运动接口失败：{error}'
            if self._stop.is_set() and state!='error':
                message='文字动作已取消，运动接口已释放。'
            self._notify(request_id,state,message)

    def cancel(self):
        self._stop.set()

    def close(self):
        self.cancel()
        thread=self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2*max(config.ROBOT_SDK_RPC_TIMEOUT,config.ROBOT_COMMAND_ACK_TIMEOUT)+4.)
            if thread.is_alive():
                raise RuntimeError('文字动作仍在停止，暂不能切换控制源。')
