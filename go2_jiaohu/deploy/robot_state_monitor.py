"""Read-only Go2 stance/odometry feedback; does not start navigation."""
import threading
import time


class RobotStateMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = None
        self._error = ''
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='visual-sdk-state')
        self._thread.start()

    def _run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from unitree_go.msg import SportModeState
            context = Context()
            rclpy.init(context=context)
            node = Node('visual_sdk_readonly_state', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            def receive(message):
                state = dict(observed_at=time.monotonic(), height=float(message.body_height),
                    rpy=tuple(float(v) for v in message.imu_state.rpy),
                    position=tuple(float(v) for v in message.position),
                    velocity=tuple(float(v) for v in message.velocity), yaw_speed=float(message.yaw_speed))
                with self._lock:
                    self._state = state
            node.create_subscription(SportModeState, '/lf/sportmodestate', receive, qos_profile_sensor_data)
            while not self._stop.is_set() and context.ok():
                executor.spin_once(timeout_sec=.05)
        except Exception as error:
            self._error = str(error)
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.)
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                context.shutdown()

    def get(self):
        with self._lock:
            state = self._state
        if state is None or time.monotonic()-state['observed_at'] > .3:
            raise ValueError('机器狗姿态反馈未就绪或已过期。'+self._error)
        return state

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.)
