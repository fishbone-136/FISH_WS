#!/usr/bin/python3
"""Read-only standing/stationary preflight for the real Go2 navigation entry."""
import argparse
import math
import time


def stance_error(height, vx, vy, wz, minimum_height=.18):
    if not all(math.isfinite(v) for v in (height, vx, vy, wz)):
        return '底盘姿态反馈包含无效数值'
    if height < minimum_height:
        return '机器狗尚未站稳（机身高度 %.3f m，要求至少 %.2f m）' % (height, minimum_height)
    if math.hypot(vx, vy) > .06 or abs(wz) > .10:
        return '机器狗仍在移动，请保持静止后再启动定位'
    return None


class StableStance:
    def __init__(self, stable_seconds=1.5):
        self.stable_seconds = stable_seconds
        self.since = self.received = None
        self.reason = '未收到机器狗姿态反馈，请检查机器人连接'

    def observe(self, now, height, vx, vy, wz):
        if self.received is not None and now-self.received > .3:
            self.since = None
        self.received = now
        self.reason = stance_error(height, vx, vy, wz)
        if self.reason:
            self.since = None
        elif self.since is None:
            self.since = now

    def ready(self, now):
        return (self.since is not None and self.received is not None and
                0 <= now-self.received <= .3 and now-self.since >= self.stable_seconds)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from unitree_go.msg import SportModeState
    rclpy.init()
    node = rclpy.create_node('navigation_stance_preflight')
    gate = StableStance()
    node.create_subscription(SportModeState, '/lf/sportmodestate',
        lambda m: gate.observe(time.monotonic(), float(m.body_height),
                              float(m.velocity[0]), float(m.velocity[1]), float(m.yaw_speed)),
        qos_profile_sensor_data)
    end = time.monotonic()+args.timeout
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if gate.ready(time.monotonic()):
                print('站稳检查通过；定位初始化期间请继续保持静止。', flush=True)
                return 0
        if gate.received is not None and time.monotonic()-gate.received > .3:
            reason = '机器狗姿态反馈已过期'
        else:
            reason = gate.reason or '站立静止时间不足 1.5 秒'
        print('导航未启动：'+reason+'。请先用遥控器让机器狗站稳，再运行启动脚本。', flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
