"""Convert validated relative requests into metric Nav2 behavior goals."""
from dataclasses import dataclass, replace
import math

from voice import config


KINDS = {'forward', 'backward', 'turn_left', 'turn_right'}


@dataclass(frozen=True)
class NavigationStep:
    kind: str
    quantity: float  # signed radians for spin, signed metres for translation
    speed: float


def supports_navigation(actions):
    return bool(actions) and all(a.action in KINDS for a in actions)


def navigation_steps(actions):
    if not supports_navigation(actions) or len(actions) > 4:
        raise ValueError('此序列不支持导航相对动作，请在关闭导航后使用 SDK。')
    steps = []
    for action in actions:
        if not all(math.isfinite(v) for v in (action.speed, action.duration, action.angle)):
            raise ValueError('相对动作包含无效数值')
        if action.action.startswith('turn_'):
            if not 0. < action.angle <= 180.:
                raise ValueError('相对转向角度无效')
            step = NavigationStep('spin', math.radians(action.angle)*(1 if action.action=='turn_left' else -1), 0.)
            if steps and steps[-1].kind == 'spin' and steps[-1].quantity*step.quantity > 0.:
                steps[-1] = replace(step, quantity=steps[-1].quantity+step.quantity)
                continue
        else:
            if config.LINEAR_TIME_MULTIPLIER <= 0.:
                raise ValueError('距离补偿配置无效')
            distance = action.speed*action.duration/config.LINEAR_TIME_MULTIPLIER
            if not 0. < distance <= 3. or not .1 <= action.speed <= config.MAX_LINEAR_SPEED:
                raise ValueError('相对直行超出三米或限速范围')
            step = NavigationStep('forward' if action.action=='forward' else 'backup',
                distance*(1 if action.action=='forward' else -1), min(action.speed, .3))
        steps.append(step)
    if sum(abs(s.quantity) for s in steps if s.kind=='spin') > 2*math.pi+.001:
        raise ValueError('连续相对转角超出范围')
    if sum(abs(s.quantity) for s in steps if s.kind!='spin') > 6.:
        raise ValueError('连续相对距离超出六米范围')
    return tuple(steps)
