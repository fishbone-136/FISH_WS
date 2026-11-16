"""Complete relative action intents shared by voice and visual text control."""
import re
import math

from . import config
from .command_safety import gate_command, parse_deterministic_command, validate_actions


_NUMBER = r'(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百点半]+)'
_MOVE = (r'(?:前进|后退|向前走|往前走|朝前走|向后走|往后走|朝后走|'
         r'向前移动|往前移动|向后移动|往后移动|向前|往前|向后|往后|'
         r'向左移动|往左移动|向左平移|左移|向右移动|往右移动|向右平移|右移)')
_TURN = r'(?:向左转|往左转|左转|向右转|往右转|右转)'
_POSTURE = r'(?:趴下|卧倒|坐下|站起|站起来|起立|恢复站立|恢复站起|停止|停下|急停)'
_RELATIVE = re.compile(rf'(?:{_MOVE}(?:\s*{_NUMBER}\s*(?:米|m|步))?|'
                       rf'{_TURN}(?:\s*{_NUMBER}\s*(?:度|°))?|{_POSTURE})', re.IGNORECASE)
_NOT_IMPERATIVE = r'如果|假如|比如|示例|如何|怎么|怎样|什么|意思|解释|是否|能否|会不会|能不能|可不可以|不要|不许|不准|别|[？?]|(?:吗|么|呢)$'


def _command_body(text):
    text = text.strip().rstrip('。！! ')
    authorized = gate_command(text)
    if authorized.authorized:
        return authorized.command_text
    return re.sub(r'^(?:邦邦[,， ]*)?(?:现在)?(?:请你|请)?(?:帮我)?\s*', '', text)


def is_motion_request(text):
    """Recognize unsupported imperatives so they cannot masquerade as scene chat."""
    if re.search(_NOT_IMPERATIVE, text):
        return False
    return bool(re.match(r'(?:转身|掉头|调头|直行|直走|前进|后退|左转|右转|左移|右移|'
                         r'向[前后左右]|往[前后左右]|朝着|朝向|走到|移动到|靠近|寻找)', _command_body(text)))


def _normalize_alias(chunk):
    # Accept natural requests addressed to the robot, while preserving the
    # whole-clause match below. “我站起来了 / 照片里的人站立” cannot match it.
    chunk = re.sub(r'^(?:(?:邦邦|bangbang|机器狗)\s*[,，:：]?\s*)?', '', chunk, flags=re.IGNORECASE)
    chunk = re.sub(r'^(?:(?:我想让你|我希望你|请你|麻烦你|帮我|给我|现在|马上|立即|'
                   r'请|麻烦|你|先|最后)\s*)+', '', chunk)
    chunk = re.sub(r'^(?:做(?:一个|个|一下)?|一个|个|一下)\s*', '', chunk)
    chunk = re.sub(r'\s*(?:吧|啊|呀|啦|哦)[。！! ]*$', '', chunk).strip()
    chunk = re.sub(r'(?:的)?动作$', '', chunk).strip()
    if re.fullmatch(r'(?:恢复站姿|恢复站立|恢复站起|重新站起来|重新站起)(?:一下)?', chunk):
        return '恢复站立'
    if re.fullmatch(r'(?:站立起来|站起来|站起|站立|起立|起身站立|起身|起来|立起来|站好|站稳|保持站姿|保持站立|站)(?:一下|一下子)?(?:的)?', chunk):
        return '站起'
    if re.fullmatch(r'(?:趴下来|趴下|趴着|卧倒|坐下来|坐下|躺下来|躺下)(?:一下|一下子)?', chunk):
        return '趴下'
    if re.fullmatch(r'(?:停止|停下|停下来|停住|停一停|停|站住|急停)(?:一下|一下子|了)?|(?:别动|不要动)(?:了)?|别走了', chunk):
        return '停止'
    chunk = re.sub(r'^(?:转个身|转一下身|转过身|掉个头)', '转身', chunk)
    chunk = re.sub(r'((?:左|右)转)一下$', r'\1', chunk)
    chunk = re.sub(r'^(?:向前行走|往前行走|向前挪|往前挪)', '前进', chunk)
    chunk = re.sub(r'^(?:往后退|向后退|向后挪|往后挪)', '后退', chunk)
    chunk = re.sub(r'^(?:直行|直走)', '前进', chunk)
    turn = re.fullmatch(rf'(转身|掉头|调头|向后转|往后转|向左掉头|向右掉头)\s*({_NUMBER}\s*(?:度|°))?', chunk)
    if turn:
        chunk = ('右转' if turn[1] == '向右掉头' else '左转') + (turn[2] or '180度')
    return chunk


def normalize_relative_language(text):
    """Normalize synonyms and request wording without opening a control gate."""
    text = text.strip().rstrip('。！!;； ')
    text = re.sub(r'^(?:邦邦|bangbang|机器狗)\s*[,，:：]?\s*', '', text, flags=re.IGNORECASE)
    text = re.sub(r'^(?:(?:我想让你|我希望你|请你|麻烦你|帮我|给我|现在|马上|立即|'
                  r'请|麻烦|你|先|最后)\s*[,，:：]?\s*)+', '', text)
    chunks = [value.strip() for value in re.split(r'然后|接着|随后|之后|并且|再|[，,；;]', text) if value.strip()]
    chunks = [re.sub(r'^执行\s*(?:(?:指令|命令|动作|一个|个|一下)\s*[:：]?\s*)?', '', value) for value in chunks]
    normalized = [_normalize_alias(value) for value in chunks]
    # Do not erase an incomplete trailing clause and execute only its prefix.
    return '，'.join(normalized) if all(normalized) else text


def describe_action(action):
    names = dict(forward='前进', backward='后退', left='左移', right='右移',
                 turn_left='左转', turn_right='右转', stand='站起', sit='趴下', recovery='恢复站立', stop='停止')
    name = names.get(action.action, action.action)
    if action.action.startswith('turn_'):
        return f'{name} {action.angle:g}°'
    if action.action in ('forward', 'backward', 'left', 'right'):
        distance = action.speed*action.duration/config.LINEAR_TIME_MULTIPLIER
        return f'{name}约 {distance:g} 米'
    return name


def parse_relative_actions(text):
    """Return validated actions only for a complete bounded imperative grammar.

    Object requests must go through depth/navigation, never the permissive
    keyword extractor's default one-metre fallback.
    """
    return parse_relative_command_body(_command_body(text)) if not re.search(_NOT_IMPERATIVE, text) else None


def parse_relative_command_body(text):
    """Parse a complete action body, without granting voice authorization.

    This entry point deliberately does not call gate_command: the voice gate
    also uses the grammar to check an explicitly prefixed sequence.
    """
    if re.search(_NOT_IMPERATIVE, text):
        return None
    text = normalize_relative_language(text)
    chunks = [value for value in text.split('，') if value]
    if not chunks:
        return None
    if not all(_RELATIVE.fullmatch(value) for value in chunks):
        return None
    if len(chunks)>4:
        raise ValueError('一次最多执行四个连续动作，请拆分后发送。')
    raw=parse_deterministic_command('，'.join(chunks).replace('朝后走','向后走'))
    if raw is None:
        raise ValueError('动作距离或角度无效，未下发机器狗控制。')
    # Preserve the requested angle when the existing per-action duration limit
    # would truncate a 180-degree turn. Never raise motor limits to fit a turn.
    expanded = []
    for action in raw:
        count = 1
        if action['action'].startswith('turn_'):
            if not 0. < action['angle'] <= 180.:
                raise ValueError('转向角度必须大于零且不超过 180 度，请拆分指令。')
            speed = min(max(abs(action['speed']), .10), config.MAX_TURN_SPEED)
            duration = math.radians(min(action['angle'], 180.))/speed*config.TURN_TIME_MULTIPLIER
            count = max(1, math.ceil(duration/config.MAX_ACTION_DURATION-1.e-9))
        expanded.extend([{**action, 'angle': action['angle']/count} for _ in range(count)])
    if len(expanded) > 4:
        raise ValueError('转向分段后超过四个动作，请拆分指令后发送。')
    actions,warnings=validate_actions(expanded)
    if not actions:
        raise ValueError('动作参数无效，未下发机器狗控制。')
    if warnings:
        # Never silently truncate a requested distance or drop the tail of a
        # sequence. The caller must ask for a shorter, complete command.
        raise ValueError('动作距离或总时长超过安全上限，请缩短或拆分指令。')
    return actions,warnings
