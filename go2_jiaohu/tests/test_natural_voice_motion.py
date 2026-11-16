"""Natural voice wording, ambiguity rejection, and stop priority; fake SDK only."""
import pytest
from voice.command_safety import gate_command, is_emergency_stop
from voice.asr_worker import _choose_rescue_text
from tests.test_speech_controller import _CapturingController


@pytest.mark.parametrize('text', [
    '执行站立动作', '站立', '站起来', '站起', '起立', '起身', '起来一下',
    '站立一下', '站立起来', '请你站好', '邦邦，帮我站起来',
    '麻烦你起身一下吧', '现在站起来', '我想让你站起来', '给我站一下',
    '请你，站起来', '执行动作，站立', '执行站立的动作', '保持站立',
    '执行一个站立动作', '做一个站立动作', '帮我做个站立动作', '请帮我执行一下站立动作',
])
def test_standing_synonyms_execute_same_action_without_model_or_tts(text):
    ctrl = _CapturingController()
    ctrl._call_llm(text)
    assert [a.action for a in ctrl.executed] == ['stand']
    assert ctrl._llm.parse_calls == []
    assert ctrl.events == ['execute']


@pytest.mark.parametrize('text,expected', [
    ('请趴下来吧', ['sit']), ('帮我躺下', ['sit']), ('恢复站姿', ['recovery']),
    ('请先起身，然后向前走半米，最后趴下来吧', ['stand', 'forward', 'sit']),
    ('邦邦，往后退半米，然后往右转九十度', ['backward', 'turn_right']),
    ('麻烦转个身，然后直行半米', ['turn_left', 'turn_left', 'forward']),
])
def test_natural_complete_sequences_preserve_order(text, expected):
    ctrl = _CapturingController()
    ctrl._call_llm(text)
    assert [a.action for a in ctrl.executed] == expected
    assert ctrl._llm.parse_calls == []


@pytest.mark.parametrize('text', [
    '不要站立', '别站起来', '请不要起身', '你会站起来吗', '站立是什么意思',
    '如果站起来会怎么样', '我已经站起来了', '照片里的人站立',
    '刚才听到电视里说站起来', '帮我看看前面站立的人', '我想站起来',
    '先站起来，然后跳舞', '站起来，然后请你', '站起来，然后不要前进',
    '往前走8米', '向前走搬拟', '执行站立动作，然后未知动作',
])
def test_chat_negation_unknown_or_partial_intent_never_executes(text):
    ctrl = _CapturingController()
    ctrl._call_llm(text)
    assert ctrl.executed == []


@pytest.mark.parametrize('text', ['停一下', '请停下来', '邦邦，别动了', '站住', '不要动', '停一停'])
def test_natural_stop_has_priority_while_busy(text):
    assert is_emergency_stop(text)
    ctrl = _CapturingController()
    ctrl.is_action_busy = True
    ctrl._call_llm(text)
    assert [a.action for a in ctrl.executed] == ['stop']


def test_asr_rescue_compares_meaning_not_exact_standing_word():
    rescue='请你起身一下吧'
    assert _choose_rescue_text('执行站立动作', rescue) == (rescue, 'applied_rescue')
    assert _choose_rescue_text('站起来', '趴下') == ('', 'command_disagreement')
    assert not gate_command('你会站起来吗').authorized
