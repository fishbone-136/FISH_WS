import unittest

from offline_test.speech_controller import SpeechController, _choose_input_device


class _FakeParser:
    def __init__(self):
        self.parse_calls = []

    def parse_command(self, text):
        self.parse_calls.append(text)
        return {
            "reply": "收到",
            "actions": [{"action": "forward", "speed": 0.3, "duration": 1.0}],
        }

    def chat(self, text):
        return "聊天回复"


class _CapturingController(SpeechController):
    def __init__(self):
        super().__init__()
        self._llm = _FakeParser()
        self.executed = []
        self.events = []

    def _execute_actions(self, actions):
        self.executed.extend(actions)
        self.events.append("execute")
        return True

    def _speak(self, text):
        self.events.append("speak")


class _EmptyActionParser(_FakeParser):
    def parse_command(self, text):
        self.parse_calls.append(text)
        return {"reply": "请重新说明", "actions": []}


class SpeechSafetyIntegrationTests(unittest.TestCase):
    def test_natural_imperative_reaches_executor_without_fixed_phrase(self):
        controller = _CapturingController()
        reply = controller._call_llm("前进一米")
        self.assertEqual([a.action for a in controller.executed], ["forward"])
        self.assertIn("已下发动作", reply)

    def test_authorized_motion_reaches_executor(self):
        controller = _CapturingController()
        reply = controller._call_llm("执行指令，前进一米")
        self.assertEqual([action.action for action in controller.executed], ["forward"])
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertEqual(controller.events, ["execute"])
        self.assertIn("已下发动作", reply)

    def test_observed_misheard_backward_command_reaches_executor(self):
        controller = _CapturingController()

        reply = controller._call_llm("直旋后退动作好")

        self.assertEqual([action.action for action in controller.executed], ["backward"])
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertLess(controller.executed[0].speed, 0.45)
        self.assertGreater(controller.executed[0].duration, 0.0)
        self.assertIn("已下发动作", reply)

    def test_asr_terminal_semicolon_command_reaches_executor(self):
        controller = _CapturingController()

        reply = controller._call_llm("邦邦执行后退动作；")

        self.assertEqual([action.action for action in controller.executed], ["backward"])
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertIn("已下发动作", reply)

    def test_observed_misheard_forward_command_reaches_executor(self):
        controller = _CapturingController()

        reply = controller._call_llm("执行全劲动作")

        self.assertEqual([action.action for action in controller.executed], ["forward"])
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertIn("已下发动作", reply)

    def test_observed_misheard_left_turn_command_reaches_executor(self):
        controller = _CapturingController()

        reply = controller._call_llm("执行折断动作")

        self.assertEqual([action.action for action in controller.executed], ["turn_left"])
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertIn("已下发动作", reply)

    def test_unknown_target_is_not_guessed_by_qwen(self):
        controller = _CapturingController()
        controller._call_llm("执行指令，去门口")
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertEqual(controller.executed, [])

    def test_unknown_fixed_action_is_rejected_before_qwen(self):
        controller = _CapturingController()

        reply = controller._call_llm("执行火山动作")

        self.assertEqual(controller._llm.parse_calls, [])
        self.assertEqual(controller.executed, [])
        self.assertIn("没理解", reply)

    def test_basic_motion_does_not_depend_on_qwen(self):
        controller = _CapturingController()
        controller._llm = _EmptyActionParser()

        controller._call_llm("執行命令，向前走兩步")

        self.assertEqual(controller._llm.parse_calls, [])
        self.assertEqual([action.action for action in controller.executed], ["forward"])

    def test_chat_never_reaches_executor(self):
        controller = _CapturingController()
        self.assertEqual(controller._call_llm("你叫什么名字"), "聊天回复")
        self.assertEqual(controller.executed, [])

    def test_logged_sequence_executes_whole_plan_without_qwen_or_tts(self):
        controller = _CapturingController()
        controller._call_llm("执行转身动作；然后前进半米。")
        self.assertEqual([a.action for a in controller.executed],
                         ["turn_left", "turn_left", "forward"])
        self.assertEqual(sum(a.angle for a in controller.executed), 180)
        self.assertAlmostEqual(controller.executed[-1].speed * controller.executed[-1].duration / 1.3, .5)
        self.assertEqual(controller._llm.parse_calls, [])
        self.assertEqual(controller.events, ["execute"])

    def test_partial_or_overlong_plan_never_executes(self):
        for text in ("执行指令，前进半米，然后未知动作", "执行指令，前进8米",
                     "使用前进搬拟然后左转", "执行转身路过然后前进半米", "站起来，然后请你"):
            controller = _CapturingController()
            controller._call_llm(text)
            self.assertEqual(controller.executed, [], text)

    def test_busy_sequence_keeps_stop_listening_without_tts(self):
        controller = _CapturingController()
        controller.is_action_busy = True
        controller._call_llm("执行前进动作")
        self.assertEqual(controller.executed, [])
        self.assertEqual(controller.events, [])
        controller._call_llm("停止")
        self.assertEqual([a.action for a in controller.executed], ["stop"])


class MicrophoneSelectionTests(unittest.TestCase):
    def setUp(self):
        self.devices = [
            {"index": 23, "name": "UACDemoV1.0: USB Audio (hw:2,0)", "maxInputChannels": 0},
            {"index": 24, "name": "pulse", "maxInputChannels": 32},
            {"index": 25, "name": "default", "maxInputChannels": 32},
        ]

    def test_selects_named_pulse_input(self):
        self.assertEqual(_choose_input_device(self.devices, "pulse"), (24, "pulse"))

    def test_rejects_output_only_hardware_device(self):
        with self.assertRaises(RuntimeError):
            _choose_input_device(self.devices, "UACDemoV1.0")


if __name__ == "__main__":
    unittest.main()
