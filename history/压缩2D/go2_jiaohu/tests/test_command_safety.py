import math
import unittest

from voice.command_safety import (
    gate_command,
    is_asr_prompt_echo,
    is_alarm_request,
    is_emergency_stop,
    is_fixed_action_command,
    parse_deterministic_command,
    validate_actions,
)


class CommandGateTests(unittest.TestCase):
    def test_chinese_authorization_phrases(self):
        self.assertEqual(gate_command("执行指令，前进一米").command_text, "前进一米")
        self.assertEqual(gate_command("執行命令，向前走兩步").command_text, "向前走两步")
        self.assertEqual(gate_command("执行左转九十度动作").command_text, "左转九十度")
        self.assertEqual(gate_command("执行动作，沿地面引导线前进").command_text, "沿地面引导线前进")

    def test_asr_terminal_semicolon_does_not_block_authorization(self):
        for punctuation in (";", "；"):
            text = f"邦邦执行后退动作{punctuation}"
            result = gate_command(text)
            self.assertTrue(result.authorized, text)
            self.assertEqual(result.command_text, "后退")
            self.assertTrue(is_fixed_action_command(text), text)

        self.assertTrue(is_emergency_stop("邦邦马上停止；"))

    def test_recovers_observed_execute_action_asr_errors(self):
        self.assertEqual(gate_command("直銜后退动作").command_text, "后退")
        self.assertEqual(gate_command("直旋后退动作好").command_text, "后退")
        self.assertEqual(gate_command("提醒后退动作").command_text, "后退")
        self.assertEqual(gate_command("执行全劲动作").command_text, "前进")
        self.assertEqual(gate_command("直行雄徑动作").command_text, "前进")
        self.assertEqual(gate_command("只想闪电动作").command_text, "前进")
        self.assertEqual(gate_command("执行折断动作").command_text, "左转")
        self.assertEqual(gate_command("直行截断动作好啊").command_text, "左转")
        self.assertEqual(gate_command("行前进动作").command_text, "前进")
        self.assertEqual(gate_command("执行后退动作吧").command_text, "后退")
        self.assertEqual(gate_command("实行右转正左").command_text, "右转")
        self.assertEqual(gate_command("直行右转頓座").command_text, "右转")
        self.assertEqual(gate_command("直行后退頓座").command_text, "后退")
        self.assertEqual(gate_command("执行帕萨顿作").command_text, "趴下")
        self.assertEqual(gate_command("执行哈萨动作").command_text, "趴下")

    def test_identifies_closed_fixed_action_frame(self):
        self.assertTrue(is_fixed_action_command("执行前进动作"))
        self.assertTrue(is_fixed_action_command("执行哈萨动作"))
        self.assertFalse(is_fixed_action_command("执行指令，去门口"))

    def test_execute_asr_recovery_remains_narrow(self):
        for text in (
            "直线后退动作",
            "直旋后退",
            "直旋后退动作是什么意思",
            "如何说直旋后退动作",
            "折断动作",
        ):
            self.assertFalse(gate_command(text).authorized, text)

    def test_english_authorization_phrases(self):
        self.assertEqual(
            gate_command("execute command, move forward").command_text,
            "move forward",
        )
        self.assertEqual(gate_command("execute a left turn action").command_text, "a left turn")

    def test_motion_without_authorization_is_chat(self):
        for text in ("前进一米", "邦邦左转", "go forward", "turn right"):
            self.assertFalse(gate_command(text).authorized, text)

    def test_questions_negations_and_examples_never_authorize(self):
        samples = (
            "如果我说执行指令前进会怎样",
            "如何说执行前进动作",
            "不要执行指令前进",
            "执行指令，不要前进",
            "what if I say execute command move forward",
            "do not execute command move forward",
            "我听到电视里说执行命令向前走两步",
            "视频中有人说 execute command move forward",
        )
        for text in samples:
            self.assertFalse(gate_command(text).authorized, text)
        self.assertFalse(gate_command("执行指令，后退吗；").authorized)

    def test_robot_name_may_precede_authorization_phrase(self):
        self.assertEqual(
            gate_command("邦邦，执行命令，后退一米").command_text,
            "后退一米",
        )

    def test_alarm_and_stop_safety_shortcuts(self):
        self.assertTrue(is_alarm_request("邦邦，一键报警"))
        self.assertTrue(is_alarm_request("call for help"))
        self.assertFalse(is_alarm_request("如何一键报警"))
        self.assertFalse(is_alarm_request("what is an emergency alert"))
        self.assertTrue(is_emergency_stop("邦邦马上停止"))
        self.assertTrue(is_emergency_stop("emergency stop now"))

    def test_asr_prompt_echo_cannot_trigger_alarm_or_motion(self):
        leaked_prompt = (
            "Bangbang, execute command, execute forward action, emergency alert, "
            "stop, ground guide line, traffic light"
        )
        self.assertTrue(is_asr_prompt_echo(leaked_prompt))
        self.assertFalse(is_alarm_request(leaked_prompt))
        self.assertFalse(gate_command(leaked_prompt).authorized)
        self.assertEqual(gate_command(leaked_prompt).reason, "asr_prompt_echo")


class ActionValidationTests(unittest.TestCase):
    def test_parses_common_commands_without_llm(self):
        forward = parse_deterministic_command("前进一米")
        turn = parse_deterministic_command("左转九十度")
        sequence = parse_deterministic_command("后退半米，然后右转30度")
        english = parse_deterministic_command(
            "move forward two meters then turn left forty five degrees"
        )

        self.assertEqual(forward[0]["action"], "forward")
        self.assertAlmostEqual(forward[0]["duration"], 1.0 / 0.3)
        self.assertEqual(turn[0]["action"], "turn_left")
        self.assertEqual(turn[0]["angle"], 90.0)
        self.assertEqual([item["action"] for item in sequence], ["backward", "turn_right"])
        self.assertEqual(sequence[1]["angle"], 30.0)
        self.assertEqual([item["action"] for item in english], ["forward", "turn_left"])
        self.assertAlmostEqual(english[0]["duration"], 2.0 / 0.3)
        self.assertEqual(english[1]["angle"], 45.0)

        steps = parse_deterministic_command("向前走两步")
        self.assertAlmostEqual(steps[0]["duration"], 0.8 / 0.3)

    def test_deterministic_parser_rejects_ambiguous_text(self):
        self.assertIsNone(parse_deterministic_command("去门口看看情况"))
        self.assertIsNone(parse_deterministic_command("前进并后退"))

    def test_rejects_unknown_and_non_finite_actions(self):
        actions, warnings = validate_actions([
            {"action": "run_shell", "duration": 1},
            {"action": "forward", "speed": math.inf, "duration": 2},
        ])
        self.assertEqual(actions, [])
        self.assertEqual(len(warnings), 2)

    def test_clamps_motion_and_visual_guide(self):
        actions, _ = validate_actions([
            {"action": "forward", "speed": 9, "duration": 999},
            {"action": "visual_guide", "speed": 2, "duration": 9999},
        ])
        self.assertLessEqual(actions[0].speed, 0.45)
        self.assertLessEqual(actions[0].duration, 12)
        self.assertLessEqual(actions[1].speed, 0.25)
        self.assertLessEqual(actions[1].duration, 120)

    def test_final_duration_is_computed_once_and_total_limit_is_real(self):
        actions, warnings = validate_actions([
            {"action": "turn_left", "speed": 0.1, "duration": 0, "angle": 180},
            {"action": "turn_right", "speed": 0.1, "duration": 0, "angle": 180},
            {"action": "turn_left", "speed": 0.1, "duration": 0, "angle": 180},
        ])
        self.assertLessEqual(sum(action.duration for action in actions), 30.0)
        self.assertTrue(any("总时长" in warning for warning in warnings))

    def test_speed_clamp_preserves_target_distance_before_compensation(self):
        actions, _ = validate_actions([
            {"action": "forward", "speed": 0.5, "duration": 2.0},
        ])
        self.assertAlmostEqual(actions[0].speed, 0.45)
        self.assertAlmostEqual(actions[0].duration, 1.0 / 0.45 * 1.3)


if __name__ == "__main__":
    unittest.main()
