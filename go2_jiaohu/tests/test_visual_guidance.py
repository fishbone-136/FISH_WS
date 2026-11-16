import unittest

import cv2
import numpy as np

from voice.visual_guidance import analyze_frame, detect_ground_guide, detect_traffic_light


class LocalPerceptionTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.rectangle(self.frame, (295, 180), (345, 359), (255, 255, 255), -1)

    def test_center_ground_guide(self):
        found, offset = detect_ground_guide(self.frame)
        self.assertTrue(found)
        self.assertAlmostEqual(offset, 0.0, delta=0.08)

    def test_red_has_safety_priority(self):
        cv2.circle(self.frame, (250, 80), 18, (0, 0, 255), -1)
        cv2.circle(self.frame, (390, 80), 18, (0, 255, 0), -1)
        signal, confidence = detect_traffic_light(self.frame)
        self.assertEqual(signal, "red")
        self.assertGreater(confidence, 0)

    def test_green_light_and_depth_obstacle(self):
        cv2.circle(self.frame, (320, 80), 18, (0, 255, 0), -1)
        depth = np.full((360, 640), 2000, dtype=np.uint16)
        depth[160:310, 230:410] = 420
        result = analyze_frame(self.frame, depth)
        self.assertEqual(result.traffic_light, "green")
        self.assertTrue(result.line_found)
        self.assertLess(result.obstacle_distance_m, 0.6)


if __name__ == "__main__":
    unittest.main()

