import unittest

from line_core import LineTracker, steering_from_image


ROWS = (0.82, 0.71, 0.60, 0.49, 0.38)


class LineTrackerTest(unittest.TestCase):
    def test_straight_center_line_has_zero_turn(self):
        tracker = LineTracker(ROWS)
        near, far, confidence, _, valid = tracker.detect([[0.0]] * len(ROWS))
        self.assertAlmostEqual(near, 0.0)
        self.assertAlmostEqual(far, 0.0)
        self.assertEqual(confidence, 1.0)
        self.assertTrue(all(valid))
        self.assertAlmostEqual(steering_from_image(near, far), 0.0)

    def test_three_candidates_select_middle_line(self):
        tracker = LineTracker(ROWS)
        near, far, _, _, _ = tracker.detect(
            [[-0.5, 0.0, 0.5]] * len(ROWS)
        )
        self.assertAlmostEqual(near, 0.0)
        self.assertAlmostEqual(far, 0.0)

    def test_line_to_left_steers_left(self):
        tracker = LineTracker(ROWS)
        near, far, _, _, _ = tracker.detect(
            [[0.15], [0.25], [0.35], [0.45], [0.55]]
        )
        self.assertGreater(near, 0.0)
        self.assertGreater(far, near)
        self.assertGreater(steering_from_image(near, far), 0.0)

    def test_line_to_right_steers_right(self):
        tracker = LineTracker(ROWS)
        near, far, _, _, _ = tracker.detect(
            [[-0.15], [-0.25], [-0.35], [-0.45], [-0.55]]
        )
        self.assertLess(near, 0.0)
        self.assertLess(far, near)
        self.assertLess(steering_from_image(near, far), 0.0)

    def test_edge_line_does_not_replace_acquired_center(self):
        tracker = LineTracker(ROWS)
        tracker.detect([[0.0]] * len(ROWS))
        _, _, _, _, valid = tracker.detect([[0.5]] * len(ROWS))
        self.assertFalse(any(valid))

    def test_one_inconsistent_band_is_rejected(self):
        tracker = LineTracker(ROWS)
        _, _, _, _, valid = tracker.detect(
            [[0.0], [0.0], [0.0], [0.0], [0.8]]
        )
        self.assertEqual(sum(valid), 4)
        self.assertFalse(valid[-1])

    def test_blank_frame_has_zero_turn(self):
        tracker = LineTracker(ROWS)
        near, far, confidence, _, valid = tracker.detect([[]] * len(ROWS))
        self.assertEqual(confidence, 0.0)
        self.assertFalse(any(valid))
        self.assertEqual(steering_from_image(near, far), 0.0)

    def test_yaw_limit_is_applied(self):
        self.assertLessEqual(abs(steering_from_image(1.0, 1.0)), 0.12)


if __name__ == "__main__":
    unittest.main()
