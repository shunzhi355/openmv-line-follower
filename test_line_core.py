import math
import unittest

from line_core import LineTracker, steering_from_image, steering_from_rois


ROWS = (0.82, 0.71, 0.60, 0.49, 0.38)
WEIGHTS = (0.40, 0.25, 0.18, 0.11, 0.06)


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


class WeightedROITest(unittest.TestCase):
    def steer(self, centers, valid=None, near=0.0, far=0.0, **kwargs):
        if valid is None:
            valid = [True] * len(ROWS)
        return steering_from_rois(
            ROWS, centers, valid, WEIGHTS, 160, 120, near, far, **kwargs
        )

    def test_centered_line_has_zero_turn(self):
        self.assertEqual(self.steer([0.0] * 5), 0.0)

    def test_offset_matches_upstream_bearing_and_sign(self):
        expected = 0.30 * math.atan(0.2 * 159 / 120)
        self.assertAlmostEqual(self.steer([0.2] * 5), expected)
        self.assertAlmostEqual(self.steer([-0.2] * 5), -expected)

    def test_missing_near_bands_do_not_bias_weighted_center(self):
        # The first two entries are stale; they must not affect the denominator
        # or be included as measured positions when their bands disappear.
        partial = self.steer([0.9, -0.8, 0.2, 0.2, 0.2], [False, False, True, True, True])
        self.assertAlmostEqual(partial, self.steer([0.2] * 5))

    def test_near_band_has_more_influence_than_far_band(self):
        near_turn = self.steer([0.2, 0.0, 0.0, 0.0, 0.0])
        far_turn = self.steer([0.0, 0.0, 0.0, 0.0, 0.2])
        self.assertGreater(near_turn, far_turn)
        self.assertGreater(far_turn, 0.0)

    def test_reference_stays_neutral_when_bands_disappear(self):
        centers = [0.1 + 0.2 * (ROWS[0] - row) / (ROWS[0] - ROWS[-1]) for row in ROWS]
        for valid in ([True] * 5, [False, False, True, True, True]):
            self.assertAlmostEqual(
                self.steer(centers, valid, near=0.1, far=0.3,
                           near_reference=0.1, heading_reference=0.2),
                0.0,
            )

    def test_fewer_than_three_bands_cannot_command_a_turn(self):
        for valid in ([False] * 5, [True, True, False, False, False]):
            self.assertEqual(self.steer([0.3] * 5, valid, near=0.3, far=0.8), 0.0)

    def test_heading_correction_and_yaw_limits_are_retained(self):
        self.assertAlmostEqual(self.steer([0.0] * 5, far=0.1), 0.08)
        self.assertEqual(self.steer([0.8] * 5, far=0.8), 0.12)
        self.assertEqual(self.steer([0.8] * 5, far=0.8, turn_sign=-1), -0.12)

    def test_deadbands_clear_small_errors(self):
        self.assertEqual(
            self.steer([0.01] * 5, far=0.01, lateral_deadband=0.02, heading_deadband=0.02),
            0.0,
        )

    def test_three_line_tracking_feeds_selected_center_to_roi_steering(self):
        tracker = LineTracker(ROWS)
        near, far, _, centers, valid = tracker.detect([[-0.5, 0.0, 0.5]] * 5)
        self.assertEqual(self.steer(centers, valid, near, far), 0.0)
        near, far, _, centers, valid = tracker.detect([[0.5]] * 5)
        self.assertEqual(self.steer(centers, valid, near, far), 0.0)
        self.assertFalse(any(valid))

    def test_synthetic_bends_keep_left_right_symmetry(self):
        turns = []
        for direction in (1, -1):
            candidates = [[direction * x] for x in (0.02, 0.04, 0.09, 0.17, 0.28)]
            near, far, _, centers, valid = LineTracker(ROWS).detect(candidates)
            self.assertGreaterEqual(sum(valid), 3)
            turns.append(self.steer(centers, valid, near, far))
        self.assertGreater(turns[0], 0.0)
        self.assertAlmostEqual(turns[0], -turns[1])

    def test_invalid_weights_are_rejected(self):
        for weights in ((1.0,), (1.0, 1.0, 0.0, 1.0, 1.0)):
            with self.assertRaises(ValueError):
                steering_from_rois(ROWS, [0.0] * 5, [True] * 5, weights, 160, 120, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
