"""Platform-independent line association and steering logic for OpenMV.

Coordinates are normalized to [-1, 1]. Positive values point toward the
left side of the image, matching the Orange Pi reference implementation.
"""


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class LineTracker:
    """Track one physical line across several horizontal scan bands."""

    def __init__(
        self,
        rows,
        min_valid_rows=3,
        selection="auto",
        max_jump=0.20,
        ambiguity_margin=0.05,
        startup_max_offset=0.45,
        outlier_residual=0.12,
    ):
        if len(rows) < 3:
            raise ValueError("At least three scan rows are required")
        if not all(0.0 < row < 1.0 for row in rows):
            raise ValueError("Scan rows must be normalized image positions")
        if not all(first > second for first, second in zip(rows, rows[1:])):
            raise ValueError("Scan rows must be ordered from near to far")
        if selection not in ("auto", "middle", "single"):
            raise ValueError("selection must be auto, middle, or single")
        if not 1 <= min_valid_rows <= len(rows):
            raise ValueError("Invalid min_valid_rows")
        self.rows = tuple(rows)
        self.min_valid_rows = min_valid_rows
        self.selection = selection
        self.max_jump = max_jump
        self.ambiguity_margin = ambiguity_margin
        self.startup_max_offset = startup_max_offset
        self.outlier_residual = outlier_residual
        self.reset()

    def reset(self):
        count = len(self.rows)
        self.previous = [0.0] * count
        self.has_track = [False] * count
        self.last_valid = [False] * count

    def _normalized_times(self):
        return [
            (self.rows[0] - row) / (self.rows[0] - self.rows[-1])
            for row in self.rows
        ]

    def _choose(self, candidates, index, predicted):
        if not candidates:
            return None

        if self.has_track[index]:
            ordered = sorted(
                candidates,
                key=lambda value: abs(value - self.previous[index]),
            )
            best_distance = abs(ordered[0] - self.previous[index])
            if best_distance > self.max_jump:
                return None
            if len(ordered) > 1:
                second_distance = abs(ordered[1] - self.previous[index])
                if second_distance - best_distance < self.ambiguity_margin:
                    return None
            return ordered[0]

        if len(candidates) == 3 and self.selection != "single":
            choice = sorted(candidates)[1]
        elif len(candidates) == 1 and self.selection != "middle":
            choice = candidates[0]
        else:
            return None

        if abs(choice - predicted) > self.startup_max_offset:
            return None
        return choice

    def _reject_outliers(self, centers, valid):
        valid_count = sum(valid)
        if valid_count < 3:
            return valid

        indices = [index for index, is_valid in enumerate(valid) if is_valid]
        times = self._normalized_times()
        best = valid[:]
        best_count = -1
        best_error = 0.0

        for first_pos in range(len(indices)):
            for second_pos in range(first_pos + 1, len(indices)):
                first = indices[first_pos]
                second = indices[second_pos]
                dt = times[second] - times[first]
                if abs(dt) < 1e-6:
                    continue

                slope = (centers[second] - centers[first]) / dt
                inliers = [False] * len(self.rows)
                error = 0.0
                count = 0
                for index in indices:
                    expected = centers[first] + slope * (times[index] - times[first])
                    residual = abs(centers[index] - expected)
                    if residual <= self.outlier_residual:
                        inliers[index] = True
                        error += residual
                        count += 1

                if count > best_count or (
                    count == best_count and error < best_error
                ):
                    best = inliers
                    best_count = count
                    best_error = error

        return best

    def detect(self, candidates_by_row):
        """Process candidate x offsets and return near, far and confidence.

        ``candidates_by_row`` must contain one list of normalized x offsets
        for each configured scan row, ordered near to far.
        """

        if len(candidates_by_row) != len(self.rows):
            raise ValueError("Candidate count does not match scan rows")

        centers = self.previous[:]
        valid = [False] * len(self.rows)

        for index in range(len(self.rows)):
            if index and valid[index - 1]:
                predicted = centers[index - 1]
            else:
                predicted = 0.0
            choice = self._choose(candidates_by_row[index], index, predicted)
            if choice is not None:
                centers[index] = choice
                valid[index] = True

        valid = self._reject_outliers(centers, valid)
        for index in range(len(self.rows)):
            if valid[index]:
                self.previous[index] = centers[index]
                self.has_track[index] = True

        self.last_valid = valid
        valid_indices = [index for index, item in enumerate(valid) if item]
        confidence = float(len(valid_indices)) / float(len(self.rows))
        if len(valid_indices) < 2:
            return 0.0, 0.0, confidence, centers, valid

        normalized_times = self._normalized_times()
        times = [normalized_times[index] for index in valid_indices]
        values = [centers[index] for index in valid_indices]
        mean_t = sum(times) / float(len(times))
        mean_x = sum(values) / float(len(values))
        denominator = sum((value - mean_t) ** 2 for value in times)

        if denominator < 1e-9:
            near = mean_x
            far = mean_x
        else:
            slope = (
                sum(
                    (times[index] - mean_t) * (values[index] - mean_x)
                    for index in range(len(times))
                )
                / denominator
            )
            intercept = mean_x - slope * mean_t
            near = intercept
            far = intercept + slope

        return clamp(near, -1.0, 1.0), clamp(far, -1.0, 1.0), confidence, centers, valid


def deadband(error, width):
    if error > width:
        return error - width
    if error < -width:
        return error + width
    return 0.0


def steering_from_image(
    near,
    far,
    near_reference=0.0,
    heading_reference=0.0,
    lateral_gain=0.35,
    heading_gain=0.8,
    lateral_deadband=0.0,
    heading_deadband=0.0,
    yaw_limit=0.12,
    turn_sign=1,
):
    """Return the reference image-space yaw command in radians per second."""

    lateral_error = near - near_reference
    heading_error = far - near - heading_reference
    lateral_error = deadband(lateral_error, lateral_deadband)
    heading_error = deadband(heading_error, heading_deadband)
    yaw = lateral_gain * lateral_error + heading_gain * heading_error
    return clamp(turn_sign * yaw, -yaw_limit, yaw_limit)
