"""OpenMV black-line follower adapted from the Orange Pi reference project.

Upload this file and line_core.py to the same directory on the OpenMV camera.
The UART output is a simple ASCII command stream; see README.md.
"""

import time

import sensor
import image
from pyb import UART

from line_core import LineTracker, steering_from_image


# Camera --------------------------------------------------------------------
CAMERA_ROTATED_180 = False
FRAME_SIZE = sensor.QQVGA

# Black-line detection ------------------------------------------------------
# Rows are ordered near (bottom) to far (top), as in the Orange Pi project.
SCAN_ROWS = (0.82, 0.71, 0.60, 0.49, 0.38)
BAND_HEIGHT = 7
LINE_SELECTION = "auto"  # auto, middle, or single
MIN_VALID_ROWS = 3
ACQUIRE_FRAMES = 3
MIN_LINE_WIDTH_PX = 2
MAX_LINE_WIDTH_FRACTION = 0.12
MIN_BAND_COVERAGE = 0.55
MIN_CONTRAST = 30
MAX_BLACK_THRESHOLD = 100
ADAPTIVE_BLACK = True

# Tracking gates ------------------------------------------------------------
MAX_JUMP = 0.20
AMBIGUITY_MARGIN = 0.05
STARTUP_MAX_OFFSET = 0.45
OUTLIER_RESIDUAL = 0.12

# Steering ------------------------------------------------------------------
NEAR_REFERENCE = 0.0
HEADING_REFERENCE = 0.0
LATERAL_GAIN = 0.35
HEADING_GAIN = 0.80
LATERAL_DEADBAND = 0.0
HEADING_DEADBAND = 0.0
TURN_SIGN = 1  # Set to -1 if the vehicle turns in the wrong direction.
YAW_LIMIT = 0.12
YAW_SMOOTHING = 0.35

# Safety --------------------------------------------------------------------
DEPARTURE_LOCK_ENABLED = True
MAX_NEAR_OFFSET = 0.35
DEPARTURE_HOLD_MS = 300

# UART ----------------------------------------------------------------------
UART_ENABLED = True
UART_ID = 3
UART_BAUDRATE = 115200
DRAW_DEBUG = True


def ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.time() * 1000)


def ticks_diff(new_value, old_value):
    try:
        return time.ticks_diff(new_value, old_value)
    except AttributeError:
        return new_value - old_value


def setup_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(FRAME_SIZE)
    try:
        sensor.set_hmirror(CAMERA_ROTATED_180)
        sensor.set_vflip(CAMERA_ROTATED_180)
    except Exception:
        pass
    sensor.skip_frames(time=2000)


def setup_uart():
    if not UART_ENABLED:
        return None
    return UART(UART_ID, baudrate=UART_BAUDRATE)


def normalized_x(center_x, width):
    return 1.0 - 2.0 * float(center_x) / float(width - 1)


def center_pixel(candidate, width):
    return int(round((1.0 - candidate) * float(width - 1) * 0.5))


def blob_value(blob, name):
    """Read a Blob field from both current attrtuple and older method APIs."""

    value = getattr(blob, name)
    if callable(value):
        return value()
    return value


def grayscale_statistic(statistics, index, name):
    """Read OpenMV statistics across old tuple and new attrtuple firmware.

    Current firmware exposes integer fields such as ``statistics.mean``.
    Older builds may expose methods, so only call the field when callable.
    """

    try:
        return float(statistics[index])
    except (IndexError, KeyError, TypeError):
        pass
    value = getattr(statistics, name)
    if callable(value):
        return float(value())
    return float(value)


def mean_luma(img, roi):
    statistics = img.get_statistics(roi=roi)
    return grayscale_statistic(statistics, 0, "mean")


def scan_luma_stats(img, roi):
    statistics = img.get_statistics(roi=roi)
    return (
        grayscale_statistic(statistics, 0, "mean"),
        grayscale_statistic(statistics, 4, "min"),
        grayscale_statistic(statistics, 5, "max"),
    )


def threshold_for_frame(img, width, height):
    near_y = int(SCAN_ROWS[0] * (height - 1))
    far_y = int(SCAN_ROWS[-1] * (height - 1))
    top = max(0, far_y - BAND_HEIGHT // 2)
    bottom = min(height, near_y + BAND_HEIGHT // 2 + 1)
    mean, low, high = scan_luma_stats(img, (0, top, width, bottom - top))

    if high - low < MIN_CONTRAST:
        return None
    if not ADAPTIVE_BLACK:
        return MAX_BLACK_THRESHOLD

    threshold = int(round(0.5 * (mean + low) + 5.0))
    threshold = max(int(low) + 3, min(MAX_BLACK_THRESHOLD, threshold))
    return threshold


def local_contrast_ok(img, x, y, width, height):
    if width < 1 or height < 1:
        return False
    try:
        inside = mean_luma(img, (x, y, width, height))
        outside_means = []
        padding = max(3, width)
        left_x = max(0, x - padding)
        if left_x < x:
            outside_means.append(
                mean_luma(img, (left_x, y, x - left_x, height))
            )
        right_x = min(img.width(), x + width + padding)
        if right_x > x + width:
            outside_means.append(
                mean_luma(
                    img,
                    (x + width, y, right_x - (x + width), height),
                )
            )
        if not outside_means:
            return True
        return max(outside_means) - inside >= MIN_CONTRAST
    except Exception:
        return True


def candidates_for_row(img, row_y, width, threshold):
    half_band = BAND_HEIGHT // 2
    top = max(0, row_y - half_band)
    roi_height = min(img.height() - top, BAND_HEIGHT)
    min_pixels = max(
        MIN_LINE_WIDTH_PX,
        int(roi_height * MIN_LINE_WIDTH_PX * MIN_BAND_COVERAGE),
    )
    blobs = img.find_blobs(
        [(0, threshold)],
        roi=(0, top, width, roi_height),
        pixels_threshold=min_pixels,
        area_threshold=max(1, MIN_LINE_WIDTH_PX * roi_height // 2),
        merge=True,
        margin=0,
    )

    max_width = int(width * MAX_LINE_WIDTH_FRACTION)
    candidates = []
    for blob in blobs:
        blob_width = blob_value(blob, "w")
        blob_x = blob_value(blob, "x")
        if blob_width < MIN_LINE_WIDTH_PX or blob_width > max_width:
            continue
        if blob_value(blob, "h") < int(roi_height * MIN_BAND_COVERAGE):
            continue
        if blob_x <= 1 or blob_x + blob_width >= width - 1:
            continue
        if not local_contrast_ok(
            img,
            blob_x,
            blob_value(blob, "y"),
            blob_width,
            blob_value(blob, "h"),
        ):
            continue
        candidates.append(normalized_x(blob_value(blob, "cx"), width))
    return candidates


def draw_debug(img, centers, valid, near, far, active, yaw):
    width = img.width()
    height = img.height()
    for index, fraction in enumerate(SCAN_ROWS):
        row_y = int(fraction * (height - 1))
        img.draw_line((0, row_y, width - 1, row_y), color=100, thickness=1)
        if valid[index]:
            x = center_pixel(centers[index], width)
            img.draw_circle((x, row_y, 4), color=255, thickness=2)

    text = "A%d Y%+04d N%+04d F%+04d" % (
        active,
        int(round(yaw * 1000.0)),
        int(round(near * 1000.0)),
        int(round(far * 1000.0)),
    )
    img.draw_string(2, 2, text, color=255, scale=1)


def send_uart(uart, sequence, active, near, far, yaw):
    if uart is None:
        return
    # LF,sequence,active,near_milli,far_milli,yaw_milli
    line = "LF,%d,%d,%d,%d,%d\r\n" % (
        sequence,
        active,
        int(round(near * 1000.0)),
        int(round(far * 1000.0)),
        int(round(yaw * 1000.0)),
    )
    uart.write(line.encode())


def run():
    setup_camera()
    uart = setup_uart()
    tracker = LineTracker(
        SCAN_ROWS,
        min_valid_rows=MIN_VALID_ROWS,
        selection=LINE_SELECTION,
        max_jump=MAX_JUMP,
        ambiguity_margin=AMBIGUITY_MARGIN,
        startup_max_offset=STARTUP_MAX_OFFSET,
        outlier_residual=OUTLIER_RESIDUAL,
    )

    sequence = 0
    acquired_frames = 0
    depart_started = None
    departure_locked = False
    smoothed_yaw = 0.0
    last_report = ticks_ms()
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()
        width = img.width()
        height = img.height()
        threshold = threshold_for_frame(img, width, height)

        if threshold is None:
            candidates = [[] for _ in SCAN_ROWS]
            near, far, _, centers, valid = tracker.detect(candidates)
        else:
            candidates = [
                candidates_for_row(
                    img,
                    int(fraction * (height - 1)),
                    width,
                    threshold,
                )
                for fraction in SCAN_ROWS
            ]
            near, far, _, centers, valid = tracker.detect(candidates)

        valid_count = sum(valid)
        detected = valid_count >= MIN_VALID_ROWS
        now = ticks_ms()

        lateral_error = near - NEAR_REFERENCE
        if DEPARTURE_LOCK_ENABLED and detected:
            if abs(lateral_error) > MAX_NEAR_OFFSET:
                if depart_started is None:
                    depart_started = now
                elif ticks_diff(now, depart_started) >= DEPARTURE_HOLD_MS:
                    departure_locked = True
            else:
                depart_started = None
        elif not detected:
            depart_started = None

        if departure_locked:
            acquired_frames = 0
            active = 0
            yaw = 0.0
        elif detected:
            if acquired_frames < ACQUIRE_FRAMES:
                acquired_frames += 1
            if acquired_frames >= ACQUIRE_FRAMES:
                target_yaw = steering_from_image(
                    near,
                    far,
                    near_reference=NEAR_REFERENCE,
                    heading_reference=HEADING_REFERENCE,
                    lateral_gain=LATERAL_GAIN,
                    heading_gain=HEADING_GAIN,
                    lateral_deadband=LATERAL_DEADBAND,
                    heading_deadband=HEADING_DEADBAND,
                    yaw_limit=YAW_LIMIT,
                    turn_sign=TURN_SIGN,
                )
                smoothed_yaw += YAW_SMOOTHING * (target_yaw - smoothed_yaw)
                if (
                    abs(near - NEAR_REFERENCE) <= LATERAL_DEADBAND
                    and abs(far - near - HEADING_REFERENCE) <= HEADING_DEADBAND
                ):
                    smoothed_yaw = 0.0
                active = 1
                yaw = smoothed_yaw
            else:
                active = 0
                yaw = 0.0
                smoothed_yaw = 0.0
        else:
            acquired_frames = 0
            active = 0
            smoothed_yaw = 0.0
            yaw = 0.0

        send_uart(uart, sequence, active, near, far, yaw)
        sequence = (sequence + 1) & 0xFF

        if DRAW_DEBUG:
            draw_debug(img, centers, valid, near, far, active, yaw)

        if ticks_diff(now, last_report) >= 500:
            print(
                "LF fps=%.1f valid=%d near=%.3f far=%.3f active=%d yaw=%+.3f"
                % (clock.fps(), valid_count, near, far, active, yaw)
            )
            last_report = now


if __name__ == "__main__":
    try:
        run()
    finally:
        # Runs when the script is stopped from OpenMV IDE or reset.
        try:
            if UART_ENABLED:
                emergency_uart = UART(UART_ID, baudrate=UART_BAUDRATE)
                emergency_uart.write(b"LF,0,0,0,0,0\r\n")
        except Exception:
            pass
