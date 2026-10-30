import sys
import types
import unittest
from unittest import mock


class FakeStatistics:
    def __init__(self, mean, low, high):
        self.values = (mean, mean, mean, 0, low, high)

    def __getitem__(self, index):
        return self.values[index]


class FakeBlob:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.w = width
        self.h = height
        self.cx = x + width // 2


class FakeFrame:
    def __init__(self, line_center=80, line_width=3, crossbar_rows=(), empty=False):
        self.line_center = line_center
        self.line_width = line_width
        self.crossbar_rows = crossbar_rows
        self.empty = empty
        self.drawn_strings = []

    def width(self):
        return 160

    def height(self):
        return 120

    def _overlap(self, x, width):
        if self.empty:
            return 0
        start = max(x, self.line_center - self.line_width // 2)
        end = min(x + width, self.line_center + self.line_width // 2 + 1)
        return max(0, end - start)

    def get_statistics(self, roi):
        x, _, width, height = roi
        line_pixels = self._overlap(x, width) * height
        total = max(1, width * height)
        if line_pixels:
            mean = int(round((230.0 * (total - line_pixels) + 20.0 * line_pixels) / total))
            return FakeStatistics(mean, 20, 240)
        return FakeStatistics(230, 225, 240)

    def find_blobs(self, thresholds, roi, **kwargs):
        x, y, width, height = roi
        if any(y <= crossbar_y < y + height for crossbar_y in self.crossbar_rows):
            return [FakeBlob(x, y, width, height)]
        overlap = self._overlap(x, width)
        if not overlap:
            return []
        left = max(x, self.line_center - self.line_width // 2)
        return [FakeBlob(left, y, overlap, height)]

    def draw_line(self, *args, **kwargs):
        if not isinstance(args[0], tuple):
            raise TypeError("draw_line expects a coordinate tuple")

    def draw_circle(self, *args, **kwargs):
        if not isinstance(args[0], tuple):
            raise TypeError("draw_circle expects a coordinate tuple")

    def draw_string(self, *args, **kwargs):
        if not isinstance(args[0], tuple):
            raise TypeError("object 'int' isn't a tuple or list")
        if len(args) != 2 or len(args[0]) != 2:
            raise ValueError("draw_string expects (x, y), text")
        if not isinstance(args[1], str):
            raise TypeError("draw_string expects text after the position tuple")
        self.drawn_strings.append((args[0], args[1], kwargs))


class FakeSensor:
    QQVGA = (160, 120)
    GRAYSCALE = 1
    snapshots = 0

    @classmethod
    def reset(cls):
        cls.snapshots = 0

    @classmethod
    def set_pixformat(cls, value):
        pass

    @classmethod
    def set_framesize(cls, value):
        pass

    @classmethod
    def set_hmirror(cls, value):
        pass

    @classmethod
    def set_vflip(cls, value):
        pass

    @classmethod
    def skip_frames(cls, time):
        pass

    @classmethod
    def snapshot(cls):
        if cls.snapshots >= 6:
            raise RuntimeError("mock camera stopped")
        cls.snapshots += 1
        return FakeFrame()


class FakeUART:
    writes = []

    def __init__(self, *args, **kwargs):
        pass

    def write(self, data):
        self.__class__.writes.append(data)


class FakeClock:
    def tick(self):
        pass

    def fps(self):
        return 20.0


class MainHardwareFlowTest(unittest.TestCase):
    def setUp(self):
        sensor_module = types.ModuleType("sensor")
        sensor_module.QQVGA = FakeSensor.QQVGA
        sensor_module.GRAYSCALE = FakeSensor.GRAYSCALE
        sensor_module.reset = FakeSensor.reset
        sensor_module.set_pixformat = FakeSensor.set_pixformat
        sensor_module.set_framesize = FakeSensor.set_framesize
        sensor_module.set_hmirror = FakeSensor.set_hmirror
        sensor_module.set_vflip = FakeSensor.set_vflip
        sensor_module.skip_frames = FakeSensor.skip_frames
        sensor_module.snapshot = FakeSensor.snapshot

        pyb_module = types.ModuleType("pyb")
        pyb_module.UART = FakeUART

        modules = mock.patch.dict(sys.modules, {
            "sensor": sensor_module,
            "image": types.ModuleType("image"),
            "pyb": pyb_module,
        })
        modules.start()
        self.addCleanup(modules.stop)
        sys.modules.pop("main", None)

        import main

        self.main = main
        clock_patch = mock.patch.object(main.time, "clock", lambda: FakeClock(), create=True)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)
        FakeUART.writes = []

    def run_frames(self, frames):
        with mock.patch.object(
            self.main.sensor, "snapshot", side_effect=list(frames) + [RuntimeError("mock camera stopped")]
        ):
            with self.assertRaisesRegex(RuntimeError, "mock camera stopped"):
                self.main.run()
        return [[int(field) for field in item.decode().strip().split(",")[1:]]
                for item in FakeUART.writes]

    def test_centered_line_reaches_active_uart_command(self):
        with self.assertRaisesRegex(RuntimeError, "mock camera stopped"):
            self.main.run()

        messages = [item.decode().strip() for item in FakeUART.writes]
        active_messages = [item for item in messages if item.split(",")[2] == "1"]
        self.assertTrue(active_messages)
        fields = active_messages[-1].split(",")
        self.assertEqual(fields[0], "LF")
        self.assertEqual(int(fields[2]), 1)
        self.assertLess(abs(int(fields[5])), 30)

    def test_loss_stops_immediately_and_requires_three_frames_to_restart(self):
        messages = self.run_frames(
            [FakeFrame(72)] * 4 + [FakeFrame(empty=True)] + [FakeFrame(72)] * 3
        )
        self.assertEqual([message[1] for message in messages], [0, 0, 1, 1, 0, 0, 0, 1])
        self.assertEqual(messages[4][-1], 0)
        self.assertGreater(messages[-1][-1], 0)

    def test_crossbar_in_one_band_preserves_other_bands(self):
        messages = self.run_frames([FakeFrame(crossbar_rows=(71,))] * 4)
        self.assertEqual(messages[-1][1], 1)
        self.assertLess(abs(messages[-1][-1]), 30)

    def test_crossbar_hiding_three_bands_stops_motion(self):
        messages = self.run_frames(
            [FakeFrame()] * 3 + [FakeFrame(crossbar_rows=(97, 84, 71))]
        )
        self.assertEqual(messages[-2][1], 1)
        self.assertEqual(messages[-1][1], 0)
        self.assertEqual(messages[-1][-1], 0)

    def test_default_mode_uses_weighted_bearing_in_uart(self):
        self.main.YAW_SMOOTHING = 1.0
        messages = self.run_frames([FakeFrame(72)] * 3)
        # x=72 -> normalized offset=15/159. The upstream bearing is atan(15/120).
        # Weighted mode gives ~37 mrad/s; legacy mode would give ~33 mrad/s.
        self.assertEqual(messages[-1][-1], 37)

    def test_original_steering_mode_remains_selectable(self):
        self.main.STEERING_MODE = "near_far"
        self.main.YAW_SMOOTHING = 1.0
        messages = self.run_frames([FakeFrame(72)] * 3)
        self.assertEqual(messages[-1][-1], 33)

    def test_right_offset_produces_negative_uart_yaw(self):
        messages = self.run_frames([FakeFrame(87)] * 3)
        self.assertLess(messages[-1][-1], 0)

    def test_old_method_and_current_attribute_blob_apis(self):
        class OldBlob:
            def w(self):
                return 3
        self.assertEqual(self.main.blob_value(OldBlob(), "w"), 3)
        self.assertEqual(self.main.blob_value(FakeBlob(78, 50, 3, 7), "w"), 3)

    def test_debug_text_accepts_current_tuple_coordinate_api(self):
        frame = FakeFrame()
        self.main.draw_debug(frame, [0.0] * 5, [True] * 5, 0.0, 0.0, 1, 0.0)
        self.assertEqual(frame.drawn_strings, [
            ((2, 2), "A1 Y+000 N+000 F+000", {"color": 255, "scale": 1})
        ])

    def test_debug_text_falls_back_to_legacy_api(self):
        for error_type in (TypeError, ValueError):
            with self.subTest(error_type=error_type):
                class LegacyFrame(FakeFrame):
                    def draw_string(self, *args, **kwargs):
                        if isinstance(args[0], tuple):
                            raise error_type("legacy firmware requires x, y, text")
                        if len(args) != 3:
                            raise ValueError("expected three arguments")
                        self.drawn_strings.append(((args[0], args[1]), args[2], kwargs))

                frame = LegacyFrame()
                self.main.draw_debug(frame, [0.0] * 5, [True] * 5, 0.0, 0.0, 1, 0.0)
                self.assertEqual(frame.drawn_strings, [
                    ((2, 2), "A1 Y+000 N+000 F+000", {"color": 255, "scale": 1})
                ])


if __name__ == "__main__":
    unittest.main()
