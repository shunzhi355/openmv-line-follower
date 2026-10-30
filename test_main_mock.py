import sys
import types
import unittest


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
    def __init__(self):
        self.line_center = 80
        self.line_width = 3

    def width(self):
        return 160

    def height(self):
        return 120

    def _overlap(self, x, width):
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
        if not isinstance(args[0], int) or not isinstance(args[1], int):
            raise TypeError("draw_string expects x and y integers")


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
    def test_centered_line_reaches_active_uart_command(self):
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

        sys.modules["sensor"] = sensor_module
        sys.modules["image"] = types.ModuleType("image")
        sys.modules["pyb"] = pyb_module
        sys.modules.pop("main", None)

        import main

        main.time.clock = lambda: FakeClock()
        FakeUART.writes = []
        with self.assertRaisesRegex(RuntimeError, "mock camera stopped"):
            main.run()

        messages = [item.decode().strip() for item in FakeUART.writes]
        active_messages = [item for item in messages if ",1," in item]
        self.assertTrue(active_messages)
        fields = active_messages[-1].split(",")
        self.assertEqual(fields[0], "LF")
        self.assertEqual(int(fields[2]), 1)
        self.assertLess(abs(int(fields[5])), 30)


if __name__ == "__main__":
    unittest.main()
