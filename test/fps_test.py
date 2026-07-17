#!/usr/bin/env python3
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import fps


class FakeADC:
    def __init__(self):
        self.calls = []
    def setup_adc_sample(self, *args):
        self.calls.append(("sample", args))
    def setup_adc_stream(self, **kwargs):
        self.calls.append(("stream", kwargs))
    def setup_adc_callback(self, callback):
        self.callback = callback


class FakePins:
    def __init__(self, adc):
        self.adc = adc
    def setup_pin(self, pin_type, pin):
        assert (pin_type, pin) == ("adc", "fps:PA2")
        return self.adc


class FakePrinter:
    def __init__(self, adc):
        self.pins = FakePins(adc)
    def get_reactor(self):
        return object()
    def add_object(self, name, value):
        pass
    def lookup_object(self, name):
        assert name == "pins"
        return self.pins


class FakeConfig:
    def __init__(self, adc, use_stream=True):
        self.printer = FakePrinter(adc)
        self.use_stream = use_stream
    def get_printer(self):
        return self.printer
    def get_name(self):
        return "fps"
    def get(self, name):
        assert name == "pin"
        return "fps:PA2"
    def getint(self, name, default):
        return default
    def getfloat(self, name, default):
        return default
    def getboolean(self, name, default):
        return self.use_stream if name == "use_adc_stream" else default


def test_helix_fps_schedule_and_opt_out():
    adc = FakeADC()
    fps.FPS(FakeConfig(adc))
    assert adc.calls == [
        ("stream", {"report_class": 1}),
        ("sample", (.100, .005, 5)),
    ]

    legacy = FakeADC()
    fps.FPS(FakeConfig(legacy, use_stream=False))
    assert legacy.calls == [("sample", (.100, .005, 5))]


if __name__ == "__main__":
    test_helix_fps_schedule_and_opt_out()
    print("PASS: FPS DMA opt-in, timing contract, and legacy override")
