import importlib.util
import math
from pathlib import Path
import sys


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'dock_beacon_adapter.py'
)
SPEC = importlib.util.spec_from_file_location('dock_beacon_adapter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def decode(left_mask, right_mask):
    return MODULE.decode_beacon(left_mask, right_mask, math.atan2(0.30, 0.60))


def test_no_beacon_is_not_visible():
    assert decode(0x00, 0x00) == (False, None)


def test_centered_far_and_close_captures_have_zero_bearing():
    assert decode(0x12, 0x12) == (True, 0.0)
    assert decode(0x14, 0x18) == (True, 0.0)


def test_left_receiver_only_places_beacon_to_robot_left():
    visible, bearing = decode(0x1A, 0x00)
    assert visible
    assert bearing > 0.0


def test_right_receiver_only_places_beacon_to_robot_right():
    visible, bearing = decode(0x00, 0x15)
    assert visible
    assert bearing < 0.0
