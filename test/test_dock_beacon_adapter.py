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


def test_rich_signal_preserves_all_five_classes_and_masks():
    signal = MODULE.decode_signal(0x15, 0x1A, 0.5)

    assert signal.left_mask == 0x15
    assert signal.right_mask == 0x1A
    assert signal.visible
    assert signal.bearing == 0.0
    assert signal.confidence == 1.0
    assert signal.far_left
    assert signal.far_right
    assert signal.close_left
    assert signal.close_right
    assert signal.center


def test_rich_signal_reports_one_receiver_as_half_confidence():
    signal = MODULE.decode_signal(0x00, 0x15, 0.5)

    assert signal.visible
    assert signal.bearing == -0.5
    assert signal.confidence == 0.5
    assert signal.far_left
    assert not signal.far_right
    assert signal.close_left
    assert not signal.close_right
    assert signal.center


def test_rich_signal_removes_bits_outside_the_five_signal_classes():
    signal = MODULE.decode_signal(0xE0, 0x80, 0.5)

    assert signal.left_mask == 0
    assert signal.right_mask == 0
    assert not signal.visible
    assert signal.bearing is None
    assert signal.confidence == 0.0
