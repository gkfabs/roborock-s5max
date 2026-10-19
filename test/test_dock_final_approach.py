import importlib.util
import math
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'dock_final_approach.py'
)
SPEC = importlib.util.spec_from_file_location('dock_final_approach', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_steering_uses_beacon_bearing_sign():
    left = MODULE.steering_command(0.4, 0.035, 1.5, 0.4)
    right = MODULE.steering_command(-0.4, 0.035, 1.5, 0.4)

    assert math.isclose(left[0], 0.035)
    assert math.isclose(left[1], 0.4)
    assert math.isclose(right[0], 0.035)
    assert math.isclose(right[1], -0.4)


def test_zero_bearing_drives_straight():
    linear, angular = MODULE.steering_command(0.0, 0.035, 1.5, 0.4)

    assert math.isclose(linear, 0.035)
    assert math.isclose(angular, 0.0)


def test_planar_distance_bounds_total_travel():
    assert math.isclose(
        MODULE.planar_distance(1.0, 2.0, 1.12, 2.16),
        0.2,
    )
