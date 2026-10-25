import importlib.util
import math
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'dock_scan_fault_adapter.py'
)
SPEC = importlib.util.spec_from_file_location(
    'dock_scan_fault_adapter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_clean_profile_does_not_change_ranges():
    output = MODULE.apply_lidar_noise(
        [0.2, 0.3], 0.1, 8.0, 'clean', 0.0, 0.0, 1, 1)

    assert output == [0.2, 0.3]


def test_realistic_profile_is_deterministic():
    first = MODULE.apply_lidar_noise(
        [0.5] * 20, 0.1, 8.0, 'realistic', 0.04, 0.2, 7, 3)
    second = MODULE.apply_lidar_noise(
        [0.5] * 20, 0.1, 8.0, 'realistic', 0.04, 0.2, 7, 3)

    assert first == second


def test_unknown_profile_fails():
    with pytest.raises(ValueError):
        MODULE.lidar_noise_parameters('unknown', 0.0, 0.0)


def test_front_outage_hides_only_the_front_sector():
    output = MODULE.apply_front_sector_outage(
        [1.0] * 5, -2.0, 1.0, True, sector_angle=0.6)

    assert output[0] == 1.0
    assert output[1] == 1.0
    assert math.isinf(output[2])
    assert output[3] == 1.0
    assert output[4] == 1.0


def test_outage_window_has_a_bounded_end():
    assert MODULE.outage_is_active(2.0, 1.0, 2.0)
    assert not MODULE.outage_is_active(3.0, 1.0, 2.0)
