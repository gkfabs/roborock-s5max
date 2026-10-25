import importlib.util
import math
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'dock_sim_adapter.py'
)
SPEC = importlib.util.spec_from_file_location('dock_sim_adapter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_far_centered_signal_uses_the_common_far_classes():
    left, right, bearing = MODULE.simulated_signal(
        -0.4, 0.0, 0.0, 0.18, 0.0)

    assert left == 0x12
    assert right == 0x12
    assert math.isclose(bearing, 0.0)


def test_close_centered_signal_uses_the_measured_close_classes():
    left, right, bearing = MODULE.simulated_signal(
        -0.1, 0.0, 0.0, 0.18, 0.0)

    assert left == 0x14
    assert right == 0x18
    assert math.isclose(bearing, 0.0)


def test_large_left_error_activates_only_left_receiver():
    left, right, bearing = MODULE.simulated_signal(
        -0.4, -0.2, 0.0, 0.18, 0.0)

    assert left == 0x15
    assert right == 0
    assert bearing > 0.0


def test_contact_requires_lateral_and_heading_alignment():
    assert MODULE.simulated_contact(
        -0.0365, 0.0, 0.0, -0.0365, 0.015, 0.015, 0.05)
    assert not MODULE.simulated_contact(
        -0.0365, 0.02, 0.0, -0.0365, 0.015, 0.015, 0.05)
    assert not MODULE.simulated_contact(
        -0.0365, 0.0, 0.06, -0.0365, 0.015, 0.015, 0.05)


def test_contact_remains_latched_during_small_physics_drift():
    assert MODULE.retain_simulated_contact(
        True, False, -0.075, 0.035, 0.15, -0.0365)


def test_contact_releases_after_the_robot_moves_away():
    assert not MODULE.retain_simulated_contact(
        True, False, -0.12, 0.0, 0.0, -0.0365)


def test_missed_contact_blocks_the_first_attempt():
    assert not MODULE.scenario_contact(
        'missed-contact', False, 'WAIT_FOR_CHARGE', True, True)


def test_missed_contact_accepts_bumper_contact_after_retry():
    assert MODULE.scenario_contact(
        'missed-contact', True, 'WAIT_FOR_CHARGE', True, False)


def test_regular_scenario_keeps_measured_contact():
    assert MODULE.scenario_contact(
        'centered', False, 'CONTROLLING', False, True)


def test_world_pose_applies_the_spawn_offset():
    x, y, yaw = MODULE.world_pose(
        0.1, 0.0, 0.2, -0.45, 0.05, 0.0)

    assert math.isclose(x, -0.35)
    assert math.isclose(y, 0.05)
    assert math.isclose(yaw, 0.2)
