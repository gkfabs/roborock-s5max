import importlib.util
import math
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'dock_sim_adapter.py'
)
SPEC = importlib.util.spec_from_file_location('dock_sim_adapter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_centered_signal_matches_the_physical_capture():
    front, right, bearing = MODULE.simulated_signal(
        -0.45, 0.0, 0.0, 0.17, 0.0)

    assert front == 0x15
    assert right == 0x18
    assert math.isclose(bearing, 0.0)


def test_robot_left_uses_only_the_right_receiver():
    front, right, bearing = MODULE.simulated_signal(
        -0.45, 0.15, 0.0, 0.17, 0.0)

    assert front == 0
    assert right == 0x15
    assert bearing < 0.0


def test_robot_right_matches_overlapping_front_and_right_receivers():
    front, right, bearing = MODULE.simulated_signal(
        -0.45, -0.10, 0.0, 0.17, 0.0)

    assert front == 0x1A
    assert right == 0x18
    assert bearing > 0.0


def test_contact_requires_lateral_and_heading_alignment():
    assert MODULE.simulated_contact(
        -0.060, 0.0, 0.0, -0.060, 0.015, 0.015, 0.05)
    assert not MODULE.simulated_contact(
        -0.060, 0.02, 0.0, -0.060, 0.015, 0.015, 0.05)
    assert not MODULE.simulated_contact(
        -0.060, 0.0, 0.06, -0.060, 0.015, 0.015, 0.05)


def test_contact_remains_latched_during_small_physics_drift():
    assert MODULE.retain_simulated_contact(
        True, False, -0.075, 0.035, 0.15, -0.060)


def test_contact_releases_after_the_robot_moves_away():
    assert not MODULE.retain_simulated_contact(
        True, False, -0.121, 0.0, 0.0, -0.060)


def test_missed_contact_blocks_the_first_attempt():
    assert not MODULE.scenario_contact(
        'missed-contact', False, 'WAIT_FOR_CHARGE', True, True)


def test_missed_contact_does_not_report_bumper_as_electrical_contact():
    assert not MODULE.scenario_contact(
        'missed-contact', True, 'WAIT_FOR_CHARGE', True, False)


def test_missed_contact_accepts_electrical_contact_after_retry():
    assert MODULE.scenario_contact(
        'missed-contact', True, 'WAIT_FOR_CHARGE', False, True)


def test_regular_scenario_keeps_measured_contact():
    assert MODULE.scenario_contact(
        'centered', False, 'CONTROLLING', False, True)


def test_world_pose_applies_the_spawn_offset():
    x, y, yaw = MODULE.world_pose(
        0.1, 0.0, 0.2, -0.45, 0.05, 0.0)

    assert math.isclose(x, -0.35)
    assert math.isclose(y, 0.05)
    assert math.isclose(yaw, 0.2)
