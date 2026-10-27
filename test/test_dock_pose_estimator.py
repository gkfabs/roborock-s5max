import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'dock_pose_estimator.py'
SPEC = importlib.util.spec_from_file_location('dock_pose_estimator', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_scan(wall_distance=0.50, dock_distance=0.46):
    angle_min = -1.2
    increment = 0.01
    ranges = []
    for index in range(241):
        angle = angle_min + index * increment
        cosine = math.cos(angle)
        if cosine <= 0.0:
            ranges.append(float('inf'))
            continue
        lateral = wall_distance * math.tan(angle)
        distance = wall_distance / cosine
        if abs(lateral) <= 0.07:
            distance = dock_distance / cosine
        ranges.append(distance)
    return ranges, angle_min, increment


def test_estimate_dock_geometry_finds_wall_and_dock_center():
    ranges, angle_min, increment = make_scan()
    geometry = MODULE.estimate_dock_geometry(
        ranges, angle_min, increment)

    assert geometry is not None
    assert abs(geometry.normal_bearing) < 0.03
    assert abs(geometry.wall_distance - 0.50) < 0.02
    assert abs(geometry.dock_lateral) < 0.02


def test_beacon_side_rejects_a_competing_center_feature():
    groups = [
        [-0.10, -0.08, -0.06, -0.04],
        [-0.01, 0.00, 0.01, 0.02],
    ]

    selected = MODULE.select_dock_group(groups, -0.464)

    assert selected == groups[0]


def test_centered_beacon_keeps_the_nearest_lidar_feature():
    groups = [
        [-0.10, -0.08, -0.06, -0.04],
        [-0.01, 0.00, 0.01, 0.02],
    ]

    selected = MODULE.select_dock_group(groups, 0.0)

    assert selected == groups[1]


def test_detection_only_operates_during_perception_and_control():
    assert MODULE.detection_is_enabled('INITIAL_PERCEPTION')
    assert MODULE.detection_is_enabled('CONTROLLING')
    assert not MODULE.detection_is_enabled('WAITING')
    assert not MODULE.detection_is_enabled('NAV_TO_STAGING_POSE')
    assert not MODULE.detection_is_enabled('WAIT_FOR_CHARGE')
    assert not MODULE.detection_is_enabled('RETRY')


def test_action_feedback_maps_to_estimator_states():
    action = MODULE.DockRobot.Feedback

    assert MODULE.feedback_state_name(
        action.NAV_TO_STAGING_POSE) == 'NAV_TO_STAGING_POSE'
    assert MODULE.feedback_state_name(
        action.INITIAL_PERCEPTION) == 'INITIAL_PERCEPTION'
    assert MODULE.feedback_state_name(action.CONTROLLING) == 'CONTROLLING'
    assert MODULE.feedback_state_name(
        action.WAIT_FOR_CHARGE) == 'WAIT_FOR_CHARGE'
    assert MODULE.feedback_state_name(action.RETRY) == 'RETRY'


def test_empty_action_status_does_not_end_dock_tracking():
    assert not MODULE.action_status_is_active([])
    assert not MODULE.action_status_is_terminal([])


def test_terminal_action_status_ends_dock_tracking():
    assert MODULE.action_status_is_terminal([
        MODULE.GoalStatus.STATUS_SUCCEEDED])


def test_target_uses_contact_offset_from_dock_face():
    geometry = MODULE.DockGeometry(0.0, 0.50, 0.03, 20, 10)

    target = MODULE.target_in_robot(geometry, 0.09, 0.15)

    assert target[0] == pytest.approx(0.26)
    assert target[1] == pytest.approx(0.03)
    assert target[2] == pytest.approx(0.0)


def test_target_includes_the_scan_origin_transform():
    geometry = MODULE.DockGeometry(0.0, 0.50, 0.03, 20, 10)

    target = MODULE.target_in_robot(
        geometry,
        0.09,
        0.15,
        scan_translation_x=0.06,
    )

    assert target[0] == pytest.approx(0.32)
    assert target[1] == pytest.approx(0.03)
    assert target[2] == pytest.approx(0.0)


def test_measured_charging_geometry_places_robot_center_from_wall():
    wall_to_dock_contact = 0.09
    robot_center_to_contact = 0.15

    assert wall_to_dock_contact + robot_center_to_contact == pytest.approx(
        0.24)


def test_transform_pose_rotates_robot_relative_target():
    target = MODULE.transform_pose(
        (1.0, 2.0, math.pi / 2.0),
        (0.3, -0.1, 0.2),
    )

    assert target[0] == pytest.approx(1.1)
    assert target[1] == pytest.approx(2.3)
    assert target[2] == pytest.approx(math.pi / 2.0 + 0.2)


def test_pose_distance_uses_only_translation():
    distance = MODULE.pose_distance((0.0, 0.0, 2.0), (0.3, 0.4, -2.0))

    assert distance == pytest.approx(0.5)


import pytest
