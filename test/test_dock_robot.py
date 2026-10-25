import importlib.util
import json
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'dock_robot.py'
SPEC = importlib.util.spec_from_file_location('dock_robot', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_dock_pose_from_stage_moves_along_stage_heading():
    pose = MODULE.dock_pose_from_stage((1.0, 2.0, math.pi / 2.0), 0.215)

    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(2.215)
    assert pose[2] == pytest.approx(math.pi / 2.0)


def test_read_dock_pose_supports_existing_stage_file(tmp_path):
    path = tmp_path / 'dock.json'
    path.write_text(json.dumps({
        'map': 'downstairs',
        'frame': 'map',
        'stage': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
    }), encoding='utf-8')

    pose = MODULE.read_dock_pose(path, 'downstairs', 0.215)

    assert pose == pytest.approx((1.215, 2.0, 0.0))


def test_read_dock_pose_prefers_calibrated_dock_pose(tmp_path):
    path = tmp_path / 'dock.json'
    path.write_text(json.dumps({
        'map': 'downstairs',
        'frame': 'map',
        'stage': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
        'dock': {'x': 1.21, 'y': 2.01, 'yaw': 0.03},
    }), encoding='utf-8')

    pose = MODULE.read_dock_pose(path, 'downstairs', 0.215)

    assert pose == pytest.approx((1.21, 2.01, 0.03))


def test_feedback_states_cover_the_opennav_action():
    assert set(MODULE.FEEDBACK_STATES.values()) == {
        'NAV_TO_STAGING_POSE',
        'INITIAL_PERCEPTION',
        'CONTROLLING',
        'WAIT_FOR_CHARGE',
        'RETRY',
    }


def test_skip_staging_does_not_publish_the_empty_navigation_phase():
    assert not MODULE.publishes_feedback_state(
        'NAV_TO_STAGING_POSE', navigate_to_staging=False)
    assert MODULE.publishes_feedback_state(
        'INITIAL_PERCEPTION', navigate_to_staging=False)
    assert MODULE.publishes_feedback_state(
        'NAV_TO_STAGING_POSE', navigate_to_staging=True)


def test_parse_bool_accepts_launch_values():
    assert MODULE.parse_bool('true')
    assert not MODULE.parse_bool('false')


def test_parse_bool_rejects_unknown_values():
    with pytest.raises(ValueError):
        MODULE.parse_bool('yes')
