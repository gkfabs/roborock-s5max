import importlib.util
import math
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'dock_stage.py'
SPEC = importlib.util.spec_from_file_location('dock_stage', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_stable_pose_averages_position_and_wrapped_yaw():
    samples = [
        (1.0 + index * 0.001, 2.0, math.radians(179.0))
        for index in range(8)
    ]
    samples[-1] = (1.007, 2.0, math.radians(-179.0))

    pose = MODULE.stable_pose(samples)

    assert pose is not None
    assert math.isclose(pose[0], 1.0035)
    assert math.isclose(pose[1], 2.0)
    assert abs(abs(math.degrees(pose[2])) - 179.25) < 0.1


def test_unstable_pose_is_rejected():
    samples = [(0.0, 0.0, 0.0)] * 7 + [(0.1, 0.0, 0.0)]

    assert MODULE.stable_pose(samples) is None


def test_planar_distance_detects_near_staging_pose():
    distance = MODULE.planar_distance((1.60, -2.89), (1.45, -2.73))

    assert math.isclose(distance, math.hypot(0.15, -0.16))
    assert distance < 0.30


def test_approach_pose_is_before_stage_along_robot_heading():
    pose = MODULE.approach_pose((1.45, -2.73, 0.0), 0.20)

    assert pose == (1.25, -2.73, 0.0)


def test_approach_pose_uses_stage_yaw():
    pose = MODULE.approach_pose((1.0, 2.0, math.pi / 2.0), 0.20)

    assert math.isclose(pose[0], 1.0)
    assert math.isclose(pose[1], 1.8)
    assert math.isclose(pose[2], math.pi / 2.0)


def test_cancel_all_request_uses_zero_goal_identifier():
    request = MODULE.CancelGoal.Request()

    assert bytes(request.goal_info.goal_id.uuid) == bytes(16)
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_stage_file_round_trip(tmp_path):
    path = tmp_path / 'apartment.dock.json'

    MODULE.write_stage(path, 'apartment', (1.2, -0.4, 0.7))

    assert MODULE.read_stage(path, 'apartment') == (1.2, -0.4, 0.7)


def test_estimate_recorder_uses_exact_map_pose():
    message = MODULE.PoseWithCovarianceStamped()
    message.header.frame_id = 'map'
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.orientation.z = math.sin(0.3)
    message.pose.pose.orientation.w = math.cos(0.3)

    pose = MODULE.pose_from_estimate(message)

    assert math.isclose(pose[0], 1.25)
    assert math.isclose(pose[1], -2.5)
    assert math.isclose(pose[2], 0.6)
