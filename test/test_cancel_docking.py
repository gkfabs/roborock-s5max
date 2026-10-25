import importlib.util
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'cancel_docking.py'
SPEC = importlib.util.spec_from_file_location('cancel_docking', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def status(goal_uuid, state):
    return SimpleNamespace(
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=list(goal_uuid)),
        ),
        status=state,
    )


def test_target_goal_is_not_stopped_while_canceling():
    target = bytes(range(16))

    assert not MODULE.target_goals_are_stopped(
        [status(target, GoalStatus.STATUS_CANCELING)],
        {target},
    )


def test_target_goal_is_stopped_after_cancellation():
    target = bytes(range(16))

    assert MODULE.target_goals_are_stopped(
        [status(target, GoalStatus.STATUS_CANCELED)],
        {target},
    )


def test_other_active_goal_does_not_block_target_confirmation():
    target = bytes(range(16))
    other = bytes(reversed(range(16)))

    assert MODULE.target_goals_are_stopped(
        [status(other, GoalStatus.STATUS_EXECUTING)],
        {target},
    )
