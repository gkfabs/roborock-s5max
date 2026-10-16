import importlib.util
from pathlib import Path
import sys


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'safety_adapter.py'
)
SPEC = importlib.util.spec_from_file_location('safety_adapter', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def recovery():
    return MODULE.CliffRecovery(
        stop_seconds=0.1,
        backup_seconds=0.6,
        backup_speed=-0.05,
    )


def test_moving_robot_stops_then_reverses_once():
    controller = recovery()
    controller.update(True, 10.0, allow_start=True)

    assert controller.command(10.0).state == MODULE.STOP
    backup = controller.command(10.2)
    assert backup.state == MODULE.BACKUP
    assert backup.linear_x == -0.05
    assert controller.command(10.8).state == MODULE.LOCKOUT


def test_idle_cliff_does_not_move_the_robot():
    controller = recovery()
    controller.update(True, 20.0, allow_start=False)

    assert controller.command(20.2).state == MODULE.IDLE


def test_active_cliff_cannot_start_a_second_recovery():
    controller = recovery()
    controller.update(True, 30.0, allow_start=True)
    controller.update(True, 31.0, allow_start=True)

    assert controller.command(31.0).state == MODULE.LOCKOUT


def test_cleared_cliff_stops_and_arms_the_next_event():
    controller = recovery()
    controller.update(True, 40.0, allow_start=True)
    controller.update(False, 40.2, allow_start=False)
    assert controller.command(40.2).state == MODULE.IDLE

    controller.update(True, 41.0, allow_start=True)
    assert controller.command(41.2).state == MODULE.BACKUP
