import importlib.util
from pathlib import Path
import sys


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'scripts' / 'contact_reflex.py'
)
SPEC = importlib.util.spec_from_file_location('contact_reflex', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def recovery():
    return MODULE.ContactRecovery(
        stop_seconds=0.1,
        backup_seconds=0.6,
        backup_hard_limit_seconds=1.5,
        turn_seconds=0.6,
        settle_seconds=0.1,
        backup_speed=-0.05,
        turn_speed=0.45,
    )


def test_left_contact_stops_backs_up_and_turns_right():
    controller = recovery()
    controller.update_contacts(True, False, 10.0)

    assert controller.command(10.0).state == MODULE.STOP
    backup = controller.command(10.2)
    assert backup.state == MODULE.BACKUP
    assert backup.linear_x == -0.05
    controller.update_contacts(False, False, 10.7)
    turn = controller.command(10.8)
    assert turn.state == MODULE.TURN
    assert turn.angular_z == -0.45


def test_right_contact_turns_left():
    controller = recovery()
    controller.update_contacts(False, True, 20.0)
    controller.update_contacts(False, False, 20.7)

    turn = controller.command(20.8)
    assert turn.state == MODULE.TURN
    assert turn.angular_z == 0.45


def test_recovery_releases_after_contact_clears():
    controller = recovery()
    controller.update_contacts(True, False, 30.0)
    controller.update_contacts(False, False, 30.2)

    command = controller.command(31.5)
    assert command.state == MODULE.IDLE
    assert not command.active


def test_pressed_bumper_holds_zero_after_recovery():
    controller = recovery()
    controller.update_contacts(True, False, 40.0)

    command = controller.command(41.7)
    assert command.state == MODULE.LOCKOUT
    assert command.active
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0

    controller.update_contacts(False, False, 41.8)
    assert controller.command(41.8).state == MODULE.IDLE


def test_pressed_bumper_keeps_backing_before_the_hard_limit():
    controller = recovery()
    controller.update_contacts(True, True, 40.0)

    command = controller.command(40.9)
    assert command.state == MODULE.BACKUP
    assert command.linear_x == -0.05


def test_head_on_contacts_alternate_turn_direction():
    controller = recovery()
    controller.update_contacts(True, True, 50.0)
    controller.update_contacts(False, False, 50.7)
    first = controller.command(50.8).angular_z
    controller.update_contacts(False, False, 51.5)
    controller.command(51.5)
    controller.update_contacts(True, True, 52.0)
    controller.update_contacts(False, False, 52.7)
    second = controller.command(52.8).angular_z

    assert first == -second


def test_idle_contact_does_not_start_recovery():
    controller = recovery()
    controller.update_contacts(True, False, 60.0, allow_start=False)

    command = controller.command(60.1)
    assert command.state == MODULE.IDLE
    assert not command.active


def test_cancel_stops_active_recovery():
    controller = recovery()
    controller.update_contacts(True, True, 70.0)
    assert controller.command(70.2).active

    controller.cancel()

    command = controller.command(70.3)
    assert command.state == MODULE.IDLE
    assert not command.active
