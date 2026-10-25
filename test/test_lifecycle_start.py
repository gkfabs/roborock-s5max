import importlib.util
from pathlib import Path

from lifecycle_msgs.msg import State


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'lifecycle_start.py'
SPEC = importlib.util.spec_from_file_location('lifecycle_start', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeLifecycleBackend:
    def __init__(self):
        self.states = {
            'docking_server': State.PRIMARY_STATE_UNCONFIGURED,
        }
        self.start_requests = 0
        self.errors = []

    def read_states(self):
        return dict(self.states)

    def request_start(self):
        self.start_requests += 1
        self.states['docking_server'] = State.PRIMARY_STATE_ACTIVE
        return True

    def wait_for_active(self):
        return all(
            state == State.PRIMARY_STATE_ACTIVE
            for state in self.states.values()
        )

    def info(self, _message):
        pass

    def error(self, message):
        self.errors.append(message)


def test_starting_the_lifecycle_twice_sends_one_transition():
    backend = FakeLifecycleBackend()
    startup = MODULE.LifecycleStartup(backend)

    assert startup.start()
    assert startup.start()
    assert backend.start_requests == 1
    assert not backend.errors


def test_mixed_active_states_do_not_start_the_group_again():
    backend = FakeLifecycleBackend()
    backend.states['controller_server'] = State.PRIMARY_STATE_ACTIVE
    startup = MODULE.LifecycleStartup(backend)

    assert not startup.start()
    assert backend.start_requests == 0
    assert backend.errors
