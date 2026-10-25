import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'cleaning_actuator_manager.py'
)
SPEC = importlib.util.spec_from_file_location(
    'cleaning_actuator_manager', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_active_state_does_not_report_completion():
    latch = MODULE.CompletionLatch()

    assert not latch.update(False)
    assert not latch.update(True)
    assert not latch.update(False)


def test_repeated_active_state_does_not_complete():
    latch = MODULE.CompletionLatch()

    assert not latch.update(True)
    assert not latch.update(True)


def test_success_result_completes_successfully():
    latch = MODULE.CompletionLatch()

    assert latch.set_result('succeeded')
    assert latch.success


def test_failure_result_completes_unsuccessfully():
    latch = MODULE.CompletionLatch()

    assert latch.set_result('failed: recovery limit reached')
    assert not latch.success
