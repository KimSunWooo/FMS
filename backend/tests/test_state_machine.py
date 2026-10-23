from app.core.state_machine import TaskStatus, can_transition


def test_happy_path_and_rejections():
    assert can_transition(TaskStatus.CREATED, TaskStatus.PENDING)
    assert can_transition(TaskStatus.PENDING, TaskStatus.ASSIGNED)
    assert can_transition(TaskStatus.RUNNING, TaskStatus.COMPLETED)
    assert can_transition(TaskStatus.RUNNING, TaskStatus.CANCEL_REQUESTED)
    assert can_transition(TaskStatus.CANCEL_REQUESTED, TaskStatus.CANCELED)
    assert can_transition(TaskStatus.RECOVERY_REQUIRED, TaskStatus.FAILED)
    assert not can_transition(TaskStatus.RECOVERY_REQUIRED, TaskStatus.COMPLETED)
    assert not can_transition(TaskStatus.RECOVERY_REQUIRED, TaskStatus.RUNNING)
    assert not can_transition(TaskStatus.CANCEL_REQUESTED, TaskStatus.COMPLETED)
    assert not can_transition(TaskStatus.COMPLETED, TaskStatus.RUNNING)
    assert not can_transition(TaskStatus.PENDING, TaskStatus.COMPLETED)
    assert not can_transition(TaskStatus.RUNNING, TaskStatus.CANCELED)
