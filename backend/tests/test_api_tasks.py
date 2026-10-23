def test_mock_task_reaches_completed_only_after_goal_evidence(env):
    client, runtime = env
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.0, "y": 6.5, "yaw": 0.2, "label": "open-bay"}, "priority": 1},
    )
    assert created.status_code == 202
    body = created.json()
    assert body["accepted"] is True
    assert body["motion_completed"] is False
    assert body["task"]["status"] == "PENDING"
    task_id = body["task"]["task_id"]

    before = client.get(f"/api/tasks/{task_id}").json()
    assert before["status"] == "PENDING"
    assert before["completion_evidence"] is None

    runtime.tick()
    done = client.get(f"/api/tasks/{task_id}").json()
    assert done["status"] == "COMPLETED"
    assert done["navigation_goal_count"] == 1
    assert done["completion_evidence"]["goal_status"] == "succeeded"
    assert done["completion_evidence"]["distance_m"] <= 0.25
    assert done["completion_evidence"]["goal_id"] == done["goal_id"]
    assert done["completion_evidence"]["execution_id"] == done["execution_id"]
    history = [event["to_status"] for event in done["events"]]
    assert history[:2] == ["CREATED", "PENDING"]
    assert "RUNNING" in history
    assert history[-1] == "COMPLETED"
    missing = client.post(f"/api/tasks/{task_id}/complete")
    assert missing.status_code == 404


def test_duplicate_idempotency_key_returns_the_same_task(env):
    client, _runtime = env
    first = client.post(
        "/api/tasks",
        headers={"Idempotency-Key": "shift-14"},
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}},
    )
    second = client.post(
        "/api/tasks",
        headers={"Idempotency-Key": "shift-14"},
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}},
    )
    assert first.status_code == 202
    assert second.status_code == 202
    assert first.json()["task"]["task_id"] == second.json()["task"]["task_id"]
    listed = client.get("/api/tasks").json()["tasks"]
    assert len([row for row in listed if row["destination"]["label"] == "open-bay"]) == 1

    clash = client.post(
        "/api/tasks",
        headers={"Idempotency-Key": "shift-14"},
        json={"destination": {"x": 4.0, "y": 1.5, "label": "other"}},
    )
    assert clash.status_code == 409
    assert clash.json()["error"]["code"] == "IDEMPOTENCY_MISMATCH"


def test_reads_do_not_move_task_state(env):
    client, runtime = env
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}},
    )
    task_id = created.json()["task"]["task_id"]
    events_before = runtime.count_task_events()
    snapshot = client.get(f"/api/tasks/{task_id}").json()
    for _ in range(4):
        assert client.get("/api/fleet/status").status_code == 200
        assert client.get("/api/tasks").status_code == 200
        assert client.get("/api/traffic/zones").status_code == 200
        assert client.get("/api/fleet/events").status_code == 200
        assert client.get(f"/api/tasks/{task_id}").status_code == 200
        assert client.get("/api/robots/pinky-1/status").status_code == 200
    after = client.get(f"/api/tasks/{task_id}").json()
    assert runtime.count_task_events() == events_before
    assert after["state_version"] == snapshot["state_version"]
    assert after["status"] == snapshot["status"]
    assert after["updated_at"] == snapshot["updated_at"]


def test_health_hides_secret_and_reports_isolation(env):
    client, runtime = env
    health = client.get("/api/system/health")
    assert health.status_code == 200
    text = health.text
    assert runtime.settings.lease_secret not in text
    body = health.json()
    assert body["rmf_runtime"] == "not_used"
    assert body["physical_control"] == "blocked"
    assert body["ros_bridge"] == "disabled"
    assert body["ros_domain_id"] == 73
    ready = client.get("/api/system/readiness")
    assert ready.status_code == 200
    assert ready.json()["ready"] is True


def test_cancel_before_dispatch_is_terminal_without_a_goal(env):
    client, _runtime = env
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}, "robot_id": "pinky-3"},
    )
    task_id = created.json()["task"]["task_id"]
    canceled = client.post(f"/api/tasks/{task_id}/cancel")
    assert canceled.status_code == 202
    task = canceled.json()["task"]
    assert task["status"] == "CANCELED"
    assert task["navigation_goal_count"] == 0
    assert task["goal_may_be_in_flight"] is False
