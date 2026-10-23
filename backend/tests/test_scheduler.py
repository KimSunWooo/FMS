def test_low_battery_robot_is_not_selected(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 30
    with runtime.db.write() as session:
        from app.models.tables import Robot

        robot = session.get(Robot, "pinky-3")
        robot.battery_percent = 1
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 10.5, "y": 7.5, "label": "north-window"}},
    )
    runtime.tick()
    task = client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()
    assert task["status"] == "RUNNING"
    assert task["robot_id"] == "pinky-2"
    events = client.get("/api/fleet/events").json()["events"]
    decision = next(item for item in events if item["code"] == "scheduler_decision")
    assert "euclidean_phase1_not_obstacle_aware" in decision["payload_json"]


def test_tie_breaks_on_robot_id(bare):
    client, runtime = bare
    runtime.settings.mock_goal_duration_seconds = 30
    for robot_id in ("pinky-b", "pinky-a"):
        response = client.post(
            "/api/robots",
            json={
                "robot_id": robot_id,
                "name": robot_id,
                "mode": "mock",
                "battery_percent": 80,
                "x": 0.2,
                "y": 7.6,
                "yaw": 0,
            },
        )
        assert response.status_code == 201
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}},
    )
    runtime.tick()
    task = client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()
    assert task["robot_id"] == "pinky-a"
    assert task["status"] == "RUNNING"


def test_controller_offline_robot_is_skipped(env):
    client, runtime = env
    with runtime.db.write() as session:
        from app.models.tables import Robot

        robot = session.get(Robot, "pinky-2")
        robot.controller_ready = False
        session.get(Robot, "pinky-1").controller_ready = False
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.0, "y": 6.5, "label": "open-bay"}, "robot_id": "pinky-2"},
    )
    runtime.tick()
    task = client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()
    assert task["status"] == "PENDING"
    assert task["navigation_goal_count"] == 0
