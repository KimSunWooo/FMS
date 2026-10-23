def test_three_robots_finish_independent_tasks(env):
    client, runtime = env
    specs = [
        ("open-bay", 5.0, 6.5),
        ("west-bay", 4.0, 1.5),
        ("south-bay", 6.0, 2.0),
    ]
    ids = []
    for label, x, y in specs:
        response = client.post("/api/tasks", json={"destination": {"x": x, "y": y, "label": label}})
        ids.append(response.json()["task"]["task_id"])
    runtime.tick()
    bodies = [client.get(f"/api/tasks/{task_id}").json() for task_id in ids]
    assert {body["status"] for body in bodies} == {"COMPLETED"}
    assert len({body["robot_id"] for body in bodies}) == 3
    assert all(body["completion_evidence"]["goal_status"] == "succeeded" for body in bodies)
    assert len({body["goal_id"] for body in bodies}) == 3


def test_one_robot_failure_leaves_the_other_two_running(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 40
    ids = []
    for label, x, y in (("open-bay", 5.0, 6.5), ("west-bay", 4.0, 1.5), ("south-bay", 6.0, 2.0)):
        response = client.post("/api/tasks", json={"destination": {"x": x, "y": y, "label": label}})
        ids.append(response.json()["task"]["task_id"])
    runtime.tick()
    running = [client.get(f"/api/tasks/{task_id}").json() for task_id in ids]
    assert {body["status"] for body in running} == {"RUNNING"}
    victim = next(body for body in running if body["robot_id"] == "pinky-2")
    others = [body for body in running if body["robot_id"] != "pinky-2"]
    client.post("/api/robots/pinky-2/offline")
    runtime.clock.advance(41)
    runtime.tick()
    assert client.get(f"/api/tasks/{victim['task_id']}").json()["status"] == "RECOVERY_REQUIRED"
    for body in others:
        done = client.get(f"/api/tasks/{body['task_id']}").json()
        assert done["status"] == "COMPLETED"
        assert done["completion_evidence"]["goal_status"] == "succeeded"
        assert done["navigation_goal_count"] == 1
