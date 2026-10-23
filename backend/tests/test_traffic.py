def _zone(client, zone_id):
    payload = client.get("/api/traffic/zones").json()
    return next(zone for zone in payload["zones"] if zone["zone_id"] == zone_id)


def test_single_zone_allows_one_robot(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 0
    first = client.post(
        "/api/tasks",
        json={"destination": {"x": 10.0, "y": 3.7, "label": "elevator"}, "robot_id": "pinky-3"},
    )
    second = client.post(
        "/api/tasks",
        json={"destination": {"x": 10.1, "y": 3.8, "label": "elevator-2"}, "robot_id": "pinky-1"},
    )
    runtime.tick()
    first_body = client.get(f"/api/tasks/{first.json()['task']['task_id']}").json()
    second_body = client.get(f"/api/tasks/{second.json()['task']['task_id']}").json()
    assert first_body["status"] == "COMPLETED"
    assert second_body["status"] == "PENDING"
    assert second_body["navigation_goal_count"] == 0
    zone = _zone(client, "ZONE_C")
    assert zone["reservation"]["robot_id"] == "pinky-3"
    assert zone["reservation"]["reservation_status"] == "held"


def test_corridor_and_intersection_reject_a_second_entry(env):
    client, runtime = env
    runtime.settings.mock_goal_duration_seconds = 30
    corridor_a = client.post(
        "/api/tasks",
        json={"destination": {"x": 3.0, "y": 3.6, "label": "corridor-east"}, "robot_id": "pinky-1"},
    )
    corridor_b = client.post(
        "/api/tasks",
        json={"destination": {"x": 5.5, "y": 3.5, "label": "corridor-west"}, "robot_id": "pinky-3"},
    )
    runtime.tick()
    corridor_rows = [
        client.get(f"/api/tasks/{corridor_a.json()['task']['task_id']}").json(),
        client.get(f"/api/tasks/{corridor_b.json()['task']['task_id']}").json(),
    ]
    assert sorted(row["status"] for row in corridor_rows) == ["PENDING", "RUNNING"]
    zone = _zone(client, "ZONE_A")
    assert zone["reservation"]["reservation_status"] == "held"
    assert zone["capacity"] == 1
    assert zone["reservation"]["robot_id"] == next(row["robot_id"] for row in corridor_rows if row["status"] == "RUNNING")

    cross = client.post(
        "/api/tasks",
        json={"destination": {"x": 8.0, "y": 4.0, "label": "junction"}, "robot_id": "pinky-2"},
    )
    other = client.post(
        "/api/tasks",
        json={"destination": {"x": 8.2, "y": 4.2, "label": "junction-2"}, "robot_id": "pinky-3"},
    )
    runtime.tick()
    junction_rows = [
        client.get(f"/api/tasks/{cross.json()['task']['task_id']}").json(),
        client.get(f"/api/tasks/{other.json()['task']['task_id']}").json(),
    ]
    assert sum(row["status"] == "RUNNING" for row in junction_rows) == 1
    assert sum(row["status"] == "PENDING" for row in junction_rows) == 1
    intersection = _zone(client, "ZONE_B")
    assert intersection["reservation"]["reservation_status"] == "held"
    assert intersection["reservation"]["robot_id"] == next(
        row["robot_id"] for row in junction_rows if row["status"] == "RUNNING"
    )


def test_offline_owner_blocks_the_zone_until_an_operator_clears_it(env):
    client, runtime = env
    before = _zone(client, "ZONE_E")
    assert before["reservation"]["robot_id"] == "pinky-1"
    reservation_id = before["reservation"]["reservation_id"]
    runtime.force_reservation_expiry(reservation_id)
    runtime.tick()
    expired = _zone(client, "ZONE_E")
    assert expired["reservation"]["reservation_status"] == "held"

    client.post("/api/robots/pinky-1/offline")
    blocked = _zone(client, "ZONE_E")
    assert blocked["reservation"]["reservation_status"] == "blocked"
    created = client.post(
        "/api/tasks",
        json={"destination": {"x": 1.2, "y": 1.2, "label": "dock"}, "robot_id": "pinky-3"},
    )
    runtime.tick()
    waiting = client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()
    assert waiting["status"] == "PENDING"
    assert waiting["navigation_goal_count"] == 0

    denied = client.post(
        f"/api/traffic/reservations/{reservation_id}/release",
        json={"reason": "guessing", "position_observed_clear": False},
    )
    assert denied.status_code == 400
    assert _zone(client, "ZONE_E")["reservation"]["reservation_status"] == "blocked"
    task_status = waiting["status"]
    released = client.post(
        f"/api/traffic/reservations/{reservation_id}/release",
        json={"reason": "bay visually empty", "position_observed_clear": True},
    )
    assert released.status_code == 200
    assert client.get(f"/api/tasks/{created.json()['task']['task_id']}").json()["status"] == task_status


def test_no_wait_queue_can_deadlock(env):
    client, _runtime = env
    rows = client.get("/api/traffic/reservations").json()["reservations"]
    assert rows
    assert {row["reservation_status"] for row in rows} <= {"held", "blocked", "released"}
    assert not any(row["reservation_status"] == "waiting" for row in rows)
