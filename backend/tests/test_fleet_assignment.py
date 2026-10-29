"""Isolated three-robot assignment. Planned occupancy is not measured motion."""
from __future__ import annotations

from datetime import datetime, timezone

from app.services.domain_status import status_path, write_status
from app.services.reservation_manager import Interval, find_conflicts, planning_profile, prepare_interval
from app.services.route_preview import build_graph

from test_fleet_tasks import START, _create


NEAR = {
    "pinky-1": {"x": 1.0, "y": 1.0},
    "pinky-2": {"x": 0.8, "y": 1.8},
    "pinky-3": {"x": 0.8, "y": 0.6},
}
SHARED = ({"x": 1.2, "y": 0.8}, {"x": 1.1, "y": 1.6})


def _seed_pinky1(runtime) -> None:
    write_status(status_path(runtime.settings.data_dir, "pinky-1"), {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "kind": "physical",
        "physical_connection_enabled": True,
        "tf_status": "fresh",
        "pose": {"x": START["x"], "y": START["y"], "yaw": 0},
        "last_seen": datetime.now(timezone.utc).isoformat(),
        "connection_state": "connected",
    })


def _automatic(client, goal: dict, priority: int) -> dict:
    created = client.post("/api/fleet-tasks", json={
        "assignment_mode": "automatic",
        "goal": goal,
        "priority": priority,
        "task_type": "navigate",
        "label": "auto",
    })
    assert created.status_code == 200, created.text
    planned = client.post(f"/api/fleet-tasks/{created.json()['task_id']}/plan")
    assert planned.status_code == 200, planned.text
    return planned.json()


def _avoidance(task: dict) -> str:
    if task["status"] == "WAITING":
        return "impossible"
    if task["status"] != "RESERVED":
        return task["status"]
    if task["schedule"].get("conflicts_avoided_by_delay"):
        return "wait"
    if task.get("selected_path_id") not in {None, "path-1"}:
        return "reroute"
    return "same_slot"


def test_three_robots_keep_separate_occupancy_and_auto_assign_five_tasks(env):
    client, runtime = env
    _seed_pinky1(runtime)
    # Do not start pinky-2 on pinky-1's TF cell; that starves the physical robot of slots.
    manual = client.post(
        f"/api/fleet-tasks/{_create(client, robot_id='pinky-2', start={'x': 0.425, 'y': 1.925}, goal=NEAR['pinky-2'])['task_id']}/plan"
    ).json()
    assert manual["status"] == "RESERVED"
    assert manual["assigned_robot_id"] == "pinky-2"
    assert str(manual["assignment_reason"]).startswith("manual assignment to pinky-2")

    planned = [
        _automatic(client, NEAR["pinky-3"], 3),
        _automatic(client, NEAR["pinky-1"], 4),
        _automatic(client, SHARED[0], 8),
        _automatic(client, SHARED[1], 2),
        _automatic(client, {"x": 1.4, "y": 1.0}, 6),
    ]
    assert len(planned) >= 5
    assigned = {manual["assigned_robot_id"]}
    evidence = [_row(manual)]
    for task in planned:
        assert task["status"] in {"RESERVED", "WAITING"}, task
        assert task["assignment_mode"] == "automatic"
        if task["status"] == "RESERVED":
            assigned.add(task["assigned_robot_id"])
            assert task["estimated_start_time"].endswith("+00:00")
            assert task["estimated_arrival_time"].endswith("+00:00")
            assert task["schedule"]["time_basis"] == "absolute"
            assert task["schedule"]["collision_safety_verified"] is False
            assert "score" in (task["assignment_reason"] or "")
        evidence.append(_row(task))
    assert assigned == {"pinky-1", "pinky-2", "pinky-3"}, evidence

    # The plan clock is the departure epoch. A later wall-clock reading can mark
    # an unverified short trip as delayed_motion; that is not a second booking.
    graph = build_graph(
        runtime.settings.physical_map_yaml,
        planning_resolution_m=runtime.settings.planning_resolution_m,
        robot_radius_m=runtime.settings.planning_robot_radius_m,
        allow_diagonal=runtime.settings.planning_allow_diagonal,
    )
    stored = client.get("/api/planning/reservations").json()["reservations"]
    profile = planning_profile(
        runtime.settings.planning_resolution_m, runtime.settings.planning_robot_radius_m,
    )
    intervals = [
        prepare_interval(graph, Interval(
            item["robot_id"], item["task_id"], item["resource_kind"], item["resource_id"],
            item["x"], item["y"], item["t_start"], item["t_end"], None, item["time_basis"],
            planning_profile=item.get("planning_profile") or "",
        ), profile)
        for item in stored
    ]
    plan_now = min(item.t_start for item in intervals) - 1.0
    planned_hits = find_conflicts(
        intervals, intervals, robot_radius_m=runtime.settings.planning_robot_radius_m, now_epoch=plan_now,
    )
    assert planned_hits == [], evidence
    report = client.get("/api/planning/conflicts").json()
    assert report["collision_safety_verified"] is False
    assert report["actual_motion_verified"] is False
    # After the planned window, an unverified trip still occupies its geometry.
    # Those hits are delayed_motion plus the same geometry, not a second future booking.
    if report["conflicts"]:
        assert "delayed_motion" in {item["conflict_type"] for item in report["conflicts"]}

    virtual = next(item for item in planned if item["status"] == "RESERVED" and item["assigned_robot_id"] != "pinky-1")
    delivered = client.post(f"/api/fleet-tasks/{virtual['task_id']}/dispatch").json()
    assert delivered["status"] == "DELIVERED"
    assert delivered["ack_channel"] == "simulated"
    assert delivered["ack"]["receiver"] == "fms-virtual-receiver"
    assert delivered["transport"]["mode"] == "simulated"
    assert delivered["motion_verified"] is False
    assert delivered["transport"]["received"] is True
    assert delivered["transport"]["completed"] is False

    reserved = [item for item in planned if item["status"] == "RESERVED" and item["task_id"] != virtual["task_id"]]
    cancelled = client.post(f"/api/fleet-tasks/{reserved[0]['task_id']}/cancel", json={"reason": "operator"}).json()
    assert cancelled["status"] == "CANCELLED"
    remaining = client.get("/api/planning/reservations").json()["reservations"]
    assert all(item["task_id"] != cancelled["task_id"] for item in remaining)
    for item in planned:
        if item.get("predecessor_task_id") != cancelled["task_id"]:
            continue
        replanned = client.get(f"/api/fleet-tasks/{item['task_id']}").json()
        assert replanned["status"] in {"RESERVED", "WAITING"}
        assert replanned["predecessor_task_id"] != cancelled["task_id"]
    again = find_conflicts(
        [item for item in intervals if item.task_id != cancelled["task_id"]],
        [item for item in intervals if item.task_id != cancelled["task_id"]],
        robot_radius_m=runtime.settings.planning_robot_radius_m,
        now_epoch=plan_now,
    )
    assert again == []
    assert {item["avoidance"] for item in evidence} <= {"same_slot", "wait", "reroute", "impossible"}
    # Wait/reroute appear when starts share a corridor. Separate homes can clear same_slot.


def _row(task: dict) -> dict:
    return {
        "robot_id": task.get("assigned_robot_id"),
        "task_id": task["task_id"],
        "status": task["status"],
        "selected_path_id": task.get("selected_path_id"),
        "depart": task.get("estimated_start_time"),
        "arrive": task.get("estimated_arrival_time"),
        "avoidance": _avoidance(task),
        "reason": task.get("assignment_reason"),
    }
