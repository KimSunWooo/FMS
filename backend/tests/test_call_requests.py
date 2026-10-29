"""User-call pickup/dropoff orchestration. Isolated DB only. No domain-24 publish."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from app.main import create_app
from app.services.domain_status import status_path, write_status
from app.services.reservation_manager import Interval, find_conflicts, planning_profile, prepare_interval
from app.services.route_preview import build_graph
from conftest import FIXED, make_settings

PICKUP = {"x": 1.0, "y": 1.0}
DROPOFF = {"x": 1.2, "y": 0.8}
SAFE_P2_HOME = {"x": 0.8, "y": 1.8, "yaw": 0.0}


@pytest.fixture
def call_env(tmp_path: Path):
    settings = make_settings(
        tmp_path,
        planning_resolution_m=0.14,
        planning_robot_radius_m=0.096,
    )
    fleet = json.loads(Path(settings.physical_fleet_path).read_text(encoding="utf-8"))
    for robot in fleet["robots"]:
        if robot["robot_id"] == "pinky-2":
            robot["home"] = dict(SAFE_P2_HOME)
    copy = tmp_path / "physical_fleet_call.json"
    copy.write_text(json.dumps(fleet), encoding="utf-8")
    settings.physical_fleet_path = copy
    app = create_app(settings)
    app.state.runtime.clock.freeze(FIXED)
    with TestClient(app) as client:
        yield client, app.state.runtime


def _seed_pinky1(runtime, x: float = 0.9, y: float = 1.2) -> None:
    now = datetime.now(timezone.utc).isoformat()
    write_status(status_path(runtime.settings.data_dir, "pinky-1"), {
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "kind": "physical",
        "physical_connection_enabled": True,
        "tf_status": "fresh",
        "pose": {"x": x, "y": y, "yaw": 0},
        "worker_tf_pose": {"x": x, "y": y, "yaw": 0},
        "worker_tf_status": "fresh",
        "last_seen": now,
        "connection_state": "connected",
        "receiver_ready": True,
    })
    # Primary physical pose path: Gateway IPC RobotStatus (not registry home).
    ipc = Path(runtime.settings.data_dir) / "ros_ipc" / "pinky-1"
    ipc.mkdir(parents=True, exist_ok=True)
    (ipc / "robot_status.json").write_text(json.dumps({
        "robot_id": "pinky-1",
        "ros_domain_id": 24,
        "receiver_ready": True,
        "task_execution_enabled": False,
        "pose_valid": True,
        "pose_source": "tf:map->base_footprint",
        "observed_at": now,
        "pose_observed_at": now,
        "pose": {"x": x, "y": y, "z": 0.0, "yaw": 0.0, "frame_id": "map"},
    }), encoding="utf-8")


def test_scenario_a_creates_two_legs_and_holds_dropoff(call_env):
    client, runtime = call_env
    _seed_pinky1(runtime)
    created = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "priority": 5,
    })
    assert created.status_code == 200, created.text
    body = created.json()
    assert body["scenario_status"] == "PICKUP_READY"
    assert body["assigned_robot_id"] in {"pinky-1", "pinky-2", "pinky-3"}
    assert body["pickup_task"]["status"] == "RESERVED"
    assert body["dropoff_task"]["status"] == "CREATED"
    assert body["dropoff_task"]["predecessor_task_id"] == body["pickup_task_id"]
    assert body["dropoff_task"]["schedule"]["hold_until_boarding"] is True
    assert body["dropoff_task"]["schedule"]["boarding_required"] is True
    assert body["dropoff_task"]["schedule"]["boarding_confirmed"] is False
    assert body["pickup"] == PICKUP
    assert body["dropoff"] == DROPOFF
    # Dropoff must not plan or dispatch yet.
    blocked_plan = client.post(f"/api/fleet-tasks/{body['dropoff_task_id']}/plan")
    assert blocked_plan.status_code == 409
    blocked_dispatch = client.post(f"/api/fleet-tasks/{body['dropoff_task_id']}/dispatch")
    assert blocked_dispatch.status_code == 409


def test_scenario_b_boarding_unlocks_dropoff_after_simulated_arrival(call_env):
    client, runtime = call_env
    created = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-2",
        "priority": 4,
    }).json()
    assert created["scenario_status"] == "PICKUP_READY"
    # Boarding before arrival must fail.
    early = client.post(f"/api/call-requests/{created['call_id']}/boarding", json={
        "event_id": "board-1", "source": "mock",
    })
    assert early.status_code == 409
    delivered = client.post(f"/api/fleet-tasks/{created['pickup_task_id']}/dispatch")
    assert delivered.status_code == 200
    assert delivered.json()["status"] == "DELIVERED"
    # ACK alone is not arrival.
    still = client.post(f"/api/call-requests/{created['call_id']}/boarding", json={
        "event_id": "board-1", "source": "mock",
    })
    assert still.status_code == 409
    arrived = client.post(f"/api/call-requests/{created['call_id']}/pickup-arrived-simulated")
    assert arrived.status_code == 200, arrived.text
    assert arrived.json()["scenario_status"] == "WAIT_BOARD"
    assert arrived.json()["pickup_arrival_evidence"] == "simulated"
    boarded = client.post(f"/api/call-requests/{created['call_id']}/boarding", json={
        "event_id": "board-1", "source": "mock",
    })
    assert boarded.status_code == 200, boarded.text
    body = boarded.json()
    assert body["boarding"]["confirmed"] is True
    assert body["boarding"]["mock_only"] is True
    assert body["scenario_status"] == "DROPOFF_READY"
    assert body["dropoff_task"]["status"] == "RESERVED"
    assert body["dropoff_task"]["schedule"]["boarding_confirmed"] is True
    # Same event is idempotent; different event is rejected.
    again = client.post(f"/api/call-requests/{created['call_id']}/boarding", json={
        "event_id": "board-1", "source": "mock",
    })
    assert again.status_code == 200
    dup = client.post(f"/api/call-requests/{created['call_id']}/boarding", json={
        "event_id": "board-2", "source": "mock",
    })
    assert dup.status_code == 409
    # Virtual dropoff may use existing dispatch after boarding; still execute=false.
    dispatched = client.post(f"/api/fleet-tasks/{body['dropoff_task_id']}/dispatch")
    assert dispatched.status_code == 200
    assert dispatched.json()["ack_channel"] == "simulated"
    assert dispatched.json()["transport"]["completed"] is False


def test_scenario_c_without_boarding_keeps_dropoff_held_and_occupancy(call_env):
    client, _runtime = call_env
    created = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-3",
        "priority": 3,
    }).json()
    client.post(f"/api/fleet-tasks/{created['pickup_task_id']}/dispatch")
    arrived = client.post(f"/api/call-requests/{created['call_id']}/pickup-arrived-simulated").json()
    assert arrived["scenario_status"] == "WAIT_BOARD"
    dropoff = client.get(f"/api/fleet-tasks/{created['dropoff_task_id']}").json()
    assert dropoff["status"] == "CREATED"
    assert dropoff["schedule"]["hold_until_boarding"] is True
    reservations = client.get("/api/planning/reservations").json()["reservations"]
    assert any(item["task_id"] == created["pickup_task_id"] for item in reservations)
    blocked = client.post(f"/api/fleet-tasks/{created['dropoff_task_id']}/dispatch")
    assert blocked.status_code == 409


def test_scenario_d_three_parallel_calls_six_legs(call_env):
    client, runtime = call_env
    _seed_pinky1(runtime)
    requests = [
        {"pickup": {"x": 1.0, "y": 1.0}, "dropoff": {"x": 1.2, "y": 0.8}, "priority": 8},
        {"pickup": {"x": 0.8, "y": 0.6}, "dropoff": {"x": 1.1, "y": 1.6}, "priority": 5},
        {"pickup": {"x": 0.7, "y": 1.5}, "dropoff": {"x": 1.5, "y": 1.5}, "priority": 3},
    ]
    calls = []
    for body in requests:
        response = client.post("/api/call-requests", json=body)
        assert response.status_code == 200, response.text
        calls.append(response.json())
    planned = [item for item in calls if item["scenario_status"] == "PICKUP_READY"]
    assert len(planned) >= 2
    robots = {item["assigned_robot_id"] for item in planned}
    assert robots <= {"pinky-1", "pinky-2", "pinky-3"}
    assert len({item["pickup_task_id"] for item in planned}) == len(planned)
    assert len({item["dropoff_task_id"] for item in planned}) == len(planned)
    for item in planned:
        assert item["dropoff_task"]["schedule"]["hold_until_boarding"] is True
    graph = build_graph(
        runtime.settings.physical_map_yaml,
        planning_resolution_m=runtime.settings.planning_resolution_m,
        robot_radius_m=runtime.settings.planning_robot_radius_m,
        allow_diagonal=True,
    )
    profile = planning_profile(
        runtime.settings.planning_resolution_m, runtime.settings.planning_robot_radius_m,
    )
    stored = client.get("/api/planning/reservations").json()["reservations"]
    intervals = [
        prepare_interval(graph, Interval(
            item["robot_id"], item["task_id"], item["resource_kind"], item["resource_id"],
            item["x"], item["y"], item["t_start"], item["t_end"], None, item["time_basis"],
            planning_profile=item.get("planning_profile") or "",
        ), profile)
        for item in stored
    ]
    if intervals:
        plan_now = min(item.t_start for item in intervals) - 1.0
        hits = find_conflicts(
            intervals, intervals,
            robot_radius_m=runtime.settings.planning_robot_radius_m,
            now_epoch=plan_now,
        )
        assert hits == []


def test_scenario_e_error_gates(call_env):
    client, runtime = call_env
    # Physical without TF cannot be chosen for a forced manual call.
    blocked = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-1",
        "priority": 5,
    })
    assert blocked.status_code in {200, 422, 409}
    if blocked.status_code == 200:
        body = blocked.json()
        assert body["scenario_status"] in {"CALL_FAILED", "CALL_BLOCKED", "PICKUP_READY"}
        if body["scenario_status"] == "PICKUP_READY":
            # Only acceptable if TF was seeded; this fixture did not seed it.
            assert False, "pinky-1 must not plan without TF"
    # Wrong call boarding
    created = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-2",
    }).json()
    client.post(f"/api/fleet-tasks/{created['pickup_task_id']}/dispatch")
    client.post(f"/api/call-requests/{created['call_id']}/pickup-arrived-simulated")
    missing = client.post("/api/call-requests/does-not-exist/boarding", json={
        "event_id": "x", "source": "mock",
    })
    assert missing.status_code == 404
    # ACK-only still blocked for dropoff.
    ack_only = client.post(f"/api/fleet-tasks/{created['dropoff_task_id']}/dispatch")
    assert ack_only.status_code == 409


def test_lattice_defaults_still_fourteen_cm_for_calls(call_env):
    client, runtime = call_env
    assert runtime.settings.planning_resolution_m == 0.14
    starts = client.get("/api/planning/starts").json()["map"]
    assert starts["planning_resolution_m"] == 0.14


def test_exit_completes_mission_after_dropoff_arrival(call_env):
    client, _runtime = call_env
    created = client.post("/api/transport-missions", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-2",
        "priority": 4,
    }).json()
    assert created["mission_id"] == created["call_id"]
    assert created["mission_status"] == "PICKUP_READY"
    client.post(f"/api/fleet-tasks/{created['pickup_task_id']}/dispatch")
    client.post(f"/api/transport-missions/{created['mission_id']}/pickup-arrived-simulated")
    # EXIT before dropoff arrival rejected.
    early_exit = client.post(f"/api/transport-missions/{created['mission_id']}/exit", json={
        "event_id": "exit-1", "source": "mock",
    })
    assert early_exit.status_code == 409
    client.post(f"/api/transport-missions/{created['mission_id']}/board", json={
        "event_id": "board-1", "source": "mock",
    })
    dropoff = client.get(f"/api/call-requests/{created['call_id']}").json()["dropoff_task"]
    client.post(f"/api/fleet-tasks/{dropoff['task_id']}/dispatch")
    waiting = client.post(
        f"/api/transport-missions/{created['mission_id']}/dropoff-arrived-simulated"
    )
    assert waiting.status_code == 200, waiting.text
    assert waiting.json()["scenario_status"] == "WAIT_EXIT"
    done = client.post(f"/api/transport-missions/{created['mission_id']}/exit", json={
        "event_id": "exit-1", "source": "mock",
    })
    assert done.status_code == 200, done.text
    body = done.json()
    assert body["scenario_status"] == "DONE"
    assert body["exit"]["confirmed"] is True
    assert body["exit"]["mock_only"] is True
    assert any(item["kind"] == "EXIT" for item in body["events"])
    # Wrong mission board rejected.
    other = client.post("/api/transport-missions/does-not-exist/board", json={
        "event_id": "board-x", "source": "mock",
    })
    assert other.status_code == 404


def test_cancel_mission_does_not_dispatch_dropoff(call_env):
    client, _runtime = call_env
    created = client.post("/api/call-requests", json={
        "pickup": PICKUP,
        "dropoff": DROPOFF,
        "robot_id": "pinky-3",
    }).json()
    cancelled = client.post(f"/api/call-requests/{created['call_id']}/cancel", json={
        "reason": "operator cancel",
    })
    assert cancelled.status_code == 200, cancelled.text
    assert cancelled.json()["scenario_status"] == "CANCELED"
    blocked = client.post(f"/api/fleet-tasks/{created['dropoff_task_id']}/dispatch")
    assert blocked.status_code == 409
