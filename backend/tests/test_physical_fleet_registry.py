import math
from pathlib import Path

from app.core.config import ROOT
from app.services.fleet_registry import load_registry, present_robot
from app.services.map_frame import (
    DEFAULT_MAP,
    MapGeometry,
    describe_grid_point,
    describe_map_point,
    grid_to_world,
    load_geometry,
    world_to_grid,
)


def test_site_map_is_the_filtered_1f_map():
    geometry = load_geometry(DEFAULT_MAP)
    assert geometry.yaml_path.name == "1f_map_f.yaml"
    assert geometry.image_path.name == "1f_map_f.pgm"
    assert geometry.width_px == 186
    assert geometry.height_px == 236
    assert geometry.resolution == 0.01
    assert (geometry.origin_x, geometry.origin_y, geometry.origin_yaw) == (0.0, 0.0, 0.0)
    located = describe_map_point(geometry, 1.696, 0.103, 0.0)
    assert math.isclose(located["grid"]["col"], 169.6, abs_tol=1e-9)
    assert math.isclose(located["grid"]["row"], 10.3, abs_tol=1e-6)
    assert located["inside_map"] is True
    assert describe_map_point(geometry, 10.0, 10.0)["inside_map"] is False


def test_map_and_grid_round_trip_with_rotated_origin():
    geometry = MapGeometry(
        yaml_path=Path("synthetic.yaml"),
        image_path=Path("synthetic.pgm"),
        width_px=100,
        height_px=80,
        resolution=0.05,
        origin_x=1.0,
        origin_y=-2.0,
        origin_yaw=math.pi / 2,
    )
    col, row = world_to_grid(geometry, 3.5, 4.25)
    x, y = grid_to_world(geometry, col, row)
    assert math.isclose(x, 3.5, abs_tol=1e-9)
    assert math.isclose(y, 4.25, abs_tol=1e-9)
    restored = describe_grid_point(geometry, col, row)
    assert math.isclose(restored["x"], 3.5, abs_tol=1e-9)
    assert math.isclose(restored["y"], 4.25, abs_tol=1e-9)


def test_registry_separates_real_pinky_from_virtual_robots():
    robots = load_registry(ROOT / "config" / "physical_fleet.json")
    assert [robot["robot_id"] for robot in robots] == ["pinky-1", "pinky-2", "pinky-3"]
    assert [robot["kind"] for robot in robots] == ["physical", "virtual", "virtual"]
    geometry = load_geometry(DEFAULT_MAP)
    physical = present_robot(robots[0], ssh_robot=None, nav2=None, geometry=geometry)
    assert physical["kind"] == "physical"
    assert physical["ssh_online"] is False
    assert physical["pose"] is None
    for record in robots[1:]:
        shown = present_robot(
            record,
            ssh_robot={"host": "should-not-apply"},
            nav2={"ssh_connected": True, "pose": {"x": 1, "y": 1, "yaw": 0}, "pose_checked_at": "2026-10-28T00:00:00+00:00", "nav2_ready": True, "ros_online": True},
            geometry=geometry,
        )
        assert shown["kind"] == "virtual"
        assert shown["ssh_online"] is False
        assert shown["pose"] is None
        assert shown["pose_source"] is None
        assert shown["nav2_ready"] is None
        assert shown["home"]["source"] == "registry_home"
        assert shown["home"]["inside_map"] is True


def test_physical_tf_pose_keeps_its_source_and_gains_grid():
    record = load_registry()[0]
    geometry = load_geometry(DEFAULT_MAP)
    shown = present_robot(
        record,
        ssh_robot={"host": "192.168.0.83"},
        nav2={
            "robot_id": "pinky-1",
            "ssh_connected": True,
            "ros_online": True,
            "nav2_ready": True,
            "cmd_vel_chain_ready": True,
            "state": "ready",
            "detail": "ok",
            "pose": {"x": 1.696, "y": 0.103, "yaw": 0.0},
            "pose_checked_at": "2026-10-28T08:00:00+00:00",
        },
        geometry=geometry,
    )
    assert shown["ssh_online"] is True
    assert shown["nav2_ready"] is True
    assert shown["pose_source"] == "tf:map->base_footprint"
    assert math.isclose(shown["pose"]["grid"]["col"], 169.6, abs_tol=1e-9)
    assert math.isclose(shown["pose"]["grid"]["row"], 10.3, abs_tol=1e-9)


def test_map_and_robot_http(env, monkeypatch):
    client, runtime = env
    map_body = client.get("/api/physical/map")
    assert map_body.status_code == 200
    payload = map_body.json()
    assert payload["source"] == "1f_map_f.yaml"
    assert payload["width_px"] == 186
    assert payload["height_px"] == 236
    assert payload["grid"]["convention"] == "nav2_occupancy"
    assert payload["grid"]["row_origin"] == "image_bottom"

    image = client.get("/api/physical/map/image")
    assert image.status_code == 200
    assert image.content.startswith(b"\x89PNG")

    converted = client.post("/api/physical/map/coordinates", json={"map": {"x": 0.4, "y": 1.9, "yaw": 0}})
    assert converted.status_code == 200
    grid = converted.json()["grid"]
    assert math.isclose(grid["col"], 40.0, abs_tol=1e-9)
    assert math.isclose(grid["row"], 190.0, abs_tol=1e-9)
    back = client.post("/api/physical/map/coordinates", json={"grid": {"col": 40, "row": 190}})
    assert back.status_code == 200
    assert math.isclose(back.json()["x"], 0.4, abs_tol=1e-9)
    assert math.isclose(back.json()["y"], 1.9, abs_tol=1e-9)
    rejected = client.post("/api/physical/map/coordinates", json={"map": {"x": 0, "y": 0}, "grid": {"col": 0, "row": 0}})
    assert rejected.status_code == 422

    offline = client.get("/api/physical/robots")
    assert offline.status_code == 200
    body = offline.json()
    assert body["data_source"] == "fleet_registry"
    assert [robot["robot_id"] for robot in body["robots"]] == ["pinky-1", "pinky-2", "pinky-3"]
    assert body["physical_count"] == 1
    assert body["virtual_count"] == 2
    assert body["connected_count"] == 0
    assert body["robots"][0]["connection_status"] == "ssh_offline"
    assert all(robot["pose"] is None for robot in body["robots"])

    async def fake_ssh(robot_id, target, timeout, password=""):
        assert robot_id == "pinky-1"
        assert target == "pinky@192.168.0.83"
        return {"robot_id": robot_id, "host": "192.168.0.83"}

    monkeypatch.setattr("app.api.physical.check_ssh", fake_ssh)
    runtime.settings.physical_ssh_targets = "pinky-1=pinky@192.168.0.83"
    online = client.get("/api/physical/robots")
    assert online.status_code == 200
    robots = online.json()["robots"]
    assert robots[0]["ssh_online"] is True
    assert robots[0]["host"] == "192.168.0.83"
    assert robots[1]["ssh_online"] is False
    assert robots[2]["kind"] == "virtual"
