"""User-call orchestration on top of existing FleetTask rows.

One call creates two tasks: pickup A→B and dropoff B→C. Dropoff stays held
until boarding is confirmed. No new ROS topics. execute stays false.
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone

from sqlalchemy import select

from app.models.tables import FleetCallRequest, FleetReservationRecord, FleetTaskRecord
from app.services.call_scenario import (
    BOARDING_CONFIRMED,
    CALL_BLOCKED,
    CALL_CREATED,
    CALL_FAILED,
    DROPOFF_ARRIVED,
    DROPOFF_DISPATCHED,
    DROPOFF_READY,
    LEG_DROPOFF,
    LEG_PICKUP,
    PICKUP_ARRIVED,
    PICKUP_DISPATCHED,
    PICKUP_PLANNED,
    WAITING_FOR_BOARDING,
)
from app.services.fleet_registry import load_registry
from app.services.fleet_task_machine import CREATED, DELIVERED, RESERVED
from app.services.fleet_task_service import (
    FleetTaskError,
    FleetTaskService,
    map_revision,
    _active_intervals,
    _now as fleet_now,
)
from app.services.reservation_manager import schedule_candidates
from app.services.route_planner import k_shortest_paths, path_record
from app.services.route_preview import inspect_point


class CallRequestError(Exception):
    def __init__(self, status_code: int, detail: str, body: dict | None = None) -> None:
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail
        self.body = body or {}


class CallRequestService:
    def __init__(self, fleet: FleetTaskService) -> None:
        self.fleet = fleet
        self.settings = fleet.settings
        self.db = fleet.db

    def create(
        self,
        *,
        pickup: dict,
        dropoff: dict,
        priority: int = 5,
        robot_id: str | None = None,
        idempotency_key: str | None = None,
    ) -> dict:
        if idempotency_key:
            with self.db.read() as session:
                existing = session.scalar(
                    select(FleetCallRequest).where(FleetCallRequest.idempotency_key == idempotency_key)
                )
                if existing is not None:
                    return self._public(existing)

        graph = self.fleet._graph()
        pickup_x, pickup_y = float(pickup["x"]), float(pickup["y"])
        dropoff_x, dropoff_y = float(dropoff["x"]), float(dropoff["y"])
        pickup_check = inspect_point(graph, pickup_x, pickup_y)
        dropoff_check = inspect_point(graph, dropoff_x, dropoff_y)
        if not pickup_check["traversable"]:
            raise CallRequestError(422, f"pickup {pickup_check['reason']}", {"pickup": pickup_check})
        if not dropoff_check["traversable"]:
            raise CallRequestError(422, f"dropoff {dropoff_check['reason']}", {"dropoff": dropoff_check})

        registry = load_registry(self.settings.physical_fleet_path)
        if robot_id:
            robots = [item for item in registry if item["robot_id"] == robot_id]
            if not robots:
                raise CallRequestError(404, "robot is not registered")
            mode = "manual"
        else:
            robots = registry
            mode = "automatic"

        ranked = []
        rejected = []
        for robot in robots:
            score = self._score_robot(robot, pickup_x, pickup_y, dropoff_x, dropoff_y, graph)
            if score.get("error"):
                rejected.append(score)
                continue
            ranked.append(score)
        if not ranked:
            call = self._store_failed(
                pickup_x, pickup_y, dropoff_x, dropoff_y, priority, idempotency_key, rejected,
            )
            return call

        ranked.sort(key=lambda item: item["score"])
        winner = ranked[0]
        call_id = str(uuid.uuid4())
        now = _now()

        # Both legs must share one robot. Score already picked the winner; assign manually.
        pickup_task = self.fleet.create(
            mode="manual",
            goal_x=pickup_x,
            goal_y=pickup_y,
            robot_id=winner["robot_id"],
            start=winner.get("start_request"),
            priority=priority,
            task_type="navigate",
            label=f"call:{call_id}:pickup",
            idempotency_key=None if not idempotency_key else f"{idempotency_key}:pickup",
        )
        with self.db.write() as session:
            row = session.get(FleetTaskRecord, pickup_task["task_id"])
            seed = {
                "transport_leg": LEG_PICKUP,
                "call_request_id": call_id,
                "boarding_required": False,
                "assignment_mode_requested": mode,
            }
            row.schedule_json = json.dumps(seed)
            row.updated_at = now

        planned_pickup = self.fleet.plan(pickup_task["task_id"])
        with self.db.write() as session:
            row = session.get(FleetTaskRecord, pickup_task["task_id"])
            if row is not None:
                schedule = json.loads(row.schedule_json or "{}")
                schedule["transport_leg"] = LEG_PICKUP
                schedule["call_request_id"] = call_id
                schedule["boarding_required"] = False
                schedule["assignment_mode_requested"] = mode
                row.schedule_json = json.dumps(schedule)
        if planned_pickup["status"] != RESERVED:
            with self.db.write() as session:
                call = FleetCallRequest(
                    id=call_id,
                    scenario_status=CALL_FAILED,
                    assigned_robot_id=None,
                    pickup_x=pickup_x,
                    pickup_y=pickup_y,
                    dropoff_x=dropoff_x,
                    dropoff_y=dropoff_y,
                    priority=priority,
                    pickup_task_id=pickup_task["task_id"],
                    dropoff_task_id=None,
                    detail_json=json.dumps({
                        "rejected": rejected,
                        "pickup_status": planned_pickup["status"],
                        "pickup_failure": planned_pickup.get("failure_reason"),
                        "dropoff_draft": winner.get("dropoff_draft"),
                    }),
                    failure_reason=planned_pickup.get("failure_reason") or "pickup planning failed",
                    idempotency_key=idempotency_key,
                    created_at=now,
                    updated_at=now,
                )
                session.add(call)
            return self.get(call_id)

        dropoff_task = self._insert_held_dropoff(
            call_id=call_id,
            robot_id=planned_pickup["assigned_robot_id"],
            pickup_task_id=planned_pickup["task_id"],
            pickup_x=pickup_x,
            pickup_y=pickup_y,
            dropoff_x=dropoff_x,
            dropoff_y=dropoff_y,
            priority=priority,
            draft=winner["dropoff_draft"],
            idempotency_key=None if not idempotency_key else f"{idempotency_key}:dropoff",
        )

        with self.db.write() as session:
            call = FleetCallRequest(
                id=call_id,
                scenario_status=PICKUP_PLANNED,
                assigned_robot_id=planned_pickup["assigned_robot_id"],
                pickup_x=pickup_x,
                pickup_y=pickup_y,
                dropoff_x=dropoff_x,
                dropoff_y=dropoff_y,
                priority=priority,
                pickup_task_id=planned_pickup["task_id"],
                dropoff_task_id=dropoff_task["task_id"],
                detail_json=json.dumps({
                    "assignment": {
                        "robot_id": winner["robot_id"],
                        "score": winner["score"],
                        "pickup_travel_s": winner["pickup_travel_s"],
                        "dropoff_travel_s": winner["dropoff_travel_s"],
                        "depart_s": winner["depart_s"],
                        "queue_length": winner["queue_length"],
                        "reason": winner["reason"],
                    },
                    "alternatives": [
                        {"robot_id": item["robot_id"], "score": item["score"]}
                        for item in ranked[1:]
                    ] + rejected,
                    "dropoff_draft": winner["dropoff_draft"],
                    "boarding_input": "http_mock_or_operator",
                    "ros_topics_unchanged": True,
                }),
                failure_reason="",
                idempotency_key=idempotency_key,
                created_at=now,
                updated_at=now,
            )
            session.add(call)
        return self.get(call_id)

    def get(self, call_id: str) -> dict:
        snapshot = self._snapshot(call_id)
        if snapshot is None:
            raise CallRequestError(404, "Call request not found")
        return self._public_from_snapshot(snapshot)

    def list_calls(self) -> list[dict]:
        with self.db.read() as session:
            rows = session.scalars(
                select(FleetCallRequest).order_by(FleetCallRequest.created_at.desc())
            ).all()
            snapshots = [self._row_snapshot(row) for row in rows]
        return [self._public_from_snapshot(item) for item in snapshots]

    def confirm_boarding(self, call_id: str, *, event_id: str, source: str = "mock") -> dict:
        """Operator/mock boarding gate. Not a ROS topic and not physical proof."""
        if not event_id:
            raise CallRequestError(422, "boarding event_id is required")
        with self.db.read() as session:
            call = session.get(FleetCallRequest, call_id)
            if call is None:
                raise CallRequestError(404, "Call request not found")
            if call.boarding_confirmed and call.boarding_event_id == event_id:
                already = True
            else:
                already = False
            if call.boarding_confirmed and call.boarding_event_id != event_id:
                raise CallRequestError(409, "boarding already confirmed with a different event_id")
            if call.scenario_status in {CALL_FAILED, CALL_BLOCKED}:
                raise CallRequestError(409, f"Cannot board from {call.scenario_status}")
            dropoff_id = call.dropoff_task_id
            pickup_id = call.pickup_task_id
            assigned = call.assigned_robot_id
            arrival = call.pickup_arrival_evidence
            motion_ok = False
            if pickup_id:
                pickup = session.get(FleetTaskRecord, pickup_id)
                dropoff = session.get(FleetTaskRecord, dropoff_id) if dropoff_id else None
                if pickup is None or dropoff is None:
                    raise CallRequestError(409, "Call is missing pickup or dropoff task")
                if pickup.assigned_robot_id != assigned:
                    raise CallRequestError(409, "boarding robot does not match the call")
                motion_ok = bool(pickup.motion_verified)
        if already:
            return self.get(call_id)
        if not motion_ok:
            raise CallRequestError(
                409,
                "pickup arrival evidence is missing; ACK or ETA is not arrival",
            )
        if arrival not in {"simulated", "operator"}:
            raise CallRequestError(409, "pickup arrival evidence kind is not accepted")

        with self.db.write() as session:
            call = session.get(FleetCallRequest, call_id)
            dropoff = session.get(FleetTaskRecord, dropoff_id)
            assert call is not None and dropoff is not None
            call.boarding_confirmed = True
            call.boarding_event_id = event_id
            call.boarding_source = source
            call.boarding_confirmed_at = _now()
            call.scenario_status = BOARDING_CONFIRMED
            schedule = json.loads(dropoff.schedule_json or "{}")
            schedule["boarding_confirmed"] = True
            schedule["boarding_event_id"] = event_id
            schedule["boarding_source"] = source
            schedule["hold_until_boarding"] = False
            dropoff.schedule_json = json.dumps(schedule)
            dropoff.updated_at = _now()
            call.updated_at = _now()

        planned = self.fleet.plan(dropoff_id)  # type: ignore[arg-type]
        with self.db.write() as session:
            call = session.get(FleetCallRequest, call_id)
            dropoff = session.get(FleetTaskRecord, dropoff_id)
            assert call is not None and dropoff is not None
            schedule = json.loads(dropoff.schedule_json or "{}")
            schedule["transport_leg"] = LEG_DROPOFF
            schedule["call_request_id"] = call_id
            schedule["boarding_required"] = True
            schedule["boarding_confirmed"] = True
            schedule["boarding_event_id"] = call.boarding_event_id
            schedule["boarding_source"] = call.boarding_source
            schedule["hold_until_boarding"] = False
            schedule["predecessor_relation"] = "transport_leg"
            dropoff.schedule_json = json.dumps(schedule)
            if planned["status"] != RESERVED:
                call.scenario_status = CALL_FAILED
                call.failure_reason = planned.get("failure_reason") or "dropoff replan failed"
            else:
                call.scenario_status = DROPOFF_READY
                call.failure_reason = ""
            call.updated_at = _now()
        return self.get(call_id)

    def mark_pickup_arrived_simulated(self, call_id: str) -> dict:
        """Simulated arrival only. Keeps pickup occupancy while waiting for boarding."""
        with self.db.read() as session:
            call = session.get(FleetCallRequest, call_id)
            if call is None:
                raise CallRequestError(404, "Call request not found")
            pickup_id = call.pickup_task_id
            already = (
                bool(call.pickup_arrival_evidence)
                and call.scenario_status in {
                    PICKUP_ARRIVED, WAITING_FOR_BOARDING, BOARDING_CONFIRMED, DROPOFF_READY,
                    DROPOFF_DISPATCHED, DROPOFF_ARRIVED,
                }
            )
        if not pickup_id:
            raise CallRequestError(409, "Call has no pickup task")
        if already:
            return self.get(call_id)
        with self.db.write() as session:
            task = session.get(FleetTaskRecord, pickup_id)
            call = session.get(FleetCallRequest, call_id)
            if task is None or call is None:
                raise CallRequestError(404, "Pickup task not found")
            if task.ack_channel != "simulated" or task.status != DELIVERED:
                raise CallRequestError(
                    409,
                    "Simulated pickup arrival requires a simulated DELIVERED receipt",
                )
            task.motion_verified = True
            task.updated_at = _now()
            # Keep confirmed reservations so boarding wait still occupies space.
            hold_until = _now().timestamp() + float(
                getattr(self.settings, "call_boarding_hold_s", 300.0) or 300.0
            )
            rows = session.scalars(select(FleetReservationRecord).where(
                FleetReservationRecord.task_id == task.id,
                FleetReservationRecord.status == "confirmed",
            )).all()
            for row in rows:
                row.t_end = max(float(row.t_end), hold_until)
            call.pickup_arrival_evidence = "simulated"
            call.scenario_status = WAITING_FOR_BOARDING
            call.updated_at = _now()
            detail = json.loads(call.detail_json or "{}")
            detail["pickup_arrival"] = {
                "kind": "simulated",
                "not_physical_proof": True,
                "recorded_at": _now().isoformat(),
            }
            call.detail_json = json.dumps(detail)
        return self.get(call_id)

    def refresh_scenario(self, call_id: str) -> dict:
        """Derive scenario labels from linked FleetTask rows without inventing motion."""
        with self.db.write() as session:
            call = session.get(FleetCallRequest, call_id)
            if call is None:
                raise CallRequestError(404, "Call request not found")
            pickup = session.get(FleetTaskRecord, call.pickup_task_id) if call.pickup_task_id else None
            dropoff = session.get(FleetTaskRecord, call.dropoff_task_id) if call.dropoff_task_id else None
            if call.scenario_status in {CALL_FAILED, CALL_BLOCKED}:
                return self._public(call)
            if dropoff is not None and dropoff.motion_verified:
                call.scenario_status = DROPOFF_ARRIVED
            elif dropoff is not None and dropoff.status in {"DISPATCHING", "DELIVERED", "ACKNOWLEDGED"}:
                call.scenario_status = DROPOFF_DISPATCHED
            elif call.boarding_confirmed and dropoff is not None and dropoff.status == RESERVED:
                call.scenario_status = DROPOFF_READY
            elif call.boarding_confirmed:
                call.scenario_status = BOARDING_CONFIRMED
            elif pickup is not None and pickup.motion_verified:
                call.scenario_status = WAITING_FOR_BOARDING
            elif pickup is not None and pickup.status in {"DISPATCHING", "DELIVERED", "ACKNOWLEDGED"}:
                call.scenario_status = PICKUP_DISPATCHED
            elif pickup is not None and pickup.status == RESERVED:
                call.scenario_status = PICKUP_PLANNED
            elif pickup is None:
                call.scenario_status = CALL_CREATED
            call.updated_at = _now()
        return self.get(call_id)

    def _score_robot(self, robot: dict, pickup_x: float, pickup_y: float, dropoff_x: float, dropoff_y: float, graph) -> dict:
        state = self.fleet._robot_state(robot["robot_id"])
        if robot["kind"] == "physical":
            if not state.get("tf_fresh") or not isinstance(state.get("pose"), dict):
                return {
                    "robot_id": robot["robot_id"],
                    "error": "tf_unavailable",
                    "blocked": True,
                }
            start = {"x": float(state["pose"]["x"]), "y": float(state["pose"]["y"])}
            start_request = None
        else:
            home = robot["home"]
            start = {"x": float(home["x"]), "y": float(home["y"])}
            start_request = dict(start)

        start_loc = graph.locate(start["x"], start["y"])
        pickup_loc = graph.locate(pickup_x, pickup_y)
        dropoff_loc = graph.locate(dropoff_x, dropoff_y)
        if start_loc[0] != "ok":
            return {"robot_id": robot["robot_id"], "error": f"start {start_loc[0]}"}
        if pickup_loc[0] != "ok":
            return {"robot_id": robot["robot_id"], "error": f"pickup {pickup_loc[0]}"}
        if dropoff_loc[0] != "ok":
            return {"robot_id": robot["robot_id"], "error": f"dropoff {dropoff_loc[0]}"}

        pickup_paths = k_shortest_paths(
            graph, start_loc[1], pickup_loc[1], max(self.settings.planning_max_candidates, 1),
        )
        dropoff_paths = k_shortest_paths(
            graph, pickup_loc[1], dropoff_loc[1], max(self.settings.planning_max_candidates, 1),
        )
        if not pickup_paths:
            return {"robot_id": robot["robot_id"], "error": "no pickup path"}
        if not dropoff_paths:
            return {"robot_id": robot["robot_id"], "error": "no dropoff path"}

        speed = self.settings.planning_speed_m_s
        pickup_travel = pickup_paths[0][1] / speed
        dropoff_travel = dropoff_paths[0][1] / speed
        node_lists = [nodes for nodes, _cost in pickup_paths]
        with self.db.read() as session:
            existing = _active_intervals(session, exclude_task="")
            queued = session.scalars(select(FleetTaskRecord).where(
                FleetTaskRecord.assigned_robot_id == robot["robot_id"],
                FleetTaskRecord.status.in_([RESERVED, "DISPATCHING", DELIVERED, "WAITING", "ACKNOWLEDGED"]),
            )).all()
            queue_length = len(queued)
        scheduled = schedule_candidates(
            graph, node_lists, existing,
            robot_id=robot["robot_id"],
            task_id=f"call-score-{robot['robot_id']}",
            speed_m_s=speed,
            dwell_s=self.settings.planning_goal_dwell_s,
            robot_radius_m=self.settings.planning_robot_radius_m,
            max_depart_slots=self.settings.planning_max_depart_slots,
            origin_epoch=fleet_now().timestamp(),
            now_epoch=fleet_now().timestamp(),
            depart_slot_s=self.settings.planning_depart_slot_s,
            current_profile=self.fleet._profile(),
        )
        if scheduled is None:
            return {"robot_id": robot["robot_id"], "error": "reservation conflict"}
        depart_s = float(scheduled["depart_s"])
        score = (
            pickup_travel
            + depart_s
            + queue_length * self.settings.planning_goal_dwell_s
            + dropoff_travel
        )
        draft = []
        for rank, (nodes, cost) in enumerate(dropoff_paths, start=1):
            record = path_record(graph, nodes, cost, rank)
            record["nodes"] = [list(node) for node in nodes]
            draft.append(record)
        return {
            "robot_id": robot["robot_id"],
            "score": score,
            "pickup_travel_s": pickup_travel,
            "dropoff_travel_s": dropoff_travel,
            "depart_s": depart_s,
            "queue_length": queue_length,
            "start_request": start_request,
            "dropoff_draft": {
                "paths": draft,
                "shortest": draft[0] if draft else None,
                "is_reservation": False,
                "boarding_required": True,
            },
            "reason": (
                f"call score {score:.3f} = pickup {pickup_travel:.2f}s"
                f" + delay {depart_s:.2f}s + queue {queue_length}"
                f" + dropoff {dropoff_travel:.2f}s"
            ),
        }

    def _insert_held_dropoff(
        self,
        *,
        call_id: str,
        robot_id: str,
        pickup_task_id: str,
        pickup_x: float,
        pickup_y: float,
        dropoff_x: float,
        dropoff_y: float,
        priority: int,
        draft: dict,
        idempotency_key: str | None,
    ) -> dict:
        revision = map_revision(self.settings.physical_map_yaml)
        now = _now()
        task_id = str(uuid.uuid4())
        schedule = {
            "transport_leg": LEG_DROPOFF,
            "call_request_id": call_id,
            "boarding_required": True,
            "boarding_confirmed": False,
            "hold_until_boarding": True,
            "predecessor_relation": "transport_leg",
            "draft_paths": draft.get("paths") or [],
            "needs_replan": False,
            "note": "Dropoff is precomputed but not executable until boarding",
        }
        with self.db.write() as session:
            task = FleetTaskRecord(
                id=task_id,
                task_type="navigate",
                priority=priority,
                assignment_mode="manual",
                requested_robot_id=robot_id,
                assigned_robot_id=robot_id,
                start_x=pickup_x,
                start_y=pickup_y,
                start_source="call_pickup",
                goal_x=dropoff_x,
                goal_y=dropoff_y,
                goal_label=f"call:{call_id}:dropoff",
                map_id=str(self.settings.physical_map_yaml.stem),
                map_revision=revision,
                selected_path_id=None,
                planned_path_json="[]",
                candidate_paths_json=json.dumps(draft.get("paths") or []),
                schedule_json=json.dumps(schedule),
                status=CREATED,
                delivery_status="NOT_SENT",
                ack_status="NOT_RECEIVED",
                predecessor_task_id=pickup_task_id,
                idempotency_key=idempotency_key,
                created_at=now,
                updated_at=now,
            )
            session.add(task)
        return {"task_id": task_id, "status": CREATED}

    def _store_failed(self, pickup_x, pickup_y, dropoff_x, dropoff_y, priority, idempotency_key, rejected) -> dict:
        call_id = str(uuid.uuid4())
        now = _now()
        blocked = any(item.get("blocked") for item in rejected)
        with self.db.write() as session:
            session.add(FleetCallRequest(
                id=call_id,
                scenario_status=CALL_BLOCKED if blocked and all(
                    item.get("error") == "tf_unavailable" for item in rejected
                ) else CALL_FAILED,
                assigned_robot_id=None,
                pickup_x=pickup_x,
                pickup_y=pickup_y,
                dropoff_x=dropoff_x,
                dropoff_y=dropoff_y,
                priority=priority,
                detail_json=json.dumps({"rejected": rejected}),
                failure_reason="no assignable robot for both legs",
                idempotency_key=idempotency_key,
                created_at=now,
                updated_at=now,
            ))
        return self.get(call_id)

    def _snapshot(self, call_id: str) -> dict | None:
        with self.db.read() as session:
            call = session.get(FleetCallRequest, call_id)
            if call is None:
                return None
            return self._row_snapshot(call)

    def _row_snapshot(self, call: FleetCallRequest) -> dict:
        return {
            "id": call.id,
            "scenario_status": call.scenario_status,
            "assigned_robot_id": call.assigned_robot_id,
            "pickup_x": call.pickup_x,
            "pickup_y": call.pickup_y,
            "dropoff_x": call.dropoff_x,
            "dropoff_y": call.dropoff_y,
            "priority": call.priority,
            "pickup_task_id": call.pickup_task_id,
            "dropoff_task_id": call.dropoff_task_id,
            "boarding_confirmed": bool(call.boarding_confirmed),
            "boarding_event_id": call.boarding_event_id or "",
            "boarding_source": call.boarding_source or "",
            "boarding_confirmed_at": None if call.boarding_confirmed_at is None else call.boarding_confirmed_at.isoformat(),
            "pickup_arrival_evidence": call.pickup_arrival_evidence or "",
            "detail_json": call.detail_json or "{}",
            "failure_reason": call.failure_reason or "",
            "created_at": call.created_at.isoformat(),
            "updated_at": call.updated_at.isoformat(),
        }

    def _public(self, call: FleetCallRequest) -> dict:
        return self._public_from_snapshot(self._row_snapshot(call))

    def _public_from_snapshot(self, call: dict) -> dict:
        pickup = None
        dropoff = None
        if call["pickup_task_id"]:
            try:
                pickup = self.fleet.get(call["pickup_task_id"])
            except FleetTaskError:
                pickup = None
        if call["dropoff_task_id"]:
            try:
                dropoff = self.fleet.get(call["dropoff_task_id"])
            except FleetTaskError:
                dropoff = None
        gates = {
            "pickup_ack_is_not_arrival": True,
            "eta_is_not_arrival": True,
            "boarding_required_for_dropoff": True,
            "dropoff_dispatch_blocked_until_boarding": True,
            "physical_arrival_sensor": "BLOCKED",
            "boarding_ros_topic": "not_added",
        }
        return {
            "call_id": call["id"],
            "scenario_status": call["scenario_status"],
            "assigned_robot_id": call["assigned_robot_id"],
            "pickup": {"x": call["pickup_x"], "y": call["pickup_y"]},
            "dropoff": {"x": call["dropoff_x"], "y": call["dropoff_y"]},
            "priority": call["priority"],
            "pickup_task_id": call["pickup_task_id"],
            "dropoff_task_id": call["dropoff_task_id"],
            "pickup_task": pickup,
            "dropoff_task": dropoff,
            "boarding": {
                "confirmed": bool(call["boarding_confirmed"]),
                "event_id": call["boarding_event_id"] or None,
                "source": call["boarding_source"] or None,
                "confirmed_at": call["boarding_confirmed_at"],
                "is_physical_user_signal": call["boarding_source"] == "operator",
                "mock_only": call["boarding_source"] == "mock",
            },
            "pickup_arrival_evidence": call["pickup_arrival_evidence"] or None,
            "gates": gates,
            "detail": json.loads(call["detail_json"] or "{}"),
            "failure_reason": call["failure_reason"],
            "created_at": call["created_at"],
            "updated_at": call["updated_at"],
        }


def _now() -> datetime:
    return datetime.now(timezone.utc)
