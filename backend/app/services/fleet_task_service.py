"""Create, plan, reserve, and deliver fleet tasks.

Route geometry comes from the existing A* / Yen planner. This module does not
send Nav2 goals. Preview requests never reach this service.
"""
from __future__ import annotations

import hashlib
import json
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

from sqlalchemy import and_, func, or_, select
from sqlalchemy.exc import IntegrityError

from app.models.tables import FleetReservationRecord, FleetTaskEventRecord, FleetTaskRecord
from app.services.fleet_dispatcher import (
    DispatchError,
    cancel_local_receiver,
    cancel_simulated,
    cancel_ssh,
    deliver_local_receiver,
    deliver_simulated,
    deliver_ssh,
)
from app.services.fleet_registry import RegistryError, load_registry, update_virtual_home
from app.services.fleet_task_machine import (
    ACKNOWLEDGED,
    ACTIVE_QUEUE,
    CANCEL_PENDING,
    CANCELLED,
    CREATED,
    DELIVERED,
    DISPATCHING,
    FAILED,
    HISTORY,
    PLANNED,
    PLANNING,
    REPLANNING,
    RESERVED,
    TERMINAL,
    UNREACHABLE,
    WAITING,
    require_transition,
)
from app.services.map_frame import load_geometry
from app.services.reservation_manager import Interval, schedule_candidates
from app.services.route_planner import k_shortest_paths, path_record
from app.services.route_preview import fresh_tf_pose, inspect_point
from app.services.waypoint_graph import WaypointGraph

ROOT = Path(__file__).resolve().parents[3]


class FleetTaskError(Exception):
    def __init__(self, status_code: int, detail: str, body: dict | None = None) -> None:
        super().__init__(detail)
        self.status_code = status_code
        self.detail = detail
        self.body = body or {}


class FleetTaskService:
    def __init__(self, settings, database, snapshot_provider) -> None:
        self.settings = settings
        self.db = database
        self.snapshot_provider = snapshot_provider
        self._recovered = False

    def create(self, *, mode: str, goal_x: float, goal_y: float, robot_id: str | None,
               start: dict | None, priority: int, task_type: str, label: str,
               idempotency_key: str | None) -> dict:
        self._recover()
        if mode not in {"manual", "automatic"}:
            raise FleetTaskError(422, "assignment_mode must be manual or automatic")
        if mode == "manual" and not robot_id:
            raise FleetTaskError(422, "Manual assignment requires a robot_id")
        if mode == "automatic" and robot_id:
            raise FleetTaskError(422, "Automatic assignment must not force a robot_id")
        if priority < 1:
            raise FleetTaskError(422, "priority must be >= 1")
        self._graph()
        revision = map_revision(self.settings.physical_map_yaml)
        try:
            return self._insert_task(
                mode=mode, goal_x=goal_x, goal_y=goal_y, robot_id=robot_id, start=start,
                priority=priority, task_type=task_type, label=label,
                idempotency_key=idempotency_key, revision=revision,
            )
        except IntegrityError:
            if not idempotency_key:
                raise
            with self.db.read() as session:
                existing = session.scalar(
                    select(FleetTaskRecord).where(FleetTaskRecord.idempotency_key == idempotency_key)
                )
                if existing is not None:
                    return _public(existing)
            raise

    def _insert_task(self, *, mode, goal_x, goal_y, robot_id, start, priority, task_type, label, idempotency_key, revision) -> dict:
        with self.db.write() as session:
            if idempotency_key:
                existing = session.scalar(select(FleetTaskRecord).where(FleetTaskRecord.idempotency_key == idempotency_key))
                if existing is not None:
                    return _public(existing)
            task = FleetTaskRecord(
                id=str(uuid.uuid4()),
                task_type=task_type,
                priority=priority,
                assignment_mode=mode,
                requested_robot_id=robot_id,
                goal_x=goal_x,
                goal_y=goal_y,
                goal_label=label,
                map_id=Path(self.settings.physical_map_yaml).stem,
                map_revision=revision,
                status=CREATED,
                delivery_status="NOT_SENT",
                ack_status="NOT_RECEIVED",
                idempotency_key=idempotency_key,
                created_at=_now(),
                updated_at=_now(),
            )
            if start is not None:
                task.start_x = float(start["x"])
                task.start_y = float(start["y"])
                task.start_source = "request"
            session.add(task)
            session.flush()
            _event(session, task, None, CREATED, "created")
            session.flush()
            return _public(task)

    def list_tasks(self) -> list[dict]:
        self._recover()
        with self.db.read() as session:
            rows = session.scalars(
                select(FleetTaskRecord).order_by(FleetTaskRecord.created_at.desc(), FleetTaskRecord.id.desc())
            ).all()
            return _publish(session, rows)

    def browse(
        self,
        *,
        scope: str = "active",
        status: str | None = None,
        robot_id: str | None = None,
        query: str | None = None,
        page: int = 1,
        page_size: int = 10,
    ) -> dict:
        """Paged task list. The unfiltered list_tasks response stays available."""
        self._recover()
        if scope not in {"active", "history", "archived"}:
            raise FleetTaskError(400, "scope must be active, history, or archived")
        if page < 1:
            raise FleetTaskError(400, "page starts at 1")
        if page_size not in {10, 20, 50}:
            raise FleetTaskError(400, "page_size must be 10, 20, or 50")
        with self.db.read() as session:
            filters = []
            if scope == "archived":
                filters.append(FleetTaskRecord.is_archived.is_(True))
            elif scope == "active":
                filters.append(FleetTaskRecord.is_archived.is_(False))
                filters.append(FleetTaskRecord.status.in_(ACTIVE_QUEUE))
            else:
                filters.append(FleetTaskRecord.is_archived.is_(False))
                filters.append(FleetTaskRecord.status.in_(HISTORY))
            if status:
                filters.append(FleetTaskRecord.status == status)
            if robot_id:
                filters.append(or_(
                    FleetTaskRecord.assigned_robot_id == robot_id,
                    and_(
                        FleetTaskRecord.assigned_robot_id.is_(None),
                        FleetTaskRecord.requested_robot_id == robot_id,
                    ),
                ))
            if query:
                filters.append(func.lower(FleetTaskRecord.id).contains(query.strip().lower()))
            total = session.scalar(select(func.count()).select_from(FleetTaskRecord).where(*filters)) or 0
            rows = session.scalars(
                select(FleetTaskRecord)
                .where(*filters)
                .order_by(FleetTaskRecord.created_at.desc(), FleetTaskRecord.id.desc())
                .offset((page - 1) * page_size)
                .limit(page_size)
            ).all()
            return {
                "tasks": _publish(session, rows),
                "total": int(total),
                "page": page,
                "page_size": page_size,
                "scope": scope,
            }

    def queue(self) -> list[dict]:
        rows = [item for item in self.list_tasks() if item["status"] not in TERMINAL]
        rows.sort(key=lambda item: (-item["priority"], item["created_at"]))
        return rows

    def get(self, task_id: str) -> dict:
        self._recover()
        with self.db.read() as session:
            return _publish(session, [_one(session, task_id)])[0]

    def set_archived(self, task_id: str, archived: bool, actor: str = "operator") -> dict:
        """Hide or restore a finished task. Does not cancel, release, or contact a receiver."""
        self._recover()
        with self.db.write() as session:
            task = _one(session, task_id)
            if archived:
                if task.status not in HISTORY:
                    raise FleetTaskError(409, "Only finished tasks can be hidden")
                if not task.is_archived:
                    task.is_archived = True
                    task.archived_at = _now()
                    task.archived_by = (actor or "operator")[:80]
                    task.updated_at = _now()
                    _event(session, task, task.status, task.status, "archived")
            elif task.is_archived:
                task.is_archived = False
                task.archived_at = None
                task.archived_by = ""
                task.updated_at = _now()
                _event(session, task, task.status, task.status, "restored")
            session.flush()
            return _publish(session, [task])[0]

    def plan(self, task_id: str) -> dict:
        self._recover()
        graph = self._graph()
        revision = map_revision(self.settings.physical_map_yaml)
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.map_revision != revision:
                if task.status not in {CREATED, REPLANNING, PLANNING}:
                    _move(session, task, REPLANNING, "map revision changed")
                _release(session, task.id, "stale")
                task.map_revision = revision
            if task.status in {CREATED, WAITING, REPLANNING, FAILED, UNREACHABLE}:
                _move(session, task, PLANNING, "planning")
            elif task.status in {RESERVED, PLANNED}:
                _release(session, task.id, "released")
                _move(session, task, REPLANNING, "replan")
                _move(session, task, PLANNING, "planning")
            elif task.status != PLANNING:
                raise FleetTaskError(409, f"Cannot plan from {task.status}")
            try:
                result = self._plan_locked(session, task, graph, revision)
            except FleetTaskError:
                raise
            session.flush()
            return result

    def select_path(self, task_id: str, path_id: str) -> dict:
        self._recover()
        graph = self._graph()
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.status not in {RESERVED, PLANNED, WAITING}:
                raise FleetTaskError(409, f"Cannot select a path from {task.status}")
            candidates = json.loads(task.candidate_paths_json or "[]")
            chosen = next((item for item in candidates if item.get("path_id") == path_id), None)
            if chosen is None:
                raise FleetTaskError(404, "Candidate path is not part of this task")
            nodes = [tuple(pair) for pair in chosen["nodes"]]
            existing = _active_intervals(session, exclude_task=task.id)
            scheduled = schedule_candidates(
                graph, [nodes], existing,
                robot_id=task.assigned_robot_id or "",
                task_id=task.id,
                speed_m_s=self.settings.planning_speed_m_s,
                dwell_s=self.settings.planning_goal_dwell_s,
                robot_radius_m=self.settings.planning_robot_radius_m,
                max_depart_slots=self.settings.planning_max_depart_slots,
            )
            if scheduled is None:
                raise FleetTaskError(409, "Selected path conflicts with current reservations", {"replan": True})
            _release(session, task.id, "released")
            _store_schedule(session, task, graph, chosen, scheduled, map_revision(self.settings.physical_map_yaml))
            if task.status != RESERVED:
                _move(session, task, RESERVED, f"selected {path_id}")
            session.flush()
            return _public(task)

    def cancel(self, task_id: str, reason: str = "") -> dict:
        self._recover()
        needs_receiver = False
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.status == CANCELLED:
                return _public(task)
            if task.status == FAILED:
                raise FleetTaskError(409, "Failure history is preserved")
            if task.status == CANCEL_PENDING:
                needs_receiver = True
            elif task.status == DISPATCHING:
                self._begin_cancellation(session, task, reason)
                _move(session, task, CANCEL_PENDING, "cancel requested during dispatch")
            elif task.status in {DELIVERED, ACKNOWLEDGED}:
                self._begin_cancellation(session, task, reason)
                _move(session, task, CANCEL_PENDING, "cancel requested; waiting for receiver")
                needs_receiver = True
            elif task.status in {CREATED, PLANNING, PLANNED, RESERVED, WAITING, REPLANNING, UNREACHABLE}:
                self._begin_cancellation(session, task, reason, outcome="local_only")
                _move(session, task, CANCELLED, reason or "cancelled")
            else:
                raise FleetTaskError(409, f"Cannot cancel {task.status}")
            session.flush()
            public = _public(task)
        if needs_receiver:
            return self._complete_receiver_cancel(task_id)
        return public

    def dispatch(self, task_id: str) -> dict:
        self._recover()
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.status in {CANCELLED, CANCEL_PENDING}:
                raise FleetTaskError(409, f"Cannot dispatch {task.status}")
            if task.status == ACKNOWLEDGED or (task.status == DELIVERED and task.ack_json):
                return _public(task)
            if task.map_revision != map_revision(self.settings.physical_map_yaml):
                raise FleetTaskError(409, "Map revision changed. Plan the task again before dispatch.")
            if task.status != RESERVED:
                raise FleetTaskError(409, f"Dispatch requires RESERVED, not {task.status}")
            if not task.assigned_robot_id or not task.payload_hash:
                raise FleetTaskError(409, "Task has no reserved path to deliver")
            record = load_registry(self.settings.physical_fleet_path)
            robot = next(item for item in record if item["robot_id"] == task.assigned_robot_id)
            payload = json.loads(task.schedule_json)["payload"]
            kind = robot["kind"]
            _move(session, task, DISPATCHING, "dispatch")
            session.flush()
        try:
            if kind == "virtual":
                ack = deliver_simulated(Path(self.settings.data_dir) / "virtual_inbox", payload)
                channel = "simulated"
            else:
                ack = self._deliver_physical(payload)
                channel = "physical"
            error = None
        except DispatchError as exc:
            ack = None
            channel = None
            error = exc
        return self._apply_delivery(task_id, ack, channel, error)

    def reservations(self) -> list[dict]:
        self._recover()
        with self.db.read() as session:
            rows = session.scalars(select(FleetReservationRecord).where(FleetReservationRecord.status == "confirmed")).all()
            return [_reservation_public(row) for row in rows]

    def conflict_report(self) -> dict:
        intervals = []
        for row in self.reservations():
            intervals.append(Interval(
                row["robot_id"], row["task_id"], row["resource_kind"], row["resource_id"],
                row["x"], row["y"], row["t_start"], row["t_end"],
            ))
        from app.services.reservation_manager import find_conflicts
        hits = find_conflicts(intervals, intervals, robot_radius_m=self.settings.planning_robot_radius_m)
        # find_conflicts skips same task and same robot, so a consistent plan reports none.
        return {
            "conflicts": hits,
            "occupancy_class": "planned",
            "actual_motion_verified": False,
            "collision_safety_verified": False,
        }

    def facilities(self) -> dict:
        path = Path(self.settings.facilities_path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        items = []
        for item in payload.get("facilities", []):
            located = item.get("x") is not None and item.get("y") is not None
            items.append({
                "id": item.get("id"),
                "name": item.get("name"),
                "x": item.get("x"),
                "y": item.get("y"),
                "selectable": located,
                "detail": None if located else "좌표가 등록되지 않아 선택할 수 없습니다.",
            })
        return {"facilities": items}

    def approve_home(self, robot_id: str, x: float, y: float, yaw: float, approve: bool) -> dict:
        if not approve:
            raise FleetTaskError(422, "Registry home is unchanged until approve is true")
        graph = self._graph()
        inspection = inspect_point(graph, x, y)
        if not inspection["traversable"]:
            raise FleetTaskError(422, f"Proposed home is not traversable ({inspection['reason']})", inspection)
        try:
            record = update_virtual_home(Path(self.settings.physical_fleet_path), robot_id, x, y, yaw)
        except RegistryError as exc:
            raise FleetTaskError(404, str(exc)) from exc
        return {"robot": record, "inspection": inspection, "registry_updated": True}

    def robot_availability(self) -> list[dict]:
        graph = self._graph()
        registry = load_registry(self.settings.physical_fleet_path)
        snapshot = self.snapshot_provider()
        queue = {item["assigned_robot_id"]: 0 for item in self.queue()}
        for item in self.queue():
            if item["assigned_robot_id"]:
                queue[item["assigned_robot_id"]] = queue.get(item["assigned_robot_id"], 0) + 1
        reports = []
        for record in registry:
            if record["kind"] == "virtual":
                inspection = inspect_point(graph, record["home"]["x"], record["home"]["y"])
                reports.append({
                    "robot_id": record["robot_id"],
                    "kind": "virtual",
                    "assignable": inspection["traversable"],
                    "reason": inspection["reason"],
                    "pose_source": "registry_home",
                    "inspection": inspection,
                    "queue_length": queue.get(record["robot_id"], 0),
                    "ssh_online": False,
                    "nav2_ready": None,
                })
            else:
                pose = fresh_tf_pose(snapshot)
                if pose is None:
                    reports.append({
                        "robot_id": record["robot_id"],
                        "kind": "physical",
                        "assignable": False,
                        "reason": "tf_unavailable",
                        "pose_source": None,
                        "inspection": None,
                        "queue_length": queue.get(record["robot_id"], 0),
                    })
                else:
                    inspection = inspect_point(graph, pose["x"], pose["y"])
                    reports.append({
                        "robot_id": record["robot_id"],
                        "kind": "physical",
                        "assignable": inspection["traversable"],
                        "reason": inspection["reason"],
                        "pose_source": pose["source"],
                        "inspection": inspection,
                        "queue_length": queue.get(record["robot_id"], 0),
                    })
        return reports

    def _plan_locked(self, session, task: FleetTaskRecord, graph: WaypointGraph, revision: str) -> dict:
        registry = load_registry(self.settings.physical_fleet_path)
        snapshot = self.snapshot_provider()
        if task.assignment_mode == "manual":
            robot = next((item for item in registry if item["robot_id"] == task.requested_robot_id), None)
            if robot is None:
                _fail(session, task, FAILED, "robot is not registered")
                return _public(task)
            choice = self._score_robot(session, task, graph, robot, snapshot)
            if not choice.get("error"):
                choice["reason"] = f"manual assignment to {robot['robot_id']}"
            if choice.get("error"):
                _fail(session, task, choice["status"], choice["error"])
                task.schedule_json = json.dumps({"suggestions": choice.get("suggestions", [])})
                return _public(task)
            self._commit_choice(session, task, graph, choice, revision)
            return _public(task)
        ranked = []
        errors = []
        for robot in registry:
            choice = self._score_robot(session, task, graph, robot, snapshot)
            if choice.get("error"):
                errors.append({
                    "robot_id": robot["robot_id"],
                    "error": choice["error"],
                    "suggestions": choice.get("suggestions", []),
                })
                continue
            ranked.append(choice)
        if not ranked:
            _fail(session, task, UNREACHABLE if any(item["error"].startswith("no path") for item in errors) else FAILED,
                  "No assignable robot")
            task.assignment_reason = json.dumps(errors, ensure_ascii=False)
            task.schedule_json = json.dumps({"rejected": errors})
            return _public(task)
        ranked.sort(key=lambda item: item["score"])
        winner = ranked[0]
        winner["reason"] = (
            f"score {winner['score']:.3f} = (travel {winner['travel_s']:.2f}s"
            f" + delay {winner['depart_s']:.2f}s + queue {winner['queue_length']})"
            f" / priority {task.priority}"
        )
        self._commit_choice(session, task, graph, winner, revision)
        task.assignment_reason = winner["reason"]
        return _public(task)

    def _score_robot(self, session, task, graph, robot, snapshot) -> dict:
        start = self._resolve_start(task, robot, snapshot, graph)
        if start.get("error"):
            return start
        goal = graph.locate(task.goal_x, task.goal_y)
        if goal[0] != "ok":
            return {"error": f"goal {goal[0]}", "status": FAILED if goal[0] != "outside" else FAILED,
                    "suggestions": inspect_point(graph, task.goal_x, task.goal_y).get("suggestions", [])}
        paths = k_shortest_paths(graph, start["node"], goal[1], max(self.settings.planning_max_candidates, 1))
        if not paths:
            return {"error": "no path", "status": UNREACHABLE, "suggestions": []}
        node_lists = [nodes for nodes, _cost in paths]
        existing = _active_intervals(session, exclude_task=task.id)
        scheduled = schedule_candidates(
            graph, node_lists, existing,
            robot_id=robot["robot_id"], task_id=task.id,
            speed_m_s=self.settings.planning_speed_m_s,
            dwell_s=self.settings.planning_goal_dwell_s,
            robot_radius_m=self.settings.planning_robot_radius_m,
            max_depart_slots=self.settings.planning_max_depart_slots,
        )
        queued = session.scalars(select(FleetTaskRecord).where(
            FleetTaskRecord.assigned_robot_id == robot["robot_id"],
            FleetTaskRecord.id != task.id,
            FleetTaskRecord.status.in_([RESERVED, DISPATCHING, DELIVERED, WAITING, ACKNOWLEDGED]),
        )).all()
        queue_length = len(queued)
        if scheduled is None:
            return {"error": "reservation conflict", "status": WAITING, "suggestions": []}
        travel_s = _path_length(graph, scheduled["nodes"]) / self.settings.planning_speed_m_s
        score = (travel_s + scheduled["depart_s"] + queue_length * self.settings.planning_goal_dwell_s) / task.priority
        records = []
        for rank, (nodes, cost) in enumerate(paths, start=1):
            record = path_record(graph, nodes, cost, rank)
            record["nodes"] = [list(node) for node in nodes]
            records.append(record)
        return {
            "robot": robot,
            "start": start,
            "paths": records,
            "scheduled": scheduled,
            "travel_s": travel_s,
            "depart_s": scheduled["depart_s"],
            "queue_length": queue_length,
            "score": score,
            "reason": "",
        }

    def _resolve_start(self, task, robot, snapshot, graph) -> dict:
        if task.start_x is not None and task.start_y is not None:
            x, y, source = task.start_x, task.start_y, task.start_source or "request"
        elif robot["kind"] == "virtual":
            x, y, source = robot["home"]["x"], robot["home"]["y"], "registry_home"
        else:
            pose = fresh_tf_pose(snapshot)
            if pose is None:
                return {"error": "tf_unavailable", "status": FAILED, "suggestions": []}
            x, y, source = pose["x"], pose["y"], pose["source"]
        reason, node = graph.locate(x, y)
        if reason != "ok":
            inspection = inspect_point(graph, x, y)
            return {"error": f"start {reason}", "status": FAILED, "suggestions": inspection["suggestions"]}
        return {"x": x, "y": y, "source": source, "node": node}

    def _commit_choice(self, session, task, graph, choice, revision: str) -> None:
        scheduled = choice["scheduled"]
        selected = next(item for item in choice["paths"] if item["nodes"] == [list(node) for node in scheduled["nodes"]])
        task.assigned_robot_id = choice["robot"]["robot_id"]
        task.start_x = choice["start"]["x"]
        task.start_y = choice["start"]["y"]
        task.start_source = choice["start"]["source"]
        task.assignment_reason = choice.get("reason") or task.assignment_reason
        task.candidate_paths_json = json.dumps(choice["paths"])
        _store_schedule(session, task, graph, selected, scheduled, revision)
        _move(session, task, PLANNED, "path selected")
        _move(session, task, RESERVED, "reserved")

    def _begin_cancellation(self, session, task: FleetTaskRecord, reason: str, outcome: str = "pending") -> None:
        current = json.loads(task.cancellation_json) if task.cancellation_json else {}
        if not current.get("cancellation_request_id"):
            current = {
                "cancellation_request_id": str(uuid.uuid4()),
                "requested_at": _now().isoformat(),
                "reason": reason,
                "outcome": outcome,
                "reservations_released": 0,
                "receiver": None,
            }
        else:
            current["reason"] = reason or current.get("reason") or ""
            current["outcome"] = outcome
        current["reservations_released"] = _release(session, task.id, "released")
        task.cancellation_json = json.dumps(current)

    def _apply_delivery(self, task_id: str, ack: dict | None, channel: str | None, error: DispatchError | None) -> dict:
        finish_cancel = False
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.status == CANCEL_PENDING:
                if error is not None:
                    record = json.loads(task.cancellation_json or "{}")
                    record["outcome"] = "not_delivered"
                    record["receiver_error"] = error.detail
                    task.cancellation_json = json.dumps(record)
                    task.delivery_status = "FAILED"
                    task.failure_reason = error.detail
                    _move(session, task, CANCELLED, "cancel requested and delivery failed")
                else:
                    self._store_receipt(task, ack, channel)
                    finish_cancel = True
            elif task.status != DISPATCHING:
                return _public(task)
            elif error is not None:
                task.delivery_status = "FAILED"
                task.ack_status = "NOT_RECEIVED"
                task.failure_reason = error.detail
                _move(session, task, FAILED, error.detail)
                _release(session, task.id, "released")
            else:
                self._store_receipt(task, ack, channel)
                if channel == "simulated":
                    _move(session, task, DELIVERED, "simulated receipt")
                else:
                    _move(session, task, DELIVERED, "receiver stored payload")
                    _move(session, task, ACKNOWLEDGED, "physical ack matched")
            session.flush()
            public = _public(task)
        if finish_cancel:
            return self._complete_receiver_cancel(task_id)
        return public

    def _store_receipt(self, task: FleetTaskRecord, ack: dict | None, channel: str | None) -> None:
        task.ack_json = json.dumps(ack or {})
        task.ack_channel = channel or "none"
        if channel == "simulated":
            task.delivery_status = "SIMULATED"
            task.ack_status = "SIMULATED_RECEIPT"
        else:
            task.delivery_status = "DELIVERED"
            task.ack_status = "ACKNOWLEDGED"

    def _complete_receiver_cancel(self, task_id: str) -> dict:
        with self.db.read() as session:
            task = _one(session, task_id)
            if task.status != CANCEL_PENDING:
                return _public(task)
            request = json.loads(task.cancellation_json or "{}")
            robot_id = task.assigned_robot_id
            channel = task.ack_channel
            delivered = bool(task.ack_json)
        message = {
            "action": "cancel",
            "task_id": task_id,
            "robot_id": robot_id,
            "cancellation_request_id": request.get("cancellation_request_id"),
            "requested_at": request.get("requested_at"),
            "reason": request.get("reason") or "",
            "execute": False,
        }
        try:
            if not delivered:
                raise DispatchError("not_stored", "Task was not delivered")
            ack = self._send_receiver_cancel(robot_id or "", channel, message)
            error = None
        except DispatchError as exc:
            ack = None
            error = exc
        with self.db.write() as session:
            task = _one(session, task_id)
            if task.status != CANCEL_PENDING:
                return _public(task)
            record = json.loads(task.cancellation_json or "{}")
            if error is None and ack is not None:
                record["outcome"] = "cancel_ack"
                record["receiver"] = ack
                task.cancellation_json = json.dumps(record)
                if ack.get("channel") == "physical":
                    task.ack_status = "CANCEL_ACKNOWLEDGED"
                _move(session, task, CANCELLED, "receiver cancel ack")
            elif error is not None and error.code == "not_stored":
                record["outcome"] = "not_stored"
                record["receiver_error"] = error.detail
                task.cancellation_json = json.dumps(record)
                _move(session, task, CANCELLED, "receiver has no stored task")
            else:
                record["outcome"] = "pending"
                record["receiver_error"] = None if error is None else error.detail
                task.cancellation_json = json.dumps(record)
            session.flush()
            return _public(task)

    def _send_receiver_cancel(self, robot_id: str, channel: str, message: dict) -> dict:
        if channel == "simulated":
            return cancel_simulated(Path(self.settings.data_dir) / "virtual_inbox", message)
        if self.settings.fleet_dispatch_mode == "local":
            return cancel_local_receiver(Path(self.settings.data_dir) / "physical_inbox", message)
        from app.api.physical import parse_targets
        targets = dict(parse_targets(self.settings.physical_ssh_targets))
        target = targets.get(robot_id)
        if not target:
            raise DispatchError("ssh_unconfigured", "No SSH target for the physical robot")
        return cancel_ssh(self.settings, target, message)

    def _deliver_physical(self, payload: dict) -> dict:
        if self.settings.fleet_dispatch_mode == "local":
            return deliver_local_receiver(Path(self.settings.data_dir) / "physical_inbox", payload)
        from app.api.physical import parse_targets
        targets = dict(parse_targets(self.settings.physical_ssh_targets))
        target = targets.get(payload["robot_id"])
        if not target:
            raise DispatchError("ssh_unconfigured", "No SSH target for the physical robot")
        return deliver_ssh(self.settings, target, payload)

    def _graph(self) -> WaypointGraph:
        from app.services.route_preview import build_graph
        return build_graph(
            self.settings.physical_map_yaml,
            planning_resolution_m=self.settings.planning_resolution_m,
            robot_radius_m=self.settings.planning_robot_radius_m,
            allow_diagonal=self.settings.planning_allow_diagonal,
        )

    def _stored_revision(self, task_id: str) -> str:
        with self.db.read() as session:
            return _one(session, task_id).map_revision

    def _recover(self) -> None:
        if self._recovered:
            return
        revision = map_revision(self.settings.physical_map_yaml)
        with self.db.write() as session:
            rows = session.scalars(select(FleetTaskRecord)).all()
            for task in rows:
                if task.status in TERMINAL or task.status == CANCEL_PENDING:
                    continue
                if task.map_revision != revision:
                    if task.status == CREATED:
                        task.map_revision = revision
                    elif task.status != REPLANNING:
                        _move(session, task, REPLANNING, "recovered stale map revision")
                        _release(session, task.id, "stale")
                    else:
                        _release(session, task.id, "stale")
                elif task.status == DISPATCHING:
                    task.failure_reason = "dispatch interrupted by process restart"
                    task.delivery_status = "FAILED"
                    _move(session, task, FAILED, task.failure_reason)
                    _release(session, task.id, "released")
        self._recovered = True


def map_revision(yaml_path: Path) -> str:
    geometry = load_geometry(yaml_path)
    digest = hashlib.sha256()
    digest.update(Path(yaml_path).read_bytes())
    digest.update(geometry.image_path.read_bytes())
    digest.update(f"{geometry.origin_x},{geometry.origin_y},{geometry.origin_yaw},{geometry.resolution}".encode())
    return digest.hexdigest()[:16]


def canonical_payload(task: FleetTaskRecord, waypoints: list[dict]) -> dict:
    payload = {
        "task_id": task.id,
        "robot_id": task.assigned_robot_id,
        "task_type": task.task_type,
        "map_id": task.map_id,
        "map_revision": task.map_revision,
        "start": {"x": task.start_x, "y": task.start_y},
        "goal": {"x": task.goal_x, "y": task.goal_y, "label": task.goal_label},
        "waypoints": [{"x": item["x"], "y": item["y"], "waypoint_id": item["waypoint_id"]} for item in waypoints],
        "execute": False,
    }
    payload["payload_hash"] = hashlib.sha256(
        json.dumps({key: value for key, value in payload.items() if key != "payload_hash"}, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return payload


def _store_schedule(session, task, graph, selected, scheduled, revision: str) -> None:
    task.selected_path_id = selected["path_id"]
    task.planned_path_json = json.dumps(selected["waypoints"])
    depart = scheduled["depart_s"]
    last = max(item.t_end for item in scheduled["intervals"])
    base = _now()
    task.estimated_start_time = (base + timedelta(seconds=depart)).isoformat()
    task.estimated_arrival_time = (base + timedelta(seconds=last)).isoformat()
    payload = canonical_payload(task, selected["waypoints"])
    task.payload_hash = payload["payload_hash"]
    task.schedule_json = json.dumps({
            "depart_s": depart,
            "speed_m_s_is_estimated": True,
            "actual_motion_verified": False,
            "collision_safety_verified": False,
            "payload": payload,
            "conflicts_avoided_by_delay": scheduled["conflicts_avoided_by_delay"],
        })
    task.map_revision = revision
    for interval in scheduled["intervals"]:
        session.add(FleetReservationRecord(
            id=str(uuid.uuid4()),
            task_id=task.id,
            robot_id=task.assigned_robot_id,
            resource_kind=interval.kind,
            resource_id=interval.resource_id,
            x=interval.x,
            y=interval.y,
            t_start=interval.t_start,
            t_end=interval.t_end,
            status="confirmed",
            occupancy_class="planned",
            map_revision=revision,
            created_at=_now(),
        ))


def _path_length(graph, nodes) -> float:
    import math
    total = 0.0
    for left, right in zip(nodes, nodes[1:]):
        lx, ly = graph.world(left)
        rx, ry = graph.world(right)
        total += math.hypot(rx - lx, ry - ly)
    return total


def _active_intervals(session, exclude_task: str) -> list[Interval]:
    rows = session.scalars(select(FleetReservationRecord).where(FleetReservationRecord.status == "confirmed")).all()
    return [
        Interval(row.robot_id, row.task_id, row.resource_kind, row.resource_id, row.x, row.y, row.t_start, row.t_end)
        for row in rows if row.task_id != exclude_task
    ]


def _release(session, task_id: str, status: str) -> int:
    rows = session.scalars(select(FleetReservationRecord).where(FleetReservationRecord.task_id == task_id)).all()
    released = 0
    for row in rows:
        if row.status == "confirmed":
            row.status = status
            released += 1
    return released


def _move(session, task: FleetTaskRecord, nxt: str, reason: str) -> None:
    require_transition(task.status, nxt)
    previous = task.status
    task.status = nxt
    task.updated_at = _now()
    _event(session, task, previous, nxt, reason)


def _fail(session, task, status: str, reason: str) -> None:
    task.failure_reason = reason
    _move(session, task, status, reason)


def _event(session, task, previous, nxt, reason: str) -> None:
    session.add(FleetTaskEventRecord(
        task_id=task.id,
        from_status=previous,
        to_status=nxt,
        reason=reason,
        created_at=_now(),
    ))


def _one(session, task_id: str) -> FleetTaskRecord:
    task = session.get(FleetTaskRecord, task_id)
    if task is None:
        raise FleetTaskError(404, "Fleet task not found")
    return task


def _counts(session, task_ids: list[str]) -> dict[str, int]:
    if not task_ids:
        return {}
    rows = session.execute(
        select(FleetReservationRecord.task_id, func.count())
        .where(
            FleetReservationRecord.task_id.in_(task_ids),
            FleetReservationRecord.status == "confirmed",
        )
        .group_by(FleetReservationRecord.task_id)
    ).all()
    return {task_id: int(count) for task_id, count in rows}


def _publish(session, rows: list[FleetTaskRecord]) -> list[dict]:
    counts = _counts(session, [row.id for row in rows])
    return [_public(row, counts.get(row.id, 0)) for row in rows]


def _public(task: FleetTaskRecord, active_reservation_count: int = 0) -> dict:
    return {
        "task_id": task.id,
        "task_type": task.task_type,
        "priority": task.priority,
        "assignment_mode": task.assignment_mode,
        "requested_robot_id": task.requested_robot_id,
        "assigned_robot_id": task.assigned_robot_id,
        "start": None if task.start_x is None else {"x": task.start_x, "y": task.start_y, "source": task.start_source},
        "goal": {"x": task.goal_x, "y": task.goal_y, "label": task.goal_label},
        "map_id": task.map_id,
        "map_revision": task.map_revision,
        "selected_path_id": task.selected_path_id,
        "planned_path": json.loads(task.planned_path_json or "[]"),
        "candidate_paths": json.loads(task.candidate_paths_json or "[]"),
        "estimated_start_time": task.estimated_start_time,
        "estimated_arrival_time": task.estimated_arrival_time,
        "status": task.status,
        "delivery_status": task.delivery_status,
        "ack_status": task.ack_status,
        "ack_channel": task.ack_channel,
        "ack": json.loads(task.ack_json) if task.ack_json else None,
        "payload_hash": task.payload_hash,
        "assignment_reason": task.assignment_reason,
        "failure_reason": task.failure_reason,
        "cancellation": json.loads(task.cancellation_json) if task.cancellation_json else None,
        "is_archived": bool(task.is_archived),
        "archived_at": None if task.archived_at is None else task.archived_at.isoformat(),
        "archived_by": task.archived_by or "",
        "active_reservation_count": active_reservation_count,
        "schedule": json.loads(task.schedule_json or "{}"),
        "actual_motion_verified": False,
        "created_at": task.created_at.isoformat(),
        "updated_at": task.updated_at.isoformat(),
    }


def _reservation_public(row: FleetReservationRecord) -> dict:
    return {
        "reservation_id": row.id,
        "task_id": row.task_id,
        "robot_id": row.robot_id,
        "resource_kind": row.resource_kind,
        "resource_id": row.resource_id,
        "x": row.x,
        "y": row.y,
        "t_start": row.t_start,
        "t_end": row.t_end,
        "status": row.status,
        "occupancy_class": row.occupancy_class,
        "map_revision": row.map_revision,
        "actual_motion_verified": False,
    }


def _now() -> datetime:
    return datetime.now(timezone.utc)
