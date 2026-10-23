from __future__ import annotations

import json
import math
import uuid
from datetime import datetime, timedelta

from sqlalchemy import select
from sqlalchemy.orm import Session

from app.core.errors import ZoneConflictError
from app.models.tables import Robot, Task, TrafficZone, ZoneReservation

ACTIVE_RESERVATION = ("held", "blocked")


def point_in_zone(zone: TrafficZone, x: float, y: float) -> bool:
    return zone.bounds_x <= x <= zone.bounds_x + zone.bounds_w and zone.bounds_y <= y <= zone.bounds_y + zone.bounds_h


def distance_to_zone(zone: TrafficZone, x: float, y: float) -> float:
    if point_in_zone(zone, x, y):
        return 0.0
    nearest_x = min(max(x, zone.bounds_x), zone.bounds_x + zone.bounds_w)
    nearest_y = min(max(y, zone.bounds_y), zone.bounds_y + zone.bounds_h)
    return math.hypot(x - nearest_x, y - nearest_y)


def load_zones(session: Session) -> list[TrafficZone]:
    return list(session.scalars(select(TrafficZone).order_by(TrafficZone.zone_id)).all())


def zone_for_point(session: Session, x: float, y: float) -> TrafficZone | None:
    for zone in load_zones(session):
        if point_in_zone(zone, x, y):
            return zone
    return None


def active_reservation(session: Session, zone_id: str) -> ZoneReservation | None:
    return session.scalar(
        select(ZoneReservation).where(
            ZoneReservation.zone_id == zone_id,
            ZoneReservation.reservation_status.in_(ACTIVE_RESERVATION),
        )
    )


def reservations_for_robot(session: Session, robot_id: str) -> list[ZoneReservation]:
    return list(
        session.scalars(
            select(ZoneReservation).where(
                ZoneReservation.robot_id == robot_id,
                ZoneReservation.reservation_status.in_(ACTIVE_RESERVATION),
            )
        ).all()
    )


def zone_available_for(session: Session, zone_id: str, robot_id: str) -> bool:
    current = active_reservation(session, zone_id)
    if current is None:
        return True
    if current.reservation_status == "blocked":
        return False
    return current.robot_id == robot_id


def seed_zones(session: Session, path, now: datetime) -> None:
    del now
    payload = json.loads(path.read_text(encoding="utf-8"))
    for item in payload:
        existing = session.get(TrafficZone, item["zone_id"])
        bounds = item["bounds"]
        if existing is None:
            session.add(
                TrafficZone(
                    zone_id=item["zone_id"],
                    name=item["name"],
                    kind=item["kind"],
                    capacity=int(item.get("capacity", 1)),
                    bounds_x=bounds["x"],
                    bounds_y=bounds["y"],
                    bounds_w=bounds["w"],
                    bounds_h=bounds["h"],
                )
            )
        else:
            existing.name = item["name"]
            existing.kind = item["kind"]
            existing.capacity = int(item.get("capacity", 1))
            existing.bounds_x = bounds["x"]
            existing.bounds_y = bounds["y"]
            existing.bounds_w = bounds["w"]
            existing.bounds_h = bounds["h"]


def acquire_destination(
    session: Session,
    *,
    robot: Robot,
    task: Task,
    now: datetime,
    ttl_seconds: float,
) -> ZoneReservation | None:
    zone = zone_for_point(session, task.destination_x, task.destination_y)
    if zone is None:
        return None
    if zone.capacity != 1:
        raise ZoneConflictError("Phase 1 zones are single-occupancy only")
    if not zone_available_for(session, zone.zone_id, robot.id):
        raise ZoneConflictError(f"{zone.zone_id} is not free for {robot.id}")
    current = active_reservation(session, zone.zone_id)
    if current is not None and current.robot_id == robot.id and current.reservation_status == "held":
        current.task_id = task.id
        current.owner_epoch = task.owner_epoch
        current.hold_reason = "destination_reserve"
        current.expires_at = now + timedelta(seconds=ttl_seconds)
        current.updated_at = now
        return current
    reservation = ZoneReservation(
        reservation_id=str(uuid.uuid4()),
        zone_id=zone.zone_id,
        robot_id=robot.id,
        task_id=task.id,
        owner_epoch=task.owner_epoch,
        reservation_status="held",
        hold_reason="destination_reserve",
        acquired_at=now,
        expires_at=now + timedelta(seconds=ttl_seconds),
        released_at=None,
        updated_at=now,
    )
    session.add(reservation)
    session.flush()
    return reservation


def block_robot_zones(session: Session, robot_id: str, now: datetime, reason: str) -> None:
    for reservation in reservations_for_robot(session, robot_id):
        if reservation.reservation_status == "released":
            continue
        reservation.reservation_status = "blocked"
        reservation.hold_reason = reason
        reservation.updated_at = now


def release_reservation(session: Session, reservation: ZoneReservation, now: datetime, reason: str) -> None:
    reservation.reservation_status = "released"
    reservation.hold_reason = reason
    reservation.released_at = now
    reservation.updated_at = now


def sync_pose_occupancy(session: Session, robot: Robot, now: datetime, reservation_ttl: float) -> None:
    """Pose may keep a zone occupied. Expiry alone never releases one.

    An untrustworthy pose blocks existing holds and does not clear them.
    """
    if not robot.pose_trustworthy:
        block_robot_zones(session, robot.id, now, "fail_closed_pose_untrusted")
        return
    zones = load_zones(session)
    inside_ids = {zone.zone_id for zone in zones if point_in_zone(zone, robot.pose_x, robot.pose_y)}
    for zone_id in inside_ids:
        current = active_reservation(session, zone_id)
        if current is None:
            session.add(
                ZoneReservation(
                    reservation_id=str(uuid.uuid4()),
                    zone_id=zone_id,
                    robot_id=robot.id,
                    task_id=None,
                    owner_epoch=0,
                    reservation_status="held",
                    hold_reason="pose_occupancy",
                    acquired_at=now,
                    expires_at=now + timedelta(seconds=reservation_ttl),
                    released_at=None,
                    updated_at=now,
                )
            )
            session.flush()
        elif current.robot_id != robot.id or current.reservation_status == "blocked":
            current.reservation_status = "blocked"
            current.hold_reason = "fail_closed_pose_conflict"
            current.updated_at = now
    for reservation in reservations_for_robot(session, robot.id):
        if reservation.reservation_status != "held":
            continue
        if reservation.zone_id in inside_ids:
            continue
        task = session.get(Task, reservation.task_id) if reservation.task_id else None
        destination_zone = None
        if task is not None:
            destination_zone = zone_for_point(session, task.destination_x, task.destination_y)
        reserved_ahead = (
            task is not None
            and task.status in {"ASSIGNED", "DISPATCHING", "RUNNING", "CANCEL_REQUESTED"}
            and destination_zone is not None
            and destination_zone.zone_id == reservation.zone_id
        )
        if reserved_ahead:
            continue
        release_reservation(session, reservation, now, "pose_outside_zone")


def congestion_count(session: Session, robot: Robot, radius: float) -> int:
    count = 0
    for zone in load_zones(session):
        current = active_reservation(session, zone.zone_id)
        if current is None or current.robot_id == robot.id:
            continue
        if distance_to_zone(zone, robot.pose_x, robot.pose_y) <= radius:
            count += 1
    return count
