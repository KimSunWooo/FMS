from __future__ import annotations

import hashlib
import hmac
import uuid
from datetime import timedelta

from sqlalchemy import func, select
from sqlalchemy.orm import Session

from app.core.clock import as_utc
from app.core.errors import LeaseRejectedError
from app.models.tables import ControlLease


class MockGateway:
    """Local control-lease issuer.

    This is not the Care-Move Gateway process. The HMAC key is the new
    project's secret. Boot id lives only in this process, so a restart
    invalidates every lease this gateway previously signed.
    """

    def __init__(self, *, environment_id: str, secret: str, ttl_seconds: float):
        self.environment_id = environment_id
        self._secret = secret
        self.ttl_seconds = ttl_seconds
        self.boot_id = str(uuid.uuid4())
        self.online = True
        self.force_reject = False

    def issue(
        self,
        session: Session,
        *,
        robot_id: str,
        session_id: str,
        owner_id: str,
        now,
    ) -> ControlLease:
        if self.force_reject or not self.online:
            raise LeaseRejectedError("mock gateway refused to issue a lease")
        current = session.scalar(
            select(func.max(ControlLease.control_generation)).where(ControlLease.robot_id == robot_id)
        )
        generation = int(current or 0) + 1
        expires_at = now + timedelta(seconds=self.ttl_seconds)
        token = expires_at.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        lease_id = str(uuid.uuid4())
        lease = ControlLease(
            lease_id=lease_id,
            environment_id=self.environment_id,
            robot_id=robot_id,
            session_id=session_id,
            owner_id=owner_id,
            control_generation=generation,
            gateway_boot_id=self.boot_id,
            canonical_expires=token,
            expires_at=expires_at,
            issued_at=now,
            revoked_at=None,
            signature="",
            status="active",
        )
        lease.signature = self._sign(lease)
        session.add(lease)
        session.flush()
        return lease

    def renew(self, session: Session, lease_id: str, now) -> bool:
        lease = session.get(ControlLease, lease_id)
        if lease is None or not self.verify(session, lease, now=now):
            return False
        expires_at = now + timedelta(seconds=self.ttl_seconds)
        lease.expires_at = expires_at
        lease.canonical_expires = expires_at.strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        lease.signature = self._sign(lease)
        return True

    def revoke(self, session: Session, lease_id: str, now) -> None:
        lease = session.get(ControlLease, lease_id)
        if lease is None:
            return
        lease.status = "revoked"
        lease.revoked_at = now

    def verify(
        self,
        session: Session,
        lease: ControlLease,
        *,
        now,
        robot_id: str | None = None,
        session_id: str | None = None,
    ) -> bool:
        if self.force_reject or not self.online:
            return False
        if lease.status != "active" or lease.revoked_at is not None:
            return False
        if lease.environment_id != self.environment_id:
            return False
        if lease.gateway_boot_id != self.boot_id:
            return False
        if robot_id is not None and lease.robot_id != robot_id:
            return False
        if session_id is not None and lease.session_id != session_id:
            return False
        expires_at = as_utc(lease.expires_at)
        if expires_at is None or now >= expires_at:
            return False
        latest = session.scalar(
            select(func.max(ControlLease.control_generation)).where(ControlLease.robot_id == lease.robot_id)
        )
        if int(latest or 0) != lease.control_generation:
            return False
        expected = self._sign(lease)
        return hmac.compare_digest(expected, lease.signature)

    def _sign(self, lease: ControlLease) -> str:
        payload = "|".join(
            [
                lease.environment_id,
                lease.robot_id,
                lease.session_id,
                lease.lease_id,
                str(lease.control_generation),
                lease.gateway_boot_id,
                lease.canonical_expires,
            ]
        )
        return hmac.new(self._secret.encode("utf-8"), payload.encode("utf-8"), hashlib.sha256).hexdigest()
