from __future__ import annotations


class FmsError(Exception):
    http_status = 400
    code = "FMS_ERROR"

    def __init__(self, message: str):
        self.message = message
        super().__init__(message)


class NotFoundError(FmsError):
    http_status = 404
    code = "NOT_FOUND"


class IllegalTransitionError(FmsError):
    http_status = 409
    code = "ILLEGAL_TRANSITION"


class StaleOwnerError(FmsError):
    http_status = 409
    code = "STALE_OWNER"


class VersionConflictError(FmsError):
    http_status = 409
    code = "VERSION_CONFLICT"


class RobotBusyError(FmsError):
    http_status = 409
    code = "ROBOT_BUSY"


class NotEligibleError(FmsError):
    http_status = 409
    code = "NOT_ELIGIBLE"


class ZoneConflictError(FmsError):
    http_status = 409
    code = "ZONE_CONFLICT"


class LeaseRejectedError(FmsError):
    http_status = 403
    code = "LEASE_REJECTED"


class PhysicalControlBlocked(FmsError):
    http_status = 403
    code = "PHYSICAL_CONTROL_BLOCKED"


class IdempotencyMismatchError(FmsError):
    http_status = 409
    code = "IDEMPOTENCY_MISMATCH"


class ConfigurationError(FmsError):
    http_status = 500
    code = "CONFIGURATION_ERROR"
