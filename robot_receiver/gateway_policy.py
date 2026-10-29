"""When a gateway may publish again.

A RELIABLE sample is not an application ACK. VOLATILE durability does not
replay it after the subscriber leaves, so a later subscriber or a rewritten
command file needs another publish.
"""
from __future__ import annotations


def publish_decision(
    *,
    published: bool,
    published_mtime: int | None,
    file_mtime: int,
    subscribers: int,
    age_s: float,
    discovery_s: float,
) -> str:
    """Return ``publish``, ``wait``, or ``timeout``."""
    if subscribers < 1:
        if age_s > discovery_s:
            return "timeout"
        return "wait"
    if (not published) or published_mtime != file_mtime:
        return "publish"
    return "wait"
