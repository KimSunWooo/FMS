"""Per-robot agent process.

The process registers with the FMS, claims at most one Nav2 send, and posts a
result only after GazeboNav2Client observes an action result. Preflight failure
exits before any goal is claimed and before any success is reported.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

from robot_agent.gazebo_nav2_client import GazeboNav2Client, GazeboNav2Unavailable
from robot_agent.preflight import live_report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Care-Move FMS robot agent")
    parser.add_argument("--fms-url", default=os.environ.get("FMS_URL", "http://127.0.0.1:8741"))
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--expected-domain", type=int, default=73)
    args = parser.parse_args(argv)
    report = live_report(expected_domain=args.expected_domain)
    observed = report.ros_domain_id
    summary = {
        "robot_id": args.robot_id,
        "preflight": report.as_dict(),
        "nav2_goal_sent": False,
        "task_completed": False,
        "physical_control": "blocked",
    }
    if not report.ready:
        summary["error"] = "gazebo preflight failed; no Nav2 goal was sent"
        print(json.dumps(summary, ensure_ascii=False))
        return 2
    client = GazeboNav2Client(
        mode="gazebo",
        expected_domain=args.expected_domain,
        observed_domain=observed,
        preflight=report,
    )
    try:
        client.assert_can_send()
    except GazeboNav2Unavailable as exc:
        summary["error"] = str(exc)
        print(json.dumps(summary, ensure_ascii=False))
        return 2
    try:
        return _run_one_assignment(args.fms_url.rstrip("/"), args.robot_id, client, summary)
    except GazeboNav2Unavailable as exc:
        summary["error"] = str(exc)
        summary["nav2_goal_sent"] = False
        summary["task_completed"] = False
        print(json.dumps(summary, ensure_ascii=False))
        return 2


def _run_one_assignment(base: str, robot_id: str, client: GazeboNav2Client, summary: dict) -> int:
    opened = _request("POST", f"{base}/api/agent/session", {"robot_id": robot_id})
    session_id = opened["session_id"]
    summary["session_id"] = session_id
    try:
        return _run_attached(base, robot_id, session_id, client, summary)
    finally:
        # Drop the in-memory session when this process leaves. Do not send
        # another Nav2 goal from here.
        try:
            _request(
                "POST",
                f"{base}/api/agent/stop",
                {"robot_id": robot_id, "session_id": session_id},
            )
        except GazeboNav2Unavailable:
            pass


def _run_attached(base: str, robot_id: str, session_id: str, client: GazeboNav2Client, summary: dict) -> int:
    # The scheduler will not assign until this session is alive. Wait for one
    # pending goal. This is not a Nav2 resend: nothing has been claimed yet.
    deadline = time.monotonic() + 30.0
    task = None
    while time.monotonic() < deadline:
        assignment = _request(
            "GET",
            f"{base}/api/agent/assignment?robot_id={robot_id}&session_id={session_id}",
            None,
        )
        candidate = assignment.get("assignment")
        if candidate and assignment.get("send_navigate_to_pose"):
            task = candidate
            break
        time.sleep(0.25)
    if task is None:
        summary["error"] = "no pending goal to send"
        print(json.dumps(summary, ensure_ascii=False))
        return 0
    claim = _request(
        "POST",
        f"{base}/api/agent/claim",
        {
            "robot_id": robot_id,
            "session_id": session_id,
            "owner_epoch": task["owner_epoch"],
            "lease_id": task["lease_id"],
            "goal_id": task["goal_id"],
        },
    )
    if not claim.get("send_navigate_to_pose"):
        summary["error"] = "duplicate or rejected claim; goal was not sent"
        print(json.dumps(summary, ensure_ascii=False))
        return 0
    destination = claim["destination"]
    frame_id = destination.get("frame_id") or "map"
    heartbeats = _start_heartbeats(base, robot_id, session_id)
    try:
        observed = client.send_goal(
            robot_id=robot_id,
            goal_id=claim["goal_id"],
            x=destination["x"],
            y=destination["y"],
            yaw=destination["yaw"],
            frame_id=frame_id,
            action_name="/navigate_to_pose",
            use_sim_time=True,
        )
    except GazeboNav2Unavailable:
        heartbeats.set()
        _post_event(
            base,
            task,
            session_id,
            robot_id,
            kind="disconnected",
            outcome="disconnected",
            pose_trustworthy=False,
            nav2_action_goal_id=None,
        )
        summary["error"] = "Nav2 result was not observed; FMS was told disconnected, not completed"
        print(json.dumps(summary, ensure_ascii=False))
        return 2
    pose = observed.get("pose")
    pose_ok = (
        isinstance(pose, dict)
        and pose.get("frame_id") == frame_id
        and "x" in pose
        and "y" in pose
    )
    summary["nav2_goal_sent"] = True
    summary["nav2_outcome"] = observed["outcome"]
    summary["nav2_action_goal_id"] = observed.get("nav2_action_goal_id")
    summary["nav2_action"] = observed.get("nav2_action")
    summary["nav2_status"] = observed.get("nav2_status")
    summary["goal_sent_at"] = observed.get("goal_sent_at")
    summary["goal_accepted_at"] = observed.get("goal_accepted_at")
    summary["goal_result_at"] = observed.get("goal_result_at")
    summary["feedback_count"] = observed.get("feedback_count")
    summary["fms_goal_id"] = claim["goal_id"]
    summary["fms_task_id"] = claim["task_id"]
    summary["execution_id"] = claim.get("execution_id") or task.get("execution_id")
    summary["owner_epoch"] = claim.get("owner_epoch") or task.get("owner_epoch")
    summary["pose_observed"] = pose_ok
    summary["pose"] = pose if pose_ok else None
    summary["goal_frame_id"] = frame_id
    if observed["outcome"] == "succeeded":
        _post_event(
            base,
            task,
            session_id,
            robot_id,
            kind="accepted",
            outcome=None,
            pose_trustworthy=False,
            nav2_action_goal_id=observed.get("nav2_action_goal_id"),
        )
    # The destination is not an observation. Without a measured pose in the
    # goal frame the FMS must not treat a Nav2 status as arrival.
    posted = _post_event(
        base,
        task,
        session_id,
        robot_id,
        kind="result",
        outcome=observed["outcome"],
        pose_trustworthy=pose_ok,
        nav2_action_goal_id=observed.get("nav2_action_goal_id"),
        pose=pose if pose_ok else None,
    )
    heartbeats.set()
    summary["fms_status"] = posted.get("status")
    summary["task_completed"] = posted.get("status") == "COMPLETED"
    if observed["outcome"] == "succeeded" and not summary["task_completed"]:
        if pose_ok:
            summary["error"] = (
                "Nav2 succeeded and a pose in the goal frame was observed, "
                "but the FMS task was not marked COMPLETED."
            )
        else:
            summary["error"] = (
                "Nav2 status was observed without a trustworthy pose. "
                "The FMS task was not marked COMPLETED."
            )
    print(json.dumps(summary, ensure_ascii=False))
    if summary["task_completed"]:
        return 0
    if observed["outcome"] == "succeeded":
        return 2
    return 0


def _start_heartbeats(base: str, robot_id: str, session_id: str) -> threading.Event:
    """Renew the lease while a goal is in flight. Stop does not cancel the goal."""
    stop = threading.Event()

    def _loop() -> None:
        while not stop.wait(5.0):
            try:
                _request(
                    "POST",
                    f"{base}/api/agent/heartbeat",
                    {"robot_id": robot_id, "session_id": session_id},
                )
            except GazeboNav2Unavailable:
                return

    threading.Thread(target=_loop, name="fms-lease-heartbeat", daemon=True).start()
    return stop


def _post_event(base, task, session_id, robot_id, *, kind, outcome, pose_trustworthy, nav2_action_goal_id, pose=None):
    body = {
        "robot_id": robot_id,
        "session_id": session_id,
        "task_id": task["task_id"],
        "goal_id": task["goal_id"],
        "execution_id": task["execution_id"],
        "owner_epoch": task["owner_epoch"],
        "lease_id": task["lease_id"],
        "kind": kind,
        "outcome": outcome,
        "pose_trustworthy": pose_trustworthy,
        "nav2_action_goal_id": nav2_action_goal_id,
    }
    if pose is not None:
        body["pose_x"] = pose["x"]
        body["pose_y"] = pose["y"]
        body["pose_yaw"] = pose.get("yaw", 0)
    return _request("POST", f"{base}/api/agent/goal-event", body)


def _request(method: str, url: str, payload: dict | None):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={"content-type": "application/json"} if data is not None else {},
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise GazeboNav2Unavailable(f"FMS returned {exc.code}: {detail}") from exc
    return json.loads(raw) if raw else {}


if __name__ == "__main__":
    sys.exit(main())
