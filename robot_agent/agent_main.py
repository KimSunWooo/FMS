"""Per-robot agent process.

The process registers with the FMS, observes a map pose, and claims at most
one Nav2 send. A goal that is already active on the action server blocks a
new send. Preflight failure exits before any goal is claimed.
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

from robot_agent.gazebo_nav2_client import (
    GazeboNav2Client,
    GazeboNav2Unavailable,
    SimNavigation,
    goal_statuses_block_send,
)
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
        return _run_one_assignment(args.fms_url.rstrip("/"), args.robot_id, summary)
    except GazeboNav2Unavailable as exc:
        # A failed FMS write must not become a completed task. Keep the fact
        # that a goal was already sent when that is already known.
        summary["error"] = str(exc)
        summary["task_completed"] = False
        summary["nav2_goal_sent"] = bool(summary.get("nav2_goal_sent"))
        print(json.dumps(summary, ensure_ascii=False))
        return 2


def note_fms_failure(summary: dict, exc: Exception) -> dict:
    summary["error"] = str(exc)
    summary["task_completed"] = False
    summary["nav2_goal_sent"] = bool(summary.get("nav2_goal_sent"))
    return summary


def _run_one_assignment(base: str, robot_id: str, summary: dict) -> int:
    opened = _request("POST", f"{base}/api/agent/session", {"robot_id": robot_id})
    session_id = opened["session_id"]
    summary["session_id"] = session_id
    try:
        return _run_attached(base, robot_id, session_id, summary)
    finally:
        try:
            _request(
                "POST",
                f"{base}/api/agent/stop",
                {"robot_id": robot_id, "session_id": session_id},
            )
        except GazeboNav2Unavailable:
            pass


def _run_attached(base: str, robot_id: str, session_id: str, summary: dict) -> int:
    heartbeats = _start_heartbeats(base, robot_id, session_id)
    try:
        with SimNavigation(use_sim_time=True) as nav:
            pose = nav.observe_pose("map")
            summary["start_pose"] = pose
            if not isinstance(pose, dict) or pose.get("frame_id") != "map":
                summary["error"] = "map pose was not observed; no Nav2 goal was sent"
                print(json.dumps(summary, ensure_ascii=False))
                return 2
            _request(
                "POST",
                f"{base}/api/agent/pose",
                {
                    "robot_id": robot_id,
                    "session_id": session_id,
                    "pose_x": pose["x"],
                    "pose_y": pose["y"],
                    "pose_yaw": pose.get("yaw", 0),
                    "frame_id": "map",
                },
            )
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
            statuses = nav.goal_statuses("/navigate_to_pose")
            summary["nav2_status_list"] = statuses
            if goal_statuses_block_send(statuses):
                summary["error"] = (
                    "an active Nav2 goal is already running or its status was not observed; "
                    "no new goal was sent"
                )
                print(json.dumps(summary, ensure_ascii=False))
                return 2
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
                summary["claim_reason"] = claim.get("reason")
                print(json.dumps(summary, ensure_ascii=False))
                return 0
            destination = claim["destination"]
            frame_id = destination.get("frame_id") or "map"
            cancel_event = threading.Event()
            watcher_stop = threading.Event()
            threading.Thread(
                target=_watch_cancel,
                args=(base, claim["task_id"], cancel_event, watcher_stop),
                name="fms-cancel-watch",
                daemon=True,
            ).start()
            try:
                observed = nav.send_goal(
                    goal_id=claim["goal_id"],
                    x=destination["x"],
                    y=destination["y"],
                    yaw=destination["yaw"],
                    frame_id=frame_id,
                    action_name="/navigate_to_pose",
                    cancel_event=cancel_event,
                )
            except GazeboNav2Unavailable:
                _post_event(
                    base,
                    task,
                    session_id,
                    robot_id,
                    kind="disconnected",
                    outcome="disconnected",
                    pose_trustworthy=False,
                    nav2_action_goal_id=None,
                    absent_reason="nav2_client_lost_before_result",
                )
                summary["error"] = "Nav2 result was not observed; FMS was told disconnected, not completed"
                print(json.dumps(summary, ensure_ascii=False))
                return 2
            finally:
                watcher_stop.set()
        return _publish_observation(base, task, session_id, robot_id, claim, frame_id, observed, summary)
    finally:
        heartbeats.set()


def _publish_observation(base, task, session_id, robot_id, claim, frame_id, observed, summary) -> int:
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
    summary["nav2_action_goal_id_absent_reason"] = observed.get("nav2_action_goal_id_absent_reason")
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
            absent_reason=observed.get("nav2_action_goal_id_absent_reason"),
        )
    posted = _post_event(
        base,
        task,
        session_id,
        robot_id,
        kind="result",
        outcome=observed["outcome"],
        pose_trustworthy=pose_ok,
        nav2_action_goal_id=observed.get("nav2_action_goal_id"),
        absent_reason=observed.get("nav2_action_goal_id_absent_reason"),
        pose=pose if pose_ok else None,
    )
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
    if observed["outcome"] == "succeeded" or posted.get("status") == "RECOVERY_REQUIRED":
        return 2
    return 0


def _start_heartbeats(base: str, robot_id: str, session_id: str) -> threading.Event:
    """Renew the lease from session start. Stopping the thread does not cancel a goal."""
    stop = threading.Event()

    def _loop() -> None:
        while not stop.wait(3.0):
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


def _watch_cancel(base: str, task_id: str, cancel_event: threading.Event, stop: threading.Event) -> None:
    while not stop.wait(0.5):
        try:
            body = _request("GET", f"{base}/api/tasks/{task_id}", None)
        except GazeboNav2Unavailable:
            continue
        if body.get("status") == "CANCEL_REQUESTED":
            cancel_event.set()
            return


def _post_event(
    base,
    task,
    session_id,
    robot_id,
    *,
    kind,
    outcome,
    pose_trustworthy,
    nav2_action_goal_id,
    absent_reason=None,
    pose=None,
):
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
        "nav2_action_goal_id_absent_reason": absent_reason,
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
