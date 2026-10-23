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
    assignment = _request(
        "GET",
        f"{base}/api/agent/assignment?robot_id={robot_id}&session_id={session_id}",
        None,
    )
    task = assignment.get("assignment")
    if not task or not assignment.get("send_navigate_to_pose"):
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
    try:
        observed = client.send_goal(
            robot_id=robot_id,
            goal_id=claim["goal_id"],
            x=destination["x"],
            y=destination["y"],
            yaw=destination["yaw"],
            frame_id=destination.get("frame_id") or "map",
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
        )
        summary["error"] = "Nav2 result was not observed; FMS was told disconnected, not completed"
        print(json.dumps(summary, ensure_ascii=False))
        return 2
    summary["nav2_goal_sent"] = True
    summary["nav2_outcome"] = observed["outcome"]
    summary["nav2_action_goal_id"] = observed.get("nav2_action_goal_id")
    summary["fms_goal_id"] = claim["goal_id"]
    summary["fms_task_id"] = claim["task_id"]
    summary["pose_observed"] = False
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
    # The destination is not an observation. Without a measured pose the FMS
    # must not treat a Nav2 status as arrival.
    posted = _post_event(
        base,
        task,
        session_id,
        robot_id,
        kind="result",
        outcome=observed["outcome"],
        pose_trustworthy=False,
        nav2_action_goal_id=observed.get("nav2_action_goal_id"),
    )
    summary["fms_status"] = posted.get("status")
    summary["task_completed"] = posted.get("status") == "COMPLETED"
    if observed["outcome"] == "succeeded" and not summary["task_completed"]:
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
