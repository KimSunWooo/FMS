#!/usr/bin/env python3
"""ROS Jazzy process: 0.05m Nav2 DriveOnHeading from fresh map TF, one robot.

No direct cmd_vel. Action cancellation requested on timeout or stdin 'cancel'.
stdout: newline-delimited JSON event records; ROS logs may appear between records.
"""
from __future__ import annotations

import json
import math
import select
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import DriveOnHeading
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from app.services.velocity_chain import classify_velocity, goal_status_name

DISTANCE = 0.05
MAX_SECONDS = 18.0


def emit(state: str, **data):
    print(json.dumps({'state': state, **data}, allow_nan=False), flush=True)


def pose_from_tf(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return (float(t.x), float(t.y), yaw)


def read_fresh_pose(node, buffer, deadline):
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.12)
        try:
            transform = buffer.lookup_transform('map', 'base_footprint', Time())
            clock_stamp = Time.from_msg(transform.header.stamp)
            age = (node.get_clock().now() - clock_stamp).nanoseconds / 1e9
            if -1.0 <= age <= 2.0:
                result = pose_from_tf(transform)
                if all(math.isfinite(v) for v in result):
                    return result
        except (TransformException, ValueError, TypeError):
            continue
    return None


def future_wait(node, future, deadline, *, poll_input=False):
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.10)
        if future.done():
            return True
        if poll_input and cancel_requested():
            return False
    return False


def cancel_requested():
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if readable:
        return sys.stdin.readline().strip().lower() == 'cancel'
    return False


class MotionWatch:
    """Record nav, smoothed, and final cmd_vel during one goal. Publish nothing."""

    def __init__(self, node: Node):
        self.node = node
        self.max_nav = None
        self.max_smoothed = None
        self.max_cmd = None
        self.cmd_seen = False
        self.odom_start = None
        self.odom_last = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(Twist, '/cmd_vel_nav', self._nav, qos)
        node.create_subscription(Twist, '/cmd_vel_smoothed', self._smoothed, qos)
        node.create_subscription(Twist, '/cmd_vel', self._cmd, qos)
        node.create_subscription(Odometry, '/odom', self._odom, qos)

    def _keep(self, previous, value):
        return value if previous is None else max(previous, value)

    def _nav(self, message: Twist):
        self.max_nav = self._keep(self.max_nav, float(message.linear.x))

    def _smoothed(self, message: Twist):
        self.max_smoothed = self._keep(self.max_smoothed, float(message.linear.x))

    def _cmd(self, message: Twist):
        self.cmd_seen = True
        self.max_cmd = self._keep(self.max_cmd, float(message.linear.x))

    def _odom(self, message: Odometry):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        sample = {'x': float(pose.position.x), 'y': float(pose.position.y), 'yaw': yaw}
        if self.odom_start is None:
            self.odom_start = sample
        self.odom_last = sample

    def report(self) -> dict:
        forward = None
        if self.odom_start is not None and self.odom_last is not None:
            dx = self.odom_last['x'] - self.odom_start['x']
            dy = self.odom_last['y'] - self.odom_start['y']
            yaw = self.odom_start['yaw']
            forward = round(dx * math.cos(yaw) + dy * math.sin(yaw), 4)
        case = classify_velocity(
            nav_linear_x=self.max_nav,
            smoothed_linear_x=self.max_smoothed,
            cmd_linear_x=self.max_cmd,
            cmd_seen=self.cmd_seen,
        )
        return {
            'velocity_case': case,
            'cmd_vel_nav_max': self.max_nav,
            'cmd_vel_smoothed_max': self.max_smoothed,
            'cmd_vel_max': self.max_cmd,
            'cmd_vel_seen': self.cmd_seen,
            'odom_forward_m': forward,
            'odom_start': self.odom_start,
            'odom_final': self.odom_last,
        }


def run():
    rclpy.init()
    node = Node('fms_pinky1_5cm_test')
    watch = MotionWatch(node)
    buffer = Buffer(cache_time=Duration(seconds=5))
    listener = TransformListener(buffer, node, spin_thread=False)
    client = ActionClient(node, DriveOnHeading, '/drive_on_heading')
    goal_handle = None
    goal_finished = False
    try:
        start = read_fresh_pose(node, buffer, time.monotonic() + 7)
        if start is None:
            emit('failed', detail='fresh map->base_footprint TF unavailable')
            return 2
        sx, sy, yaw = start
        gx, gy = sx + DISTANCE * math.cos(yaw), sy + DISTANCE * math.sin(yaw)
        emit('pose_verified', start={'x': sx, 'y': sy, 'yaw': yaw},
             target={'x': gx, 'y': gy, 'yaw': yaw}, requested_distance_m=DISTANCE)
        if not client.wait_for_server(timeout_sec=6.0):
            emit('failed', detail='/drive_on_heading Action server unavailable')
            return 3
        goal = DriveOnHeading.Goal()
        goal.target.x = DISTANCE
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = 0.04  # 4 cm/s; bounded straight drive.
        goal.time_allowance = Duration(seconds=10.0).to_msg()
        last_feedback = 0.0

        def feedback_cb(message):
            nonlocal last_feedback
            if time.monotonic() - last_feedback >= 1.0:
                last_feedback = time.monotonic()
                remaining = float(message.feedback.distance_traveled)
                if math.isfinite(remaining):
                    emit('running', distance_traveled_m=remaining)

        future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        if not future_wait(node, future, time.monotonic() + 6.0):
            emit('failed', detail='Goal acceptance timed out or cancelled before acceptance')
            return 4
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            emit('rejected', detail='Nav2 rejected 5cm DriveOnHeading goal')
            return 5
        emit('accepted', detail='Nav2 accepted 5cm DriveOnHeading goal')
        result_future = goal_handle.get_result_async()
        end_by = time.monotonic() + MAX_SECONDS
        reason = None
        while rclpy.ok() and time.monotonic() < end_by:
            rclpy.spin_once(node, timeout_sec=0.12)
            if result_future.done():
                break
            if cancel_requested():
                reason = 'operator_cancelled'
                break
        if not result_future.done():
            reason = reason or 'timeout'
            emit('cancelling', detail=reason)
            cancel_future = goal_handle.cancel_goal_async()
            future_wait(node, cancel_future, time.monotonic() + 4.0)
            future_wait(node, result_future, time.monotonic() + 4.0)
        final_pose = read_fresh_pose(node, buffer, time.monotonic() + 3.0)
        delta_forward = None
        delta_lateral = None
        if final_pose is not None:
            dx = final_pose[0] - sx
            dy = final_pose[1] - sy
            delta_forward = round(dx * math.cos(yaw) + dy * math.sin(yaw), 4)
            delta_lateral = round(-dx * math.sin(yaw) + dy * math.cos(yaw), 4)
        goal_finished = result_future.done()
        result_status = result_future.result().status if result_future.done() else None
        trace = watch.report()
        status_name = goal_status_name(result_status)
        measured = {
            'actual_forward_m': delta_forward,
            'actual_lateral_m': delta_lateral,
            'map_forward_m': delta_forward,
            'goal_status': result_status,
            'goal_status_name': status_name,
            'start_pose': {'x': sx, 'y': sy, 'yaw': yaw},
            'final_pose': (
                {'x': final_pose[0], 'y': final_pose[1], 'yaw': final_pose[2]}
                if final_pose is not None else None),
            **trace,
        }
        case_text = ''
        if trace['velocity_case'] == 'A':
            case_text = ' Case A: /cmd_vel stayed at 0 while nav and smoothed were forward.'
        elif trace['velocity_case'] == 'B':
            case_text = ' Case B: /cmd_vel published no message while nav and smoothed were forward.'
        elif trace['velocity_case'] == 'C':
            case_text = ' Case C: /cmd_vel was forward; compare odometry with the map TF.'
        if reason:
            emit('cancelled' if reason == 'operator_cancelled' else 'timeout', detail=reason + case_text, **measured)
            return 6
        if result_status != GoalStatus.STATUS_SUCCEEDED:
            emit('failed', detail=f'Nav2 {status_name} (status {result_status}).{case_text}', **measured)
            return 7
        # Map TF alone is not the traveled distance. Odometry is required as well.
        odom_forward = trace['odom_forward_m']
        if delta_forward is None or odom_forward is None:
            emit('unverified', detail='Nav2 succeeded; map TF and odometry were not both available', **measured)
            return 8
        if abs(delta_forward - odom_forward) > 0.03:
            emit('measurement_disagreement',
                 detail='Map TF and odometry forward displacements differ by more than 3cm', **measured)
            return 8
        if delta_forward < 0.035 or odom_forward < 0.035 or abs(delta_lateral or 0) > 0.04:
            emit('no_verified_motion',
                 detail='Nav2 succeeded but 3.5cm forward movement was not confirmed by both map TF and odometry',
                 **measured)
            return 9
        if delta_forward > 0.10 or odom_forward > 0.10:
            emit('excess_motion', detail='More than 10cm of observed forward motion', **measured)
            return 10
        emit('completed',
             detail='Nav2 SUCCEEDED. Map TF and odometry both show 3.5cm to 10cm forward. Tolerance is 1.5cm under and 5cm over the 5cm goal.',
             **measured)
        return 0
    except Exception as exc:
        emit('failed', detail=f'{type(exc).__name__}: {str(exc)[:200]}')
        return 11
    finally:
        if goal_handle is not None and not goal_finished and rclpy.ok():
            # Never abandon a potentially active physical goal on a worker error.
            try:
                cancel_future = goal_handle.cancel_goal_async()
                future_wait(node, cancel_future, time.monotonic() + 4.0)
            except Exception:
                pass
        try:
            client.destroy()
            node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(run())
