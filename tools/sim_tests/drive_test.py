#!/usr/bin/env python3
"""Closed-loop Gazebo validation of the HUNTeR replica.

Run with a simulation already up (ros2 launch hunter_gazebo sim.launch.py headless:=true):
  python3 tools/sim_tests/drive_test.py --scenario all --report /tmp/sim_report.json

Scenarios
  static   : settle, report suspension heave/roll at rest and ride height
  straight : drive straight, compare commanded vs ground-truth speed, odometry drift
  counter  : 4WS counter-phase circle; fitted radius vs commanded v/wz
  front    : 2WS front-steer circle
  crab     : in-phase crab; heading must not change, track angle = atan(vy/vx)
  minturn  : full-lock counter-phase circle -> minimum turning radius
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fit_circle(pts):
    """Algebraic (Kasa) least-squares circle fit -> (cx, cy, r)."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); syy = sum(p[1] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    sxxx = sum(p[0] ** 3 for p in pts); syyy = sum(p[1] ** 3 for p in pts)
    sxyy = sum(p[0] * p[1] ** 2 for p in pts); sxxy = sum(p[0] ** 2 * p[1] for p in pts)
    a = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    b = [-(sxxx + sxyy), -(sxxy + syyy), -(sxx + syy)]
    # Solve 3x3
    import copy
    m = copy.deepcopy(a)
    for i in range(3):
        m[i].append(b[i])
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, 3):
            f = m[r][c] / m[c][c]
            for k in range(c, 4):
                m[r][k] -= f * m[c][k]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][k] * x[k] for k in range(r + 1, 3))) / m[r][r]
    cx, cy = -x[0] / 2.0, -x[1] / 2.0
    return cx, cy, math.sqrt(max(cx * cx + cy * cy - x[2], 0.0))


class Tester(Node):
    def __init__(self):
        super().__init__("hunter_drive_test", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.gt = None
        self.odom = None
        self.js = None
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt, 50)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 50)
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", m), 50)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/four_ws_controller/steering_mode", 10)
        self.trace = []

    def _gt(self, m):
        self.gt = m
        self.trace.append((self.now(), m.pose.pose.position.x, m.pose.pose.position.y,
                           m.pose.pose.position.z, yaw_of(m.pose.pose.orientation),
                           m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_seconds, cmd=None):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sim_seconds:
            if cmd is not None:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = cmd
                self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=120.0):
        start = time.time()
        while rclpy.ok() and (self.gt is None or self.js is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise RuntimeError("simulation topics not available")

    def joint(self, name):
        i = self.js.name.index(name)
        return self.js.position[i]

    def set_mode(self, mode):
        for _ in range(5):
            self.mode_pub.publish(String(data=mode))
            self.spin_for(0.1)

    def stop(self, seconds=4.0):
        self.spin_for(seconds, (0.0, 0.0, 0.0))


def scenario_static(t):
    t.stop(6.0)
    return {
        "front_heave_m": t.joint("front_axle_heave_joint"),
        "rear_heave_m": t.joint("rear_axle_heave_joint"),
        "front_roll_rad": t.joint("front_axle_roll_joint"),
        "rear_roll_rad": t.joint("rear_axle_roll_joint"),
        "base_footprint_z_m": t.gt.pose.pose.position.z,
    }


def circle(t, mode, v, wz, seconds):
    t.set_mode(mode)
    t.spin_for(6.0, (v, 0.0, wz))  # settle into the turn
    t.trace.clear()
    t.spin_for(seconds, (v, 0.0, wz))
    pts = [(p[1], p[2]) for p in t.trace]
    cx, cy, r = fit_circle(pts)
    yaw_rate = sum(p[7] for p in t.trace) / len(t.trace)
    speed = sum(math.hypot(p[5], p[6]) for p in t.trace) / len(t.trace)
    t.stop()
    return {"mode": mode, "cmd_v": v, "cmd_wz": wz, "cmd_radius": abs(v / wz) if wz else None,
            "fit_radius_base_footprint": r, "mean_speed": speed, "mean_yaw_rate": yaw_rate}


def scenario_straight(t, v=5.0, seconds=12.0):
    t.set_mode("counter")
    t.spin_for(5.0, (v, 0.0, 0.0))
    t.trace.clear()
    o0 = t.odom
    g0 = t.gt
    t.spin_for(seconds, (v, 0.0, 0.0))
    speed = sum(p[5] for p in t.trace) / len(t.trace)
    dgt = math.hypot(t.gt.pose.pose.position.x - g0.pose.pose.position.x, t.gt.pose.pose.position.y - g0.pose.pose.position.y)
    dod = math.hypot(t.odom.pose.pose.position.x - o0.pose.pose.position.x, t.odom.pose.pose.position.y - o0.pose.pose.position.y) if o0 else None
    lateral = abs(t.trace[-1][2] - t.trace[0][2])
    t.stop(6.0)
    return {"cmd_v": v, "mean_speed_gt": speed, "distance_gt": dgt, "distance_odom": dod, "lateral_drift_m": lateral}


def scenario_crab(t, vx=1.5, vy=0.6, seconds=10.0):
    t.set_mode("crab")
    t.spin_for(6.0, (vx, vy, 0.0))
    t.trace.clear()
    t.spin_for(seconds, (vx, vy, 0.0))
    dx = t.trace[-1][1] - t.trace[0][1]
    dy = t.trace[-1][2] - t.trace[0][2]
    dyaw = t.trace[-1][4] - t.trace[0][4]
    t.stop(6.0)
    t.set_mode("counter")
    return {"cmd_track_angle_deg": math.degrees(math.atan2(vy, vx)),
            "gt_track_angle_deg": math.degrees(math.atan2(dy, dx) - t.trace[0][4]),
            "heading_change_deg": math.degrees(dyaw)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="all")
    ap.add_argument("--report", default="")
    a = ap.parse_args()
    rclpy.init()
    t = Tester()
    t.wait_ready()
    out = {}
    sc = a.scenario
    if sc in ("all", "static"):
        out["static"] = scenario_static(t)
    if sc in ("all", "straight"):
        out["straight"] = scenario_straight(t)
    if sc in ("all", "counter"):
        out["counter"] = circle(t, "counter", 2.0, 0.25, 20.0)
    if sc in ("all", "front"):
        out["front"] = circle(t, "front", 2.0, 0.15, 25.0)
    if sc in ("all", "crab"):
        out["crab"] = scenario_crab(t)
    if sc in ("all", "minturn"):
        out["minturn_counter"] = circle(t, "counter", 1.5, 1.0, 25.0)
        out["minturn_front"] = circle(t, "front", 1.5, 1.0, 35.0)
    print(json.dumps(out, indent=2))
    if a.report:
        with open(a.report, "w") as f:
            json.dump(out, f, indent=2)
    t.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
