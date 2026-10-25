#!/usr/bin/env python3
"""Closed-loop Gazebo validation of the HUNTeR replica.

Start a simulation first, then run the matching suite:
  ros2 launch hunter_gazebo sim.launch.py headless:=true rviz:=false sensors:=false
  python3 tools/sim_tests/drive_test.py --suite flat --report flat.json

  ros2 launch hunter_gazebo sim.launch.py headless:=true rviz:=false sensors:=false world:=offroad.sdf
  python3 tools/sim_tests/drive_test.py --suite offroad --report offroad.json

Every scenario prints measured vs expected values; expectations come from the
kinematics module and vehicle.yaml, so the test checks the model against its
own design numbers.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "hunter_control"))
from hunter_control.kinematics import FourWheelSteeringKinematics  # noqa: E402
from hunter_control.vehicle_config import geometry_from_config, load_vehicle  # noqa: E402

CFG = load_vehicle(os.path.join(ROOT, "hunter_description", "config", "vehicle.yaml"))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rp_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def fit_circle(pts):
    """Algebraic (Kasa) least-squares circle fit -> (cx, cy, r)."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); syy = sum(p[1] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    sxxx = sum(p[0] ** 3 for p in pts); syyy = sum(p[1] ** 3 for p in pts)
    sxyy = sum(p[0] * p[1] ** 2 for p in pts); sxxy = sum(p[0] ** 2 * p[1] for p in pts)
    m = [[sxx, sxy, sx, -(sxxx + sxyy)], [sxy, syy, sy, -(sxxy + syyy)], [sx, sy, n, -(sxx + syy)]]
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
        super().__init__("hunter_drive_test", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.gt = None
        self.odom = None
        self.js = None
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt, 50)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 50)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(String, "/four_ws_controller/steering_mode", 10)
        self.trace = []
        self.jtrace = []

    def _gt(self, m):
        self.gt = m
        q = m.pose.pose.orientation
        r, p = rp_of(q)
        self.trace.append({"t": self.now(), "x": m.pose.pose.position.x, "y": m.pose.pose.position.y,
                           "z": m.pose.pose.position.z, "yaw": yaw_of(q), "roll": r, "pitch": p,
                           "vx": m.twist.twist.linear.x, "vy": m.twist.twist.linear.y})

    def _js(self, m):
        self.js = m
        self.jtrace.append(dict(zip(m.name, m.position)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, sim_seconds, cmd=None, until=None):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < sim_seconds:
            if cmd is not None:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = cmd
                self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            if until is not None and self.gt is not None and until(self):
                return True
        return False

    def wait_ready(self, timeout=180.0):
        start = time.time()
        while rclpy.ok() and (self.gt is None or self.js is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise RuntimeError("simulation topics not available")
        self.spin_for(3.0)

    def joint(self, name):
        return self.js.position[self.js.name.index(name)]

    def set_mode(self, mode):
        for _ in range(5):
            self.mode_pub.publish(String(data=mode))
            self.spin_for(0.1)

    def stop(self, seconds=4.0):
        self.spin_for(seconds, (0.0, 0.0, 0.0))


def unwrap_rate(tr):
    if len(tr) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(tr, tr[1:]):
        d = b["yaw"] - a["yaw"]
        total += math.atan2(math.sin(d), math.cos(d))
    return total / (tr[-1]["t"] - tr[0]["t"])


# ---------------------------------------------------------------- scenarios
def scenario_static(t):
    t.stop(6.0)
    return {"front_heave_m": t.joint("front_axle_heave_joint"), "rear_heave_m": t.joint("rear_axle_heave_joint"),
            "front_roll_deg": math.degrees(t.joint("front_axle_roll_joint")),
            "rear_roll_deg": math.degrees(t.joint("rear_axle_roll_joint")),
            "base_footprint_z_m": t.gt.pose.pose.position.z,
            "expected": "heave ~0 at kerb mass (spring references set for the 2020 kerb load)"}


def scenario_straight(t, v=8.0, seconds=10.0):
    t.set_mode("counter")
    t.spin_for(6.0, (v, 0.0, 0.0))
    t.trace.clear()
    o0, g0 = t.odom, t.gt
    t.spin_for(seconds, (v, 0.0, 0.0))
    speed = sum(p["vx"] for p in t.trace) / len(t.trace)
    dgt = math.hypot(t.gt.pose.pose.position.x - g0.pose.pose.position.x, t.gt.pose.pose.position.y - g0.pose.pose.position.y)
    dod = math.hypot(t.odom.pose.pose.position.x - o0.pose.pose.position.x,
                     t.odom.pose.pose.position.y - o0.pose.pose.position.y)
    lateral = abs(t.trace[-1]["y"] - t.trace[0]["y"])
    t.stop(8.0)
    return {"cmd_v": v, "mean_speed_gt": speed, "distance_gt": dgt, "distance_odom": dod,
            "odom_error_pct": 100 * (dod - dgt) / dgt, "lateral_drift_m": lateral}


def circle(t, mode, v, wz, seconds, kin):
    vx, vy, wzc = kin.constrain(mode, v, 0.0, wz)
    vx, vy, wzc = kin.limit(vx, vy, wzc)
    exp_r = math.hypot(vx, vy) / abs(wzc) if wzc else None
    t.set_mode(mode)
    t.spin_for(8.0, (v, 0.0, wz))
    t.trace.clear()
    t.spin_for(seconds, (v, 0.0, wz))
    cx, cy, r = fit_circle([(p["x"], p["y"]) for p in t.trace])
    yaw_rate = unwrap_rate(t.trace)
    speed = sum(math.hypot(p["vx"], p["vy"]) for p in t.trace) / len(t.trace)
    t.stop(6.0)
    return {"mode": mode, "cmd_v": v, "cmd_wz": wz, "realised_wz": wzc, "expected_radius_base": exp_r,
            "fit_radius_base": r, "radius_error_pct": 100 * (r - exp_r) / exp_r if exp_r else None,
            "mean_speed": speed, "yaw_rate": yaw_rate, "expected_yaw_rate": wzc}


def scenario_crab(t, vx=1.5, vy=0.6, seconds=10.0):
    t.set_mode("crab")
    t.spin_for(6.0, (vx, vy, 0.0))
    t.trace.clear()
    t.spin_for(seconds, (vx, vy, 0.0))
    a, b = t.trace[0], t.trace[-1]
    track = math.atan2(b["y"] - a["y"], b["x"] - a["x"]) - a["yaw"]
    t.stop(6.0)
    t.set_mode("counter")
    return {"cmd_track_angle_deg": math.degrees(math.atan2(vy, vx)), "gt_track_angle_deg": math.degrees(track),
            "heading_change_deg": math.degrees(b["yaw"] - a["yaw"])}


def scenario_course(t, v=2.0):
    """Off-road course along +x: twister, washboard, step, 30 % grade."""
    t.set_mode("counter")
    t.trace.clear(); t.jtrace.clear()
    ok = t.spin_for(120.0, (v, 0.0, 0.0), until=lambda s: s.gt.pose.pose.position.x > 66.0)

    def seg(x0, x1):
        tr = [p for p in t.trace if x0 <= p["x"] <= x1]
        return tr

    out = {"completed": ok, "final_x": t.gt.pose.pose.position.x}
    tw = seg(9, 23)
    jt = t.jtrace
    out["twister"] = {
        "max_front_axle_roll_deg": max(abs(math.degrees(j.get("front_axle_roll_joint", 0))) for j in jt),
        "max_rear_axle_roll_deg": max(abs(math.degrees(j.get("rear_axle_roll_joint", 0))) for j in jt),
        "max_body_roll_deg": max(abs(math.degrees(p["roll"])) for p in tw) if tw else None,
        "heave_range_m": [min(j.get("front_axle_heave_joint", 0) for j in jt), max(j.get("front_axle_heave_joint", 0) for j in jt)],
        "articulation_limit_deg": CFG["suspension"]["articulation_limit_deg"],
    }
    gr = seg(56, 61)
    out["grade"] = {"mean_pitch_deg": sum(math.degrees(p["pitch"]) for p in gr) / len(gr) if gr else None,
                    "mean_speed": sum(math.hypot(p["vx"], p["vy"]) for p in gr) / len(gr) if gr else None,
                    "expected_pitch_deg": -math.degrees(math.atan(0.3))}
    st = seg(43.5, 48.5)
    out["step"] = {"passed": t.gt.pose.pose.position.x > 49.0,
                   "max_pitch_deg": max(abs(math.degrees(p["pitch"])) for p in st) if st else None}
    t.stop(6.0)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="flat", choices=["flat", "offroad"])
    ap.add_argument("--report", default="")
    a = ap.parse_args()
    kin = FourWheelSteeringKinematics(geometry_from_config(CFG))
    rclpy.init()
    t = Tester()
    t.wait_ready()
    out = {}
    if a.suite == "flat":
        out["static"] = scenario_static(t)
        out["straight"] = scenario_straight(t)
        out["counter"] = circle(t, "counter", 2.0, 0.25, 20.0, kin)
        out["front"] = circle(t, "front", 2.0, 0.15, 25.0, kin)
        out["crab"] = scenario_crab(t)
        out["minturn_counter"] = circle(t, "counter", 1.5, 2.0, 25.0, kin)
        out["minturn_front"] = circle(t, "front", 1.5, 2.0, 35.0, kin)
        out["turning_radius_outer_tyre_design"] = {m: kin.turning_radius(m) for m in ("counter", "front")}
    else:
        out["course"] = scenario_course(t)
    print(json.dumps(out, indent=2))
    if a.report:
        with open(a.report, "w") as f:
            json.dump(out, f, indent=2)
    t.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
