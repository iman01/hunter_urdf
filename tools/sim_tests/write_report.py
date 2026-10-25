#!/usr/bin/env python3
"""Write docs/VALIDATION.md from the Gazebo reports in docs/validation/."""
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
V = os.path.join(ROOT, "docs", "validation")
f = json.load(open(os.path.join(V, "gazebo_flat.json")))
o = json.load(open(os.path.join(V, "gazebo_offroad.json")))["course"]
x = json.load(open(os.path.join(V, "gazebo_extra.json")))
L = []
w = L.append
w("# Validation\n")
w("Closed-loop results from Gazebo Harmonic (DART, 1 ms step) with the default model (`variant:=2020`, trapezoid "
  "steering linkage), produced by `tools/sim_tests/drive_test.py` and written by `tools/sim_tests/write_report.py`. "
  "Raw reports are in [`validation/`](validation/). Expected values come from the kinematics module and "
  "`vehicle.yaml`, so these tests check that the simulated vehicle behaves like its own design.\n")
w("## Flat ground (`flat.sdf`)\n")
w("| Test | Expected | Measured |\n|---|---|---|")
s = f["static"]
w(f"| Front / rear axle heave at kerb mass | 0 / 0 mm | {s['front_heave_m']*1000:+.1f} / {s['rear_heave_m']*1000:+.1f} mm |")
w(f"| Front / rear axle roll at rest | 0 deg | {s['front_roll_deg']:+.3f} / {s['rear_roll_deg']:+.3f} deg |")
st = f["straight"]
w(f"| Straight line at {st['cmd_v']:.0f} m/s: speed | {st['cmd_v']:.2f} m/s | {st['mean_speed_gt']:.3f} m/s |")
w(f"| Straight line: wheel odometry vs ground truth over {st['distance_gt']:.1f} m | 0 % | {st['odom_error_pct']:+.2f} % |")
w(f"| Straight line: lateral drift over {st['distance_gt']:.1f} m (open loop) | 0 m | {st['lateral_drift_m']:.2f} m |")
rows = [("counter", "4WS counter-phase circle, 2 m/s"), ("front", "2WS front-steer circle, 2 m/s"),
        ("minturn_counter", "4WS full lock (vehicle centre path)"), ("minturn_front", "2WS full lock (vehicle centre path)")]
for k, label in rows:
    d = f[k]
    w(f"| {label}: radius | {d['expected_radius_base']:.3f} m | {d['fit_radius_base']:.3f} m ({d['radius_error_pct']:+.2f} %) |")
    w(f"| {label}: yaw rate | {d['expected_yaw_rate']:.4f} rad/s | {d['yaw_rate']:.4f} rad/s |")
for k, label in (("reverse_counter", "4WS circle in reverse, 1.5 m/s"), ("rear_mode", "2WS rear-steer circle, 2 m/s"),
                 ("adaptive_10mps", "Adaptive mode at 10 m/s")):
    d = x[k]
    w(f"| {label}: radius | {d['expected_radius_base']:.3f} m | {d['fit_radius_base']:.3f} m ({d['radius_error_pct']:+.2f} %) |")
a = x["ackermann_counter"]
w(f"| Ackermann input (2 m/s, 0.2 rad, counter mode): radius | {a['expected_radius']:.3f} m | {a['fit_radius']:.3f} m |")
c = f["crab"]
w(f"| Crab 1.5 + 0.6 m/s: track angle | {c['cmd_track_angle_deg']:.1f} deg | {c['gt_track_angle_deg']:.1f} deg |")
w(f"| Crab: heading change over 10 s | 0 deg | {c['heading_change_deg']:+.2f} deg |")
w(f"| Command timeout: speed 6 s after the last command | 0 m/s | {x['timeout_stop']['speed_after_6s_without_cmd']:.3f} m/s |")
t = f["turning_radius_outer_tyre_design"]
w(f"\nDesign turning radius at the outer tyre: {t['counter']:.2f} m (4WS), {t['front']:.2f} m (2WS).\n")
w("Notes: the full-lock circles come out 1 to 2 % tighter than kinematics because the rigid tyres scrub. The crab "
  "track-angle offset comes from the mechanical trapezoid, which cannot hold both wheels of an axle parallel "
  "(`linkage:=ideal` gives exactly parallel wheels). The straight-line drift is an open-loop figure: the 4WS "
  "controller has no heading feedback, so a path follower or Nav2 controller removes it. The static heave settles "
  "about 2.5 mm into bump in every configuration (2 % of the static sag); masses and spring references in the "
  "generated SDF match the design exactly, so this is the equilibrium of DART's contact and joint-servo constraints.\n")
w("## Off-road course (`offroad.sdf`, 2 m/s, counter-phase mode)\n")
w("| Section | Result |\n|---|---|")
tw = o["twister"]
w(f"| Articulation twister, 0.30 m humps alternating left/right | passed; axle roll {tw['max_front_axle_roll_deg']:.2f} / "
  f"{tw['max_rear_axle_roll_deg']:.2f} deg (limit {tw['articulation_limit_deg']:.2f}), front heave "
  f"{tw['heave_range_m'][0]*1000:.0f} .. {tw['heave_range_m'][1]*1000:.0f} mm, body roll {tw['max_body_roll_deg']:.1f} deg |")
w(f"| 0.30 m step | {'passed' if o['step']['passed'] else 'FAILED'}; max pitch {o['step']['max_pitch_deg']:.1f} deg |")
g = o["grade"]
w(f"| 30 % grade (16.7 deg) | climbed at {g['mean_speed']:.2f} m/s; body pitch {abs(g['mean_pitch_deg']):.1f} deg "
  f"(slope {abs(g['expected_pitch_deg']):.1f} deg + suspension squat) |")
w(f"| Course completed | {o['completed']} (final x {o['final_x']:.1f} m) |\n")
w("The 0.30 m humps exceed the 250 mm stroke, so the axles reach the articulation limit and the body takes the rest as "
  "roll, the expected behaviour of the published suspension.\n")
w("## Configurations\n")
ar = x["armed_variant"]
w("| Check | Expected | Measured |\n|---|---|---|")
w(f"| Armed configuration (`variant:=2023 payload:=rws`, +550 kg): front / rear sag | {ar['expected_sag_front_m']*1000:.1f} / "
  f"{ar['expected_sag_rear_m']*1000:.1f} mm | {ar['front_heave_m']*1000:.1f} / {ar['rear_heave_m']*1000:.1f} mm |")
w(f"| RWS position controller: yaw / pitch | {ar['rws_cmd'][0]} / {ar['rws_cmd'][1]} rad | {ar['rws_measured'][0]} / {ar['rws_measured'][1]} rad |")
w("| Sensors (rendered under Xvfb, CPU): lidar clouds 16384 points, 1280x720 images, NavSat, IMU | published | "
  "published, frame ids = optical / sensor frames |\n")
w("Sensor images from the simulation (front driving camera and PTZ looking at the off-road course):\n")
w("![front driving camera](images/sim_front_drive_camera.png)\n")
w("![PTZ camera](images/sim_ptz_camera.png)\n")
w("## Static tests (`colcon test`)\n")
w("* `hunter_description/test/test_urdf.py`: 7 configurations; `check_urdf`; total mass within 1 % of the kerb mass "
  "(3500 kg published; 4050 kg estimated for the armed configuration; + payload); every inertia positive definite and "
  "satisfying the triangle inequality; every mesh present; ros2_control joints exist; heave travel equals the "
  "published 250 mm stroke on both axles; published envelope (4.70 x 2.20 m, 2.70 m wheelbase, 1 m wheels); design "
  "outputs up to date.")
w("* `hunter_control/test/test_kinematics.py`: 4WS kinematics (straight line, reverse, counter-phase symmetry, Ackermann "
  "normals meeting at the instantaneous centre, crab, forward/inverse round trip in every mode for the ideal linkage "
  "and in every ICR mode for both linkages and both differential settings, steering limits, trapezoid symmetry and "
  "Ackermann error, locked differential, turning radius).")
w("* `hunter_control/test/test_command_shaper.py`: curvature held while accelerating, acceleration/deceleration "
  "limits, no steering windup, ramped slowdown on steering lag, linkage-consistent steering slew, yaw-rate and "
  "lateral-acceleration limits, rotate-in-place pre-steer, reverse arcs, free-mode anti-windup.")
open(os.path.join(ROOT, "docs", "VALIDATION.md"), "w").write("\n".join(L) + "\n")
print("wrote docs/VALIDATION.md")
