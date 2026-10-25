#!/usr/bin/env python3
"""Regenerate docs/images/*.png (needs ROS 2 xacro on PATH and the node deps in
tools/render, see render.mjs)."""

import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "hunter_control"))

from hunter_control.kinematics import FourWheelSteeringKinematics  # noqa: E402
from hunter_control.vehicle_config import geometry_from_config, load_vehicle  # noqa: E402

cfg = load_vehicle(os.path.join(ROOT, "hunter_description", "config", "vehicle.yaml"))
kin = FourWheelSteeringKinematics(geometry_from_config(cfg))
STEER = ("front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint")


def steer_for(mode, vx, vy, wz):
    c = kin.inverse(mode, vx, vy, wz)
    return {n: a for n, a in zip(STEER, c.steer)}


ART = math.radians(cfg["suspension"]["articulation_limit_deg"]) * 0.95
XRAY = {"hide": "hull,lamps_"}
XRAY_NOGROUND = {"hide": "hull,lamps_", "ground": "0"}
SCENES = [
    # name, xacro args, joints, view "az,el,dist,tx,ty,tz"
    ("hunter_iso", "", {}, "35,18,10.5,0,0,0.9", {}),
    ("hunter_iso_rear", "", {}, "215,18,10.5,0,0,0.9", {}),
    ("hunter_side", "", {}, "90,2,11,0,0,0.8", {}),
    ("hunter_front", "", {}, "0,4,8.5,0,0,0.8", {}),
    ("hunter_running_gear", "sensors:=false", {}, "30,22,7.5,0,0,0.55", XRAY),
    ("hunter_articulation", "sensors:=false", {"front_axle_roll_joint": ART, "rear_axle_roll_joint": -ART},
     "8,8,8.0,0,0,0.6", XRAY_NOGROUND),
    ("hunter_4ws_counter", "sensors:=false", steer_for("counter", 1.0, 0.0, 5.0), "90,89,8.5,0,0,0.5", XRAY),
    ("hunter_4ws_crab", "sensors:=false", steer_for("crab", 1.0, 0.6, 0.0), "90,89,8.5,0,0,0.5", XRAY),
    ("hunter_2ws_front", "sensors:=false", steer_for("front", 1.0, 0.0, 5.0), "90,89,8.5,0,0,0.5", XRAY),
]


def main():
    out = os.path.join(ROOT, "docs", "images")
    os.makedirs(out, exist_ok=True)
    xacro = os.path.join(ROOT, "hunter_description", "urdf", "hunter.urdf.xacro")
    with tempfile.TemporaryDirectory() as tmp:
        views = []
        for name, args, joints, view, opts in SCENES:
            urdf = os.path.join(tmp, f"{name}.urdf")
            raw = subprocess.check_output(["xacro", xacro] + args.split(), text=True)
            open(urdf + ".raw", "w").write(raw)
            js = ",".join(f"{k}:{v}" for k, v in joints.items())
            subprocess.check_call([sys.executable, os.path.join(HERE, "add_linkage_visuals.py"), urdf + ".raw", urdf,
                                   "--joints", js])
            views.append((urdf, dict({"name": name, "view": view, "joints": js}, **opts)))
        for urdf, v in views:
            vf = os.path.join(tmp, v["name"] + ".json")
            json.dump([v], open(vf, "w"))
            subprocess.check_call(["node", os.path.join(HERE, "render.mjs"), urdf,
                                   os.path.join(ROOT, "hunter_description"), out, vf])


if __name__ == "__main__":
    main()
