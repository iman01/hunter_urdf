#!/usr/bin/env python3
"""Clearance check: hull meshes against the running gear over the steering and
suspension envelope.

Forward kinematics come from the generated URDF itself, so the check sees the
same frames as RViz/Gazebo. Every hull mesh is split into its convex pieces;
a tyre/axle vertex inside a hull piece (or a hull vertex inside a tyre) is a
clash. Wheel travel is limited to the per-wheel bump the arches are sized for.

Usage: python3 tools/check_clearance.py [urdf]    (default: runs xacro)
Exit code 1 on any clash.
"""

from __future__ import annotations

import itertools
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "hunter_description")
V = yaml.safe_load(open(os.path.join(PKG, "config", "vehicle.yaml")))
MESHES = os.path.join(PKG, "meshes", "visual")

HULL_FILES = ["hull.stl", "hull_black.stl", "hull_trim.stl", "hull_plates.stl", "hull_details.stl",
              "hull_steel.stl", "hull_shackles.stl", "hull_tiedowns.stl", "lamps_white.stl", "lamps_amber.stl", "lamps_red.stl"]
MOVING = {"tyre.stl", "rim.stl", "hub.stl", "axle_housing.stl", "axle_motor.stl", "steering_actuator.stl",
          "knuckle_left.stl", "knuckle_right.stl"}
MARGIN = 0.005          # a vertex this far inside a hull piece counts as a clash


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tf(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def axis_motion(jtype, axis, q):
    m = np.eye(4)
    a = np.asarray(axis, float)
    if jtype == "prismatic":
        m[:3, 3] = a * q
    elif jtype in ("revolute", "continuous"):
        m[:3, :3] = trimesh.transformations.rotation_matrix(q, a)[:3, :3]
    return m


def vec(e, attr, default):
    return [float(v) for v in e.get(attr, default).split()] if e is not None else [float(v) for v in default.split()]


def load_urdf(path):
    root = ET.parse(path).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        joints[j.find("child").get("link")] = dict(
            name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
            origin=tf(vec(o, "xyz", "0 0 0"), vec(o, "rpy", "0 0 0")),
            axis=vec(j.find("axis"), "xyz", "1 0 0"))
    visuals = []
    for link in root.findall("link"):
        for v in link.findall("visual"):
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            f = os.path.basename(mesh.get("filename"))
            o = v.find("origin")
            visuals.append((link.get("name"), f, tf(vec(o, "xyz", "0 0 0"), vec(o, "rpy", "0 0 0"))))
    return joints, visuals


def link_pose(link, joints, q, cache):
    if link in cache:
        return cache[link]
    if link not in joints:
        cache[link] = np.eye(4)
        return cache[link]
    j = joints[link]
    m = link_pose(j["parent"], joints, q, cache) @ j["origin"] @ axis_motion(j["type"], j["axis"], q.get(j["name"], 0.0))
    cache[link] = m
    return m


def convex_pieces(path):
    m = trimesh.load(path)
    return [c.convex_hull for c in m.split(only_watertight=False) if len(c.vertices) >= 4]


def inside(points, hull):
    """Points deeper than MARGIN inside a convex hull."""
    n = hull.face_normals
    d = np.einsum("ij,ij->i", n, hull.triangles[:, 0, :])
    s = points @ n.T - d                        # signed distance to every face plane (outward +)
    return np.all(s < -MARGIN, axis=1)


def main():
    if len(sys.argv) > 1:
        urdf = sys.argv[1]
    else:
        urdf = os.path.join(tempfile.mkdtemp(prefix="hunter_clearance_"), "hunter.urdf")
        out = subprocess.check_output(["xacro", os.path.join(PKG, "urdf", "hunter.urdf.xacro"),
                                       f"vehicle_file:={os.path.join(PKG, 'config', 'vehicle.yaml')}",
                                       "sensors:=false"], text=True)
        open(urdf, "w").write(out)
    joints, visuals = load_urdf(urdf)

    hull = []
    for f in HULL_FILES:
        if not os.path.exists(os.path.join(MESHES, f)):
            continue
        for p in convex_pieces(os.path.join(MESHES, f)):
            hull.append((f, p))
    hull_pts = np.vstack([p.vertices for _, p in hull])
    hull_bounds = [(p.bounds, f, p) for f, p in hull]
    meshes = {f: trimesh.load(os.path.join(MESHES, f)) for f in MOVING}
    tyre_hull = meshes["tyre.stl"].convex_hull

    S = V["suspension"]
    steer = math.radians(V["steering"]["max_angle_deg"])
    roll = math.radians(S["articulation_limit_deg"])
    half_track = V["chassis"]["track"] / 2
    clashes = []
    n_cfg = 0
    for axle in ("front", "rear"):
        bump = S[f"travel_bump_{axle}"]
        reb = S[f"travel_rebound_{axle}"]
        for heave, r, st_l, st_r in itertools.product((-reb, 0.0, bump), (-roll, 0.0, roll),
                                                      (-steer, -steer / 2, 0.0, steer / 2, steer),
                                                      (-steer, 0.0, steer)):
            # Per-wheel travel within the stroke the arches are sized for
            if heave + abs(math.sin(r)) * half_track > bump + 1e-3:
                continue
            q = {f"{axle}_axle_heave_joint": heave, f"{axle}_axle_roll_joint": r,
                 f"{axle}_left_steer_joint": st_l, f"{axle}_right_steer_joint": st_r}
            cache = {}
            n_cfg += 1
            for link, f, vo in visuals:
                if f not in MOVING or not link.startswith(axle):
                    continue
                # hull meshes are authored in base_link; the URDF root is base_footprint
                T = np.linalg.inv(link_pose("base_link", joints, q, cache)) @ link_pose(link, joints, q, cache) @ vo
                pts = trimesh.transform_points(meshes[f].vertices, T)
                lo, hi = pts.min(0), pts.max(0)
                for (blo, bhi), hf, hp in hull_bounds:
                    if np.any(hi < blo - 0.01) or np.any(lo > bhi + 0.01):
                        continue
                    k = int(inside(pts, hp).sum())
                    if k:
                        clashes.append((axle, f, hf, round(heave, 3), round(math.degrees(r), 1),
                                        round(math.degrees(st_l)), round(math.degrees(st_r)), k))
                if f == "tyre.stl":
                    th = tyre_hull.copy()
                    th.apply_transform(T)
                    k = int(inside(hull_pts, th).sum())
                    if k:
                        clashes.append((axle, "hull vertices", "inside tyre", round(heave, 3),
                                        round(math.degrees(r), 1), round(math.degrees(st_l)),
                                        round(math.degrees(st_r)), k))
    print(f"{len(hull)} hull pieces, {n_cfg} suspension/steering configurations checked")
    if clashes:
        seen = set()
        for c in clashes:
            key = c[:3]
            if key in seen:
                continue
            seen.add(key)
            print("CLASH", c)
        return 1
    print("no clashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
