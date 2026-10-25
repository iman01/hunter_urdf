#!/usr/bin/env python3
"""Append the suspension/steering linkage to a URDF as fixed visual links, for
static renders (RViz shows them live through suspension_linkage_publisher).

  python3 tools/render/add_linkage_visuals.py in.urdf out.urdf \
      --joints front_axle_roll_joint:0.12,front_left_steer_joint:0.4
"""

import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "hunter_control"))

from hunter_control.suspension_linkage import SuspensionLinkage, look_rpy  # noqa: E402

MESH = "package://hunter_description/meshes/visual/"


def link_xml(name, mesh, start, end, scale, rgba):
    r, p, y = look_rpy(start, end)
    return (f'  <link name="{name}"><visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
            f'<mesh filename="{MESH}{mesh}" scale="{scale[0]:.4f} {scale[1]:.4f} {scale[2]:.4f}"/></geometry>'
            f'<material name="{name}_mat"><color rgba="{rgba}"/></material></visual></link>\n'
            f'  <joint name="{name}_joint" type="fixed"><parent link="base_link"/><child link="{name}"/>'
            f'<origin xyz="{start[0]:.5f} {start[1]:.5f} {start[2]:.5f}" rpy="{r:.6f} {p:.6f} {y:.6f}"/></joint>\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp")
    ap.add_argument("out")
    ap.add_argument("--joints", default="")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(os.path.join(ROOT, "hunter_description", "config", "vehicle.yaml")))
    joints = {k: float(v) for k, v in (kv.split(":") for kv in a.joints.split(",") if kv)}
    lk = SuspensionLinkage(cfg)
    sd = cfg["suspension"]["linkage"]["coilover"]["spring_diameter"]
    xml = ""
    for e in lk.elements(joints):
        if e.kind in ("rod", "tie_rod"):
            xml += link_xml(f"viz_{e.name}", "link_rod.stl", e.start, e.end, (e.length, e.diameter, e.diameter),
                            "0.35 0.36 0.37 1")
        else:
            d = [(b - x) / e.length for x, b in zip(e.end, e.start)]
            s0 = tuple(x + 0.03 * di for x, di in zip(e.end, d))
            s1 = tuple(x - 0.08 * di for x, di in zip(e.start, d))
            xml += link_xml(f"viz_{e.name}_body", "coilover_body.stl", e.start, e.end, (1, 1, 1), "0.15 0.15 0.16 1")
            xml += link_xml(f"viz_{e.name}_rod", "coilover_rod.stl", e.end, e.start, (1, 1, 1), "0.78 0.79 0.80 1")
            xml += link_xml(f"viz_{e.name}_spring", "coilover_spring.stl", s0, s1, (e.length - 0.11, sd, sd),
                            "0.60 0.11 0.09 1")
    src = open(a.inp).read()
    open(a.out, "w").write(src.replace("</robot>", xml + "</robot>"))


if __name__ == "__main__":
    main()
