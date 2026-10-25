"""Pose of the physical suspension and steering linkage of the HUNTeR replica.

The URDF models each rigid axle with two joints (heave + roll), because URDF
cannot hold the closed 3-link + Panhard loops. This module closes the loops
for visualisation: given the axle joint states and knuckle angles it returns
every linkage element as a segment between its two hard points, expressed in
base_link. Used by the suspension_linkage_publisher node (RViz markers) and by
tools/render (static renders).

Hard points come from vehicle.yaml (suspension.linkage), in the axle frame of
the FRONT axle: origin at the axle centre, x pointing away from the vehicle
centre. The rear axle is a front axle turned by pi about z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

Vec = Tuple[float, float, float]


@dataclass
class Element:
    name: str          # e.g. front_lower_link_left
    kind: str          # rod | coilover | tie_rod
    start: Vec         # base_link
    end: Vec           # base_link
    diameter: float

    @property
    def length(self) -> float:
        return math.dist(self.start, self.end)


def _rot_x(p: Vec, a: float) -> Vec:
    c, s = math.cos(a), math.sin(a)
    return (p[0], c * p[1] - s * p[2], s * p[1] + c * p[2])


def _rot_z(p: Vec, a: float) -> Vec:
    c, s = math.cos(a), math.sin(a)
    return (c * p[0] - s * p[1], s * p[0] + c * p[1], p[2])


class SuspensionLinkage:
    def __init__(self, cfg: Dict):
        self.wheelbase = cfg["chassis"]["wheelbase"]
        self.h_rc = cfg["suspension"]["roll_center_height"]
        self.lk = cfg["suspension"]["linkage"]
        st = cfg["steering"]
        self.kp_half = 0.5 * st["kingpin_track"]
        self.arm = st["arm_length"]
        self.beta = math.radians(st["trapezoid_angle_deg"])
        self.arm_z = st["arm_height"]

    # ----------------------------------------------------------- transforms
    def _axle_point(self, s: float, p: Vec, heave: float, roll: float) -> Vec:
        """Point given in front-axle coordinates on axle s (+1 front, -1 rear)."""
        q = (s * p[0], s * p[1], p[2] - self.h_rc)          # mirror for the rear axle, roll-axis origin
        q = _rot_x(q, roll)
        return (s * self.wheelbase / 2 + q[0], q[1], heave + self.h_rc + q[2])

    def _chassis_point(self, s: float, p: Vec) -> Vec:
        return (s * (self.wheelbase / 2 + p[0]), s * p[1], p[2])

    def _arm_end(self, s: float, side: float, steer: float) -> Vec:
        """Steering-arm ball joint, axle-centre coordinates (vehicle orientation)."""
        a, b = self.arm, self.beta
        local = (-s * a * math.cos(b), -side * a * math.sin(b), 0.0)
        rx, ry, _ = _rot_z(local, steer)
        return (rx, side * self.kp_half + ry, self.arm_z)

    # ------------------------------------------------------------- elements
    def elements(self, joints: Dict[str, float]) -> List[Element]:
        out: List[Element] = []
        for axle, s in (("front", 1.0), ("rear", -1.0)):
            hv = joints.get(f"{axle}_axle_heave_joint", 0.0)
            rl = joints.get(f"{axle}_axle_roll_joint", 0.0)

            def seg(name, kind, pc, pa, d):
                out.append(Element(f"{axle}_{name}", kind, self._chassis_point(s, pc),
                                   self._axle_point(s, pa, hv, rl), d))

            lo = self.lk["lower_link"]
            for side, sy in (("left", 1.0), ("right", -1.0)):
                # Mirrored axle: the front-frame +y side lands on the vehicle's -y for the rear axle,
                # so pick the hard point whose vehicle side matches.
                fy = sy * s
                seg(f"lower_link_{side}", "rod", (lo["chassis"][0], fy * lo["chassis"][1], lo["chassis"][2]),
                    (lo["axle"][0], fy * lo["axle"][1], lo["axle"][2]), lo["diameter"])
                co = self.lk["coilover"]
                seg(f"coilover_{side}", "coilover", (co["chassis"][0], fy * co["chassis"][1], co["chassis"][2]),
                    (co["axle"][0], fy * co["axle"][1], co["axle"][2]), co["body_diameter"])
            up = self.lk["upper_link"]
            seg("upper_link", "rod", tuple(up["chassis"]), tuple(up["axle"]), up["diameter"])
            ph = self.lk["panhard_rod"]
            seg("panhard_rod", "rod", tuple(ph["chassis"]), tuple(ph["axle"]), ph["diameter"])

            # Tie rod between the two steering-arm ends (both on the axle).
            dl = joints.get(f"{axle}_left_steer_joint", 0.0)
            dr = joints.get(f"{axle}_right_steer_joint", 0.0)
            pl = self._arm_end(s, 1.0, dl)
            pr = self._arm_end(s, -1.0, dr)
            out.append(Element(f"{axle}_tie_rod", "tie_rod",
                               self._axle_point_vehicle(s, pl, hv, rl), self._axle_point_vehicle(s, pr, hv, rl), 0.04))
        return out

    def _axle_point_vehicle(self, s: float, p: Vec, heave: float, roll: float) -> Vec:
        """Point already in vehicle orientation relative to the axle centre."""
        q = _rot_x((p[0], p[1], p[2] - self.h_rc), roll)
        return (s * self.wheelbase / 2 + q[0], q[1], heave + self.h_rc + q[2])

    def coilover_lengths(self, joints: Dict[str, float]) -> Dict[str, float]:
        return {e.name: e.length for e in self.elements(joints) if e.kind == "coilover"}


def look_rotation(start: Sequence[float], end: Sequence[float]) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) rotating +x onto end - start (no roll)."""
    dx, dy, dz = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    # q = q_yaw(z) * q_pitch(y)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def look_rpy(start: Sequence[float], end: Sequence[float]) -> Tuple[float, float, float]:
    dx, dy, dz = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    return 0.0, -math.atan2(dz, math.hypot(dx, dy)), math.atan2(dy, dx)
