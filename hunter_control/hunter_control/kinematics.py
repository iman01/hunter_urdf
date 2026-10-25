"""Planar kinematics of the HUNTeR double-steer-axle (4WS) vehicle.

Pure Python, no ROS dependency, so it can be unit tested in isolation and
reused by the controller node, the odometry and the offline design tools.

Conventions (REP 103 / REP 105):
  * base_link: x forward, y left, z up, located midway between the axles.
  * Wheel order everywhere: front_left, front_right, rear_left, rear_right.
  * Steering angle delta: rotation of the knuckle about +z (positive = wheel
    points to the left).
  * Wheel angular velocity omega: positive = vehicle moves forward.

The vehicle has one steering actuator per axle. Each rigid axle couples its
left and right knuckles through a tie rod (steering trapezoid), so by default
the outer wheel angle follows from the inner one through the four-bar linkage
(`SteeringTrapezoid`). Setting `linkage: ideal` gives exact Ackermann angles
per wheel instead (a perfect steer-by-wire assumption).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")

MODE_FRONT = "front"          # 2WS, front axle steers, ICR on rear axle line
MODE_REAR = "rear"            # 2WS, rear axle steers, ICR on front axle line
MODE_COUNTER = "counter"      # 4WS counter-phase, ICR on mid-wheelbase line
MODE_CRAB = "crab"            # 4WS in-phase, all wheels parallel, no yaw
MODE_ADAPTIVE = "adaptive"    # counter-phase at low speed blending to front-only
MODE_FREE = "free"            # any planar twist the steering limits allow
MODES = (MODE_FRONT, MODE_REAR, MODE_COUNTER, MODE_CRAB, MODE_ADAPTIVE, MODE_FREE)


def _wrap_half_pi(angle: float) -> Tuple[float, float]:
    """Fold an angle into [-pi/2, pi/2]; return (angle, rolling direction)."""
    if angle > math.pi / 2.0:
        return angle - math.pi, -1.0
    if angle < -math.pi / 2.0:
        return angle + math.pi, -1.0
    return angle, 1.0


@dataclass
class SteeringTrapezoid:
    """Ackermann steering trapezoid (four-bar) of one rigid steer axle.

    Geometry in the axle frame (x towards the vehicle front for the front
    axle): kingpins at (0, +/-kingpin_track/2), steering arms of length
    `arm_length` pointing away from the vehicle centre (towards the
    mid-wheelbase line) and inclined inwards by `trapezoid_angle`.
    The tie rod joins the arm ends.  `side` = +1 for the front axle and -1
    for the rear axle (mirror image about the mid-wheelbase line).
    """

    kingpin_track: float
    arm_length: float
    trapezoid_angle: float
    side: float = 1.0

    def __post_init__(self) -> None:
        a, b, k = self.arm_length, self.trapezoid_angle, self.kingpin_track
        self.tie_rod_length = k - 2.0 * a * math.sin(b)
        if self.tie_rod_length <= 0.0:
            raise ValueError("steering trapezoid: arms too long or angle too large")

    def _arm_end(self, left: bool, delta: float) -> Tuple[float, float]:
        a, b, k = self.arm_length, self.trapezoid_angle, self.kingpin_track
        if left:
            theta = -math.pi + b + delta
            kx, ky = 0.0, k / 2.0
        else:
            theta = math.pi - b + delta
            kx, ky = 0.0, -k / 2.0
        return kx + a * math.cos(theta), ky + a * math.sin(theta)

    def _right_from_left_front(self, delta_left: float) -> float:
        """Right knuckle angle for a given left angle, front-axle frame."""
        ax, ay = self._arm_end(True, delta_left)
        kx, ky = 0.0, -self.kingpin_track / 2.0
        a, l = self.arm_length, self.tie_rod_length
        dx, dy = ax - kx, ay - ky
        d = math.hypot(dx, dy)
        if d > a + l or d < abs(a - l):
            raise ValueError("steering trapezoid: linkage cannot close at this angle")
        # Intersection of circle(K_R, a) and circle(A_L, l).
        along = (a * a - l * l + d * d) / (2.0 * d)
        h = math.sqrt(max(a * a - along * along, 0.0))
        mx, my = kx + along * dx / d, ky + along * dy / d
        candidates = ((mx + h * dy / d, my - h * dx / d), (mx - h * dy / d, my + h * dx / d))
        # Pick the assembly branch closest to the nominal (straight-ahead) one.
        ref = self._arm_end(False, delta_left)
        px, py = min(candidates, key=lambda p: (p[0] - ref[0]) ** 2 + (p[1] - ref[1]) ** 2)
        theta = math.atan2(py - ky, px - kx)
        return math.atan2(math.sin(theta - (math.pi - self.trapezoid_angle)),
                          math.cos(theta - (math.pi - self.trapezoid_angle)))

    def outer_from_inner(self, delta_inner: float) -> float:
        """Outer knuckle angle for an inner angle >= 0 (front-axle frame,
        left turn). The linkage never toggles on this branch."""
        return self._right_from_left_front(delta_inner)

    def inner_from_outer(self, delta_outer: float) -> float:
        """Inverse of `outer_from_inner` (monotonic) by bisection."""
        lo, hi = 0.0, math.radians(80.0)
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            try:
                too_far = self.outer_from_inner(mid) > delta_outer
            except ValueError:
                too_far = True
            if too_far:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def right_from_left(self, delta_left: float) -> float:
        """Right knuckle angle produced by the linkage for a left angle."""
        d = self.side * delta_left
        if d >= 0.0:
            r = self.outer_from_inner(d)
        else:
            # Mirror symmetry: (dl, dr) valid <=> (-dr, -dl) valid.
            r = -self.inner_from_outer(-d)
        return self.side * r

    def pair_for_mean_angle(self, delta_mean: float, limit: float) -> Tuple[float, float]:
        """Left/right angles whose mean equals `delta_mean` (pure translation /
        crab: both tyres pull, so the vehicle tracks their mean direction)."""
        if abs(delta_mean) < 1e-9:
            return 0.0, 0.0
        dm = abs(delta_mean)
        lo, hi = 0.0, limit
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if 0.5 * (mid + self.outer_from_inner(mid)) > dm:
                hi = mid
            else:
                lo = mid
        d_in = 0.5 * (lo + hi)
        d_out = self.outer_from_inner(d_in)
        front_frame_positive = (self.side * delta_mean) > 0.0
        left, right = (d_in, d_out) if front_frame_positive else (-d_out, -d_in)
        return self.side * left, self.side * right

    def pair_for_center_angle(self, delta_center: float, limit: float) -> Tuple[float, float]:
        """Left/right knuckle angles whose Ackermann-equivalent centre angle
        (cot dc = (cot dl + cot dr)/2) equals `delta_center`."""
        if abs(delta_center) < 1e-9:
            return 0.0, 0.0
        dc = abs(delta_center)

        def center_of(d_in: float) -> float:
            ti, to = math.tan(d_in), math.tan(self.outer_from_inner(d_in))
            return math.atan(2.0 * ti * to / (ti + to)) if ti + to > 1e-12 else 0.0

        lo, hi = 0.0, limit
        # center_of is monotonic in the inner angle; bisection.
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if center_of(mid) > dc:
                hi = mid
            else:
                lo = mid
        d_in = 0.5 * (lo + hi)
        d_out = self.outer_from_inner(d_in)
        # Front-axle frame: positive centre angle => left wheel is inner.
        front_frame_positive = (self.side * delta_center) > 0.0
        left, right = (d_in, d_out) if front_frame_positive else (-d_out, -d_in)
        return self.side * left, self.side * right


@dataclass
class VehicleGeometry:
    front_axle_x: float          # [m] front axle position in base_link
    rear_axle_x: float           # [m] rear axle position in base_link (negative)
    kingpin_track: float         # [m] lateral distance between kingpin axes
    wheel_track: float           # [m] lateral distance between tyre centre planes
    wheel_radius: float          # [m] effective rolling radius
    max_steer: float             # [rad] max knuckle angle (either direction)
    linkage: str = "trapezoid"   # "trapezoid" or "ideal"
    arm_length: float = 0.25     # [m] steering arm length
    trapezoid_angle: float = 0.0  # [rad] 0 => auto-design for counter-phase
    trapezoids: List[SteeringTrapezoid] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if self.trapezoid_angle <= 0.0:
            # Classic rule: arm extensions meet on the line through the
            # instantaneous centre for the design mode (counter-phase => mid
            # wheelbase), i.e. tan(beta) = (K/2) / (L/2).
            half_l = 0.5 * (self.front_axle_x - self.rear_axle_x)
            self.trapezoid_angle = math.atan2(0.5 * self.kingpin_track, half_l)
        self.trapezoids = [
            SteeringTrapezoid(self.kingpin_track, self.arm_length, self.trapezoid_angle, +1.0),
            SteeringTrapezoid(self.kingpin_track, self.arm_length, self.trapezoid_angle, -1.0),
        ]

    @property
    def wheelbase(self) -> float:
        return self.front_axle_x - self.rear_axle_x

    @property
    def kingpin_offset(self) -> float:
        """Kingpin axis to tyre centre plane (spindle length) [m]."""
        return 0.5 * (self.wheel_track - self.kingpin_track)

    def pivots(self) -> List[Tuple[float, float, float]]:
        """(x, y, outward offset sign) of each kingpin, wheel order."""
        k = 0.5 * self.kingpin_track
        return [
            (self.front_axle_x, +k, +1.0),
            (self.front_axle_x, -k, -1.0),
            (self.rear_axle_x, +k, +1.0),
            (self.rear_axle_x, -k, -1.0),
        ]


@dataclass
class WheelCommand:
    steer: List[float]       # [rad] knuckle angles, wheel order
    omega: List[float]       # [rad/s] wheel spin rates, wheel order
    twist: Tuple[float, float, float]  # realised (vx, vy, wz) after limits


class FourWheelSteeringKinematics:
    """Inverse/forward kinematics for the double steer axle vehicle."""

    def __init__(self, geometry: VehicleGeometry,
                 adaptive_low_speed: float = 2.0, adaptive_high_speed: float = 8.0):
        self.g = geometry
        self.adaptive_low_speed = adaptive_low_speed
        self.adaptive_high_speed = adaptive_high_speed

    # ------------------------------------------------------------------ modes
    def icr_line_x(self, mode: str, vx: float) -> Optional[float]:
        """x position (base_link) of the lateral line holding the ICR."""
        g = self.g
        mid = 0.5 * (g.front_axle_x + g.rear_axle_x)
        if mode == MODE_FRONT:
            return g.rear_axle_x
        if mode == MODE_REAR:
            return g.front_axle_x
        if mode == MODE_COUNTER:
            return mid
        if mode == MODE_ADAPTIVE:
            lo, hi = self.adaptive_low_speed, self.adaptive_high_speed
            s = min(max((abs(vx) - lo) / max(hi - lo, 1e-6), 0.0), 1.0)
            s = s * s * (3.0 - 2.0 * s)  # smoothstep
            return mid + s * (g.rear_axle_x - mid)
        return None

    def constrain(self, mode: str, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """Project a requested twist onto the steering mode's constraint."""
        if mode not in MODES:
            raise ValueError(f"unknown steering mode '{mode}'")
        if mode == MODE_CRAB:
            return vx, vy, 0.0
        if mode == MODE_FREE:
            return vx, vy, wz
        x_icr = self.icr_line_x(mode, vx)
        # Zero lateral velocity on the ICR line: vy + wz * x_icr = 0.
        return vx, -wz * x_icr, wz

    # ------------------------------------------------------------- inverse IK
    def _ideal_wheel(self, i: int, vx: float, vy: float, wz: float) -> Tuple[Optional[float], float]:
        px, py, _ = self.g.pivots()[i]
        ux, uy = vx - wz * py, vy + wz * px
        if math.hypot(ux, uy) < 1e-9:
            return None, 0.0
        delta, _ = _wrap_half_pi(math.atan2(uy, ux))
        return delta, 0.0

    def _rolling_omega(self, i: int, delta: float, vx: float, vy: float, wz: float) -> float:
        px, py, side = self.g.pivots()[i]
        o = side * self.g.kingpin_offset
        cx, cy = px - o * math.sin(delta), py + o * math.cos(delta)
        ux, uy = vx - wz * cy, vy + wz * cx
        return (ux * math.cos(delta) + uy * math.sin(delta)) / self.g.wheel_radius

    def raw_angles(self, vx: float, vy: float, wz: float,
                   hold: Optional[Sequence[float]] = None) -> List[float]:
        """Knuckle angles for a twist (no limits). Wheels whose pivot does not
        move keep the `hold` angle (default 0)."""
        g = self.g
        out: List[float] = []
        for i in range(4):
            d, _ = self._ideal_wheel(i, vx, vy, wz)
            out.append(d if d is not None else (hold[i] if hold else 0.0))
        if g.linkage == "trapezoid":
            for axle, (il, ir) in enumerate(((0, 1), (2, 3))):
                ax_x = g.front_axle_x if axle == 0 else g.rear_axle_x
                ux, uy = vx, vy + wz * ax_x
                if math.hypot(ux, uy) < 1e-9:
                    continue
                dc, _ = _wrap_half_pi(math.atan2(uy, ux))
                if abs(wz) < 1e-12:
                    # Pure translation (crab): no ICR; the linkage dictates a
                    # slightly toed pair, centred on the commanded direction.
                    dl, dr = g.trapezoids[axle].pair_for_mean_angle(dc, 1.2 * g.max_steer)
                else:
                    dl, dr = g.trapezoids[axle].pair_for_center_angle(dc, 1.2 * g.max_steer)
                out[il], out[ir] = dl, dr
        return out

    def feasible(self, vx: float, vy: float, wz: float) -> bool:
        return all(abs(d) <= self.g.max_steer + 1e-9 for d in self.raw_angles(vx, vy, wz))

    def limit(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """Scale the steering part (vy, wz) of a twist until every knuckle is
        within its limit; vx is preserved.  Linear in (vy, wz), so the mode
        constraint vy = -wz * x_icr is preserved too."""
        if self.feasible(vx, vy, wz):
            return vx, vy, wz
        lo, hi = 0.0, 1.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if self.feasible(vx, mid * vy, mid * wz):
                lo = mid
            else:
                hi = mid
        return vx, lo * vy, lo * wz

    def inverse(self, mode: str, vx: float, vy: float, wz: float,
                hold: Optional[Sequence[float]] = None,
                differential: str = "electronic") -> WheelCommand:
        vx, vy, wz = self.constrain(mode, vx, vy, wz)
        vx, vy, wz = self.limit(vx, vy, wz)
        steer = self.raw_angles(vx, vy, wz, hold)
        steer = [max(-self.g.max_steer, min(self.g.max_steer, d)) for d in steer]
        omega = [self._rolling_omega(i, steer[i], vx, vy, wz) for i in range(4)]
        if differential == "locked":
            # Locked axle differentials: both wheels of an axle turn together.
            for il, ir in ((0, 1), (2, 3)):
                avg = 0.5 * (omega[il] + omega[ir])
                omega[il] = omega[ir] = avg
        return WheelCommand(steer=steer, omega=omega, twist=(vx, vy, wz))

    # ------------------------------------------------------------- forward FK
    def forward(self, steer: Sequence[float], omega: Sequence[float]) -> Tuple[float, float, float]:
        """Least-squares body twist from measured knuckle angles and wheel
        rates. Each wheel contributes its contact-point velocity vector."""
        g = self.g
        # Normal equations for unknowns (vx, vy, wz):
        #   vx - wz*cy = v*cos(d) ; vy + wz*cx = v*sin(d)
        a = [[0.0] * 3 for _ in range(3)]
        b = [0.0] * 3
        for i, (px, py, side) in enumerate(g.pivots()):
            d = steer[i]
            o = side * g.kingpin_offset
            cx, cy = px - o * math.sin(d), py + o * math.cos(d)
            v = omega[i] * g.wheel_radius
            rows = (([1.0, 0.0, -cy], v * math.cos(d)), ([0.0, 1.0, cx], v * math.sin(d)))
            for row, rhs in rows:
                for r in range(3):
                    b[r] += row[r] * rhs
                    for c in range(3):
                        a[r][c] += row[r] * row[c]
        return tuple(_solve3(a, b))  # type: ignore[return-value]

    # --------------------------------------------------------------- helpers
    def turning_radius(self, mode: str) -> float:
        """Kerb-to-kerb style radius of the outermost tyre centre at full
        lock for a given mode [m] (inf for crab)."""
        if mode == MODE_CRAB:
            return math.inf
        lo, hi = 1e-4, 10.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            vx, vy, wz = self.constrain(mode, 1.0, 0.0, mid)
            if self.feasible(vx, vy, wz):
                lo = mid
            else:
                hi = mid
        vx, vy, wz = self.constrain(mode, 1.0, 0.0, lo)
        cx, cy = -vy / wz, vx / wz  # ICR in base_link
        r = 0.0
        steer = self.raw_angles(vx, vy, wz)
        for i, (px, py, side) in enumerate(self.g.pivots()):
            o = side * self.g.kingpin_offset
            wx, wy = px - o * math.sin(steer[i]), py + o * math.cos(steer[i])
            r = max(r, math.hypot(wx - cx, wy - cy))
        return r


def _solve3(a: List[List[float]], b: List[float]) -> List[float]:
    """Gaussian elimination with partial pivoting for a 3x3 system."""
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return [0.0, 0.0, 0.0]
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, 3):
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    x = [0.0] * 3
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][c] * x[c] for c in range(r + 1, 3))) / m[r][r]
    return x
