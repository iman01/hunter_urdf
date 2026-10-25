"""Command shaping for the HUNTeR 4WS controller (pure Python, unit tested).

Turns a raw velocity command into a feasible, rate-limited one and the
matching knuckle/wheel commands:

  * The path geometry (curvature for the ICR modes, track angle for crab) is
    applied immediately and only the speed is ramped, so the vehicle follows
    the commanded arc while it accelerates instead of starting on a tighter
    one.
  * Speed is capped by the steering limits, the yaw-rate and lateral
    acceleration limits, and by how far the knuckles still lag behind their
    target; every cap goes through the same accel/decel ramp (no one-tick
    stops).
  * The steering reference never exceeds what the steering can realise (no
    windup).
  * Knuckles slew linkage-consistently: per axle the left knuckle is slewed
    and the right one follows from the tie-rod four-bar (trapezoid), or both
    move by the same fraction of their error (ideal linkage).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from .kinematics import MODE_CRAB, MODE_FREE, FourWheelSteeringKinematics


@dataclass
class ShaperLimits:
    max_speed: float = 13.9
    max_reverse_speed: float = 4.0
    max_accel: float = 2.0
    max_decel: float = 4.0
    max_yaw_rate: float = 1.0
    max_yaw_accel: float = 2.0
    max_lateral_accel: float = 3.5
    max_lateral_speed: float = 3.0
    steer_rate: float = math.radians(40.0)
    steer_error_slowdown: float = math.radians(8.0)
    steer_error_stop: float = math.radians(25.0)


@dataclass
class ShapedCommand:
    steer: List[float]
    omega: List[float]
    twist: Tuple[float, float, float]
    speed_scale: float


def _ramp(cur: float, target: float, up: float, down: float, dt: float) -> float:
    """Move cur toward target; `down` applies while |cur| shrinks or the sign flips."""
    shrinking = abs(target) < abs(cur) or target * cur < 0.0
    step = (down if shrinking else up) * dt
    return cur + max(-step, min(step, target - cur))


@dataclass
class CommandShaper:
    kin: FourWheelSteeringKinematics
    lim: ShaperLimits = field(default_factory=ShaperLimits)
    steer: List[float] = field(default_factory=lambda: [0.0] * 4)
    speed: float = 0.0                       # signed speed along the path [m/s]
    free_ref: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    unit: Tuple[float, float, float] = (1.0, 0.0, 0.0)   # last commanded path geometry (unit speed)

    # ------------------------------------------------------------- geometry
    def _unit_twist(self, mode: str, vx: float, vy: float, wz: float,
                    speed_for_geometry: float) -> Optional[Tuple[float, float, float]]:
        """Feasible twist with the commanded geometry, scaled to unit speed
        (forward), or None when the command carries no geometry."""
        if mode == MODE_CRAB:
            if math.hypot(vx, vy) < 1e-6:
                return None
            sgn = 1.0 if vx >= 0.0 else -1.0
            ux, uy = sgn * vx, sgn * vy                      # fold reverse onto forward geometry
            n = math.hypot(ux, uy)
            ux, uy, _ = self.kin.limit(ux / n, uy / n, 0.0)  # crab angle within steering limits
            n = math.hypot(ux, uy)
            return ux / n, uy / n, 0.0
        if abs(vx) < 1e-3:
            if abs(wz) < 1e-6:
                return None
            # Rotate-in-place request: not possible with 4WS. Pre-steer to full lock.
            kappa = math.copysign(1e3, wz)
        else:
            kappa = wz / vx                                   # path curvature (sign: left turn > 0 forward)
        g = max(abs(speed_for_geometry), 0.1)
        tx, ty, tw = self.kin.constrain(mode, g, 0.0, kappa * g)
        tx, ty, tw = self.kin.limit(tx, ty, tw)
        return tx / g, ty / g, tw / g

    # ---------------------------------------------------------------- step
    def step(self, mode: str, cmd: Tuple[float, float, float], dt: float,
             measured_steer: Optional[Sequence[float]] = None,
             differential: str = "electronic") -> ShapedCommand:
        L = self.lim
        vx_c = max(-L.max_reverse_speed, min(L.max_speed, cmd[0]))
        vy_c = max(-L.max_lateral_speed, min(L.max_lateral_speed, cmd[1]))
        wz_c = max(-L.max_yaw_rate, min(L.max_yaw_rate, cmd[2]))

        if mode == MODE_FREE:
            return self._step_free(vx_c, vy_c, wz_c, dt, measured_steer, differential)

        unit = self._unit_twist(mode, vx_c, vy_c, wz_c, self.speed)
        if mode == MODE_CRAB:
            target_speed = math.copysign(math.hypot(vx_c, vy_c), vx_c if abs(vx_c) > 1e-9 else 1.0)
        else:
            target_speed = vx_c
        has_geometry = unit is not None
        if unit is None:
            # No new geometry (e.g. a stop command): keep the last path while slowing down.
            unit = self.unit if mode != MODE_CRAB or self.unit[2] == 0.0 else (1.0, 0.0, 0.0)
        self.unit = unit
        if abs(unit[2]) > 1e-9:
            # |v * wz| <= yaw-rate limit and |v^2 * kappa| <= lateral acceleration limit
            kappa = abs(unit[2]) / max(math.hypot(unit[0], unit[1]), 1e-9)
            vmax = min(L.max_yaw_rate / abs(unit[2]), math.sqrt(L.max_lateral_accel / max(kappa, 1e-9)))
            target_speed = max(-vmax, min(vmax, target_speed))

        # Knuckle targets from the geometry alone (independent of speed).
        if has_geometry:
            target_steer = self.kin.inverse(mode, *unit, hold=self.steer, differential=differential).steer
        else:
            target_steer = list(self.steer)

        # Slow down while the knuckles have not reached the target geometry.
        ref = measured_steer if measured_steer is not None else self.steer
        lag = max(abs(t - m) for t, m in zip(target_steer, ref))
        lo, hi = L.steer_error_slowdown, L.steer_error_stop
        scale = 1.0 if lag <= lo else max(0.0, 1.0 - (lag - lo) / max(hi - lo, 1e-6))
        self.speed = _ramp(self.speed, scale * target_speed, L.max_accel, L.max_decel, dt)

        self._slew(target_steer, dt)
        if abs(self.speed) < 1e-9:
            twist = (0.0, 0.0, 0.0)
            omega = [0.0] * 4
        else:
            twist = (unit[0] * self.speed, unit[1] * self.speed, unit[2] * self.speed)
            omega = self.kin.inverse(mode, *twist, hold=self.steer, differential=differential).omega
        self.free_ref = list(twist)
        return ShapedCommand(list(self.steer), omega, twist, scale)

    def _step_free(self, vx, vy, wz, dt, measured_steer, differential) -> ShapedCommand:
        L = self.lim
        r = self.free_ref
        r[0] = _ramp(r[0], vx, L.max_accel, L.max_decel, dt)
        r[1] = _ramp(r[1], vy, L.max_accel, L.max_decel, dt)
        r[2] = _ramp(r[2], wz, L.max_yaw_accel, L.max_yaw_accel, dt)
        if abs(r[0] * r[2]) > L.max_lateral_accel:
            r[2] = math.copysign(L.max_lateral_accel / max(abs(r[0]), 1e-6), r[2])
        cmd = self.kin.inverse(MODE_FREE, *r, hold=self.steer, differential=differential)
        self.free_ref = list(cmd.twist)                      # anti-windup: keep only what is realisable
        self.speed = cmd.twist[0]
        self._slew(cmd.steer, dt)
        return ShapedCommand(list(self.steer), cmd.omega, cmd.twist, 1.0)

    # ---------------------------------------------------------------- slew
    def _slew(self, target: Sequence[float], dt: float) -> None:
        g = self.kin.g
        step = self.lim.steer_rate * dt
        for axle, (il, ir) in enumerate(((0, 1), (2, 3))):
            cl, cr = self.steer[il], self.steer[ir]
            tl, tr = target[il], target[ir]
            if g.linkage == "trapezoid":
                # One actuator per axle: slew the left knuckle, the tie rod sets the right one.
                nl = cl + max(-step, min(step, tl - cl))
                self.steer[il] = nl
                self.steer[ir] = g.trapezoids[axle].right_from_left(nl) if abs(nl) > 1e-12 else 0.0
            else:
                err = max(abs(tl - cl), abs(tr - cr))
                f = 1.0 if err <= step else step / err
                self.steer[il] = cl + f * (tl - cl)
                self.steer[ir] = cr + f * (tr - cr)
