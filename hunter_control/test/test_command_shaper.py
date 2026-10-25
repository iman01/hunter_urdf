import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hunter_control.command_shaper import CommandShaper, ShaperLimits  # noqa: E402
from hunter_control.kinematics import FourWheelSteeringKinematics  # noqa: E402
from hunter_control.vehicle_config import geometry_from_config, load_vehicle  # noqa: E402

VEHICLE_YAML = os.environ.get("HUNTER_VEHICLE_YAML", os.path.join(
    os.path.dirname(__file__), "..", "..", "hunter_description", "config", "vehicle.yaml"))
DT = 0.02


def shaper(linkage="trapezoid"):
    cfg = load_vehicle(VEHICLE_YAML)
    return CommandShaper(FourWheelSteeringKinematics(geometry_from_config(cfg, linkage)), ShaperLimits())


def run(s, mode, cmd, seconds, meas=None):
    out = []
    for _ in range(int(seconds / DT)):
        out.append(s.step(mode, cmd, DT, meas if meas is not None else None))
    return out


def test_curvature_preserved_while_accelerating():
    s = shaper()
    run(s, "counter", (0.0, 0.0, 0.0), 0.2)
    outs = run(s, "counter", (2.0, 0.0, 0.5), 3.0)
    moving = [o for o in outs if abs(o.twist[0]) > 0.05]
    assert moving, "vehicle never moved"
    for o in moving:
        assert o.twist[2] / o.twist[0] == pytest.approx(0.25, rel=1e-6)


def test_speed_respects_accel_and_decel():
    s = shaper()
    prev = 0.0
    for o in run(s, "counter", (5.0, 0.0, 0.0), 4.0) + run(s, "counter", (0.0, 0.0, 0.0), 3.0):
        dv = o.twist[0] - prev
        assert -s.lim.max_decel * DT - 1e-9 <= dv <= s.lim.max_accel * DT + 1e-9
        prev = o.twist[0]


def test_no_windup_on_release():
    s = shaper()
    run(s, "counter", (0.3, 0.0, 1.0), 3.0)                 # far beyond the steering limit
    assert max(abs(a) for a in s.steer) <= s.kin.g.max_steer + 1e-9
    before = list(s.steer)
    o = s.step("counter", (0.3, 0.0, 0.0), DT)               # release: knuckles start back at once
    assert all(abs(a) < abs(b) for a, b in zip(o.steer, before))


def test_lag_slowdown_is_ramped():
    s = shaper()
    run(s, "counter", (4.0, 0.0, 0.0), 3.0)
    # Knuckles reported far from the crab target: the speed must ramp down, not jump.
    prev = abs(s.speed)
    for _ in range(20):
        o = s.step("crab", (4.0, 1.5, 0.0), DT, measured_steer=[0.0, 0.0, 0.0, 0.0])
        v = math.hypot(o.twist[0], o.twist[1])
        assert prev - v <= s.lim.max_decel * DT + 1e-6
        prev = v


@pytest.mark.parametrize("mode,cmd", [("counter", (1.0, 0.0, 0.4)), ("front", (1.0, 0.0, -0.2)),
                                      ("crab", (1.0, 0.5, 0.0)), ("rear", (1.0, 0.0, 0.2))])
def test_slew_is_linkage_consistent(mode, cmd):
    s = shaper("trapezoid")
    for o in run(s, mode, cmd, 2.0):
        for axle, (il, ir) in enumerate(((0, 1), (2, 3))):
            l, r = o.steer[il], o.steer[ir]
            expect = s.kin.g.trapezoids[axle].right_from_left(l) if abs(l) > 1e-12 else 0.0
            assert r == pytest.approx(expect, abs=1e-9)


def test_lateral_accel_and_yaw_limits():
    s = shaper()
    outs = run(s, "counter", (13.0, 0.0, 0.4), 12.0)
    o = outs[-1]
    assert abs(o.twist[0] * o.twist[2]) <= s.lim.max_lateral_accel + 1e-6
    assert abs(o.twist[2]) <= s.lim.max_yaw_rate + 1e-9


def test_rotate_in_place_request_presteers_without_moving():
    s = shaper()
    outs = run(s, "counter", (0.0, 0.0, 0.5), 2.0)
    assert all(o.twist == (0.0, 0.0, 0.0) for o in outs)
    assert outs[-1].steer[0] == pytest.approx(s.kin.g.max_steer, abs=1e-6)


def test_reverse_arc():
    s = shaper()
    o = run(s, "counter", (-1.5, 0.0, 0.2), 4.0)[-1]
    assert o.twist[0] == pytest.approx(-1.5, abs=1e-6)
    assert o.twist[2] == pytest.approx(0.2, abs=1e-6)


def test_free_mode_anti_windup():
    s = shaper()
    o = run(s, "free", (0.5, 0.0, 1.0), 3.0)[-1]
    assert s.free_ref[2] == pytest.approx(o.twist[2])
    assert all(abs(a) <= s.kin.g.max_steer + 1e-9 for a in o.steer)
