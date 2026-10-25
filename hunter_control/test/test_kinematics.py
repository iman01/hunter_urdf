import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hunter_control.kinematics import (  # noqa: E402
    MODES, FourWheelSteeringKinematics, VehicleGeometry)
from hunter_control.vehicle_config import geometry_from_config, load_vehicle  # noqa: E402

VEHICLE_YAML = os.path.join(os.path.dirname(__file__), "..", "..", "hunter_description", "config", "vehicle.yaml")


def make(linkage="ideal"):
    cfg = load_vehicle(VEHICLE_YAML)
    return FourWheelSteeringKinematics(geometry_from_config(cfg, linkage))


@pytest.fixture(params=["ideal", "trapezoid"])
def kin(request):
    return make(request.param)


def test_straight_line(kin):
    c = kin.inverse("counter", 3.0, 0.0, 0.0)
    assert all(abs(d) < 1e-9 for d in c.steer)
    for w in c.omega:
        assert w == pytest.approx(3.0 / kin.g.wheel_radius, rel=1e-9)


def test_reverse_keeps_angles_small(kin):
    c = kin.inverse("counter", -2.0, 0.0, -0.2)
    assert all(abs(d) <= kin.g.max_steer + 1e-9 for d in c.steer)
    assert all(w < 0 for w in c.omega)


def test_counter_phase_is_symmetric(kin):
    c = kin.inverse("counter", 2.0, 0.0, 0.25)
    fl, fr, rl, rr = c.steer
    assert fl == pytest.approx(-rl, abs=1e-9)
    assert fr == pytest.approx(-rr, abs=1e-9)
    assert fl > fr > 0.0  # left turn: left (inner) wheel steers more


def test_front_mode_rear_straight(kin):
    c = kin.inverse("front", 2.0, 0.0, 0.2)
    assert c.steer[2] == pytest.approx(0.0, abs=1e-9)
    assert c.steer[3] == pytest.approx(0.0, abs=1e-9)
    assert c.steer[0] > c.steer[1] > 0.0


def test_ideal_ackermann_normals_meet_at_icr():
    kin = make("ideal")
    vx, vy, wz = kin.constrain("counter", 2.0, 0.0, 0.3)
    c = kin.inverse("counter", vx, vy, wz)
    icr = (-c.twist[1] / c.twist[2], c.twist[0] / c.twist[2])
    for (px, py, _), d in zip(kin.g.pivots(), c.steer):
        # Wheel axis through the kingpin must pass through the ICR.
        ax, ay = -math.sin(d), math.cos(d)
        rx, ry = icr[0] - px, icr[1] - py
        assert abs(rx * ay - ry * ax) < 1e-6


def test_crab_all_parallel_ideal():
    kin = make("ideal")
    c = kin.inverse("crab", 2.0, 0.6, 0.4)
    assert c.twist[2] == 0.0
    assert max(c.steer) - min(c.steer) < 1e-9
    assert c.steer[0] == pytest.approx(math.atan2(0.6, 2.0))


@pytest.mark.parametrize("mode", MODES)
def test_forward_inverts_inverse_ideal(mode):
    kin = make("ideal")
    rnd = random.Random(7)
    for _ in range(50):
        vx = rnd.uniform(-5, 10)
        vy = rnd.uniform(-1, 1)
        wz = rnd.uniform(-0.6, 0.6)
        c = kin.inverse(mode, vx, vy, wz)
        est = kin.forward(c.steer, c.omega)
        for a, b in zip(est, c.twist):
            assert a == pytest.approx(b, abs=1e-6)


def test_limits_respected(kin):
    rnd = random.Random(3)
    for mode in MODES:
        for _ in range(30):
            c = kin.inverse(mode, rnd.uniform(-3, 3), rnd.uniform(-3, 3), rnd.uniform(-3, 3))
            assert all(abs(d) <= kin.g.max_steer + 1e-9 for d in c.steer)


def test_trapezoid_close_to_ackermann_in_counter_phase():
    ideal, trap = make("ideal"), make("trapezoid")
    for wz in (0.05, 0.1, 0.2, 0.3):
        ci = ideal.inverse("counter", 2.0, 0.0, wz)
        ct = trap.inverse("counter", 2.0, 0.0, wz)
        if ci.twist != ct.twist:
            continue
        for a, b in zip(ci.steer, ct.steer):
            assert abs(a - b) < math.radians(1.0)


def test_trapezoid_symmetry_and_mirror():
    g = make("trapezoid").g
    front, rear = g.trapezoids
    for deg in (2, 10, 20, 30):
        d = math.radians(deg)
        # Mirror image: the rear axle steers opposite to the front one.
        assert rear.right_from_left(-d) == pytest.approx(-front.right_from_left(d), abs=1e-12)
        # Left turn: right (outer) wheel steers less than the left one.
        assert 0.0 < front.right_from_left(d) < d
        # Right turn: right (inner) wheel steers more than the left one.
        assert front.right_from_left(-d) < -d


def test_locked_differential_equal_axle_speeds():
    kin = make("ideal")
    c = kin.inverse("counter", 2.0, 0.0, 0.3, differential="locked")
    assert c.omega[0] == pytest.approx(c.omega[1])
    assert c.omega[2] == pytest.approx(c.omega[3])


def test_counter_phase_turns_tighter_than_front(kin):
    assert kin.turning_radius("counter") < 0.75 * kin.turning_radius("front")


def test_geometry_validation():
    with pytest.raises(ValueError):
        VehicleGeometry(1.0, -1.0, 1.0, 1.2, 0.5, 0.5, arm_length=0.6, trapezoid_angle=math.radians(60))


def test_trapezoid_crab_tracks_commanded_direction():
    kin = make("trapezoid")
    c = kin.inverse("crab", 2.0, 0.6, 0.0)
    target = math.atan2(0.6, 2.0)
    assert 0.5 * (c.steer[0] + c.steer[1]) == pytest.approx(target, abs=1e-6)
    assert 0.5 * (c.steer[2] + c.steer[3]) == pytest.approx(target, abs=1e-6)


@pytest.mark.parametrize("linkage", ["ideal", "trapezoid"])
@pytest.mark.parametrize("differential", ["electronic", "locked"])
@pytest.mark.parametrize("mode", ["front", "rear", "counter", "adaptive"])
def test_odometry_round_trip_icr_modes(linkage, differential, mode):
    """forward(inverse(twist)) recovers the twist for every linkage and
    differential setting in the modes with an instantaneous centre."""
    kin = make(linkage)
    rnd = random.Random(11)
    for _ in range(40):
        vx = rnd.uniform(0.3, 8.0) * rnd.choice((1, -1))
        c = kin.inverse(mode, vx, 0.0, rnd.uniform(-0.6, 0.6), differential=differential)
        est = kin.forward(c.steer, c.omega, differential=differential)
        speed = math.hypot(c.twist[0], c.twist[1])
        assert math.hypot(est[0] - c.twist[0], est[1] - c.twist[1]) < 1e-6 * max(speed, 1.0)
        assert est[2] == pytest.approx(c.twist[2], abs=1e-6)


def test_odometry_crab_trapezoid_close():
    kin = make("trapezoid")
    c = kin.inverse("crab", 2.0, 0.7, 0.0)
    est = kin.forward(c.steer, c.omega)
    assert abs(est[2]) < 1e-6
    track = math.atan2(est[1], est[0])
    assert track == pytest.approx(math.atan2(c.twist[1], c.twist[0]), abs=math.radians(2.0))
