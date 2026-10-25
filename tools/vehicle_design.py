#!/usr/bin/env python3
"""Engineering model of the Lukasiewicz-PIAP HUNTeR replica.

This script is the single source of truth for every number in the URDF.
Each input carries its status and source:

  published  - stated by PIAP or by press quoting PIAP (see SOURCES)
  derived    - computed from published values (formula in the report)
  estimated  - engineering estimate where nothing is published (reasoning given)
  design     - an engineering choice made for this replica, consistent with the
               published constraints (e.g. spring rates sized for the published
               25 cm stroke)

Outputs
  hunter_description/config/vehicle.yaml   consumed by xacro, controllers, mesh generator
  docs/DESIGN_REPORT.md                     every parameter with status/source + derivations

Usage:  python3 tools/vehicle_design.py [--check]
  --check   only verify that the committed outputs are up to date (CI)
"""

from __future__ import annotations

import argparse
import io
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Tuple

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "hunter_control"))

from hunter_control.kinematics import (FourWheelSteeringKinematics,  # noqa: E402
                                       SteeringTrapezoid, VehicleGeometry)

G = 9.80665
IN = 0.0254

# --------------------------------------------------------------------------
# Sources (retrieved 2026-10-25; quotes archived in docs/research/raw_findings.json)
# --------------------------------------------------------------------------
SOURCES: Dict[str, str] = {
    "POSTER21": "Lukasiewicz-PIAP, HUNTeR poster, INTARG 2021 invention fair "
                "(intarg.haller.pl/targi/pliki_wynalazki/posters/157-1623325091.pdf): 3500 kg, 470x220 cm, "
                "wheelbase 270 cm, ~50 km/h, dependent 3-link suspension with Panhard bar, coil springs, "
                "controlled-damping shock absorbers, stroke 25 cm, 110 kW generator, 40 kWh Li-Ion NMC + BTMS, "
                "600 VDC / 3x400 VAC / 24 VDC outputs, front/rear driving cameras, 2 side cameras per side, "
                "top-view cameras on the railing, PTZ, 800 m radio link",
    "JANES20": "Janes, 'Lukasiewicz-PIAP unveils Hunter UGV', 24 Sep 2020: 3.5 t, 4.7 x 2.2 m, two steering axles "
               "each integrated with an electric motor, 3-link Panhard suspension, 50 km/h, 110 kW generator, "
               "GPS, two lidars, 4 hull navigation cameras + 5 HD cameras incl. PTZ, robotic arm",
    "MILMAG20": "Milmag, 'Lukasiewicz-PIAP presents Hunter UGV', Sep 2020: 9 HD cameras (PTZ, 4 hull navigation, "
                "4 drive assistant), GPS, 2 lidars; hydraulic arm, cargo crate, GPR, UAV landing pad options; "
                "1 m wheels",
    "ARMYREC24": "Army Recognition, 'Poland deploys PIAP Hunter next-gen UGV', 2024: 3500 kg, 470x220 cm, wheelbase "
                 "270 cm, 90 kW continuous / 220 kW peak serial hybrid with traction motors integrated with the "
                 "driving axles, differential locks, two steering axles, 1 m wheels, 25 cm stroke",
    "TENDER17": "PIAP public procurement ZO/11/07/2017 'Dostawa mostow napedowych' (HUNTeR): 3 rigid FRONT drive "
                "axles, Ultimate Dana 60 or equivalent, ratio 5.38, ring gear >= 9.75 in, 70 in between wheel "
                "mounting surfaces, 8 x 6.5 in bolt pattern, 3.5 in tubes, disc brakes >= 350 mm, 2-piston calipers",
    "TENDER18": "PIAP notice ZWR/01/2018 (HUNTeR motor set): 2 x three-phase PMSM, up to 700 VDC, >= 4500 rpm at "
                "full load, 450 A peak for >= 2 min",
    "D24_23": "Defence24 (EN/PL) 2023 FEX/MSPO articles: > 4 t (armed), 4.7 m, 1 m wheels, dependent rigid-axle "
              "suspension with three reaction rods + Panhard rod, coil springs, adjustable dampers, differential "
              "locks, two steering axles, 6-7 HD cameras (2 driving + 4 perimeter), AREX ZMU-03 12.7 mm RWS",
    "MILMAG_ZMU03": "Milmag, ZMU-03/ZMU-07 shooting demonstration: ZMU-03 with 12.7 mm HMG 205 kg incl. weapon",
    "WB_ZMU05": "WB Group ZMU-05 flyer: 1630 x 750 x 660 mm, 190 kg kerb, 228 kg combat",
    "D24TV21": "Defence24 TV 2021: cargo-mule configuration, ~1000 kg payload",
    "PZ26": "Polska Zbrojna, MSPO 2026: up to 1 t payload, 450 km range on a full tank",
    "PIAP24": "Lukasiewicz-PIAP, MSPO 2024 release: latest HUNTeR adds lidars and stereovision cameras",
    "SPICER": "Spicer/Dana, Ultimate Dana 60 crate axle data: 35-spline 4340 shafts, 3.5 in tubes, 8x6.5 bolt "
              "pattern, ~358 mm vented rotors, 2-piston calipers, crate axle for Jeep JK (wheelbase 2.946 m)",
}


@dataclass
class P:
    """A documented parameter."""
    value: float
    unit: str
    status: str          # published | derived | estimated | design
    source: str          # key into SOURCES or a short reasoning
    note: str = ""


REGISTRY: Dict[str, P] = {}


def par(name: str, value, unit: str, status: str, source: str, note: str = ""):
    REGISTRY[name] = P(value, unit, status, source, note)
    return value


# ==========================================================================
# 1. Published envelope
# ==========================================================================
L_OVERALL = par("body.length", 4.70, "m", "published", "POSTER21, JANES20")
W_OVERALL = par("body.width", 2.20, "m", "published", "POSTER21, JANES20")
WHEELBASE = par("chassis.wheelbase", 2.70, "m", "published", "POSTER21, ARMYREC24",
                "Polish press also prints 2.20 m next to the 2.20 m width; 2.70 m is PIAP's own poster figure "
                "and is the only value compatible with 1 m wheels inside a 4.70 m hull")
M_CURB_2020 = par("mass.curb_2020", 3500.0, "kg", "published", "POSTER21, JANES20", "2020 prototype, cargo deck")
M_CURB_2023 = par("mass.curb_2023_armed", 4050.0, "kg", "estimated", "D24_23",
                  "'over four tonnes' kerb for the ZMU-03 armed version; 4.05 t used")
PAYLOAD_MAX = par("mass.payload_max", 1000.0, "kg", "published", "D24TV21, PZ26")
V_MAX = par("limits.max_speed", 50.0 / 3.6, "m/s", "published", "POSTER21, JANES20", "50 km/h")
STROKE = par("suspension.stroke", 0.25, "m", "published", "POSTER21, ARMYREC24")

# ==========================================================================
# 2. Wheels and tyres
# ==========================================================================
TYRE_OD = par("wheel.diameter", 1.00, "m", "published", "D24_23, MILMAG20")
TYRE_W = par("wheel.width", 0.343, "m", "estimated",
             "8 x 6.5 in bolt pattern (TENDER17) => light-truck 17 in wheels; the common ~1 m tyre on that "
             "pattern is 40x13.50R17 (section 343 mm)")
RIM_D = par("wheel.rim_diameter", 17 * IN, "m", "estimated", "see wheel.width")
RIM_W = par("wheel.rim_width", 9 * IN, "m", "estimated", "17x9 beadlock wheel")
WHEEL_OFFSET = par("wheel.offset", -0.0127, "m", "estimated",
                   "17x9 with 4.5 in backspacing => -12.7 mm offset (wheel centre plane outboard of the mounting face)")
M_TYRE = par("wheel.tyre_mass", 75.0, "kg", "estimated", "40x13.50R17 LT load range E off-road tyre")
M_RIM = par("wheel.rim_mass", 27.0, "kg", "estimated", "17x9 steel beadlock wheel")
K_TYRE = par("wheel.vertical_stiffness", 350_000.0, "N/m", "estimated",
             "40 in LT tyre at ~2.0 bar: 300-400 kN/m")
MU = par("wheel.mu", 0.9, "-", "design", "dry soil / asphalt peak friction for an off-road tread")

# ==========================================================================
# 3. Axles (Ultimate Dana 60 front steer axles, both ends)
# ==========================================================================
WMS = par("axle.wms_width", 70 * IN, "m", "published", "TENDER17", "wheel-mounting-surface to WMS")
TRACK = par("chassis.track", WMS - 2 * WHEEL_OFFSET, "m", "derived", "track = WMS - 2 * offset")
KP_TO_WMS = par("axle.kingpin_to_wms", 0.120, "m", "estimated",
                "Dana 60 ball-joint knuckle: kingpin axis ~120 mm inboard of the hub face")
KP_TRACK = par("steering.kingpin_track", WMS - 2 * KP_TO_WMS, "m", "derived", "WMS - 2 * kingpin_to_wms")
KPI = par("steering.kingpin_inclination", 8.0, "deg", "estimated", "Dana 60 ball-joint knuckles",
          "documented only; the URDF uses a vertical steering axis through the kingpin (see report)")
CASTER = par("steering.caster", 5.0, "deg", "estimated", "typical solid-axle setting; rear axle is a mirrored "
             "front axle so its caster is reversed for forward travel (documented only)")
RATIO_AXLE = par("drive.axle_ratio", 5.38, "-", "published", "TENDER17")
TUBE_D = par("axle.tube_diameter", 3.5 * IN, "m", "published", "TENDER17")
RING_GEAR = par("axle.ring_gear_diameter", 9.75 * IN, "m", "published", "TENDER17", "minimum")
ROTOR_D = par("axle.brake_rotor_diameter", 0.358, "m", "estimated", "SPICER (tender: >= 350 mm)")
DIFF_R = par("axle.diff_radius", 0.165, "m", "estimated", "9.75 in ring gear housing, cover radius")
DIFF_Y = par("axle.diff_offset_y", 0.10, "m", "estimated", "Dana 60 front: differential offset to the left")

M_AXLE_HOUSING = par("axle.housing_mass", 150.0, "kg", "estimated",
                     "Ultimate Dana 60 front ~205 kg with brakes: 150 kg housing/diff/inner shafts + 2 x 27.5 kg ends")
M_KNUCKLE = par("axle.knuckle_mass", 32.0, "kg", "estimated",
                "knuckle, hub, 358 mm rotor, caliper, outer shaft, ball joints")
M_MOTOR = par("drive.motor_assembly_mass", 85.0, "kg", "estimated",
              "45/110 kW PMSM (~45 kg) + ~3:1 reduction (~25 kg) + mounting (~15 kg), on the pinion => unsprung")
M_STEER_ACT = par("steering.actuator_mass", 25.0, "kg", "estimated", "linear actuator, tie rod, brackets")
M_LINKS_UNSPRUNG = par("suspension.links_unsprung_mass", 25.0, "kg", "estimated",
                       "half of 3 links + Panhard + 2 coil-overs, per axle")

# ==========================================================================
# 4. Tyre deflection, radii
# ==========================================================================
M_AXLE_LINK = M_AXLE_HOUSING + M_MOTOR + M_STEER_ACT + M_LINKS_UNSPRUNG
M_WHEEL = M_TYRE + M_RIM
M_UNSPRUNG_AXLE = M_AXLE_LINK + 2 * M_KNUCKLE + 2 * M_WHEEL
M_UNSPRUNG = 2 * M_UNSPRUNG_AXLE
par("mass.unsprung_per_axle", M_UNSPRUNG_AXLE, "kg", "derived", "axle + 2 knuckles + 2 wheels")

W_WHEEL_STATIC = M_CURB_2020 * G / 4.0           # symmetric design, ~50/50
TYRE_DEFL = W_WHEEL_STATIC / K_TYRE
R_FREE = TYRE_OD / 2.0
R_STATIC = par("wheel.static_loaded_radius", R_FREE - TYRE_DEFL, "m", "derived", "R - F_z / k_tyre at kerb")
R_ROLL = par("wheel.rolling_radius", R_FREE - TYRE_DEFL / 3.0, "m", "derived",
             "effective rolling radius ~ R - deflection/3 (radial tyre)")
R_COLL = R_ROLL  # rigid simulation tyre: its radius is the rolling radius the controller uses
par("wheel.collision_radius", R_COLL, "m", "design", "simulation tyre is rigid: radius = rolling radius")
Z_GROUND = -R_COLL                                # ground height in base_link at kerb static

# Wheel inertia: tread band (60 % of tyre) as thin ring at 0.95 R, sidewalls
# (40 %) as annulus rim..R, rim barrel 70 % at rim radius, rim disc 30 %.
r_rim = RIM_D / 2.0
I_tyre_spin = 0.6 * M_TYRE * (0.95 * R_FREE) ** 2 + 0.4 * M_TYRE * (R_FREE ** 2 + r_rim ** 2) / 2.0
I_rim_spin = 0.7 * M_RIM * r_rim ** 2 + 0.3 * M_RIM * r_rim ** 2 / 2.0
WHEEL_IYY = par("wheel.iyy", I_tyre_spin + I_rim_spin, "kg m^2", "derived", "ring/annulus model")
# Diametral: thin ring I_d = I_spin / 2 plus width term m w^2 / 12.
WHEEL_IXX = par("wheel.ixx", 0.5 * WHEEL_IYY + M_WHEEL * TYRE_W ** 2 / 12.0, "kg m^2", "derived",
                "I_spin/2 + m w^2/12")

# ==========================================================================
# 5. Hull geometry (base_link frame: x fwd, z up, origin mid-wheelbase at
#    wheel-centre height at kerb static; heights below are above ground)
# ==========================================================================
H_DECK_TOP = par("body.deck_height", 1.35, "m", "estimated",
                 "1 m wheels + 156 mm bump + clearance => fenders/deck at ~1.2 m; 0.15 m deck structure")
H_DECK_BOTTOM = par("body.wheel_arch_height", 1.20, "m", "derived",
                    "tyre top at full bump (R_coll + R + bump) + ~5 cm clearance")
H_RAIL = par("body.railing_height", 0.25, "m", "estimated", "deck railing carrying top-view cameras (POSTER21)")
H_TUB_BOTTOM = par("body.belly_height", 0.47, "m", "design", "battery/equipment tub between the axles")
H_KEEL_BOTTOM = par("body.keel_height", 0.86, "m", "derived",
                    "frame/tunnel above the axle housings and motors at full bump")
H_NOSE_BOTTOM = par("body.nose_bottom_height", 0.66, "m", "design", "front/rear bumper lower edge, sets ~39 deg approach")
X_TUB = par("body.tub_half_length", 0.60, "m", "design", "clear of the axle motors (end at ~0.65 m) and tyres")
Y_TUB = par("body.tub_half_width", 1.00, "m", "design", "")
Y_KEEL = par("body.keel_half_width", 0.38, "m", "derived", "inside the steered-tyre sweep (~0.45 m at 35 deg lock)")
X_NOSE = par("body.nose_start", 1.97, "m", "derived", "ahead of the steered front tyre sweep (1.93 m)")

hz = lambda h: h - R_COLL  # noqa: E731  height above ground -> base_link z
COLLISION_BOXES = []  # [cx, cy, cz, sx, sy, sz]


def box(x0, x1, y0, y1, h0, h1):
    COLLISION_BOXES.append([round((x0 + x1) / 2, 4), round((y0 + y1) / 2, 4), round((hz(h0) + hz(h1)) / 2, 4),
                            round(x1 - x0, 4), round(y1 - y0, 4), round(h1 - h0, 4)])


HL = L_OVERALL / 2.0
box(-HL + 0.05, HL - 0.05, -W_OVERALL / 2, W_OVERALL / 2, H_DECK_BOTTOM, H_DECK_TOP)       # deck / fenders
box(-X_TUB, X_TUB, -Y_TUB, Y_TUB, H_TUB_BOTTOM, H_DECK_BOTTOM)                              # centre tub
box(-X_NOSE, X_NOSE, -Y_KEEL, Y_KEEL, H_KEEL_BOTTOM, H_DECK_BOTTOM)                          # keel / tunnels
box(X_NOSE, HL, -1.0, 1.0, H_NOSE_BOTTOM + 0.12, H_DECK_BOTTOM)                             # front nose
box(-HL, -X_NOSE, -1.0, 1.0, H_NOSE_BOTTOM + 0.12, H_DECK_BOTTOM)                           # rear tail

# ==========================================================================
# 6. Sprung mass budget (2020 prototype, cargo deck, no payload)
# ==========================================================================
M_SPRUNG = M_CURB_2020 - M_UNSPRUNG
# name: (mass, (x, y, h_above_ground), (sx, sy, sz), status note)
SPRUNG_ITEMS: List[Tuple[str, float, Tuple[float, float, float], Tuple[float, float, float], str]] = [
    ("diesel genset (engine + 110 kW generator)", 360.0, (1.45, 0.0, 1.03), (1.05, 0.70, 0.30),
     "~2-3 L turbodiesel (~120 kW shaft) + PM generator, front compartment above the front axle"),
    ("40 kWh NMC battery + BTMS", 300.0, (0.0, 0.0, 0.66), (1.10, 1.80, 0.32),
     "120-150 Wh/kg pack level; low in the centre tub"),
    ("fuel (~110 L) + tank", 117.0, (-1.40, 0.0, 1.03), (0.60, 0.70, 0.30),
     "450 km range (PZ26) at ~25 L/100 km"),
    ("power electronics (2 traction inverters, generator inverter, DC/DC, 400 VAC)", 120.0,
     (-1.85, 0.0, 1.03), (0.80, 0.70, 0.30), "rear compartment"),
    ("cooling (radiators, BTMS chiller, fans)", 70.0, (2.15, 0.0, 1.00), (0.35, 1.60, 0.45), "behind the front grille"),
    ("24 V system, computers, radios, harness", 140.0, (0.0, 0.0, 1.00), (1.00, 1.60, 0.30), "centre tub, upper"),
    ("sensors, PTZ mast, lights", 45.0, (1.6, 0.0, 1.55), (1.0, 1.6, 0.4), "front sensor area"),
    ("suspension (sprung half of links, coil-overs, anti-roll bars)", 70.0, (0.0, 0.0, 0.95), (2.7, 1.3, 0.3), ""),
    ("deck plates and railing", 300.0, (0.0, 0.0, 1.30), (4.6, 2.2, 0.15), "steel/aluminium deck"),
    ("left side panels, fender, bumper half", 110.0, (0.0, 0.95, 1.00), (4.6, 0.15, 0.5), ""),
    ("right side panels, fender, bumper half", 110.0, (0.0, -0.95, 1.00), (4.6, 0.15, 0.5), ""),
]
m_listed = sum(i[1] for i in SPRUNG_ITEMS)
M_FRAME = M_SPRUNG - m_listed
SPRUNG_ITEMS.append(("frame (ladder/space frame) + tub structure + misc", M_FRAME, (0.0, 0.0, 0.92),
                     (4.3, 1.4, 0.55), "remainder to the published 3500 kg kerb mass"))


def composite(items):
    m = sum(i[1] for i in items)
    cx = sum(i[1] * i[2][0] for i in items) / m
    cy = sum(i[1] * i[2][1] for i in items) / m
    ch = sum(i[1] * i[2][2] for i in items) / m
    ixx = iyy = izz = ixy = ixz = iyz = 0.0
    for _, mi, (x, y, h), (sx, sy, sz), _ in items:
        dx, dy, dz = x - cx, y - cy, h - ch
        ixx += mi * (sy * sy + sz * sz) / 12 + mi * (dy * dy + dz * dz)
        iyy += mi * (sx * sx + sz * sz) / 12 + mi * (dx * dx + dz * dz)
        izz += mi * (sx * sx + sy * sy) / 12 + mi * (dx * dx + dy * dy)
        ixy -= mi * dx * dy
        ixz -= mi * dx * dz
        iyz -= mi * dy * dz
    return m, (cx, cy, ch), dict(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)


M_CH, COM_CH, I_CH = composite(SPRUNG_ITEMS)
H_CG_SPRUNG = COM_CH[2]

# Axle inertia (housing as a 1.55 m tube + diff + motor), about its own COM
ax_len = KP_TRACK
I_AXLE = dict(
    ixx=M_AXLE_HOUSING * ax_len ** 2 / 12 + M_MOTOR * (0.12 ** 2) + M_STEER_ACT * 0.3 ** 2,
    iyy=M_AXLE_HOUSING * 0.12 ** 2 + M_MOTOR * (0.42 ** 2 + 0.12 ** 2 / 2),
    izz=M_AXLE_HOUSING * ax_len ** 2 / 12 + M_MOTOR * 0.42 ** 2 + M_STEER_ACT * 0.3 ** 2,
)
AXLE_COM_X_TOWARD_CENTRE = M_MOTOR * 0.42 / M_AXLE_LINK  # motor sits 0.42 m toward the vehicle centre

# Whole-vehicle CG
m_total = M_CH + M_UNSPRUNG
h_cg_total = (M_CH * H_CG_SPRUNG + M_UNSPRUNG * R_COLL) / m_total
x_cg_total = (M_CH * COM_CH[0]) / m_total
par("mass.cg_height_kerb", h_cg_total, "m", "derived", "composite of the mass budget")
par("mass.cg_x_kerb", x_cg_total, "m", "derived", "+ forward of mid-wheelbase")

# ==========================================================================
# 7. Axle loads
# ==========================================================================
front_frac = 0.5 + x_cg_total / WHEELBASE
F_AXLE_F = m_total * G * front_frac
F_AXLE_R = m_total * G * (1 - front_frac)
# Sprung load carried by each axle's springs (unsprung weight goes straight to the tyres)
front_frac_s = 0.5 + COM_CH[0] / WHEELBASE
W_SPRUNG_F = M_CH * G * front_frac_s
W_SPRUNG_R = M_CH * G * (1 - front_frac_s)
par("suspension.static_load_front", W_SPRUNG_F, "N", "derived", "sprung weight on the front springs (kerb 2020)")
par("suspension.static_load_rear", W_SPRUNG_R, "N", "derived", "sprung weight on the rear springs (kerb 2020)")

# ==========================================================================
# 8. Suspension design
# ==========================================================================
F_RIDE = par("suspension.ride_frequency", 1.50, "Hz", "design",
             "off-road military practice 1.2-1.7 Hz at kerb; drops to ~1.3 Hz at max payload")
ZETA = par("suspension.damping_ratio", 0.35, "-", "design",
           "nominal setting of the controlled (semi-active) dampers; range 0.2-0.6")
m_corner = M_CH / 4.0
K_RIDE = m_corner * (2 * math.pi * F_RIDE) ** 2
K_WHEEL = par("suspension.wheel_rate", K_RIDE * K_TYRE / (K_TYRE - K_RIDE), "N/m", "derived",
              "k_w = K_ride k_t / (k_t - K_ride), K_ride = m_corner (2 pi f)^2")
C_WHEEL = par("suspension.damping_wheel", 2 * ZETA * math.sqrt(K_WHEEL * m_corner), "N s/m", "derived",
              "c = 2 zeta sqrt(k_w m_corner)")
SAG = W_SPRUNG_F / 2.0 / K_WHEEL
REBOUND = par("suspension.travel_rebound", round(SAG, 3), "m", "design",
              "coil spring just unloaded at full droop (no preload) => droop = static sag")
BUMP = par("suspension.travel_bump", round(STROKE - REBOUND, 3), "m", "derived", "stroke - droop")
S_SPRING = par("suspension.spring_track", 1.24, "m", "design",
               "coil-overs on the axle tubes just inboard of the knuckles (clear of the steered tyre)")
K_ARB = par("suspension.anti_roll_stiffness", 15_000.0, "N m/rad", "design",
            "per axle; brings the roll gradient to ~6 deg/g with the RWS fitted")
H_RC = par("suspension.roll_center_height", 0.10, "m", "design",
           "Panhard rod 0.10 m above the axle centreline, horizontal at ride height")
H_RC_GROUND = R_COLL + H_RC
K_ROLL_AXLE = 2 * K_WHEEL * (S_SPRING / 2) ** 2 + K_ARB
roll_arm = H_CG_SPRUNG - H_RC_GROUND
ROLL_GRAD = M_CH * G * roll_arm / (2 * K_ROLL_AXLE - M_CH * G * roll_arm)
par("suspension.roll_gradient", math.degrees(ROLL_GRAD), "deg/g", "derived",
    "m_s g (h_s - h_rc) / (K_phi - m_s g (h_s - h_rc))")
ART = par("suspension.articulation_limit_deg", math.degrees((BUMP + REBOUND) / TRACK), "deg", "derived",
          "one wheel at full bump, the other at full droop: (bump + droop) / track")
f_hop = math.sqrt(2 * (K_WHEEL + K_TYRE) / M_UNSPRUNG_AXLE) / (2 * math.pi)
par("suspension.wheel_hop_frequency", f_hop, "Hz", "derived", "unsprung axle on springs + tyres")

# Linkage hard points (axle frame: origin axle centre, x toward the vehicle
# OUTSIDE i.e. forward for the front axle; the rear axle is the mirror image).
LINKAGE = {
    "lower_link": {"axle": [-0.05, 0.42, -0.13], "chassis": [-0.95, 0.40, -0.015], "diameter": 0.06},
    "upper_link": {"axle": [-0.10, 0.10, 0.24], "chassis": [-0.80, 0.10, 0.36], "diameter": 0.055},
    "panhard_rod": {"axle": [0.10, -0.62, 0.10], "chassis": [0.10, 0.55, 0.10], "diameter": 0.05},
    "coilover": {"axle": [0.0, 0.62, 0.06], "chassis": [0.0, 0.58, 0.70], "body_diameter": 0.075,
                 "spring_diameter": 0.115, "rod_diameter": 0.032},
    "anti_roll_bar": {"arm_length": 0.35, "bar_width": 1.05, "diameter": 0.032},
}
lo, up = LINKAGE["lower_link"], LINKAGE["upper_link"]
LINK_LEN = {k: math.dist(v["axle"], v["chassis"]) for k, v in LINKAGE.items() if "axle" in v}
# Side-view instant centre of the 3-link (lower vs upper link lines, x-z plane)
sl = (lo["chassis"][2] - lo["axle"][2]) / (lo["chassis"][0] - lo["axle"][0])
su = (up["chassis"][2] - up["axle"][2]) / (up["chassis"][0] - up["axle"][0])
x_ic = (up["axle"][2] - lo["axle"][2] + sl * lo["axle"][0] - su * up["axle"][0]) / (sl - su)
z_ic = lo["axle"][2] + sl * (x_ic - lo["axle"][0])
panhard_shift = (BUMP ** 2) / (2 * LINK_LEN["panhard_rod"])

# ==========================================================================
# 9. Steering
# ==========================================================================
STEER_MAX = par("steering.max_angle_deg", 35.0, "deg", "estimated",
                "Dana 60 ball-joint knuckles with ~1 m tyres: 33-38 deg lock")
STEER_RATE = par("steering.max_rate_deg_s", 40.0, "deg/s", "estimated", "electric linear actuator, ~3.5 s lock-to-lock")
ARM = par("steering.arm_length", 0.155, "m", "estimated", "Dana 60 knuckle steering arm")


def trapezoid_error(beta):
    t = SteeringTrapezoid(KP_TRACK, ARM, beta, 1.0)
    half_l = WHEELBASE / 2
    err = 0.0
    n = 0
    for i in range(1, 36):
        din = math.radians(i * STEER_MAX / 35.0)
        ideal = math.atan(1.0 / (1.0 / math.tan(din) + KP_TRACK / half_l))
        try:
            err += (t.outer_from_inner(din) - ideal) ** 2
        except ValueError:
            return 1e9
        n += 1
    return math.sqrt(err / n)


betas = [math.radians(b / 10) for b in range(100, 450)]
BETA = min(betas, key=trapezoid_error)
par("steering.trapezoid_angle_deg", round(math.degrees(BETA), 2), "deg", "design",
    "optimised for counter-phase 4WS (ICR on the mid-wheelbase line), min RMS outer-angle error")
TRAP_RMS = math.degrees(trapezoid_error(BETA))
BETA_JK = math.atan((KP_TRACK / 2) / 2.946)
TRAP_RMS_JK = math.degrees(trapezoid_error(BETA_JK))

geom = VehicleGeometry(WHEELBASE / 2, -WHEELBASE / 2, KP_TRACK, TRACK, R_ROLL, math.radians(STEER_MAX),
                       "trapezoid", ARM, BETA)
geom_ideal = VehicleGeometry(WHEELBASE / 2, -WHEELBASE / 2, KP_TRACK, TRACK, R_ROLL, math.radians(STEER_MAX),
                             "ideal", ARM, BETA)
kin = FourWheelSteeringKinematics(geom)
kin_ideal = FourWheelSteeringKinematics(geom_ideal)
TURN = {m: kin.turning_radius(m) for m in ("front", "counter")}
TURN_IDEAL = {m: kin_ideal.turning_radius(m) for m in ("front", "counter")}
par("steering.turning_radius_4ws", TURN["counter"], "m", "derived", "outer tyre centre, counter-phase, full lock")
par("steering.turning_radius_2ws", TURN["front"], "m", "derived", "outer tyre centre, front-only, full lock")

# ==========================================================================
# 10. Drive train
# ==========================================================================
P_CONT = par("drive.power_continuous", 90_000.0, "W", "published", "ARMYREC24", "total, 2 motors")
P_PEAK = par("drive.power_peak", 220_000.0, "W", "published", "ARMYREC24", "total, 2 motors")
RPM_MIN_FULL = par("drive.motor_speed_full_load", 4500.0, "rpm", "published", "TENDER18", ">= 4500 rpm")
V_DESIGN_TOP = 52.0 / 3.6
RED = par("drive.reduction_ratio", round((RPM_MIN_FULL * 2 * math.pi / 60) / (V_DESIGN_TOP / R_ROLL) / RATIO_AXLE, 2),
          "-", "estimated", "motor reaches 4500 rpm at ~52 km/h ('over 50 km/h')")
RATIO_TOTAL = RED * RATIO_AXLE
BASE_RPM = par("drive.motor_base_speed", 1800.0, "rpm", "estimated", "typical traction PMSM corner speed")
T_MOTOR_PEAK = (P_PEAK / 2) / (BASE_RPM * 2 * math.pi / 60)
ETA = 0.92
T_WHEEL_PEAK = par("drive.wheel_torque_limit", round(T_MOTOR_PEAK * RATIO_TOTAL * ETA / 2, -1), "N m", "derived",
                   "per wheel: (P_peak/2)/omega_base * ratio * eta / 2")
W_WHEEL_MAX = par("drive.wheel_max_speed", round(V_DESIGN_TOP / R_ROLL, 2), "rad/s", "derived", "52 km/h / r_roll")
F_TRACT = 4 * T_WHEEL_PEAK / R_ROLL
par("drive.tractive_force_peak", F_TRACT, "N", "derived", "motor-limited; traction-limited at mu m g")
grade_cont = (P_CONT / V_MAX - 900.0) / (m_total * G)

# ==========================================================================
# 11. Performance checks
# ==========================================================================
SSF = TRACK / (2 * h_cg_total)
tilt = math.degrees(math.atan(SSF))
CLR_DIFF = R_COLL - DIFF_R


def tangent_angle(px, ph, cx, ch, r):
    """Angle below horizontal of the lower tangent from (px, ph) to circle."""
    d = math.hypot(cx - px, ch - ph)
    return math.degrees(math.atan2(ph - ch, abs(cx - px)) + math.asin(r / d))


APPROACH = tangent_angle(HL, H_NOSE_BOTTOM, WHEELBASE / 2, R_COLL, R_FREE)
BREAKOVER = 2 * tangent_angle(0.0, H_TUB_BOTTOM, WHEELBASE / 2, R_COLL, R_FREE)
par("performance.approach_angle", APPROACH, "deg", "derived", "nose lower edge tangent to the front tyre")
par("performance.breakover_angle", BREAKOVER, "deg", "derived", "belly centre tangent to both tyres")
par("performance.ground_clearance", CLR_DIFF, "m", "derived", "under the differential housings")

# ==========================================================================
# 12. Sensors (positions in base_link; heights above ground converted)
# ==========================================================================
z_deck = hz(H_DECK_TOP)
z_rail = hz(H_DECK_TOP + H_RAIL)


def S(name, kind, xyz, rpy, **kw):
    d = {"name": name, "type": kind, "xyz": [round(v, 4) for v in xyz], "rpy": [round(v, 4) for v in rpy]}
    d.update(kw)
    return d


SENSORS = [
    # Two 3D lidars (JANES20/MILMAG20): front and rear, on sensor brackets above the deck edge.
    S("front_lidar", "lidar", (HL - 0.10, 0.0, z_deck + 0.22), (0, 0, 0),
      model="32-channel spinning 3D lidar (Ouster OS1-32 class)", hfov_deg=360, vfov_deg=45,
      h_samples=1024, v_samples=32, range_min=0.5, range_max=120.0, rate=10.0, variants=["2020", "2023"]),
    S("rear_lidar", "lidar", (-HL + 0.10, 0.0, z_deck + 0.22), (0, 0, math.pi),
      model="32-channel spinning 3D lidar (Ouster OS1-32 class)", hfov_deg=360, vfov_deg=45,
      h_samples=1024, v_samples=32, range_min=0.5, range_max=120.0, rate=10.0, variants=["2020", "2023"]),
    # Driving cameras front and rear (POSTER21 'front and rear driving cameras').
    S("front_drive_camera", "camera", (HL - 0.02, 0.0, hz(1.10)), (0, math.radians(8), 0),
      hfov_deg=100, width=1280, height=720, rate=30.0, variants=["2020", "2023"]),
    S("rear_drive_camera", "camera", (-HL + 0.02, 0.0, hz(1.10)), (0, math.radians(8), math.pi),
      hfov_deg=100, width=1280, height=720, rate=30.0, variants=["2020", "2023"]),
    # Side / perimeter cameras, 2 per side (POSTER21; D24_23 'four perimeter cameras').
    S("left_front_camera", "camera", (1.10, W_OVERALL / 2 - 0.01, hz(1.28)), (0, math.radians(25), math.radians(60)),
      hfov_deg=120, width=1280, height=720, rate=15.0, variants=["2020", "2023"]),
    S("left_rear_camera", "camera", (-1.10, W_OVERALL / 2 - 0.01, hz(1.28)), (0, math.radians(25), math.radians(120)),
      hfov_deg=120, width=1280, height=720, rate=15.0, variants=["2020", "2023"]),
    S("right_front_camera", "camera", (1.10, -W_OVERALL / 2 + 0.01, hz(1.28)), (0, math.radians(25), math.radians(-60)),
      hfov_deg=120, width=1280, height=720, rate=15.0, variants=["2020", "2023"]),
    S("right_rear_camera", "camera", (-1.10, -W_OVERALL / 2 + 0.01, hz(1.28)), (0, math.radians(25), math.radians(-120)),
      hfov_deg=120, width=1280, height=720, rate=15.0, variants=["2020", "2023"]),
    # Top-view cameras on the deck railing (POSTER21), looking down at the wheels/terrain ahead and behind.
    S("top_front_camera", "camera", (HL - 0.35, 0.0, z_rail + 0.05), (0, math.radians(55), 0),
      hfov_deg=110, width=1280, height=720, rate=15.0, variants=["2020"]),
    S("top_rear_camera", "camera", (-HL + 0.35, 0.0, z_rail + 0.05), (0, math.radians(55), math.pi),
      hfov_deg=110, width=1280, height=720, rate=15.0, variants=["2020"]),
    # 2024 stereo pair (PIAP24), front, under the lidar.
    S("front_stereo_camera", "stereo", (HL - 0.04, 0.0, z_deck + 0.06), (0, math.radians(5), 0),
      baseline=0.12, hfov_deg=90, width=1280, height=720, rate=15.0, variants=["2023"]),
    S("gnss", "navsat", (-1.60, 0.30, z_rail + 0.05), (0, 0, 0), rate=10.0, variants=["2020", "2023"]),
    S("imu", "imu", (0.0, 0.0, hz(0.95)), (0, 0, 0), rate=200.0, variants=["2020", "2023"]),
]
PTZ = {"mount_xyz": [round(HL - 0.55, 4), 0.0, round(z_deck, 4)], "mast_height": 0.55, "head_size": 0.22,
       "hfov_wide_deg": 60.0, "hfov_tele_deg": 3.0, "tilt_min_deg": -45.0, "tilt_max_deg": 90.0,
       "variants": ["2020"]}

# ==========================================================================
# 13. Payload modules
# ==========================================================================
PAYLOADS = {
    "cargo": {"mass": 600.0, "box": [1.8, 1.7, 0.6], "xyz": [-0.55, 0.0, round(z_deck + 0.30, 4)],
              "note": "crates; total up to 1000 kg (D24TV21) - default 600 kg"},
    "rws": {"model": "AREX ZMU-03 12.7 mm", "mass": 205.0, "base_xyz": [-0.20, 0.0, round(z_deck, 4)],
            "base_diameter": 0.62, "height": 0.66, "length": 1.63, "yaw_height": 0.18, "pitch_height": 0.40,
            "pitch_min_deg": -20.0, "pitch_max_deg": 60.0, "note": "205 kg incl. weapon (MILMAG_ZMU03); envelope "
            "from ZMU-05 1630x750x660 mm (WB_ZMU05)"},
    "uav_pad": {"mass": 45.0, "size": [1.2, 1.2, 0.05], "xyz": [-1.25, 0.0, round(z_deck + 0.20, 4)],
                "note": "UAV landing pad option (MILMAG20); UAV relay was part of the HUNTeR project"},
}
UPGRADE_2023 = {"mass": round(M_CURB_2023 - M_CURB_2020 - PAYLOADS["rws"]["mass"], 1),
                "xyz": [round(COM_CH[0], 4), 0.0, round(hz(H_CG_SPRUNG), 4)],
                "note": "armour/electronics/sensor growth to the 'over 4 t' armed kerb mass (D24_23)"}

# ==========================================================================
# 14. Assemble vehicle.yaml
# ==========================================================================
r4 = lambda v: round(float(v), 4)  # noqa: E731

VEHICLE = {
    "meta": {
        "model": "Lukasiewicz-PIAP HUNTeR (replica)",
        "generated_by": "tools/vehicle_design.py - edit that file and re-run, do not edit by hand",
        "frames": "base_link: mid-wheelbase, wheel-centre height at kerb static, x forward, z up",
    },
    "chassis": {"wheelbase": WHEELBASE, "track": r4(TRACK)},
    "body": {
        "length": L_OVERALL, "width": W_OVERALL, "deck_height": H_DECK_TOP, "wheel_arch_height": H_DECK_BOTTOM,
        "railing_height": H_RAIL, "belly_height": H_TUB_BOTTOM, "keel_height": H_KEEL_BOTTOM,
        "nose_bottom_height": H_NOSE_BOTTOM, "tub_half_length": X_TUB, "tub_half_width": Y_TUB,
        "keel_half_width": Y_KEEL, "nose_start": X_NOSE, "ground_z": r4(Z_GROUND),
        "collision_boxes": COLLISION_BOXES,
    },
    "mass": {
        "curb_2020": M_CURB_2020, "curb_2023_armed": M_CURB_2023, "payload_max": PAYLOAD_MAX,
        "chassis": r4(M_CH),
        "chassis_com": [r4(COM_CH[0]), r4(COM_CH[1]), r4(hz(COM_CH[2]))],
        "chassis_inertia": {k: r4(v) for k, v in I_CH.items()},
        "upgrade_2023": UPGRADE_2023,
    },
    "wheel": {
        "diameter": TYRE_OD, "width": TYRE_W, "rim_diameter": r4(RIM_D), "rim_width": r4(RIM_W),
        "offset": WHEEL_OFFSET, "rolling_radius": r4(R_ROLL), "static_loaded_radius": r4(R_STATIC),
        "collision_radius": r4(R_COLL), "mass": M_WHEEL, "ixx": r4(WHEEL_IXX), "iyy": r4(WHEEL_IYY),
        "vertical_stiffness": K_TYRE, "mu": MU, "mu2": MU, "bolt_count": 8, "bolt_circle": r4(6.5 * IN),
    },
    "axle": {
        "mass": M_AXLE_LINK, "knuckle_mass": M_KNUCKLE, "com_x_toward_centre": r4(AXLE_COM_X_TOWARD_CENTRE),
        "com_z_offset": 0.03, "ixx": r4(I_AXLE["ixx"]), "iyy": r4(I_AXLE["iyy"]), "izz": r4(I_AXLE["izz"]),
        "wms_width": r4(WMS), "tube_diameter": r4(TUBE_D), "diff_radius": DIFF_R, "diff_offset_y": DIFF_Y,
        "housing_radius": r4(TUBE_D / 2), "brake_rotor_diameter": ROTOR_D,
        "motor": {"radius": 0.12, "length": 0.50, "x_toward_centre": 0.42, "z": 0.10},
    },
    "suspension": {
        "type": "dependent rigid axles, 3-link + Panhard rod, coil-over springs, controlled dampers",
        "stroke": STROKE, "travel_bump": BUMP, "travel_rebound": REBOUND,
        "wheel_rate": r4(K_WHEEL), "damping_wheel": r4(C_WHEEL), "ride_frequency": F_RIDE, "damping_ratio": ZETA,
        "spring_track": S_SPRING, "anti_roll_stiffness": K_ARB, "roll_center_height": H_RC,
        "articulation_limit_deg": r4(ART),
        "static_load_front": r4(W_SPRUNG_F), "static_load_rear": r4(W_SPRUNG_R),
        "joint_effort_limit": 200000.0, "joint_velocity_limit": 3.0, "coulomb_friction": 0.0,
        "linkage": LINKAGE,
    },
    "steering": {
        "max_angle_deg": STEER_MAX, "max_rate_deg_s": STEER_RATE, "kingpin_track": r4(KP_TRACK),
        "kingpin_inclination_deg": KPI, "caster_deg": CASTER, "arm_length": ARM,
        "trapezoid_angle_deg": round(math.degrees(BETA), 2), "arm_height": -0.07, "linkage": "trapezoid",
        "sim_position_gain": 20.0, "knuckle_torque_limit": 6000.0, "knuckle_damping": 80.0,
        "adaptive": {"low_speed": 2.0, "high_speed": 8.0},
    },
    "drive": {
        "axle_ratio": RATIO_AXLE, "reduction_ratio": RED, "power_continuous": P_CONT, "power_peak": P_PEAK,
        "wheel_torque_limit": T_WHEEL_PEAK, "wheel_max_speed": W_WHEEL_MAX, "wheel_bearing_damping": 1.0,
    },
    "limits": {
        "max_speed": r4(V_MAX), "max_reverse_speed": 4.0, "max_accel": 2.0, "max_decel": 4.0,
        "max_yaw_rate": 1.0, "max_lateral_accel": 3.5, "max_crab_speed": 3.0,
    },
    "sensors": SENSORS,
    "ptz": PTZ,
    "payloads": PAYLOADS,
}


# ==========================================================================
# 15. Report
# ==========================================================================
def report() -> str:
    o = io.StringIO()
    w = o.write
    w("# HUNTeR replica: design report\n\n")
    w("_Generated by `tools/vehicle_design.py`. Do not edit by hand._\n\n")
    w("Every number in the URDF comes from this report. Status: **published** (PIAP or press quoting PIAP), "
      "**derived** (computed from published values), **estimated** (engineering estimate where nothing is "
      "published), **design** (an engineering choice for this replica that respects the published constraints).\n\n")
    w("## Key results\n\n")
    rows = [
        ("Kerb mass (2020 prototype)", f"{m_total:.0f} kg", "published 3500 kg"),
        ("Sprung / unsprung", f"{M_CH:.0f} / {M_UNSPRUNG:.0f} kg", "mass budget"),
        ("CG height (kerb)", f"{h_cg_total:.3f} m", ""),
        ("CG x (from mid-wheelbase)", f"{x_cg_total:+.3f} m", f"front axle share {front_frac * 100:.1f} %"),
        ("Track (tyre centres)", f"{TRACK:.4f} m", "Dana 60 WMS 70 in + wheel offset"),
        ("Kingpin track", f"{KP_TRACK:.3f} m", ""),
        ("Rolling / static loaded radius", f"{R_ROLL:.4f} / {R_STATIC:.4f} m", f"tyre deflection {TYRE_DEFL * 1000:.1f} mm"),
        ("Wheel rate / damping per corner", f"{K_WHEEL / 1000:.1f} kN/m / {C_WHEEL / 1000:.2f} kN s/m",
         f"{F_RIDE} Hz, zeta {ZETA}"),
        ("Travel bump / droop", f"{BUMP * 1000:.0f} / {REBOUND * 1000:.0f} mm", "published stroke 250 mm"),
        ("Roll gradient", f"{math.degrees(ROLL_GRAD):.1f} deg/g", f"roll centre {H_RC_GROUND:.3f} m"),
        ("Axle articulation", f"+/-{ART:.1f} deg", ""),
        ("Wheel-hop frequency", f"{f_hop:.1f} Hz", "heavy e-axles"),
        ("Steering lock", f"{STEER_MAX:.0f} deg", f"trapezoid beta {math.degrees(BETA):.1f} deg, RMS Ackermann "
         f"error {TRAP_RMS:.2f} deg (stock Jeep JK arms: {TRAP_RMS_JK:.1f} deg)"),
        ("Turning radius 4WS / 2WS", f"{TURN['counter']:.2f} / {TURN['front']:.2f} m",
         f"outer tyre centre; ideal Ackermann {TURN_IDEAL['counter']:.2f} / {TURN_IDEAL['front']:.2f} m"),
        ("Total reduction motor-wheel", f"{RATIO_TOTAL:.2f}", f"{RED} x 5.38"),
        ("Wheel torque limit", f"{T_WHEEL_PEAK:.0f} N m", f"tractive force {F_TRACT / 1000:.1f} kN "
         f"({F_TRACT / (m_total * G):.2f} g)"),
        ("Continuous grade at 50 km/h", f"{grade_cont * 100:.0f} %", "90 kW"),
        ("Static stability factor", f"{SSF:.2f}", f"tilt-table angle {tilt:.1f} deg"),
        ("Ground clearance (diffs)", f"{CLR_DIFF:.3f} m", ""),
        ("Approach / departure angle", f"{APPROACH:.0f} deg", "symmetric"),
        ("Ramp breakover angle", f"{BREAKOVER:.0f} deg", ""),
        ("3-link instant centre", f"{x_ic:+.2f} m, {z_ic:+.2f} m", "axle frame, x outward"),
        ("Panhard lateral shift at full bump", f"{panhard_shift * 1000:.1f} mm", "neglected in the URDF"),
    ]
    w("| Quantity | Value | Note |\n|---|---|---|\n")
    for r in rows:
        w(f"| {r[0]} | {r[1]} | {r[2]} |\n")
    w("\n## Parameters\n\n| Parameter | Value | Unit | Status | Source / reasoning |\n|---|---|---|---|---|\n")
    for k, p in REGISTRY.items():
        v = p.value
        vs = f"{v:.4g}" if isinstance(v, float) else str(v)
        src = p.source + (f". {p.note}" if p.note else "")
        w(f"| `{k}` | {vs} | {p.unit} | {p.status} | {src} |\n")
    w("\n## Sprung mass budget (2020 prototype)\n\n| Item | Mass [kg] | Position x, y, h [m] | Note |\n|---|---|---|---|\n")
    for name, m, pos, _, note in SPRUNG_ITEMS:
        w(f"| {name} | {m:.0f} | {pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:.2f} | {note} |\n")
    w(f"| **total sprung** | **{M_CH:.0f}** | COM {COM_CH[0]:+.3f}, {COM_CH[1]:+.3f}, {COM_CH[2]:.3f} | |\n")
    w(f"\nChassis inertia about its COM [kg m^2]: Ixx {I_CH['ixx']:.0f}, Iyy {I_CH['iyy']:.0f}, Izz {I_CH['izz']:.0f}, "
      f"Ixz {I_CH['ixz']:.1f}.\n\n")
    w("Unsprung, per axle: housing/diff/inner shafts {:.0f} kg, motor + reduction {:.0f} kg, steering actuator "
      "{:.0f} kg, links (half) {:.0f} kg => axle link {:.0f} kg; knuckles 2 x {:.0f} kg; wheels 2 x {:.0f} kg "
      "=> {:.0f} kg.\n\n".format(M_AXLE_HOUSING, M_MOTOR, M_STEER_ACT, M_LINKS_UNSPRUNG, M_AXLE_LINK, M_KNUCKLE,
                                M_WHEEL, M_UNSPRUNG_AXLE))
    w("## Sources\n\n")
    for k, v in SOURCES.items():
        w(f"- **{k}**: {v}\n")
    return o.getvalue()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    out_yaml = os.path.join(ROOT, "hunter_description", "config", "vehicle.yaml")
    out_md = os.path.join(ROOT, "docs", "DESIGN_REPORT.md")
    header = ("# GENERATED by tools/vehicle_design.py - do not edit by hand.\n"
              "# Sources, status and derivation of every value: docs/DESIGN_REPORT.md\n")
    y = header + yaml.safe_dump(VEHICLE, sort_keys=False, default_flow_style=None, width=110)
    md = report()
    if a.check:
        ok = open(out_yaml).read() == y and open(out_md).read() == md
        print("up to date" if ok else "OUT OF DATE: run tools/vehicle_design.py")
        return 0 if ok else 1
    with open(out_yaml, "w") as f:
        f.write(y)
    with open(out_md, "w") as f:
        f.write(md)
    print(f"wrote {out_yaml}\nwrote {out_md}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
