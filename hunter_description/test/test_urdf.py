"""Static checks of the generated HUNTeR URDF in every configuration.

Run with:  colcon test --packages-select hunter_description
or:        python3 -m pytest hunter_description/test   (xacro and check_urdf on PATH)
"""

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(PKG, "urdf", "hunter.urdf.xacro")
VEHICLE = yaml.safe_load(open(os.path.join(PKG, "config", "vehicle.yaml")))

CONFIGS = [
    ("", 3500.0),
    ("payload:=cargo", 3500.0 + VEHICLE["payloads"]["cargo"]["mass"]),
    ("payload:=uav_pad", 3500.0 + VEHICLE["payloads"]["uav_pad"]["mass"]),
    ("variant:=2023 payload:=rws", VEHICLE["mass"]["curb_2023_armed"]),
    ("sensors:=false visual_detail:=primitive", 3500.0),
    ("sim_mode:=gz controllers_file:=/tmp/c.yaml variant:=2023 payload:=rws", VEHICLE["mass"]["curb_2023_armed"]),
    ("sim_mode:=mock", 3500.0),
]
# Sensors, PTZ and small parts are not in the 3500 kg mass budget: allow 1 %.
MASS_TOL = 0.01


def xacro(args):
    if shutil.which("xacro") is None:
        pytest.skip("xacro not available")
    # Includes are resolved relative to the top-level file ($(dirname)); pass the
    # parameter file explicitly so the source tree under test is used throughout.
    return subprocess.check_output(
        ["xacro", XACRO, f"vehicle_file:={os.path.join(PKG, 'config', 'vehicle.yaml')}"] + args.split(), text=True)


@pytest.fixture(scope="module", params=CONFIGS, ids=[c[0] or "default" for c in CONFIGS])
def model(request):
    args, mass = request.param
    return ET.fromstring(xacro(args)), mass, args


def test_check_urdf(model, tmp_path):
    if shutil.which("check_urdf") is None:
        pytest.skip("check_urdf not available")
    root, _, _ = model
    f = tmp_path / "h.urdf"
    f.write_text(ET.tostring(root, encoding="unicode"))
    out = subprocess.run(["check_urdf", str(f)], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr


def test_total_mass(model):
    root, expected, _ = model
    total = sum(float(l.find("inertial/mass").get("value")) for l in root.findall("link")
                if l.find("inertial") is not None)
    assert abs(total - expected) / expected < MASS_TOL, f"total {total:.1f} kg, expected {expected:.1f} kg"


def test_inertias_physical(model):
    root, _, _ = model
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        i = inertial.find("inertia").attrib
        m = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                      [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                      [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
        ev = np.linalg.eigvalsh(m)
        assert ev.min() > 0, f"{link.get('name')}: inertia not positive definite"
        a, b, c = sorted(ev)
        assert a + b >= c * (1 - 1e-6), f"{link.get('name')}: violates triangle inequality"


def test_meshes_exist(model):
    root, _, _ = model
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        assert uri.startswith("package://hunter_description/")
        path = os.path.join(PKG, uri[len("package://hunter_description/"):])
        assert os.path.isfile(path), f"missing mesh {path}"


def test_ros2_control_joints_exist(model):
    root, _, args = model
    joints = {j.get("name") for j in root.findall("joint")}
    rc = root.find("ros2_control")
    if "sim_mode" not in args:
        assert rc is None
        return
    for j in rc.findall("joint"):
        assert j.get("name") in joints


def test_kinematic_chain_and_limits(model):
    root, _, _ = model
    j = {e.get("name"): e for e in root.findall("joint")}
    for axle in ("front", "rear"):
        heave = j[f"{axle}_axle_heave_joint"]
        assert heave.get("type") == "prismatic"
        lim = heave.find("limit")
        travel = float(lim.get("upper")) - float(lim.get("lower"))
        assert travel == pytest.approx(VEHICLE["suspension"]["stroke"], abs=1e-6)  # published 25 cm stroke
        assert j[f"{axle}_axle_roll_joint"].get("type") == "revolute"
        for side in ("left", "right"):
            st = j[f"{axle}_{side}_steer_joint"]
            assert float(st.find("limit").get("upper")) == pytest.approx(np.radians(VEHICLE["steering"]["max_angle_deg"]))
            assert j[f"{axle}_{side}_wheel_joint"].get("type") == "continuous"


def test_published_envelope():
    """Model geometry against the published 4.70 x 2.20 m, 2.70 m wheelbase, 1 m wheels."""
    root = ET.fromstring(xacro("sensors:=false"))
    joints = {e.get("name"): e for e in root.findall("joint")}
    xf = float(joints["front_axle_heave_joint"].find("origin").get("xyz").split()[0])
    xr = float(joints["rear_axle_heave_joint"].find("origin").get("xyz").split()[0])
    assert xf - xr == pytest.approx(2.70)
    assert VEHICLE["body"]["length"] == 4.70 and VEHICLE["body"]["width"] == 2.20
    assert VEHICLE["wheel"]["diameter"] == 1.0
    boxes = np.array(VEHICLE["body"]["collision_boxes"])
    assert (boxes[:, 0] + boxes[:, 3] / 2).max() <= 2.35 + 1e-6
    assert (boxes[:, 1] + boxes[:, 4] / 2).max() <= 1.10 + 1e-6


def test_design_outputs_up_to_date():
    tool = os.path.join(os.path.dirname(PKG), "tools", "vehicle_design.py")
    if not os.path.isfile(tool):
        pytest.skip("source tree tools not available")
    out = subprocess.run(["python3", tool, "--check"], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout
