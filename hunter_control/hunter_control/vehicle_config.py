"""Load the single-source vehicle parameter file (hunter_description/config/vehicle.yaml)."""

from __future__ import annotations

import math
import os
from typing import Any, Dict, Optional

import yaml

from .kinematics import VehicleGeometry


def default_vehicle_file() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("hunter_description"), "config", "vehicle.yaml")
    except Exception:  # pragma: no cover - running outside a ROS workspace
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, "..", "..", "hunter_description", "config", "vehicle.yaml"))


def load_vehicle(path: Optional[str] = None) -> Dict[str, Any]:
    with open(path or default_vehicle_file(), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def geometry_from_config(cfg: Dict[str, Any], linkage: Optional[str] = None) -> VehicleGeometry:
    ch = cfg["chassis"]
    wh = cfg["wheel"]
    st = cfg["steering"]
    return VehicleGeometry(
        front_axle_x=0.5 * ch["wheelbase"],
        rear_axle_x=-0.5 * ch["wheelbase"],
        kingpin_track=st["kingpin_track"],
        wheel_track=ch["track"],
        wheel_radius=wh["rolling_radius"],
        max_steer=math.radians(st["max_angle_deg"]),
        linkage=linkage or st.get("linkage", "trapezoid"),
        arm_length=st["arm_length"],
        trapezoid_angle=math.radians(st.get("trapezoid_angle_deg", 0.0)),
    )
