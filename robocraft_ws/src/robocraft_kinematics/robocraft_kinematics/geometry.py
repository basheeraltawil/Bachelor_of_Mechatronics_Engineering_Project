"""Cell geometry model.

Mirrors ``robocraft_description/config/cell.yaml``.  The dataclasses carry the
same defaults so the library (and its tests) work without a ROS installation;
:func:`load_cell` overrides them from the YAML file.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    velocity: float
    effort: float

    def contains(self, value: float, tol: float = 1e-9) -> bool:
        return self.lower - tol <= value <= self.upper + tol


@dataclass(frozen=True)
class ArmGeometry:
    """Geometry of one 4-axis SCARA arm (J1, J2 revolute; J3 quill; J4 wrist)."""

    name: str
    base_xy: np.ndarray                 # column axis position in world [m]
    base_yaw: float                     # direction of J1 = 0 in world [rad]
    l1: float = 0.220
    l2: float = 0.220
    j1: JointLimit = JointLimit(-2.62, 2.62, 3.14, 4.0)
    j2: JointLimit = JointLimit(-2.53, 2.53, 3.14, 4.0)
    j3: JointLimit = JointLimit(0.0, 0.170, 0.20, 120.0)
    j4: JointLimit = JointLimit(-6.28, 6.28, 6.28, 1.0)
    tcp_z0: float = 0.175               # TCP height at q3 = 0 [m]

    @property
    def limits(self) -> List[JointLimit]:
        return [self.j1, self.j2, self.j3, self.j4]

    @property
    def reach(self) -> float:
        return self.l1 + self.l2


@dataclass(frozen=True)
class PlatformGeometry:
    leg_radius: float = 0.060
    plate_radius: float = 0.075
    initial_yaw: float = math.radians(30.0)
    grip_height: float = 0.070
    lift_height: float = 0.015

    def leg_angle(self, leg: int) -> float:
        """Angle of leg ``leg`` (0..5) in the platform frame."""
        return leg * math.pi / 3.0

    def leg_offset(self, leg: int, yaw: float) -> np.ndarray:
        a = yaw + self.leg_angle(leg)
        return self.leg_radius * np.array([math.cos(a), math.sin(a)])


@dataclass(frozen=True)
class SafetyConfig:
    link_clearance: float = 0.035
    column_clearance: float = 0.070
    speed_override: float = 0.6


@dataclass
class CellGeometry:
    arms: Dict[str, ArmGeometry]
    platform: PlatformGeometry = field(default_factory=PlatformGeometry)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    plate_radius: float = 0.40
    column_half_diag: float = math.hypot(0.05, 0.04)

    @property
    def arm_names(self) -> List[str]:
        return list(self.arms.keys())


def _tcp_z0(arm_cfg: dict, cell_cfg: dict) -> float:
    """TCP height at q3 = 0, derived exactly like the xacro model does."""
    z_plate = cell_cfg["base_plate_thickness"]
    j1_z = z_plate + arm_cfg["column_height"] + arm_cfg["hub_height"]
    link2_z = j1_z + arm_cfg["link_thickness"] / 2 - arm_cfg["link2_drop"]
    flange_z = link2_z - arm_cfg["link_thickness"] / 2 - arm_cfg["flange_offset"]
    finger_tip = flange_z - arm_cfg["gripper_body_height"] - arm_cfg["finger_length"]
    return finger_tip + arm_cfg["tcp_above_finger_tip"]


def build_cell(cfg: dict) -> CellGeometry:
    cell_cfg, arm_cfg, lim = cfg["cell"], cfg["arm"], cfg["limits"]
    r = cell_cfg["base_circle_radius"]
    arms: Dict[str, ArmGeometry] = {}
    for name, ang_deg in zip(cfg["arms"]["names"], cfg["arms"]["base_angles_deg"]):
        a = math.radians(ang_deg)
        arms[name] = ArmGeometry(
            name=name,
            base_xy=np.array([r * math.cos(a), r * math.sin(a)]),
            base_yaw=_wrap(a + math.pi),
            l1=arm_cfg["link1_length"],
            l2=arm_cfg["link2_length"],
            j1=JointLimit(**lim["j1"]),
            j2=JointLimit(**lim["j2"]),
            j3=JointLimit(**lim["j3"]),
            j4=JointLimit(**lim["j4"]),
            tcp_z0=_tcp_z0(arm_cfg, cell_cfg),
        )
    p = cfg["platform"]
    platform = PlatformGeometry(
        leg_radius=p["leg_radius"],
        plate_radius=p["plate_radius"],
        initial_yaw=math.radians(p["initial_yaw_deg"]),
        grip_height=p["grip_height"],
        lift_height=p["lift_height"],
    )
    s = cfg.get("safety", {})
    return CellGeometry(
        arms=arms,
        platform=platform,
        safety=SafetyConfig(**s) if s else SafetyConfig(),
        plate_radius=cell_cfg["base_plate_radius"],
        column_half_diag=math.hypot(arm_cfg["column_width"], arm_cfg["column_depth"]) / 2,
    )


def default_config_path() -> Optional[Path]:
    """Locate cell.yaml in an installed ROS workspace or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("robocraft_description")) / "config" / "cell.yaml"
        if p.exists():
            return p
    except Exception:  # noqa: BLE001 - ament not available / package not built
        pass
    here = Path(__file__).resolve()
    for parent in here.parents:
        p = parent / "robocraft_description" / "config" / "cell.yaml"
        if p.exists():
            return p
    return None


def load_cell(path: Optional[str] = None) -> CellGeometry:
    import yaml

    p = Path(path) if path else default_config_path()
    if p is None:
        raise FileNotFoundError("cell.yaml not found; pass the path explicitly")
    with open(p, "r", encoding="utf-8") as f:
        return build_cell(yaml.safe_load(f))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


wrap_angle = _wrap
