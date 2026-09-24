"""Parallel (closed-chain) kinematics: 2 or 3 arms carrying the hex platform.

With every gripper clamped on a platform leg and the leg bearing unlocked the
cell is a planar n-RRR parallel manipulator (n = 2 or 3).  Mobility is 3
(x, y, yaw) while 2n motors are available, i.e. the system is *redundantly
actuated* — exactly the thesis configuration (6 motors, 3 DOF).

Inverse kinematics is decoupled: every leg position follows from the platform
pose, then each arm solves its own serial IK.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence

import numpy as np

from .geometry import CellGeometry, wrap_angle
from .scara import IKError, TcpPose, fk, ik


@dataclass
class PlatformPose:
    x: float
    y: float
    yaw: float
    z: float  # gripping height of the TCPs (leg grip point)

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.yaw, self.z])

    @staticmethod
    def from_array(a: Sequence[float]) -> "PlatformPose":
        return PlatformPose(float(a[0]), float(a[1]), float(a[2]), float(a[3]))


def leg_world_xy(cell: CellGeometry, pose: PlatformPose, leg: int) -> np.ndarray:
    return np.array([pose.x, pose.y]) + cell.platform.leg_offset(leg, pose.yaw)


def nearest_leg(cell: CellGeometry, pose: PlatformPose, arm: str, exclude: Sequence[int] = ()) -> int:
    """Leg whose position is closest to the arm column (the natural grip)."""
    base = cell.arms[arm].base_xy
    legs = [k for k in range(6) if k not in exclude]
    return min(legs, key=lambda k: float(np.linalg.norm(leg_world_xy(cell, pose, k) - base)))


def platform_ik(cell: CellGeometry, pose: PlatformPose, grips: Mapping[str, int],
                q_seed: Optional[Mapping[str, Sequence[float]]] = None,
                elbows: Optional[Mapping[str, int]] = None,
                wrist_yaw: Optional[Mapping[str, float]] = None) -> Dict[str, np.ndarray]:
    """Joint vectors for every gripping arm.

    ``wrist_yaw``: gripper yaw per arm in the world frame. Since the leg bearing
    is passive, the wrist can hold any yaw; by default it keeps the seed value.
    """
    out: Dict[str, np.ndarray] = {}
    for arm_name, leg in grips.items():
        arm = cell.arms[arm_name]
        xy = leg_world_xy(cell, pose, leg)
        seed = None if q_seed is None else q_seed.get(arm_name)
        if wrist_yaw is not None and arm_name in wrist_yaw:
            yaw = wrist_yaw[arm_name]
        elif seed is not None:
            yaw = fk(arm, seed).yaw
        else:
            yaw = arm.base_yaw
        elbow = None if elbows is None else elbows.get(arm_name)
        try:
            out[arm_name] = ik(arm, TcpPose(xy[0], xy[1], pose.z, yaw), seed, elbow)
        except IKError as e:
            raise IKError(f"platform pose ({pose.x:.3f},{pose.y:.3f},{math.degrees(pose.yaw):.1f}deg): {e}") from e
    return out


def platform_fk(cell: CellGeometry, joints: Mapping[str, Sequence[float]],
                grips: Mapping[str, int]) -> PlatformPose:
    """Platform pose from >= 2 gripping arms (least-squares rigid fit)."""
    if len(grips) < 2:
        raise ValueError("platform FK needs at least two gripping arms")
    world, local = [], []
    z = []
    for arm_name, leg in grips.items():
        p = fk(cell.arms[arm_name], joints[arm_name])
        world.append([p.x, p.y])
        z.append(p.z)
        a = cell.platform.leg_angle(leg)
        local.append([cell.platform.leg_radius * math.cos(a), cell.platform.leg_radius * math.sin(a)])
    W, L = np.array(world), np.array(local)
    wc, lc = W.mean(axis=0), L.mean(axis=0)
    H = (L - lc).T @ (W - wc)
    yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = wc - R @ lc
    return PlatformPose(float(t[0]), float(t[1]), wrap_angle(yaw), float(np.mean(z)))


def closure_error(cell: CellGeometry, joints: Mapping[str, Sequence[float]],
                  grips: Mapping[str, int]) -> float:
    """Max distance between a TCP and the leg it should hold [m].

    Non-zero values mean the redundant actuators fight each other (internal
    forces) — used as a runtime safety monitor.
    """
    pose = platform_fk(cell, joints, grips)
    err = 0.0
    for arm_name, leg in grips.items():
        p = fk(cell.arms[arm_name], joints[arm_name])
        err = max(err, float(np.linalg.norm(np.array([p.x, p.y]) - leg_world_xy(cell, pose, leg))))
    return err
