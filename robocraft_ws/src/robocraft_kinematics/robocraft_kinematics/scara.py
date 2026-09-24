"""Serial kinematics of one RoboCraft SCARA arm.

The planar part (J1, J2) is exactly the thesis model (Section 2.2)::

    x = x_i + a*cos(th1) + b*cos(th1 + th2)
    y = y_i + a*sin(th1) + b*sin(th1 + th2)

with ``th1`` measured in the world frame.  Here ``th1 = base_yaw + q1`` so
that every column can be mounted facing the cell centre.  J3 (quill) and J4
(wrist) are the industrial extension: ``z = tcp_z0 - q3`` and
``tool_yaw = base_yaw + q1 + q2 + q4``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from .geometry import ArmGeometry, wrap_angle


class IKError(ValueError):
    """Target is unreachable or violates joint limits."""


@dataclass
class TcpPose:
    x: float
    y: float
    z: float
    yaw: float

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.yaw])


def fk_planar(arm: ArmGeometry, q1: float, q2: float) -> np.ndarray:
    th1 = arm.base_yaw + q1
    return arm.base_xy + np.array([
        arm.l1 * math.cos(th1) + arm.l2 * math.cos(th1 + q2),
        arm.l1 * math.sin(th1) + arm.l2 * math.sin(th1 + q2),
    ])


def elbow_xy(arm: ArmGeometry, q1: float) -> np.ndarray:
    th1 = arm.base_yaw + q1
    return arm.base_xy + arm.l1 * np.array([math.cos(th1), math.sin(th1)])


def fk(arm: ArmGeometry, q: Sequence[float]) -> TcpPose:
    q1, q2, q3, q4 = q
    xy = fk_planar(arm, q1, q2)
    return TcpPose(xy[0], xy[1], arm.tcp_z0 - q3, wrap_angle(arm.base_yaw + q1 + q2 + q4))


def jacobian(arm: ArmGeometry, q1: float, q2: float) -> np.ndarray:
    """Planar 2x2 Jacobian  [xdot, ydot]^T = J [q1dot, q2dot]^T  (thesis 2.2.3)."""
    th1 = arm.base_yaw + q1
    s1, c1 = math.sin(th1), math.cos(th1)
    s12, c12 = math.sin(th1 + q2), math.cos(th1 + q2)
    return np.array([
        [-arm.l1 * s1 - arm.l2 * s12, -arm.l2 * s12],
        [arm.l1 * c1 + arm.l2 * c12, arm.l2 * c12],
    ])


def manipulability(arm: ArmGeometry, q2: float) -> float:
    """|det J| = a*b*|sin(q2)|  -> 0 at the stretched/folded singularity."""
    return abs(arm.l1 * arm.l2 * math.sin(q2))


def ik_planar_solutions(arm: ArmGeometry, x: float, y: float) -> List[np.ndarray]:
    """Both closed-form solutions (elbow +/-) of the planar 2R chain.

    Uses the thesis derivation  f^2 + l^2 = a^2 + b^2 + 2ab*cos(th2).
    Solutions are returned as (q1, q2) *joint* values (base yaw removed) and
    are *not* filtered by joint limits.
    """
    dx, dy = x - arm.base_xy[0], y - arm.base_xy[1]
    r2 = dx * dx + dy * dy
    c2 = (r2 - arm.l1 ** 2 - arm.l2 ** 2) / (2 * arm.l1 * arm.l2)
    if c2 > 1.0 + 1e-9 or c2 < -1.0 - 1e-9:
        return []
    c2 = max(-1.0, min(1.0, c2))
    sols = []
    for sign in (+1.0, -1.0):
        q2 = sign * math.acos(c2)
        th1 = math.atan2(dy, dx) - math.atan2(arm.l2 * math.sin(q2), arm.l1 + arm.l2 * math.cos(q2))
        q1 = wrap_angle(th1 - arm.base_yaw)
        sols.append(np.array([q1, q2]))
    return sols


def ik(arm: ArmGeometry, pose: TcpPose, q_seed: Optional[Sequence[float]] = None,
       elbow: Optional[int] = None) -> np.ndarray:
    """Full 4-axis IK with limit checking.

    ``elbow``: +1 / -1 to force a branch, ``None`` to pick the valid solution
    closest to ``q_seed`` (industrial controllers keep the configuration
    flag constant during LIN motion — pass ``elbow`` for that).
    """
    sols = ik_planar_solutions(arm, pose.x, pose.y)
    if not sols:
        raise IKError(f"{arm.name}: ({pose.x:.3f}, {pose.y:.3f}) outside reach {arm.reach:.3f} m")
    q3 = arm.tcp_z0 - pose.z
    if not arm.j3.contains(q3):
        raise IKError(f"{arm.name}: z={pose.z:.3f} needs q3={q3:.3f} outside quill travel")
    candidates = []
    for s in sols:
        if elbow is not None and np.sign(s[1]) not in (elbow, 0.0):
            continue
        if not (arm.j1.contains(s[0]) and arm.j2.contains(s[1])):
            continue
        q4 = pose.yaw - arm.base_yaw - s[0] - s[1]
        q4 = _nearest_equivalent(q4, q_seed[3] if q_seed is not None else 0.0, arm.j4.lower, arm.j4.upper)
        if q4 is None:
            continue
        candidates.append(np.array([s[0], s[1], q3, q4]))
    if not candidates:
        raise IKError(f"{arm.name}: ({pose.x:.3f}, {pose.y:.3f}) violates joint limits (elbow={elbow})")
    if q_seed is None:
        return candidates[0]
    seed = np.asarray(q_seed, dtype=float)
    w = np.array([1.0, 1.0, 0.1, 0.2])
    return min(candidates, key=lambda c: float(np.sum(w * np.abs(c - seed))))


def elbow_sign(q2: float) -> int:
    return 1 if q2 >= 0.0 else -1


def within_limits(arm: ArmGeometry, q: Sequence[float], tol: float = 1e-6) -> bool:
    return all(lim.contains(v, tol) for lim, v in zip(arm.limits, q))


def _nearest_equivalent(angle: float, seed: float, lo: float, hi: float) -> Optional[float]:
    """Pick angle + 2*pi*k inside [lo, hi] closest to ``seed``."""
    best = None
    for k in range(-3, 4):
        a = angle + 2 * math.pi * k
        if lo - 1e-9 <= a <= hi + 1e-9 and (best is None or abs(a - seed) < abs(best - seed)):
            best = a
    return best
