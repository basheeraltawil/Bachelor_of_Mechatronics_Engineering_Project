"""Inter-arm interference checking (industrial "interference zones").

The three arms share one workspace.  Links of different arms that sit on the
same height layer can collide, and link 2 / the gripper of one arm can hit a
foreign column.  Each arm is approximated by capsules:

* layer L1 (z 0.30..0.32): link 1 from the motor-2 tail to the elbow
* layer L2 (z 0.275..0.295): link 2 from the elbow to the TCP
* column + hub of every arm up to z 0.30 (a vertical cylinder)
* quill / gripper: a vertical cylinder at the TCP

Every planned trajectory is checked sample by sample before execution.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from .geometry import ArmGeometry, CellGeometry
from .scara import elbow_xy, fk_planar

LINK1_TAIL = 0.120
LINK_HALF_WIDTH = 0.0125
QUILL_RADIUS = 0.030
# gripper body 32 x 75 mm (fingers slide along the wrist y axis): a capsule
GRIPPER_HALF_LEN = 0.0215
GRIPPER_RADIUS = 0.016
GRIPPER_CLEARANCE = 0.004


def _seg_seg_distance(p1, q1, p2, q2) -> float:
    """Minimum distance between 2-D segments p1q1 and p2q2."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e < 1e-12:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def _seg_point_distance(p, q, x) -> float:
    return _seg_seg_distance(p, q, x, x)


@dataclass
class ArmShape:
    base: np.ndarray
    tail: np.ndarray
    elbow: np.ndarray
    tcp: np.ndarray
    tool_yaw: float = 0.0

    def gripper_segment(self):
        d = GRIPPER_HALF_LEN * np.array([-math.sin(self.tool_yaw), math.cos(self.tool_yaw)])
        return self.tcp - d, self.tcp + d


def arm_shape(arm: ArmGeometry, q: Sequence[float]) -> ArmShape:
    th1 = arm.base_yaw + q[0]
    tail = arm.base_xy - LINK1_TAIL * np.array([math.cos(th1), math.sin(th1)])
    return ArmShape(arm.base_xy.copy(), tail, elbow_xy(arm, q[0]), fk_planar(arm, q[0], q[1]),
                    arm.base_yaw + q[0] + q[1] + q[3])


@dataclass
class Interference:
    arm_a: str
    arm_b: str
    what: str
    margin: float  # negative = collision

    def __str__(self) -> str:
        return f"{self.arm_a}/{self.arm_b}: {self.what} (margin {self.margin * 1000:.1f} mm)"


def pair_margin(sa: ArmShape, sb: ArmShape, link_clearance: float, column_clearance: float):
    """Smallest margin between two arms and the name of the critical pair."""
    checks = [
        ("link1-link1", _seg_seg_distance(sa.tail, sa.elbow, sb.tail, sb.elbow) - link_clearance),
        ("link2-link2", _seg_seg_distance(sa.elbow, sa.tcp, sb.elbow, sb.tcp) - link_clearance),
        ("gripper-gripper", _seg_seg_distance(*sa.gripper_segment(), *sb.gripper_segment())
         - 2 * GRIPPER_RADIUS - GRIPPER_CLEARANCE),
    ]
    for (x, y, tag) in ((sa, sb, "A"), (sb, sa, "B")):
        checks.append((f"link2({tag})-column", _seg_point_distance(x.elbow, x.tcp, y.base) - column_clearance))
        checks.append((f"link2({tag})-motor2", _seg_point_distance(x.elbow, x.tcp, y.tail) - column_clearance / 2))
        checks.append((f"quill({tag})-link1", _seg_point_distance(y.tail, y.elbow, x.tcp) - QUILL_RADIUS - LINK_HALF_WIDTH))
    return min(checks, key=lambda c: c[1])


def check_configuration(cell: CellGeometry, joints: Mapping[str, Sequence[float]]):
    """Return the worst :class:`Interference` over all arm pairs (or None if no arms)."""
    shapes = {n: arm_shape(cell.arms[n], q) for n, q in joints.items()}
    worst = None
    for a, b in itertools.combinations(sorted(shapes), 2):
        what, margin = pair_margin(shapes[a], shapes[b], cell.safety.link_clearance,
                                   cell.safety.column_clearance)
        if worst is None or margin < worst.margin:
            worst = Interference(a, b, what, margin)
    return worst


def first_collision(cell: CellGeometry, samples: Sequence[Mapping[str, Sequence[float]]]):
    """First sample index with negative margin, or None if the motion is safe."""
    for i, joints in enumerate(samples):
        w = check_configuration(cell, joints)
        if w is not None and w.margin < 0.0:
            return i, w
    return None
