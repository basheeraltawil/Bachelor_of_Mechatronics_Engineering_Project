"""Thesis via-point generator and obstacle-aware path selection (Sections 5.1, 5.4).

The thesis builds a via point as the intersection of two circles of radius
``L = (W/2)/cos(theta)`` centred at the start and the goal (``W`` = distance,
``theta`` = "bending angle", 30 deg by default).  The two intersections are the
two candidate detours.  With an obstacle (the green cube from the camera) the
generator keeps the candidate whose polyline is collision-free, increasing the
bending angle if neither is.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class CircleObstacle:
    x: float
    y: float
    radius: float


def via_point_candidates(p0: Sequence[float], pf: Sequence[float], bend_deg: float = 30.0) -> Tuple[np.ndarray, np.ndarray]:
    """Both circle-intersection via points (closed form of the thesis sympy solve)."""
    p0, pf = np.asarray(p0, float), np.asarray(pf, float)
    d = pf - p0
    W = float(np.linalg.norm(d))
    if W < 1e-9:
        return p0.copy(), p0.copy()
    mid = (p0 + pf) / 2
    h = (W / 2) * math.tan(math.radians(bend_deg))  # sqrt(L^2 - (W/2)^2)
    n = np.array([-d[1], d[0]]) / W
    return mid + h * n, mid - h * n


def segment_point_distance(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> float:
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / denom))
    return float(np.linalg.norm(a + t * ab - p))


def polyline_clearance(points: Sequence[np.ndarray], obstacles: Sequence[CircleObstacle]) -> float:
    """Smallest (distance - obstacle radius) of a polyline to all obstacles."""
    best = math.inf
    for a, b in zip(points[:-1], points[1:]):
        for ob in obstacles:
            best = min(best, segment_point_distance(np.asarray(a), np.asarray(b), np.array([ob.x, ob.y])) - ob.radius)
    return best


def plan_detour(p0: Sequence[float], pf: Sequence[float], obstacles: Sequence[CircleObstacle],
                clearance: float, bend_deg: float = 30.0, max_bend_deg: float = 75.0,
                step_deg: float = 5.0) -> Optional[List[np.ndarray]]:
    """Return [p0, (via), pf] keeping ``clearance`` beyond every obstacle radius, or None.

    A straight line is used when it is already clear (no via point needed).
    """
    p0, pf = np.asarray(p0, float), np.asarray(pf, float)
    if not obstacles or polyline_clearance([p0, pf], obstacles) >= clearance:
        return [p0, pf]
    bend = bend_deg
    while bend <= max_bend_deg + 1e-9:
        cands = []
        for via in via_point_candidates(p0, pf, bend):
            path = [p0, via, pf]
            cands.append((polyline_clearance(path, obstacles), path))
        cands.sort(key=lambda c: -c[0])
        if cands[0][0] >= clearance:
            return cands[0][1]
        bend += step_deg
    return None
