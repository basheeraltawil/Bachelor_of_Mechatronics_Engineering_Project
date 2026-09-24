"""Trajectory generation.

* :func:`cubic_via_coefficients` — the thesis method (Section 5.1): two cubic
  segments through a via point, zero velocity at both ends, continuous
  velocity *and* acceleration at the via point (8x8 linear system).  The
  appendix code had a typo in the velocity-continuity row
  (``2*tf+3*tf*tf`` in one column); this is the corrected system.
* :func:`quintic` / :func:`ptp_duration` — synchronised PTP motion as used by
  industrial controllers (all axes start and stop together).
* :class:`TrapezoidProfile` — path-parameter profile for LIN (straight-line)
  Cartesian motion, e.g. the laser-cutting square.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Sequence

import numpy as np


# --------------------------------------------------------------------------- thesis cubic
def cubic_via_coefficients(q0: float, qv: float, qf: float, t1: float, t2: float) -> np.ndarray:
    """Return [a0..a3, b0..b3] for s1(t)=sum a_i t^i (0<=t<=t1), s2(tau)=sum b_i tau^i (0<=tau<=t2)."""
    A = np.array([
        [1, 0, 0, 0, 0, 0, 0, 0],                      # s1(0)    = q0
        [1, t1, t1 ** 2, t1 ** 3, 0, 0, 0, 0],         # s1(t1)   = qv
        [0, 0, 0, 0, 1, 0, 0, 0],                      # s2(0)    = qv
        [0, 0, 0, 0, 1, t2, t2 ** 2, t2 ** 3],         # s2(t2)   = qf
        [0, 1, 0, 0, 0, 0, 0, 0],                      # s1'(0)   = 0
        [0, 0, 0, 0, 0, 1, 2 * t2, 3 * t2 ** 2],       # s2'(t2)  = 0
        [0, 1, 2 * t1, 3 * t1 ** 2, 0, -1, 0, 0],      # s1'(t1)  = s2'(0)
        [0, 0, 2, 6 * t1, 0, 0, -2, 0],                # s1''(t1) = s2''(0)
    ], dtype=float)
    B = np.array([q0, qv, qv, qf, 0, 0, 0, 0], dtype=float)
    return np.linalg.solve(A, B)


def eval_cubic_via(c: np.ndarray, t1: float, t2: float, t: float) -> tuple:
    """(position, velocity, acceleration) of the two-segment cubic at time t."""
    if t <= t1:
        a, tau = c[:4], t
    else:
        a, tau = c[4:], min(t - t1, t2)
    p = a[0] + a[1] * tau + a[2] * tau ** 2 + a[3] * tau ** 3
    v = a[1] + 2 * a[2] * tau + 3 * a[3] * tau ** 2
    acc = 2 * a[2] + 6 * a[3] * tau
    return p, v, acc


# --------------------------------------------------------------------------- PTP
def quintic(q0: np.ndarray, qf: np.ndarray, T: float, t: float):
    """Rest-to-rest quintic: returns (q, qd, qdd)."""
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)
    if T <= 0:
        return qf.copy(), np.zeros_like(qf), np.zeros_like(qf)
    s = min(max(t / T, 0.0), 1.0)
    d = qf - q0
    pos = q0 + d * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    vel = d * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
    acc = d * (60 * s - 180 * s ** 2 + 120 * s ** 3) / T ** 2
    return pos, vel, acc


def ptp_duration(q0: Sequence[float], qf: Sequence[float], vmax: Sequence[float],
                 amax: Sequence[float], override: float = 1.0, t_min: float = 0.3) -> float:
    """Synchronised PTP time: the slowest axis defines the motion time."""
    override = min(max(override, 0.05), 1.0)
    T = t_min
    for a, b, v, acc in zip(q0, qf, vmax, amax):
        d = abs(b - a)
        if d < 1e-9:
            continue
        T = max(T, 1.875 * d / (v * override), math.sqrt(5.774 * d / (acc * override)))
    return T


# --------------------------------------------------------------------------- LIN
@dataclass
class TrapezoidProfile:
    """Trapezoidal (or triangular) velocity profile over a path of length L."""

    length: float
    vmax: float
    amax: float
    t_acc: float = field(init=False)
    t_flat: float = field(init=False)
    v_peak: float = field(init=False)

    def __post_init__(self) -> None:
        L, v, a = max(self.length, 0.0), self.vmax, self.amax
        if L < 1e-12:
            self.t_acc = self.t_flat = self.v_peak = 0.0
            return
        if v * v / a > L:  # triangular
            self.v_peak = math.sqrt(L * a)
            self.t_acc = self.v_peak / a
            self.t_flat = 0.0
        else:
            self.v_peak = v
            self.t_acc = v / a
            self.t_flat = (L - v * v / a) / v

    @property
    def duration(self) -> float:
        return 2 * self.t_acc + self.t_flat

    def s(self, t: float) -> float:
        """Arc length travelled at time t."""
        if self.length < 1e-12:
            return 0.0
        ta, tf = self.t_acc, self.t_flat
        a = self.v_peak / ta if ta > 0 else 0.0
        t = min(max(t, 0.0), self.duration)
        if t < ta:
            return 0.5 * a * t * t
        if t < ta + tf:
            return 0.5 * a * ta * ta + self.v_peak * (t - ta)
        td = t - ta - tf
        return 0.5 * a * ta * ta + self.v_peak * tf + self.v_peak * td - 0.5 * a * td * td


@dataclass
class JointTrajectory:
    """Time-stamped joint samples (seconds, joint vectors)."""

    joint_names: List[str]
    times: List[float] = field(default_factory=list)
    positions: List[np.ndarray] = field(default_factory=list)
    velocities: List[np.ndarray] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.times[-1] if self.times else 0.0

    def append(self, t: float, q: np.ndarray, qd: np.ndarray | None = None) -> None:
        self.times.append(float(t))
        self.positions.append(np.asarray(q, float))
        self.velocities.append(np.zeros_like(q) if qd is None else np.asarray(qd, float))

    def fill_velocities(self) -> None:
        """Central finite differences, zero at both ends (rest-to-rest)."""
        n = len(self.times)
        for i in range(n):
            if i == 0 or i == n - 1:
                self.velocities[i] = np.zeros_like(self.positions[i])
            else:
                dt = self.times[i + 1] - self.times[i - 1]
                self.velocities[i] = (self.positions[i + 1] - self.positions[i - 1]) / dt

    def max_velocity_ratio(self, vmax: Sequence[float]) -> float:
        if len(self.times) < 2:
            return 0.0
        ratio = 0.0
        for i in range(1, len(self.times)):
            dt = self.times[i] - self.times[i - 1]
            if dt <= 0:
                continue
            v = np.abs(self.positions[i] - self.positions[i - 1]) / dt
            ratio = max(ratio, float(np.max(v / np.asarray(vmax))))
        return ratio

    def scaled(self, factor: float) -> "JointTrajectory":
        out = JointTrajectory(list(self.joint_names))
        for t, q in zip(self.times, self.positions):
            out.append(t * factor, q)
        out.fill_velocities()
        return out


def sample_ptp(names: List[str], q0, qf, T: float, dt: float = 0.05) -> JointTrajectory:
    traj = JointTrajectory(names)
    n = max(2, int(math.ceil(T / dt)) + 1)
    for i in range(n):
        t = T * i / (n - 1)
        q, qd, _ = quintic(q0, qf, T, t)
        traj.append(t, q, qd)
    return traj


def sample_profile(profile: TrapezoidProfile, fn: Callable[[float], np.ndarray],
                   dt: float = 0.05) -> List[tuple]:
    """Evaluate ``fn(s)`` (s = arc length) along a profile. Returns (t, value)."""
    T = profile.duration
    n = max(2, int(math.ceil(T / dt)) + 1)
    return [(T * i / (n - 1), fn(profile.s(T * i / (n - 1)))) for i in range(n)]
