"""High-level, backend-agnostic motion commands for the RoboCraft cell.

Serial mode   – every arm is an independent 4-axis SCARA (PTP / LIN / pick / place).
Parallel mode – 2 or 3 arms clamp the hex platform legs and move it as a
                redundantly actuated n-RRR parallel manipulator.

Every plan is checked for joint limits, IK feasibility, joint speed limits
(with the industrial speed override) and inter-arm interference *before* it is
sent to the robot.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from robocraft_kinematics.geometry import CellGeometry, wrap_angle
from robocraft_kinematics.interference import check_configuration
from robocraft_kinematics.parallel import PlatformPose, leg_world_xy, nearest_leg
from robocraft_kinematics.scara import IKError, TcpPose, elbow_sign, fk, ik
from robocraft_kinematics.trajectory import (JointTrajectory, TrapezoidProfile, ptp_duration,
                                             sample_ptp)

from .backend import Backend, ExecutionError

HOME = np.array([-1.6, 2.5, 0.0, 0.0])
FINGER_OPEN = 0.022
FINGER_LEG = 0.0        # 12 mm leg: fingers touch the rubber sleeve
FINGER_PART = 0.014     # 40 mm cube
SAFE_Z = 0.170          # TCP travel height (clears legs, parts, obstacles)


class PlanningError(RuntimeError):
    pass


class InterferenceError(PlanningError):
    pass


class CellCommander:
    def __init__(self, cell: CellGeometry, backend: Backend, override: Optional[float] = None,
                 check_interference: bool = True, dt: float = 0.04):
        self.cell = cell
        self.backend = backend
        self.override = cell.safety.speed_override if override is None else override
        self.check_interference = check_interference
        self.dt = dt
        p = cell.platform
        self.platform = PlatformPose(0.0, 0.0, p.initial_yaw, p.grip_height)
        gt = backend.platform_ground_truth()
        if gt is not None:
            self.platform = replace(gt, z=p.grip_height)
        self.grips: Dict[str, int] = {}
        self.locked: set = set()
        self.last_margin = math.inf
        # interlock for concurrently moving arms (serial mode, one thread per arm)
        self._zone_lock = threading.Lock()
        self._active: Dict[str, Tuple[float, JointTrajectory]] = {}
        self.conflict_timeout = 60.0
        a = cell.arms[cell.arm_names[0]]
        self.vmax = np.array([a.j1.velocity, a.j2.velocity, a.j3.velocity, a.j4.velocity])
        self.amax = np.array([6.0, 6.0, 0.8, 15.0])

    # ================================================================ state
    @property
    def arms(self) -> List[str]:
        return self.cell.arm_names

    def q(self, arm: str) -> np.ndarray:
        return self.backend.get_q(arm)

    def tcp(self, arm: str) -> TcpPose:
        return fk(self.cell.arms[arm], self.q(arm))

    @property
    def mode(self) -> str:
        return "parallel" if len(self.grips) >= 2 else ("serial" if self.grips else "serial")

    # ================================================================ planning core
    def _interp(self, traj: JointTrajectory, t: float) -> np.ndarray:
        times = traj.times
        if t <= times[0]:
            return traj.positions[0]
        if t >= times[-1]:
            return traj.positions[-1]
        i = int(np.searchsorted(times, t))
        t0, t1 = times[i - 1], times[i]
        w = (t - t0) / (t1 - t0)
        return (1 - w) * traj.positions[i - 1] + w * traj.positions[i]

    def check(self, plans: Mapping[str, JointTrajectory],
              active: Optional[Mapping[str, Tuple[float, JointTrajectory]]] = None) -> float:
        """Interference check of simultaneous plans.

        Arms not in ``plans`` either follow their active trajectory (started at
        wall time t0) or keep their current pose.
        """
        active = {a: v for a, v in (active or {}).items() if a not in plans}
        static = {a: self.q(a) for a in self.arms if a not in plans and a not in active}
        now = time.monotonic()
        T = max((p.duration for p in plans.values()), default=0.0)
        worst = math.inf
        for t in np.arange(0.0, T + self.dt, self.dt):
            joints = dict(static)
            joints.update({a: self._interp(tr, now - t0 + t) for a, (t0, tr) in active.items()})
            joints.update({a: self._interp(p, t) for a, p in plans.items()})
            w = check_configuration(self.cell, joints)
            if w is None:
                continue
            worst = min(worst, w.margin)
            if self.check_interference and w.margin < 0.0:
                raise InterferenceError(f"interference at t={t:.2f}s: {w}")
        self.last_margin = worst
        return worst

    def execute(self, plans: Mapping[str, JointTrajectory],
                platform_path: Optional[List[Tuple[float, PlatformPose]]] = None) -> None:
        if not plans:
            return
        deadline = time.monotonic() + self.conflict_timeout
        while True:
            with self._zone_lock:
                try:
                    self.check(plans, self._active)
                    t0 = time.monotonic()
                    for a, p in plans.items():
                        self._active[a] = (t0, p)
                    break
                except InterferenceError:
                    # another arm is moving through the zone: wait like an interlock
                    if not self._active or time.monotonic() > deadline:
                        raise
            self.backend.status("waiting for interference zone to clear")
            time.sleep(0.2)
        try:
            if platform_path is not None and hasattr(self.backend, "execute_with_platform"):
                self.backend.execute_with_platform(plans, platform_path)
            else:
                self.backend.execute(plans)
        finally:
            with self._zone_lock:
                for a in plans:
                    self._active.pop(a, None)

    def plan_ptp(self, goals: Mapping[str, Sequence[float]], sync: bool = True) -> Dict[str, JointTrajectory]:
        """Synchronised PTP for one or more arms (all finish together)."""
        starts = {a: self.q(a) for a in goals}
        for a, g in goals.items():
            arm = self.cell.arms[a]
            if not all(l.contains(v, 1e-6) for l, v in zip(arm.limits, g)):
                raise PlanningError(f"{a}: PTP goal {np.round(g, 3)} outside joint limits")
        durations = {a: ptp_duration(starts[a], goals[a], self.vmax, self.amax, self.override) for a in goals}
        T = max(durations.values()) if goals else 0.0
        return {a: sample_ptp(self.backend.joint_names(a), starts[a], np.asarray(goals[a], float),
                              T if sync else durations[a], self.dt)
                for a in goals}

    def plan_cartesian(self, pose_fns: Mapping[str, Callable[[float], TcpPose]], length: float,
                       speed: float, accel: float = 0.15,
                       elbows: Optional[Mapping[str, int]] = None) -> Tuple[Dict[str, JointTrajectory], TrapezoidProfile]:
        """Sample Cartesian paths (s = arc length) for several arms on one time base."""
        speed = max(1e-4, speed * self.override)
        prof = TrapezoidProfile(length, speed, accel)
        T = prof.duration
        n = max(2, int(math.ceil(T / self.dt)) + 1)
        elbows = dict(elbows or {a: elbow_sign(self.q(a)[1]) for a in pose_fns})
        plans = {a: JointTrajectory(self.backend.joint_names(a)) for a in pose_fns}
        seeds = {a: self.q(a) for a in pose_fns}
        for i in range(n):
            t = T * i / (n - 1)
            s = prof.s(t)
            for a, fn in pose_fns.items():
                try:
                    q = ik(self.cell.arms[a], fn(s), seeds[a], elbows.get(a))
                except IKError as e:
                    raise PlanningError(f"LIN path not feasible at s={s:.3f} m: {e}") from e
                plans[a].append(t, q)
                seeds[a] = q
        # respect joint speed limits: stretch time if necessary (same factor for all arms)
        ratio = max(p.max_velocity_ratio(self.vmax * self.override) for p in plans.values())
        if ratio > 1.0:
            plans = {a: p.scaled(ratio * 1.05) for a, p in plans.items()}
        else:
            for p in plans.values():
                p.fill_velocities()
        return plans, prof

    # ================================================================ serial primitives
    def move_joints(self, goals: Mapping[str, Sequence[float]]) -> None:
        self.execute(self.plan_ptp(goals))

    def ptp(self, arm: str, pose: TcpPose, elbow: Optional[int] = None) -> None:
        q = ik(self.cell.arms[arm], pose, self.q(arm), elbow)
        self.move_joints({arm: q})

    def lin(self, targets: Mapping[str, TcpPose], speed: float = 0.08) -> None:
        """Straight-line TCP motion of one or more arms (common time base)."""
        starts = {a: self.tcp(a) for a in targets}
        lengths = {a: float(np.linalg.norm(targets[a].as_array()[:3] - starts[a].as_array()[:3])) for a in targets}
        L = max(max(lengths.values(), default=0.0), 1e-4)

        def make(a):
            p0, p1 = starts[a].as_array(), targets[a].as_array()
            dyaw = wrap_angle(p1[3] - p0[3])
            return lambda s: TcpPose(*(p0[:3] + (p1[:3] - p0[:3]) * min(s / L, 1.0)), p0[3] + dyaw * min(s / L, 1.0))

        plans, _ = self.plan_cartesian({a: make(a) for a in targets}, L, speed)
        self.execute(plans)

    def z_move(self, arms: Iterable[str], z: float, speed: float = 0.1) -> None:
        self.lin({a: replace(self.tcp(a), z=z) for a in arms}, speed)

    def gripper(self, arms: Iterable[str], position: float) -> None:
        for a in arms:
            self.backend.gripper(a, position)

    def home(self, arms: Optional[Iterable[str]] = None) -> None:
        arms = list(arms or self.arms)
        up = {a: self.q(a) for a in arms}
        for a in arms:
            up[a][2] = 0.0
        self.move_joints(up)
        self.move_joints({a: HOME.copy() for a in arms})

    def pick(self, arm: str, x: float, y: float, z: float, yaw: Optional[float] = None,
             width: float = FINGER_PART) -> None:
        yaw = self.cell.arms[arm].base_yaw if yaw is None else yaw
        self.backend.status(f"{arm}: pick at ({x:.3f}, {y:.3f})")
        self.gripper([arm], FINGER_OPEN)
        self.ptp(arm, TcpPose(x, y, SAFE_Z, yaw))
        self.lin({arm: TcpPose(x, y, z, yaw)})
        self.gripper([arm], width)
        if not self.backend.grasp(arm):
            raise ExecutionError(f"{arm}: grasp failed at ({x:.3f}, {y:.3f})")
        self.z_move([arm], SAFE_Z)

    def place(self, arm: str, x: float, y: float, z: float, yaw: Optional[float] = None) -> None:
        yaw = self.cell.arms[arm].base_yaw if yaw is None else yaw
        self.backend.status(f"{arm}: place at ({x:.3f}, {y:.3f})")
        self.ptp(arm, TcpPose(x, y, SAFE_Z, yaw))
        self.lin({arm: TcpPose(x, y, z + 0.002, yaw)})
        self.backend.release(arm)
        self.gripper([arm], FINGER_OPEN)
        self.z_move([arm], SAFE_Z)

    # ================================================================ parallel primitives
    def facing_legs(self, arms: Iterable[str], pose: Optional[PlatformPose] = None,
                    exclude: Sequence[int] = ()) -> Dict[str, int]:
        pose = pose or self.platform
        used = list(exclude) + list(self.grips.values())
        out = {}
        for a in arms:
            k = nearest_leg(self.cell, pose, a, exclude=used)
            out[a] = k
            used.append(k)
        return out

    def _grip_yaw(self, arm: str, xy: np.ndarray) -> float:
        """Finger axis radial to the platform centre, so grippers on adjacent
        legs (60 mm apart) do not clash; of the two options take the one
        closest to the current tool yaw (less J4 motion)."""
        radial = math.atan2(xy[1] - self.platform.y, xy[0] - self.platform.x)
        cur = self.tcp(arm).yaw
        return min((radial - math.pi / 2, radial + math.pi / 2), key=lambda y: abs(wrap_angle(y - cur)))

    def couple(self, arm_legs: Mapping[str, int], locked: bool = False) -> None:
        """Approach and clamp platform legs (attach = serial -> parallel)."""
        pose = self.platform
        z_grip = self.platform.z
        self.backend.status("couple " + ", ".join(f"{a}->leg{k}" for a, k in arm_legs.items()))
        self.gripper(arm_legs, FINGER_OPEN)
        above, down = {}, {}
        for a, k in arm_legs.items():
            xy = leg_world_xy(self.cell, pose, k)
            yaw = self._grip_yaw(a, xy)
            above[a] = ik(self.cell.arms[a], TcpPose(xy[0], xy[1], SAFE_Z, yaw), self.q(a))
            down[a] = TcpPose(xy[0], xy[1], z_grip, yaw)
        self.move_joints(above)
        self.lin(down, speed=0.08)
        self.gripper(arm_legs, FINGER_LEG)
        for a, k in arm_legs.items():
            if not self.backend.grasp(a, locked=locked):
                raise ExecutionError(f"{a}: could not clamp platform leg {k}")
            self.grips[a] = k
            if locked:
                self.locked.add(a)

    def decouple(self, arms: Optional[Iterable[str]] = None, retract: bool = True) -> None:
        arms = list(arms if arms is not None else self.grips)
        self.backend.status("decouple " + ", ".join(arms))
        for a in arms:
            self.backend.release(a)
            self.grips.pop(a, None)
            self.locked.discard(a)
        self.gripper(arms, FINGER_OPEN)
        if retract:
            self.z_move(arms, SAFE_Z)

    def _platform_path(self, waypoints: Sequence[PlatformPose]):
        pts = [self.platform] + list(waypoints)
        r = self.cell.platform.leg_radius
        seg = []
        for p0, p1 in zip(pts[:-1], pts[1:]):
            d = math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2
                          + (r * wrap_angle(p1.yaw - p0.yaw)) ** 2)
            seg.append(d)
        cum = np.concatenate([[0.0], np.cumsum(seg)])

        def at(s: float) -> PlatformPose:
            if cum[-1] < 1e-12:
                return pts[-1]
            s = min(max(s, 0.0), cum[-1])
            i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
            w = 0.0 if seg[i] < 1e-12 else (s - cum[i]) / seg[i]
            p0, p1 = pts[i], pts[i + 1]
            return PlatformPose(p0.x + w * (p1.x - p0.x), p0.y + w * (p1.y - p0.y),
                                p0.yaw + w * wrap_angle(p1.yaw - p0.yaw), p0.z + w * (p1.z - p0.z))

        return at, float(cum[-1])

    def move_platform(self, waypoints: Sequence[PlatformPose], speed: float = 0.03,
                      laser: bool = False) -> None:
        """Move the clamped platform through waypoints with straight segments (LIN)."""
        if not self.grips:
            raise PlanningError("no arm holds the platform")
        yaw_change = any(abs(wrap_angle(w.yaw - self.platform.yaw)) > 1e-9 for w in waypoints)
        if len(self.grips) == 1 and not self.locked and yaw_change:
            raise PlanningError("one unlocked arm cannot control the platform yaw (engage the brake)")
        at, length = self._platform_path(waypoints)
        yaw0 = self.platform.yaw
        tool0 = {a: self.tcp(a).yaw for a in self.grips}

        def make(a, leg):
            def fn(s):
                p = at(s)
                xy = leg_world_xy(self.cell, p, leg)
                # the wrist follows the platform yaw: grippers stay radial and the
                # (passive) leg bearing does not have to turn
                yaw = tool0[a] + (p.yaw - yaw0)
                return TcpPose(xy[0], xy[1], p.z, yaw)
            return fn

        plans, prof = self.plan_cartesian({a: make(a, k) for a, k in self.grips.items()}, length, speed,
                                          accel=0.08)
        T_plan = next(iter(plans.values())).duration
        scale = T_plan / prof.duration if prof.duration > 0 else 1.0
        path = [(t * scale, at(prof.s(t))) for t in np.arange(0.0, prof.duration + self.dt, self.dt)]
        if laser:
            self.backend.set_laser(True)
        try:
            self.execute(plans, platform_path=path)
        finally:
            if laser:
                self.backend.set_laser(False)
        self.platform = waypoints[-1]
        self.backend.publish_platform(self.platform)

    def lift(self, dz: float) -> None:
        self.move_platform([replace(self.platform, z=self.platform.z + dz)], speed=0.05)

    def rotate_platform(self, dyaw: float, speed: float = 0.03) -> None:
        self.move_platform([replace(self.platform, yaw=self.platform.yaw + dyaw)], speed)

    def translate_platform(self, x: float, y: float, speed: float = 0.03, laser: bool = False) -> None:
        self.move_platform([replace(self.platform, x=x, y=y)], speed, laser)
