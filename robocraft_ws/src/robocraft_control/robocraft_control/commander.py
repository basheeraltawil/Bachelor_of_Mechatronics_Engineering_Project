"""CellCommander: the high-level "robot language" of the RoboCraft cell.

Scenarios are written with the methods of this class, like a program on an
industrial robot controller:

    ===================  ======================================================
    Serial mode          every arm is an independent 4-axis SCARA
    -------------------  ------------------------------------------------------
    move_joints(goals)   synchronised PTP in joint space (all arms finish together)
    ptp(arm, pose)       PTP to a Cartesian TCP pose
    lin(targets)         straight-line TCP motion of one or more arms
    pick / place         complete pick-and-place sequences with grasp check
    home(arms)           quill up, then park
    -------------------  ------------------------------------------------------
    Parallel mode        2-3 arms clamp the hex platform = one parallel robot
    -------------------  ------------------------------------------------------
    couple(arm_legs)     approach and clamp platform legs (serial -> parallel)
    move_platform(wps)   move the platform through waypoints (LIN)
    lift / rotate_platform / translate_platform   convenience wrappers
    decouple(arms)       release and retract (parallel -> serial)
    ===================  ======================================================

Safety: before *anything* moves, every plan is checked for inverse-kinematics
feasibility, joint limits, joint speed limits (scaled by the speed override,
like the override knob on a teach pendant) and collisions between the arms.
If two arms run in different threads, an interlock makes one wait until the
other one is out of the way.

The commander never talks to hardware directly; it uses a
:class:`~robocraft_control.backend.Backend` (dry-run, ROS/Gazebo or real).
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
from robocraft_kinematics.trajectory import JointTrajectory, TrapezoidProfile, ptp_duration, sample_ptp

from .backend import Backend, ExecutionError

# ---------------------------------------------------------------- constants
HOME = np.array([-1.6, 2.5, 0.0, 0.0])   # parked pose [J1 rad, J2 rad, J3 m, J4 rad]: elbow folded outwards
FINGER_OPEN = 0.022                      # finger opening per side [m] (fully open)
FINGER_LEG = 0.0                         # closed on a 12 mm platform leg
FINGER_PART = 0.014                      # closed on a 40 mm cube
SAFE_Z = 0.170                           # TCP travel height [m]: clears legs, parts and obstacles
JOINT_ACCEL = np.array([6.0, 6.0, 0.8, 15.0])   # max joint acceleration [rad/s^2, rad/s^2, m/s^2, rad/s^2]


class PlanningError(RuntimeError):
    """A motion cannot be planned (unreachable, outside limits, ...)."""


class InterferenceError(PlanningError):
    """A planned motion would make two arms collide."""


class CellCommander:
    """Plans, checks and executes motions of the whole cell."""

    def __init__(self, cell: CellGeometry, backend: Backend, override: Optional[float] = None,
                 check_interference: bool = True, dt: float = 0.04):
        """
        cell      geometry and limits (from cell.yaml)
        backend   dry-run, ROS (mock / Gazebo) or real hardware
        override  speed override 0..1 (default from cell.yaml)
        dt        trajectory sample time [s]
        """
        self.cell = cell
        self.backend = backend
        self.override = cell.safety.speed_override if override is None else override
        self.check_interference = check_interference
        self.dt = dt

        # where the platform is and who holds it
        p = cell.platform
        self.platform = PlatformPose(0.0, 0.0, p.initial_yaw, p.grip_height)
        ground_truth = backend.platform_ground_truth()        # Gazebo knows the real pose
        if ground_truth is not None:
            self.platform = replace(ground_truth, z=p.grip_height)
        self.grips: Dict[str, int] = {}                       # arm -> clamped leg index
        self.locked: set = set()                              # arms clamping with the brake engaged
        self.last_margin = math.inf                           # smallest arm-to-arm clearance of the last plan

        # interlock for arms moving concurrently in different threads
        self._zone_lock = threading.Lock()
        self._active: Dict[str, Tuple[float, JointTrajectory]] = {}   # arm -> (start time, trajectory)
        self.conflict_timeout = 60.0

        a = cell.arms[cell.arm_names[0]]
        self.vmax = np.array([a.j1.velocity, a.j2.velocity, a.j3.velocity, a.j4.velocity])
        self.amax = JOINT_ACCEL

    # ================================================================ state
    @property
    def arms(self) -> List[str]:
        return self.cell.arm_names

    @property
    def mode(self) -> str:
        """'parallel' when two or more arms hold the platform, otherwise 'serial'."""
        return "parallel" if len(self.grips) >= 2 else "serial"

    def q(self, arm: str) -> np.ndarray:
        """Current joint vector [J1, J2, J3, J4] of an arm."""
        return self.backend.get_q(arm)

    def tcp(self, arm: str) -> TcpPose:
        """Current tool-centre-point pose of an arm (forward kinematics)."""
        return fk(self.cell.arms[arm], self.q(arm))

    # ================================================================ checking and execution
    @staticmethod
    def _interp(traj: JointTrajectory, t: float) -> np.ndarray:
        """Joint vector of a sampled trajectory at time t (linear interpolation)."""
        times = traj.times
        if t <= times[0]:
            return traj.positions[0]
        if t >= times[-1]:
            return traj.positions[-1]
        i = int(np.searchsorted(times, t))
        w = (t - times[i - 1]) / (times[i] - times[i - 1])
        return (1 - w) * traj.positions[i - 1] + w * traj.positions[i]

    def check(self, plans: Mapping[str, JointTrajectory],
              active: Optional[Mapping[str, Tuple[float, JointTrajectory]]] = None) -> float:
        """Check simultaneous plans for collisions between arms, sample by sample.

        Arms without a plan either follow the trajectory they are executing
        right now (``active``: arm -> (start time, trajectory)) or stand still.
        Returns the smallest clearance [m]; raises InterferenceError if < 0.
        """
        active = {a: v for a, v in (active or {}).items() if a not in plans}
        standing = {a: self.q(a) for a in self.arms if a not in plans and a not in active}
        now = time.monotonic()
        T = max((p.duration for p in plans.values()), default=0.0)
        worst = math.inf
        for t in np.arange(0.0, T + self.dt, self.dt):
            joints = dict(standing)
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
        """Check and run plans for one or more arms with a common start time.

        If another arm (another thread) is in the way, wait until it has
        passed - like the interference-zone interlock of an industrial cell.
        """
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
                    if not self._active or time.monotonic() > deadline:
                        raise                      # a real collision, not just a busy zone
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

    # ================================================================ planning
    def plan_ptp(self, goals: Mapping[str, Sequence[float]], sync: bool = True) -> Dict[str, JointTrajectory]:
        """Joint-space PTP (quintic profile) for one or more arms.

        With ``sync`` all arms get the duration of the slowest one, so they
        start and stop together.
        """
        starts = {a: self.q(a) for a in goals}
        for a, g in goals.items():
            if not all(lim.contains(v, 1e-6) for lim, v in zip(self.cell.arms[a].limits, g)):
                raise PlanningError(f"{a}: PTP goal {np.round(g, 3)} outside joint limits")
        durations = {a: ptp_duration(starts[a], goals[a], self.vmax, self.amax, self.override) for a in goals}
        T = max(durations.values()) if goals else 0.0
        return {a: sample_ptp(self.backend.joint_names(a), starts[a], np.asarray(goals[a], float),
                              T if sync else durations[a], self.dt)
                for a in goals}

    def plan_cartesian(self, pose_fns: Mapping[str, Callable[[float], TcpPose]], length: float,
                       speed: float, accel: float = 0.15,
                       elbows: Optional[Mapping[str, int]] = None
                       ) -> Tuple[Dict[str, JointTrajectory], TrapezoidProfile]:
        """Cartesian (LIN) plans for several arms on one common time base.

        ``pose_fns[arm](s)`` gives the TCP pose after travelling ``s`` metres
        along the path. The path speed follows a trapezoidal profile; inverse
        kinematics is solved at every sample with the elbow configuration kept
        constant. If any joint would exceed its speed limit, time is stretched
        for *all* arms, so they stay synchronised.
        """
        speed = max(1e-4, speed * self.override)
        prof = TrapezoidProfile(length, speed, accel)
        n = max(2, int(math.ceil(prof.duration / self.dt)) + 1)
        elbows = dict(elbows or {a: elbow_sign(self.q(a)[1]) for a in pose_fns})
        plans = {a: JointTrajectory(self.backend.joint_names(a)) for a in pose_fns}
        seeds = {a: self.q(a) for a in pose_fns}
        for i in range(n):
            t = prof.duration * i / (n - 1)
            s = prof.s(t)
            for a, fn in pose_fns.items():
                try:
                    q = ik(self.cell.arms[a], fn(s), seeds[a], elbows.get(a))
                except IKError as e:
                    raise PlanningError(f"LIN path not feasible at s={s:.3f} m: {e}") from e
                plans[a].append(t, q)
                seeds[a] = q
        ratio = max(p.max_velocity_ratio(self.vmax * self.override) for p in plans.values())
        if ratio > 1.0:
            plans = {a: p.scaled(ratio * 1.05) for a, p in plans.items()}
        else:
            for p in plans.values():
                p.fill_velocities()
        return plans, prof

    # ================================================================ serial-mode commands
    def move_joints(self, goals: Mapping[str, Sequence[float]]) -> None:
        """Synchronised PTP of one or more arms to joint goals."""
        self.execute(self.plan_ptp(goals))

    def ptp(self, arm: str, pose: TcpPose, elbow: Optional[int] = None) -> None:
        """PTP of one arm to a Cartesian pose (the path in between is curved)."""
        self.move_joints({arm: ik(self.cell.arms[arm], pose, self.q(arm), elbow)})

    def lin(self, targets: Mapping[str, TcpPose], speed: float = 0.08) -> None:
        """Straight-line TCP motion of one or more arms (they arrive together)."""
        starts = {a: self.tcp(a) for a in targets}
        L = max([float(np.linalg.norm(targets[a].as_array()[:3] - starts[a].as_array()[:3])) for a in targets]
                + [1e-4])

        def line(a):
            p0, p1 = starts[a].as_array(), targets[a].as_array()
            dyaw = wrap_angle(p1[3] - p0[3])
            return lambda s: TcpPose(*(p0[:3] + (p1[:3] - p0[:3]) * min(s / L, 1.0)), p0[3] + dyaw * min(s / L, 1.0))

        plans, _ = self.plan_cartesian({a: line(a) for a in targets}, L, speed)
        self.execute(plans)

    def z_move(self, arms: Iterable[str], z: float, speed: float = 0.1) -> None:
        """Move the quill(s) straight up or down to TCP height z."""
        self.lin({a: replace(self.tcp(a), z=z) for a in arms}, speed)

    def gripper(self, arms: Iterable[str], position: float) -> None:
        """Open/close fingers (opening per finger in metres)."""
        for a in arms:
            self.backend.gripper(a, position)

    def home(self, arms: Optional[Iterable[str]] = None) -> None:
        """Retract the quill first (never sweep over parts), then park."""
        arms = list(arms or self.arms)
        up = {a: self.q(a) for a in arms}
        for a in arms:
            up[a][2] = 0.0
        self.move_joints(up)
        self.move_joints({a: HOME.copy() for a in arms})

    def pick(self, arm: str, x: float, y: float, z: float, yaw: Optional[float] = None,
             width: float = FINGER_PART) -> None:
        """Open, go above the part, descend (LIN), close, verify the grasp, lift."""
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
        """Go above the target, descend (LIN), release, open, lift."""
        yaw = self.cell.arms[arm].base_yaw if yaw is None else yaw
        self.backend.status(f"{arm}: place at ({x:.3f}, {y:.3f})")
        self.ptp(arm, TcpPose(x, y, SAFE_Z, yaw))
        self.lin({arm: TcpPose(x, y, z + 0.002, yaw)})     # 2 mm above the table: gentle drop
        self.backend.release(arm)
        self.gripper([arm], FINGER_OPEN)
        self.z_move([arm], SAFE_Z)

    # ================================================================ parallel-mode commands
    def facing_legs(self, arms: Iterable[str], pose: Optional[PlatformPose] = None,
                    exclude: Sequence[int] = ()) -> Dict[str, int]:
        """For each arm the free platform leg closest to its column."""
        pose = pose or self.platform
        used = list(exclude) + list(self.grips.values())
        out = {}
        for a in arms:
            out[a] = nearest_leg(self.cell, pose, a, exclude=used)
            used.append(out[a])
        return out

    def _grip_yaw(self, arm: str, xy: np.ndarray) -> float:
        """Gripper yaw for clamping a leg: finger axis radial to the platform,
        so grippers on neighbouring legs (60 mm apart) do not touch. Of the two
        symmetric options, the one needing less wrist (J4) motion is used."""
        radial = math.atan2(xy[1] - self.platform.y, xy[0] - self.platform.x)
        cur = self.tcp(arm).yaw
        return min((radial - math.pi / 2, radial + math.pi / 2), key=lambda y: abs(wrap_angle(y - cur)))

    def couple(self, arm_legs: Mapping[str, int], locked: bool = False) -> None:
        """Clamp platform legs: open, go above the legs, descend together, close,
        verify every grasp. ``locked=True`` also engages the leg brake."""
        self.backend.status("couple " + ", ".join(f"{a}->leg{k}" for a, k in arm_legs.items()))
        self.gripper(arm_legs, FINGER_OPEN)
        above, down = {}, {}
        for a, k in arm_legs.items():
            xy = leg_world_xy(self.cell, self.platform, k)
            yaw = self._grip_yaw(a, xy)
            above[a] = ik(self.cell.arms[a], TcpPose(xy[0], xy[1], SAFE_Z, yaw), self.q(a))
            down[a] = TcpPose(xy[0], xy[1], self.platform.z, yaw)
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
        """Release legs, open the grippers and lift the quills."""
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
        """Piecewise-linear platform path; returns (pose_at(s), total length).

        Rotation is converted to a length with the leg radius, so 'metres
        along the path' is meaningful for mixed translation + rotation.
        """
        pts = [self.platform] + list(waypoints)
        r = self.cell.platform.leg_radius
        seg = [math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2
                         + (r * wrap_angle(p1.yaw - p0.yaw)) ** 2) for p0, p1 in zip(pts[:-1], pts[1:])]
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
        """Move the clamped platform through waypoints along straight segments.

        Parallel kinematics: every leg position follows from the platform pose,
        and each clamping arm solves its own inverse kinematics to that leg.
        ``laser=True`` switches the laser on for the whole motion.
        """
        if not self.grips:
            raise PlanningError("no arm holds the platform")
        turns = any(abs(wrap_angle(w.yaw - self.platform.yaw)) > 1e-9 for w in waypoints)
        if len(self.grips) == 1 and not self.locked and turns:
            raise PlanningError("one unlocked arm cannot control the platform yaw (engage the brake)")
        at, length = self._platform_path(waypoints)
        yaw0 = self.platform.yaw
        tool0 = {a: self.tcp(a).yaw for a in self.grips}

        def leg_path(a, leg):
            def pose(s):
                p = at(s)
                xy = leg_world_xy(self.cell, p, leg)
                # the wrist turns with the platform: grippers stay radial and the
                # passive leg bearing does not need to turn
                return TcpPose(xy[0], xy[1], p.z, tool0[a] + (p.yaw - yaw0))
            return pose

        plans, prof = self.plan_cartesian({a: leg_path(a, k) for a, k in self.grips.items()}, length, speed,
                                          accel=0.08)
        # platform poses on the (possibly stretched) plan time base, for visualisation
        scale = next(iter(plans.values())).duration / prof.duration if prof.duration > 0 else 1.0
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
        """Raise (dz > 0) or lower the clamped platform."""
        self.move_platform([replace(self.platform, z=self.platform.z + dz)], speed=0.05)

    def rotate_platform(self, dyaw: float, speed: float = 0.03) -> None:
        """Turn the clamped platform about its centre by dyaw [rad]."""
        self.move_platform([replace(self.platform, yaw=self.platform.yaw + dyaw)], speed)

    def translate_platform(self, x: float, y: float, speed: float = 0.03, laser: bool = False) -> None:
        """Move the platform centre to (x, y) in a straight line, keeping its orientation."""
        self.move_platform([replace(self.platform, x=x, y=y)], speed, laser)
