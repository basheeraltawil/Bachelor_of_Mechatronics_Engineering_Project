"""RoboCraft scenarios: industrial tasks for the reconfigurable 3-arm cell.

Every scenario is a plain Python function ``run_<name>(cmd, **options)`` that
uses the high-level :class:`~robocraft_control.commander.CellCommander`
(``pick``, ``place``, ``couple``, ``move_platform``, ...). Because the
commander hides the hardware, the *same* function runs in the unit tests
(dry-run backend), in RViz (mock hardware), in Gazebo and on the real robot.

==========================  ========  ==========================================  =========
scenario                    mode      real-world problem it represents            thesis
==========================  ========  ==========================================  =========
serial_pick_place           serial    three independent machine-tending cells     1.2
vision_sorting              serial    camera quality inspection + reject sorting  5.5
parallel_square             parallel  laser engraving of a simple pattern         5.3
laser_cutting_profile       parallel  cutting a sheet part: outline + bolt hole    5.3
obstacle_square             parallel  cutting around clamps / keep-out zones       5.4, 5.5
pure_rotation               parallel  re-orienting a workpiece beyond one robot's  5.2
                                      range by regrasping (handover)
locked_transport            serial    one robot moves a fixture / pallet alone     3.2
reconfiguration_demo        both      flexible manufacturing: same robots switch   1.2
                                      between single and cooperative work
==========================  ========  ==========================================  =========

Add a scenario: write ``run_my_task(cmd, ...)`` below and register it in
``SCENARIOS`` at the end of the file; it is then available as
``ros2 run robocraft_control scenario my_task``.
"""
from __future__ import annotations

import math
import threading
from dataclasses import replace
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from robocraft_kinematics.via_point import plan_detour

from ..commander import CellCommander, PlanningError

# ---------------------------------------------------------------- cell layout
PART_GRASP_Z = 0.040            # TCP height to grip the 40 mm cubes (cube centre z = 0.030)
STATION_RADIUS = 0.18           # infeed / outfeed stations: distance from the column [m]
STATION_ANGLE = math.radians(35.0)   # ... and angle to either side of the arm's axis
REJECT_RADIUS = 0.32            # reject bin at the plate edge
REJECT_ANGLE = math.radians(-65.0)
PART_COLOURS = ("red", "blue", "yellow")


def _polar(cmd: CellCommander, arm: str, radius: float, angle: float) -> Tuple[float, float]:
    """Point at (radius, angle) in the arm's own frame (angle 0 = towards the cell centre)."""
    a = cmd.cell.arms[arm]
    t = a.base_yaw + angle
    return float(a.base_xy[0] + radius * math.cos(t)), float(a.base_xy[1] + radius * math.sin(t))


def station(cmd: CellCommander, arm: str, side: int) -> Tuple[float, float]:
    """Part station of an arm: side +1 = infeed (left), -1 = outfeed / good-parts bin (right)."""
    return _polar(cmd, arm, STATION_RADIUS, side * STATION_ANGLE)


def reject_bin(cmd: CellCommander, arm: str) -> Tuple[float, float]:
    """Reject bin of an arm, at the plate edge between two arms."""
    return _polar(cmd, arm, REJECT_RADIUS, REJECT_ANGLE)


def _run_arms_concurrently(arms: Sequence[str], job: Callable[[str], None]) -> None:
    """Run ``job(arm)`` for every arm in its own thread and re-raise the first error.

    Collisions between the threads are prevented by the commander's
    interference interlock (an arm waits while another one is in the way).
    """
    errors: List[BaseException] = []

    def wrapper(arm: str) -> None:
        try:
            job(arm)
        except BaseException as e:  # noqa: BLE001 - re-raised in the main thread
            errors.append(e)

    threads = [threading.Thread(target=wrapper, args=(a,), daemon=True) for a in arms]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if errors:
        raise errors[0]


# ============================================================ serial mode
def run_serial_pick_place(cmd: CellCommander, cycles: int = 2, concurrent: bool = True,
                          arms: Optional[Sequence[str]] = None, start_side: int = +1) -> None:
    """Machine tending: every arm moves its own part from the infeed to the
    outfeed and back (ping-pong), all arms at the same time.

    ``start_side=-1`` starts at the outfeed (used after a previous run)."""
    arms = list(arms or cmd.arms)

    def job(arm: str) -> None:
        src, dst = (+1, -1) if start_side >= 0 else (-1, +1)
        for _ in range(cycles):
            cmd.pick(arm, *station(cmd, arm, src), PART_GRASP_Z)
            cmd.place(arm, *station(cmd, arm, dst), PART_GRASP_Z)
            src, dst = dst, src
        cmd.home([arm])

    if concurrent:
        _run_arms_concurrently(arms, job)
    else:
        for a in arms:
            job(a)


def run_vision_sorting(cmd: CellCommander, reject: Sequence[str] = ("red",), concurrent: bool = True,
                       vision_timeout: float = 10.0) -> Dict[str, int]:
    """Quality inspection: the overhead camera classifies every part by colour
    and locates it (no taught positions). Parts of a *reject* colour go to the
    reject bin at the plate edge, all others to the good-parts bin.

    Each part is handled by the arm whose column is closest; the arms work in
    parallel. Returns the counts {"good": n, "rejected": m}.
    """
    reject = {reject} if isinstance(reject, str) else set(reject)
    parts = [d for d in cmd.backend.detections(timeout=vision_timeout) if d.label in PART_COLOURS]
    if not parts:
        raise PlanningError("the camera sees no parts to sort (is the vision node running?)")

    jobs: Dict[str, list] = {a: [] for a in cmd.arms}
    for d in parts:
        arm = min(cmd.arms, key=lambda a: float(np.hypot(*(cmd.cell.arms[a].base_xy - (d.x, d.y)))))
        jobs[arm].append(d)
    cmd.backend.status("inspection: " + ", ".join(f"{d.label} at ({d.x:.3f}, {d.y:.3f})" for d in parts))
    counts = {"good": 0, "rejected": 0}
    lock = threading.Lock()

    def job(arm: str) -> None:
        for d in jobs[arm]:
            bad = d.label in reject
            cmd.pick(arm, d.x, d.y, PART_GRASP_Z)
            target = reject_bin(cmd, arm) if bad else station(cmd, arm, -1)
            cmd.place(arm, *target, PART_GRASP_Z)
            with lock:
                counts["rejected" if bad else "good"] += 1
        cmd.home([arm])

    busy = [a for a in cmd.arms if jobs[a]]
    if concurrent:
        _run_arms_concurrently(busy, job)
    else:
        for a in busy:
            job(a)
    cmd.backend.status(f"sorting done: {counts['good']} good, {counts['rejected']} rejected")
    return counts


def run_locked_transport(cmd: CellCommander, arm: str = "arm1", dx: float = 0.04,
                         dyaw_deg: float = 30.0) -> None:
    """Fixture transfer by one robot (thesis 3.2 lock mechanism).

    With the leg brake engaged a single arm holds the platform rigidly, so it
    can shift *and* rotate it without help from the other arms."""
    cmd.couple(cmd.facing_legs([arm]), locked=True)
    cmd.lift(cmd.cell.platform.lift_height)
    p0 = cmd.platform
    axis = -cmd.cell.arms[arm].base_xy / np.linalg.norm(cmd.cell.arms[arm].base_xy)
    side = np.array([-axis[1], axis[0]])          # sideways from the arm's point of view
    cmd.move_platform([replace(p0, x=p0.x + dx * side[0], y=p0.y + dx * side[1])], speed=0.03)
    cmd.rotate_platform(math.radians(dyaw_deg))
    cmd.rotate_platform(-math.radians(dyaw_deg))
    cmd.move_platform([p0], speed=0.03)
    detach_platform(cmd)


# ============================================================ parallel mode helpers
def attach_platform(cmd: CellCommander, arms: Sequence[str]) -> Dict[str, int]:
    """Serial -> parallel: each arm clamps the leg facing it, then all lift together."""
    legs = cmd.facing_legs(arms)
    cmd.couple(legs)
    cmd.lift(cmd.cell.platform.lift_height)
    return legs


def detach_platform(cmd: CellCommander, home: bool = True) -> None:
    """Parallel -> serial: set the platform down, open, retract, park."""
    if cmd.platform.z > cmd.cell.platform.grip_height + 1e-6:
        cmd.move_platform([replace(cmd.platform, z=cmd.cell.platform.grip_height)], speed=0.05)
    arms = list(cmd.grips)
    cmd.decouple(arms)
    if home:
        cmd.home(arms)


def square_corners(cx: float, cy: float, side: float) -> List[Tuple[float, float]]:
    """Closed square path (5 points, first = last)."""
    h = side / 2
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h), (cx - h, cy - h)]


def cut_contour(cmd: CellCommander, points: Sequence[Tuple[float, float]], speed: float) -> None:
    """Travel to the first point with the laser off, then follow the contour with the laser on."""
    cmd.translate_platform(*points[0], speed=max(speed, 0.03))
    cmd.move_platform([replace(cmd.platform, x=x, y=y) for x, y in points[1:]], speed=speed, laser=True)


def part_profile(cx: float, cy: float, width: float, height: float, corner_radius: float,
                 hole_diameter: float, arc_step_deg: float = 10.0) -> List[List[Tuple[float, float]]]:
    """Contours of a rectangular plate with rounded corners and a centre hole.

    Returned in cutting order: inner contours first (the hole), then the
    outline - otherwise the part would fall out before its holes are cut.
    """
    hole = []
    if hole_diameter > 0:
        r = hole_diameter / 2
        n = int(360 / arc_step_deg)
        hole = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]
    rc = min(corner_radius, width / 2, height / 2)
    hx, hy = width / 2 - rc, height / 2 - rc
    outline = []
    n_arc = max(2, int(90 / arc_step_deg))
    # corners counter-clockwise, starting bottom-right: centre of each corner arc and start angle
    for (sx, sy), a0 in (((+1, -1), -90), ((+1, +1), 0), ((-1, +1), 90), ((-1, -1), 180)):
        for k in range(n_arc + 1):
            a = math.radians(a0 + 90 * k / n_arc)
            outline.append((cx + sx * hx + rc * math.cos(a), cy + sy * hy + rc * math.sin(a)))
    outline.append(outline[0])
    return [c for c in (hole, outline) if c]


# ============================================================ parallel scenarios
def run_parallel_square(cmd: CellCommander, side: float = 0.12, speed: float = 0.03,
                        arms: Optional[Sequence[str]] = None) -> None:
    """Laser engraving of a square by pure translation (thesis 5.3).
    2 or 3 arms act as one parallel manipulator; the laser is in the platform centre."""
    arms = list(arms or cmd.arms)
    attach_platform(cmd, arms)
    c = (cmd.platform.x, cmd.platform.y)
    cmd.backend.status(f"parallel square {side * 1000:.0f} mm with {len(arms)} arms")
    cut_contour(cmd, square_corners(*c, side), speed)
    cmd.translate_platform(*c, speed=0.03)
    detach_platform(cmd)


def run_laser_cutting_profile(cmd: CellCommander, width: float = 0.10, height: float = 0.07,
                              corner_radius: float = 0.012, hole_diameter: float = 0.03,
                              speed: float = 0.015) -> None:
    """Cut a real part: a plate with rounded corners and a bolt hole.

    Typical laser-cutting job (sheet metal, acrylic, wood). The platform keeps
    its orientation, the laser is switched off for travel moves, and the hole
    is cut before the outline."""
    attach_platform(cmd, cmd.arms)
    c = (cmd.platform.x, cmd.platform.y)
    contours = part_profile(*c, width, height, corner_radius, hole_diameter)
    length = sum(math.dist(p, q) for cont in contours for p, q in zip(cont[:-1], cont[1:]))
    cmd.backend.status(f"cutting {width * 1000:.0f} x {height * 1000:.0f} mm part, "
                       f"{len(contours)} contours, {length * 1000:.0f} mm of cut")
    for contour in contours:
        cut_contour(cmd, contour, speed)
    cmd.translate_platform(*c, speed=0.03)
    detach_platform(cmd)


def run_obstacle_square(cmd: CellCommander, side: float = 0.12, speed: float = 0.03,
                        margin: float = 0.020, vision_timeout: float = 10.0) -> None:
    """Cutting around a keep-out zone (thesis 5.4 + 5.5).

    The camera finds the green obstacle (e.g. a clamp); every edge of the square
    that would pass too close is replaced by the thesis via-point detour."""
    obstacles = cmd.backend.obstacles(timeout=vision_timeout)
    cmd.backend.status(f"vision: {len(obstacles)} obstacle(s)")
    attach_platform(cmd, cmd.arms)
    c = (cmd.platform.x, cmd.platform.y)
    corners = square_corners(*c, side)

    def route(p0, p1) -> List[np.ndarray]:
        path = plan_detour(p0, p1, obstacles, clearance=margin)   # margin beyond each obstacle radius
        if path is None:
            raise PlanningError(f"no collision-free detour from {p0} to {p1}")
        return path[1:]

    for p in route(c, corners[0]):
        cmd.translate_platform(float(p[0]), float(p[1]), speed=speed)
    waypoints = [replace(cmd.platform, x=float(p[0]), y=float(p[1]))
                 for p0, p1 in zip(corners[:-1], corners[1:]) for p in route(p0, p1)]
    cmd.backend.status(f"square with {len(waypoints) - (len(corners) - 1)} via-point detour(s)")
    cmd.move_platform(waypoints, speed=speed, laser=True)
    for p in route(corners[-1], c):
        cmd.translate_platform(float(p[0]), float(p[1]), speed=speed)
    detach_platform(cmd)


def run_pure_rotation(cmd: CellCommander, cycles: int = 6, step_deg: float = 60.0,
                      holders: Sequence[str] = ("arm1", "arm2"), helper: str = "arm3") -> None:
    """Re-orient the workpiece holder by regrasping (thesis 5.2).

    Two arms turn the platform by 60 deg; the third arm takes over while the
    first two regrasp. Six cycles give a full 360 deg turn - far beyond what
    a single arm could rotate."""
    attach_platform(cmd, holders)
    for i in range(cycles):
        cmd.backend.status(f"pure rotation cycle {i + 1}/{cycles}")
        cmd.rotate_platform(math.radians(step_deg))
        cmd.couple(cmd.facing_legs([helper]))            # helper clamps a free leg
        for arm in holders:                              # holders regrasp one by one
            cmd.decouple([arm])
            cmd.couple(cmd.facing_legs([arm]))
        cmd.decouple([helper])                           # helper lets go and waits
        cmd.home([helper])
    detach_platform(cmd)


def run_reconfiguration_demo(cmd: CellCommander) -> None:
    """Flexible manufacturing: serial work -> couple -> cooperative work -> decouple -> serial."""
    run_serial_pick_place(cmd, cycles=1)
    run_parallel_square(cmd, side=0.08)
    run_pure_rotation(cmd, cycles=1)
    run_serial_pick_place(cmd, cycles=1, start_side=-1)   # parts are at the outfeed now


SCENARIOS = {
    "serial_pick_place": run_serial_pick_place,
    "vision_sorting": run_vision_sorting,
    "parallel_square": run_parallel_square,
    "laser_cutting_profile": run_laser_cutting_profile,
    "obstacle_square": run_obstacle_square,
    "pure_rotation": run_pure_rotation,
    "locked_transport": run_locked_transport,
    "reconfiguration_demo": run_reconfiguration_demo,
}

__all__ = ["SCENARIOS", "station", "reject_bin", "part_profile"] + [f.__name__ for f in SCENARIOS.values()]
