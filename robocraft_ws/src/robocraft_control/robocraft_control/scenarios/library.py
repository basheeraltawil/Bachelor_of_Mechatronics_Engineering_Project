"""RoboCraft simulation scenarios.

Each scenario is a plain function ``run_<name>(cmd, **params)`` operating on a
:class:`~robocraft_control.commander.CellCommander`, so the *same* code runs
in unit tests (dry-run backend), RViz (mock hardware), Gazebo and on the real
cell.

=====================  =========  =================================================
scenario               mode       thesis reference
=====================  =========  =================================================
serial_pick_place      serial     purpose of the project (independent arms, 1.2)
parallel_square        parallel   5.3 drawing square without obstacle (laser cutting)
pure_rotation          parallel   5.2 pure rotation of the platform with handover
obstacle_square        parallel   5.4 + 5.5 square with green obstacle (camera)
locked_transport       serial     3.2 lock mechanism (single arm carries platform)
reconfiguration_demo   both       full serial -> parallel -> serial cycle
=====================  =========  =================================================
"""
from __future__ import annotations

import math
import threading
from dataclasses import replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from robocraft_kinematics.via_point import plan_detour

from ..commander import CellCommander, PlanningError

PART_GRASP_Z = 0.040        # TCP height for the 40 mm cubes (centre at 0.030)
STATION_RADIUS = 0.18       # distance of the part stations from the column
STATION_ANGLE = math.radians(35.0)  # stations stay outside the platform sweep


def station(cmd: CellCommander, arm: str, side: int) -> Tuple[float, float]:
    """Part station of an arm: side +1 = infeed, -1 = outfeed."""
    a = cmd.cell.arms[arm]
    t = a.base_yaw + side * STATION_ANGLE
    p = a.base_xy + STATION_RADIUS * np.array([math.cos(t), math.sin(t)])
    return float(p[0]), float(p[1])


# ============================================================ serial mode
def run_serial_pick_place(cmd: CellCommander, cycles: int = 2, concurrent: bool = True,
                          arms: Optional[Sequence[str]] = None, start_side: int = +1) -> None:
    """Every arm works on its own: pick its part at the infeed, place at the
    outfeed, then back (ping-pong); ``start_side=-1`` starts at the outfeed.  Arms run concurrently in their own
    threads; the commander's interference interlock serialises motions that
    would collide."""
    arms = list(arms or cmd.arms)
    errors: List[BaseException] = []

    def worker(arm: str) -> None:
        try:
            src, dst = (+1, -1) if start_side >= 0 else (-1, +1)
            for _ in range(cycles):
                sx, sy = station(cmd, arm, src)
                dx, dy = station(cmd, arm, dst)
                cmd.pick(arm, sx, sy, PART_GRASP_Z)
                cmd.place(arm, dx, dy, PART_GRASP_Z)
                src, dst = dst, src
            cmd.home([arm])
        except BaseException as e:  # noqa: BLE001 - reported below
            errors.append(e)

    if concurrent:
        threads = [threading.Thread(target=worker, args=(a,), daemon=True) for a in arms]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    else:
        for a in arms:
            worker(a)
    if errors:
        raise errors[0]


# ============================================================ parallel mode helpers
def attach_platform(cmd: CellCommander, arms: Sequence[str]) -> Dict[str, int]:
    """Serial -> parallel: clamp the legs facing each arm and lift."""
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
    h = side / 2
    return [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h), (cx - h, cy - h)]


# ============================================================ parallel scenarios
def run_parallel_square(cmd: CellCommander, side: float = 0.12, speed: float = 0.03,
                        arms: Optional[Sequence[str]] = None) -> None:
    """Thesis 5.3: the platform (laser head in the centre) draws a square by pure
    translation while 2 or 3 arms work as one parallel manipulator."""
    arms = list(arms or cmd.arms)
    attach_platform(cmd, arms)
    c = (cmd.platform.x, cmd.platform.y)
    corners = square_corners(c[0], c[1], side)
    cmd.backend.status(f"parallel square {side * 1000:.0f} mm with {len(arms)} arms")
    cmd.translate_platform(*corners[0], speed=speed)
    cmd.move_platform([replace(cmd.platform, x=x, y=y) for x, y in corners[1:]], speed=speed, laser=True)
    cmd.translate_platform(*c, speed=speed)
    detach_platform(cmd)


def run_pure_rotation(cmd: CellCommander, cycles: int = 6, step_deg: float = 60.0,
                      holders: Sequence[str] = ("arm1", "arm2"), helper: str = "arm3") -> None:
    """Thesis 5.2: two arms rotate the platform 60 deg, the third arm takes over
    while the first two re-grasp; six cycles give a full 360 deg turn."""
    attach_platform(cmd, holders)
    for i in range(cycles):
        cmd.backend.status(f"pure rotation cycle {i + 1}/{cycles}")
        cmd.rotate_platform(math.radians(step_deg))
        cmd.couple(cmd.facing_legs([helper]))            # M3 clamps a free leg
        for arm in holders:                              # M1, then M2 re-grasp
            cmd.decouple([arm])
            cmd.couple(cmd.facing_legs([arm]))
        cmd.decouple([helper])                           # M3 lets go and waits
        cmd.home([helper])
    detach_platform(cmd)


def run_obstacle_square(cmd: CellCommander, side: float = 0.12, speed: float = 0.03,
                        margin: float = 0.020, vision_timeout: float = 10.0) -> None:
    """Thesis 5.4/5.5: the camera finds the green obstacle; every square edge
    whose laser path would cross it is replaced by the thesis via-point detour."""
    obstacles = cmd.backend.obstacles(timeout=vision_timeout)
    cmd.backend.status(f"vision: {len(obstacles)} obstacle(s)")
    attach_platform(cmd, cmd.arms)
    c = (cmd.platform.x, cmd.platform.y)
    corners = square_corners(c[0], c[1], side)

    def route(p0, p1) -> List[np.ndarray]:
        path = plan_detour(p0, p1, obstacles, clearance=margin)  # margin beyond each radius
        if path is None:
            raise PlanningError(f"no collision-free detour from {p0} to {p1}")
        return path[1:]

    for p in route(c, corners[0]):
        cmd.translate_platform(float(p[0]), float(p[1]), speed=speed)
    waypoints = []
    for p0, p1 in zip(corners[:-1], corners[1:]):
        for p in route(p0, p1):
            waypoints.append(replace(cmd.platform, x=float(p[0]), y=float(p[1])))
    detours = len(waypoints) - (len(corners) - 1)
    cmd.backend.status(f"square with {detours} via-point detour(s)")
    cmd.move_platform(waypoints, speed=speed, laser=True)
    for p in route(corners[-1], c):
        cmd.translate_platform(float(p[0]), float(p[1]), speed=speed)
    detach_platform(cmd)


def run_locked_transport(cmd: CellCommander, arm: str = "arm1", dx: float = 0.04,
                         dyaw_deg: float = 30.0) -> None:
    """Thesis 3.2 lock mechanism: with the leg brake engaged a single arm holds
    the platform rigidly, so it can move *and* orient it (J4) on its own."""
    legs = cmd.facing_legs([arm])
    cmd.couple(legs, locked=True)
    cmd.lift(cmd.cell.platform.lift_height)
    p0 = cmd.platform
    ax = cmd.cell.arms[arm].base_xy
    direction = -ax / np.linalg.norm(ax)        # towards the cell centre line
    perp = np.array([-direction[1], direction[0]])
    cmd.move_platform([replace(p0, x=p0.x + dx * perp[0], y=p0.y + dx * perp[1])], speed=0.03)
    cmd.rotate_platform(math.radians(dyaw_deg))
    cmd.rotate_platform(-math.radians(dyaw_deg))
    cmd.move_platform([p0], speed=0.03)
    detach_platform(cmd)


def run_reconfiguration_demo(cmd: CellCommander) -> None:
    """Serial work -> automatic coupling -> parallel task -> decoupling -> serial."""
    run_serial_pick_place(cmd, cycles=1)
    run_parallel_square(cmd, side=0.08)
    run_pure_rotation(cmd, cycles=1)
    run_serial_pick_place(cmd, cycles=1, start_side=-1)   # parts are at the outfeed now


SCENARIOS = {
    "serial_pick_place": run_serial_pick_place,
    "parallel_square": run_parallel_square,
    "pure_rotation": run_pure_rotation,
    "obstacle_square": run_obstacle_square,
    "locked_transport": run_locked_transport,
    "reconfiguration_demo": run_reconfiguration_demo,
}

__all__ = ["SCENARIOS", "station"] + [f.__name__ for f in SCENARIOS.values()]
