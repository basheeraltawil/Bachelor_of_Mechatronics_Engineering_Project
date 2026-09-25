"""Every scenario must plan and "execute" on the kinematic dry-run backend.

This catches, without ROS or Gazebo: unreachable targets, joint-limit
violations, joint-speed violations and collisions between the arms.
"""
import math

import numpy as np
import pytest

from robocraft_kinematics import TcpPose, ik, load_cell
from robocraft_kinematics.via_point import CircleObstacle
from robocraft_control.backend import Detection, DryRunBackend
from robocraft_control.commander import HOME, CellCommander, InterferenceError
from robocraft_control.scenarios.library import SCENARIOS, part_profile, reject_bin, station

OBSTACLE = CircleObstacle(0.0, -0.11, 0.036)       # same place as in the obstacle world


def make(obstacles=None, detections=None):
    cell = load_cell()
    backend = DryRunBackend({a: HOME for a in cell.arm_names}, obstacles, detections)
    return CellCommander(cell, backend), backend


def camera_view(cmd):
    """What the camera reports in the default world: one cube on each infeed."""
    colours = {"arm1": "red", "arm2": "blue", "arm3": "yellow"}
    return [Detection(colours[a], *station(cmd, a, +1), 0.028) for a in cmd.arms]


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_scenario_dry_run(name):
    cmd, backend = make([OBSTACLE] if name == "obstacle_square" else None)
    if name == "vision_sorting":
        backend._detections = camera_view(cmd)
    kwargs = {"concurrent": False} if name in ("serial_pick_place", "vision_sorting") else {}
    SCENARIOS[name](cmd, **kwargs)
    assert backend.plans, "scenario produced no motion"
    assert not cmd.grips, "platform still clamped at the end"
    for arm in cmd.arms:
        assert np.allclose(backend.get_q(arm), HOME, atol=1e-6), f"{arm} not parked"


def test_vision_sorting_decisions():
    cmd, backend = make()
    backend._detections = camera_view(cmd)
    counts = SCENARIOS["vision_sorting"](cmd, concurrent=False)
    assert counts == {"good": 2, "rejected": 1}
    assert any(line == "grasp arm1" for line in backend.log)          # red part is handled by arm1


def test_vision_sorting_needs_camera():
    cmd, _ = make()
    with pytest.raises(Exception, match="sees no parts"):
        SCENARIOS["vision_sorting"](cmd)


def test_part_profile_cuts_hole_first():
    contours = part_profile(0, 0, 0.10, 0.07, 0.012, 0.03)
    hole, outline = contours
    assert max(math.hypot(*p) for p in hole) == pytest.approx(0.015)          # hole first
    xs, ys = zip(*outline)
    assert max(xs) == pytest.approx(0.05) and max(ys) == pytest.approx(0.035)   # 100 x 70 mm outline
    assert outline[0] == outline[-1]                                            # closed contour


def test_bins_are_reachable_and_separate():
    cmd, _ = make()
    spots = [station(cmd, a, s) for a in cmd.arms for s in (1, -1)] + [reject_bin(cmd, a) for a in cmd.arms]
    for a in cmd.arms:
        for p in (station(cmd, a, 1), station(cmd, a, -1), reject_bin(cmd, a)):
            ik(cmd.cell.arms[a], TcpPose(*p, 0.04, 0.0))
    gaps = [math.dist(p, q) for i, p in enumerate(spots) for q in spots[i + 1:]]
    assert min(gaps) > 0.08


def test_obstacle_square_uses_detour():
    cmd, backend = make([OBSTACLE])
    SCENARIOS["obstacle_square"](cmd)
    assert "laser on" in backend.log


def test_pure_rotation_turns_full_circle():
    cmd, _ = make()
    yaw0 = cmd.platform.yaw
    SCENARIOS["pure_rotation"](cmd)
    assert math.isclose(math.remainder(cmd.platform.yaw - yaw0, 2 * math.pi), 0.0, abs_tol=1e-6)


def test_interference_is_detected():
    cmd, _ = make()
    goal1 = ik(cmd.cell.arms["arm1"], TcpPose(0.05, -0.02, 0.17, 0.0))   # two TCPs on the same spot
    goal2 = ik(cmd.cell.arms["arm2"], TcpPose(0.06, -0.02, 0.17, 0.0))
    with pytest.raises(InterferenceError):
        cmd.move_joints({"arm1": goal1, "arm2": goal2})
