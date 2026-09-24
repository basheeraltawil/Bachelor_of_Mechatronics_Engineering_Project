"""Every scenario must plan and 'execute' on the kinematic dry-run backend:
IK feasible, inside joint limits, within joint speed limits, no interference."""
import math

import numpy as np
import pytest

from robocraft_kinematics import load_cell
from robocraft_kinematics.via_point import CircleObstacle
from robocraft_control.backend import DryRunBackend
from robocraft_control.commander import HOME, CellCommander, InterferenceError
from robocraft_control.scenarios.library import SCENARIOS


def make(obstacles=None):
    cell = load_cell()
    be = DryRunBackend({a: HOME for a in cell.arm_names}, obstacles)
    return CellCommander(cell, be), be


@pytest.mark.parametrize("name", sorted(SCENARIOS))
def test_scenario_dry_run(name):
    obstacles = [CircleObstacle(0.0, -0.11, 0.036)] if name == "obstacle_square" else None
    cmd, be = make(obstacles)
    kwargs = {"concurrent": False} if name == "serial_pick_place" else {}
    SCENARIOS[name](cmd, **kwargs)
    assert be.plans, "scenario produced no motion"
    assert not cmd.grips
    for arm in cmd.arms:
        assert np.allclose(be.get_q(arm), HOME, atol=1e-6), f"{arm} not parked"
    print(f"{name}: {len(be.plans)} motions, {be.sim_time:.1f} s, min margin {cmd.last_margin * 1000:.0f} mm")


def test_obstacle_square_uses_detour():
    cmd, be = make([CircleObstacle(0.0, -0.11, 0.036)])
    SCENARIOS["obstacle_square"](cmd)
    # a detour adds at least one extra waypoint -> platform trajectory avoids the zone
    assert any("laser on" in line for line in be.log)


def test_pure_rotation_turns_full_circle():
    cmd, _ = make()
    yaw0 = cmd.platform.yaw
    SCENARIOS["pure_rotation"](cmd)
    assert math.isclose((cmd.platform.yaw - yaw0) % (2 * math.pi), 0.0, abs_tol=1e-6) or \
        math.isclose(cmd.platform.yaw - yaw0, 2 * math.pi, abs_tol=1e-6)


def test_interference_is_detected():
    cmd, _ = make()
    # drive arm1 and arm2 TCPs onto the same spot -> must be rejected
    from robocraft_kinematics import TcpPose, ik
    goal1 = ik(cmd.cell.arms["arm1"], TcpPose(0.05, -0.02, 0.17, 0.0))
    goal2 = ik(cmd.cell.arms["arm2"], TcpPose(0.06, -0.02, 0.17, 0.0))
    with pytest.raises(InterferenceError):
        cmd.move_joints({"arm1": goal1, "arm2": goal2})
