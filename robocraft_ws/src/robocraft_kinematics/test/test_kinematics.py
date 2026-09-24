import math

import numpy as np
import pytest

from robocraft_kinematics import (IKError, PlatformPose, TcpPose, closure_error, fk, ik,
                                  ik_planar_solutions, jacobian, load_cell, platform_fk, platform_ik)
from robocraft_kinematics.interference import check_configuration
from robocraft_kinematics.protocol import Command, State, crc8
from robocraft_kinematics.trajectory import (TrapezoidProfile, cubic_via_coefficients, eval_cubic_via,
                                             ptp_duration, quintic)
from robocraft_kinematics.via_point import CircleObstacle, plan_detour, via_point_candidates


@pytest.fixture(scope="module")
def cell():
    return load_cell()


def test_base_positions_match_thesis(cell):
    # thesis / base plate markings: (0,-320), (277.13,160), (-277.13,160) mm
    assert np.allclose(cell.arms["arm1"].base_xy, [0.0, -0.320], atol=1e-6)
    assert np.allclose(cell.arms["arm2"].base_xy, [0.27713, 0.160], atol=1e-5)
    assert np.allclose(cell.arms["arm3"].base_xy, [-0.27713, 0.160], atol=1e-5)
    assert cell.arms["arm1"].l1 == pytest.approx(0.220)


def test_thesis_forward_formula(cell):
    arm = cell.arms["arm2"]
    q1, q2 = 0.3, -0.7
    th1 = arm.base_yaw + q1
    x = arm.base_xy[0] + arm.l2 * (math.cos(th1) * math.cos(q2) - math.sin(th1) * math.sin(q2)) + arm.l1 * math.cos(th1)
    y = arm.base_xy[1] + arm.l2 * (math.cos(th1) * math.sin(q2) + math.cos(q2) * math.sin(th1)) + arm.l1 * math.sin(th1)
    p = fk(arm, [q1, q2, 0.0, 0.0])
    assert p.x == pytest.approx(x) and p.y == pytest.approx(y)


@pytest.mark.parametrize("arm_name", ["arm1", "arm2", "arm3"])
def test_ik_roundtrip(cell, arm_name):
    arm = cell.arms[arm_name]
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = np.array([rng.uniform(-2.0, 2.0), rng.uniform(0.2, 2.4) * rng.choice([-1, 1]),
                      rng.uniform(0, 0.17), rng.uniform(-3, 3)])
        pose = fk(arm, q)
        q_ik = ik(arm, pose, q_seed=q)
        assert np.allclose(q_ik, q, atol=1e-6)


def test_ik_unreachable(cell):
    arm = cell.arms["arm1"]
    with pytest.raises(IKError):
        ik(arm, TcpPose(0.0, 0.5, 0.1, 0.0))
    assert ik_planar_solutions(arm, 5.0, 5.0) == []


def test_singularity_jacobian(cell):
    arm = cell.arms["arm1"]
    assert abs(np.linalg.det(jacobian(arm, 0.2, 0.0))) < 1e-12        # stretched arm (thesis 2.2.3)
    assert abs(np.linalg.det(jacobian(arm, 0.2, 1.0))) > 1e-3


def test_platform_ik_fk_roundtrip(cell):
    grips = {"arm1": 4, "arm2": 0, "arm3": 2}
    pose = PlatformPose(0.02, -0.01, math.radians(35), cell.platform.grip_height)
    q = platform_ik(cell, pose, grips)
    est = platform_fk(cell, q, grips)
    assert est.x == pytest.approx(pose.x, abs=1e-9)
    assert est.y == pytest.approx(pose.y, abs=1e-9)
    assert est.yaw == pytest.approx(pose.yaw, abs=1e-9)
    assert closure_error(cell, q, grips) < 1e-9


def test_start_configuration_is_interference_free(cell):
    grips = {"arm1": 4, "arm2": 0, "arm3": 2}
    pose = PlatformPose(0.0, 0.0, cell.platform.initial_yaw, cell.platform.grip_height)
    q = platform_ik(cell, pose, grips)
    w = check_configuration(cell, q)
    assert w.margin > 0.0, str(w)


def test_cubic_via_continuity():
    c = cubic_via_coefficients(10.0, 40.0, 25.0, 4.0, 3.0)
    p0, v0, _ = eval_cubic_via(c, 4.0, 3.0, 0.0)
    pv1, vv1, av1 = eval_cubic_via(c, 4.0, 3.0, 4.0)
    pv2, vv2, av2 = eval_cubic_via(c, 4.0, 3.0, 4.0 + 1e-9)
    pf, vf, _ = eval_cubic_via(c, 4.0, 3.0, 7.0)
    assert p0 == pytest.approx(10.0) and v0 == pytest.approx(0.0)
    assert pv1 == pytest.approx(40.0) and pf == pytest.approx(25.0) and vf == pytest.approx(0.0, abs=1e-9)
    assert vv1 == pytest.approx(vv2, abs=1e-6) and av1 == pytest.approx(av2, abs=1e-6)


def test_quintic_and_ptp():
    q, qd, _ = quintic(np.zeros(2), np.array([1.0, -2.0]), 2.0, 2.0)
    assert np.allclose(q, [1.0, -2.0]) and np.allclose(qd, 0.0)
    T = ptp_duration([0, 0], [1.0, 0.1], [1.0, 1.0], [5.0, 5.0])
    assert T == pytest.approx(1.875, rel=1e-3)


def test_trapezoid_profile():
    p = TrapezoidProfile(0.1, 0.05, 0.2)
    assert p.s(0.0) == 0.0
    assert p.s(p.duration) == pytest.approx(0.1)
    tri = TrapezoidProfile(0.001, 1.0, 1.0)
    assert tri.s(tri.duration) == pytest.approx(0.001)


def test_thesis_via_point_geometry():
    a, b = via_point_candidates([0, 0], [0.1, 0], 30.0)
    L = 0.05 / math.cos(math.radians(30))
    for v in (a, b):
        assert np.linalg.norm(v) == pytest.approx(L)
        assert np.linalg.norm(v - np.array([0.1, 0])) == pytest.approx(L)


def test_detour_avoids_obstacle():
    ob = [CircleObstacle(0.05, 0.0, 0.02)]
    path = plan_detour([0, 0], [0.1, 0], ob, clearance=0.02)
    assert path is not None and len(path) == 3
    assert plan_detour([0, 0], [0.1, 0], [], 0.03) is not None


def test_protocol_roundtrip():
    c = Command(7, math.radians(12.34), math.radians(-56.78), 90, True, True)
    d = Command.decode(c.encode())
    assert d.seq == 7 and math.degrees(d.q1) == pytest.approx(12.34) and d.brake
    s = State(3, 0.5, -0.25, 0b1001)
    assert State.decode(s.encode()).status == 0b1001
    bad = bytearray(c.encode())
    bad[3] ^= 0x10
    with pytest.raises(ValueError):
        Command.decode(bytes(bad))
    assert crc8(b"123456789") == 0xF4  # CRC-8/SMBUS check value
