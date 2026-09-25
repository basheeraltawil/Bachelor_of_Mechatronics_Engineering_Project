"""05 - Trajectory planning (thesis Section 5.1) compared with industrial profiles.

Thesis method: go from P0 to Pf through a via point Pv.
  * Via point: intersection of two circles of radius L = (W/2)/cos(theta)
    around P0 and Pf (W = |Pf - P0|, theta = "bending angle" = 30 deg).
  * Every joint follows two cubic segments  s(t) = c0 + c1 t + c2 t^2 + c3 t^3
    with 8 conditions -> an 8x8 linear system:
      s1(0)=q0, s1(t1)=qv, s2(0)=qv, s2(t2)=qf        (positions)
      s1'(0)=0, s2'(t2)=0                             (start/stop at rest)
      s1'(t1)=s2'(0), s1''(t1)=s2''(0)                (smooth at the via point)
    (the appendix code had "2*tf+3*tf^2" in one column of the velocity row;
     the corrected matrix is in robocraft_kinematics/trajectory.py)

Industrial alternatives used by the ROS 2 cell:
  * PTP quintic:  s = 10 u^3 - 15 u^4 + 6 u^5  (zero speed AND acceleration at
    both ends -> smoother than a cubic, which jumps in acceleration)
  * LIN: straight TCP line with a trapezoidal speed profile, IK at every sample.

Run:  python3 analysis/05_trajectory_planning.py
"""
import matplotlib.pyplot as plt
import numpy as np

from common import cell, header, save
from robocraft_kinematics.scara import TcpPose, fk_planar, ik
from robocraft_kinematics.trajectory import (TrapezoidProfile, cubic_via_coefficients, eval_cubic_via,
                                             quintic)
from robocraft_kinematics.via_point import via_point_candidates

header("05 Trajectory planning")
arm = cell().arms["arm1"]
P0, Pf = np.array([-0.12, -0.10]), np.array([0.12, -0.05])
Pv = via_point_candidates(P0, Pf, 30.0)[0]
q = {n: ik(arm, TcpPose(*p, 0.10, 0.0))[:2] for n, p in (("0", P0), ("v", Pv), ("f", Pf))}
t1 = t2 = 2.0
print(f"P0 {P0}, via {np.round(Pv, 3)}, Pf {Pf} (bending angle 30 deg)")

t = np.linspace(0, t1 + t2, 400)
thesis = np.zeros((len(t), 2, 3))
for j in range(2):
    c = cubic_via_coefficients(q["0"][j], q["v"][j], q["f"][j], t1, t2)
    thesis[:, j, :] = [eval_cubic_via(c, t1, t2, ti) for ti in t]
    print(f"J{j + 1}: cubic coefficients seg1 = {np.round(c[:4], 4)}, seg2 = {np.round(c[4:], 4)}")

ptp = np.array([[*quintic(q["0"], q["f"], t1 + t2, ti)] for ti in t])      # (n, 3, 2)
prof = TrapezoidProfile(float(np.linalg.norm(Pf - P0)), 0.1, 0.2)
lin_xy = np.array([P0 + (Pf - P0) * prof.s(ti) / prof.length for ti in np.linspace(0, prof.duration, 100)])
path_thesis = np.array([fk_planar(arm, *thesis[i, :, 0]) for i in range(len(t))])
path_ptp = np.array([fk_planar(arm, *ptp[i, 0, :]) for i in range(len(t))])

fig, axs = plt.subplots(1, 3, figsize=(15, 4.3))
for j, style in ((0, "-"), (1, "--")):
    axs[0].plot(t, np.degrees(thesis[:, j, 0]), style, label=f"J{j + 1} thesis cubic + via")
    axs[0].plot(t, np.degrees(ptp[:, 0, j]), style, alpha=0.5, label=f"J{j + 1} PTP quintic")
    axs[1].plot(t, np.degrees(thesis[:, j, 2]), style, label=f"J{j + 1} thesis cubic")
    axs[1].plot(t, np.degrees(ptp[:, 2, j]), style, alpha=0.5, label=f"J{j + 1} quintic")
axs[0].axvline(t1, color="grey", lw=0.8)
axs[0].set_title("Joint angle")
axs[0].set_ylabel("deg")
axs[1].set_title("Joint acceleration (cubic jumps at start/end)")
axs[1].set_ylabel("deg/s^2")
for ax in axs[:2]:
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
axs[2].plot(*path_thesis.T * 1000, label="thesis: cubic via point")
axs[2].plot(*path_ptp.T * 1000, label="PTP (joint space, curved)")
axs[2].plot(*lin_xy.T * 1000, label="LIN (straight line)")
axs[2].plot(*np.array([P0, Pv, Pf]).T * 1000, "ko:", ms=4, label="P0 - via - Pf")
axs[2].set_aspect("equal")
axs[2].set_title("TCP path seen from above")
axs[2].set_xlabel("x [mm]")
axs[2].set_ylabel("y [mm]")
axs[2].legend(fontsize=7)
axs[2].grid(alpha=0.3)
save(fig, "05_trajectory_planning.png")
