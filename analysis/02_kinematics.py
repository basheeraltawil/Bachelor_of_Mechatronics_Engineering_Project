"""02 - Forward / inverse kinematics, Jacobian and singularities (thesis 2.2).

Forward kinematics (arm i, base at (x_i, y_i), th1 = base yaw + q1):
    x = x_i + a cos th1 + b cos(th1 + q2)
    y = y_i + a sin th1 + b sin(th1 + q2)

Inverse kinematics (square and add the two equations):
    f^2 + l^2 = a^2 + b^2 + 2ab cos q2   ->  q2 = +/- acos(...)   (two elbows)
    q1 = atan2(dy, dx) - atan2(b sin q2, a + b cos q2) - base yaw

Jacobian  [xdot, ydot]^T = J [q1dot, q2dot]^T,   det J = a b sin q2
    det J = 0  <=>  q2 = 0 (arm stretched) or q2 = 180 deg (folded): singular.
Yoshikawa manipulability  w = |det J|  and condition number  k = s_max / s_min
tell how "well" the arm can move in every direction at a pose.

This script (1) checks FK/IK against each other on random poses, (2) maps
manipulability and conditioning over the workspace of one arm.

Run:  python3 analysis/02_kinematics.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np

from common import cell, header, save
from robocraft_kinematics.scara import fk, ik, jacobian

header("02 Kinematics, Jacobian, singularities")
c = cell()
arm = c.arms["arm1"]

# (1) FK -> IK round trip on random joint vectors ------------------------------
rng = np.random.default_rng(0)
errors = []
for _ in range(2000):
    q = np.array([rng.uniform(-2.5, 2.5), rng.choice([-1, 1]) * rng.uniform(0.1, 2.5),
                  rng.uniform(0, 0.17), rng.uniform(-3, 3)])
    q_back = ik(arm, fk(arm, q), q_seed=q)
    errors.append(np.max(np.abs(q_back - q)))
print(f"FK->IK round trip on 2000 random poses: max joint error = {max(errors):.2e} rad")

# (2) manipulability / condition number over the arm's reach ------------------
a, b = arm.l1, arm.l2
q2 = np.linspace(-arm.j2.upper, arm.j2.upper, 400)
w = np.abs(a * b * np.sin(q2))
print(f"manipulability max = {w.max():.4f} m^2 at |q2| = {abs(math.degrees(q2[np.argmax(w)])):.0f} deg "
      f"(elbow at 90 deg is the most dexterous pose)")

r = np.linspace(0.01, a + b - 1e-4, 300)          # distance column -> TCP
cos_q2 = np.clip((r ** 2 - a ** 2 - b ** 2) / (2 * a * b), -1, 1)
cond = []
for cq in cos_q2:
    J = jacobian(arm, 0.0, math.acos(cq))
    s = np.linalg.svd(J, compute_uv=False)
    cond.append(s[0] / max(s[1], 1e-12))
cond = np.array(cond)
good = r[cond < 3.0]
print(f"well-conditioned band (condition number < 3): r = {good.min() * 1000:.0f} ... {good.max() * 1000:.0f} mm "
      f"from the column")

fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))
axs[0].plot(np.degrees(q2), w * 1e4)
axs[0].set_xlabel("elbow angle q2 [deg]")
axs[0].set_ylabel("manipulability |det J| [cm^2 x 100]")
axs[0].set_title("Singular at q2 = 0 (stretched) - best at +/-90 deg")
axs[0].grid(alpha=0.3)
axs[1].semilogy(r * 1000, cond)
axs[1].axhspan(1, 3, color="tab:green", alpha=0.15, label="well conditioned (k < 3)")
axs[1].set_xlabel("distance column -> TCP [mm]")
axs[1].set_ylabel("condition number k")
axs[1].set_title("Avoid the fully stretched and fully folded arm")
axs[1].legend()
axs[1].grid(alpha=0.3, which="both")
save(fig, "02_kinematics.png")
