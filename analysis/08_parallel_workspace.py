"""08 - Workspace of the parallel mode (platform carried by 2 or 3 arms).

Script 01 used the strict "full 360 deg rotation" definition. For real tasks
(laser cutting, engraving, transport) the platform usually keeps one
orientation, so the useful workspace is the *constant-orientation* workspace:
all platform centres p for which every clamping arm reaches its leg

    leg_k(p, phi) = p + R(phi) * rho * [cos 60k deg, sin 60k deg],   rho = 60 mm

inside its joint limits (inverse kinematics of script 02 for each arm).

The second map shows the *rotation capability*: starting from the nominal
orientation (30 deg), how many degrees the platform can turn at each point
without any arm leaving its limits.

Run:  python3 analysis/08_parallel_workspace.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np

from common import cell, header, save
from robocraft_kinematics.workspace import reachable_mask

header("08 Parallel-mode workspace")
c = cell()
rho, yaw0 = c.platform.leg_radius, c.platform.initial_yaw
step = 0.004
xs = np.arange(-0.25, 0.25 + step / 2, step)
X, Y = np.meshgrid(xs, xs)
# legs facing each column at the nominal orientation: arm1 -> leg 4, arm2 -> leg 0, arm3 -> leg 2
grips = {"arm1": 4, "arm2": 0, "arm3": 2}


def feasible(arms, yaw):
    ok = np.ones_like(X, dtype=bool)
    for arm in arms:
        ang = yaw + grips[arm] * math.pi / 3
        ok &= reachable_mask(c.arms[arm], X + rho * math.cos(ang), Y + rho * math.sin(ang))
    return ok


three = feasible(["arm1", "arm2", "arm3"], yaw0)
two = feasible(["arm1", "arm2"], yaw0)
area = lambda m: m.sum() * step ** 2 * 1e6  # noqa: E731
print(f"constant orientation (30 deg), 3 arms: {area(three):8.0f} mm^2")
print(f"constant orientation (30 deg), 2 arms: {area(two):8.0f} mm^2")

# rotation capability: largest symmetric +/- range around yaw0 that stays feasible
rot = np.zeros_like(X)
alive = three.copy()
for d in range(1, 91):
    alive &= feasible(["arm1", "arm2", "arm3"], yaw0 + math.radians(d))
    alive &= feasible(["arm1", "arm2", "arm3"], yaw0 - math.radians(d))
    rot[alive] = d
print(f"rotation capability at the centre: +/-{rot[len(xs) // 2, len(xs) // 2]:.0f} deg "
      f"(the handover scenario uses 60 deg steps with regrasping)")

fig, axs = plt.subplots(1, 2, figsize=(12, 5.5))
ext = [xs[0] * 1000, xs[-1] * 1000, xs[0] * 1000, xs[-1] * 1000]
axs[0].imshow(two, origin="lower", extent=ext, cmap="Blues", alpha=0.5)
axs[0].imshow(np.ma.masked_where(~three, three), origin="lower", extent=ext, cmap="autumn", alpha=0.8)
axs[0].set_title("Platform centre, fixed orientation\nred: 3 arms, blue: 2 arms (arm1 + arm2)")
im = axs[1].imshow(np.ma.masked_where(~three, rot), origin="lower", extent=ext, cmap="viridis")
fig.colorbar(im, ax=axs[1], label="possible rotation +/- [deg] (capped at 90)")
axs[1].set_title("How far the platform can turn (3 arms)")
for ax in axs:
    ax.plot([-60, 60, 60, -60, -60], [-60, -60, 60, 60, -60], "k--", lw=1, label="120 mm square task")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.legend(loc="lower right", fontsize=8)
save(fig, "08_parallel_workspace.png")
