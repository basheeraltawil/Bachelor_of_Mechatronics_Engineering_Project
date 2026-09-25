"""01 - Workspace of the three serial arms (thesis Section 2.1).

Question: which points of the 800 mm base plate can the arms reach, and how
large is the "dexterous" area?

Method
------
* A point (x, y) is *reachable* by an arm when the 2-link inverse kinematics
  has a solution whose joint angles are inside the joint limits:
      cos q2 = (r^2 - a^2 - b^2) / (2ab),   |cos q2| <= 1
* A point is *dexterous* (thesis definition: "reachable with any orientation")
  when the whole circle of radius rho = 60 mm around it is reachable: the arm
  can then hold a platform leg while the platform turns a full 360 deg.
* Performance measure used in the thesis:
      PC1 = dexterous area / (0.8 * footprint area),  footprint = pi * 400^2

Run:  python3 analysis/01_workspace.py
"""
import math

import matplotlib.pyplot as plt

from common import cell, header, save
from robocraft_kinematics.workspace import analyse, pc1

header("01 Workspace analysis")
c = cell()
res = analyse(c, step=0.004)

rows = []
for name in c.arm_names:
    rows.append((name, res.area(res.reach[name]), res.area(res.dexterous[name] & res.on_plate)))
serial = res.area(res.serial_dexterous)
parallel = res.area(res.parallel_dexterous)

print(f"{'arm':6s} {'reachable [mm^2]':>18s} {'dexterous [mm^2]':>18s}")
for name, r, d in rows:
    print(f"{name:6s} {r * 1e6:18.0f} {d * 1e6:18.0f}")
print(f"\nserial mode   (union of the arms)      : {serial * 1e6:9.0f} mm^2   PC1 = {pc1(serial, c.plate_radius):.2f}")
print(f"parallel mode (all 3 arms, 360 deg turn): {parallel * 1e6:9.0f} mm^2   PC1 = {pc1(parallel, c.plate_radius):.2f}")
print("thesis value (SolidWorks, no joint limits): 383438 mm^2   PC1 = 0.95")

fig, axs = plt.subplots(1, 2, figsize=(12, 6))
ext = [res.X.min(), res.X.max(), res.Y.min(), res.Y.max()]
colors = {"arm1": "Reds", "arm2": "Blues", "arm3": "Greens"}
for name in c.arm_names:
    axs[0].contour(res.X, res.Y, res.reach[name].astype(float), levels=[0.5], cmap=colors[name])
axs[0].imshow(res.serial_dexterous, origin="lower", extent=ext, cmap="Greys", alpha=0.35)
axs[0].set_title("Reachable borders (lines) and dexterous area (grey)")
axs[1].imshow(res.parallel_dexterous, origin="lower", extent=ext, cmap="Oranges", alpha=0.9)
axs[1].set_title("All 3 arms can hold the platform during a full turn")
for ax in axs:
    ax.add_patch(plt.Circle((0, 0), c.plate_radius, fill=False, lw=2))
    for name, arm in c.arms.items():
        ax.plot(*arm.base_xy, "ks")
        ax.annotate(name, arm.base_xy, xytext=(6, 6), textcoords="offset points")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
save(fig, "01_workspace.png")

assert math.isclose(c.arms["arm2"].base_xy[0], 0.27713, abs_tol=1e-4)
