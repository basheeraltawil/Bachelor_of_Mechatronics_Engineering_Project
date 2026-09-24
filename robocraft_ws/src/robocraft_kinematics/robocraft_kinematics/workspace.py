"""Workspace analysis (thesis Section 2.1) with the real joint limits.

Dexterous point (thesis definition): a point the gripper can reach "with any
orientation".  For the hex platform that means the whole circle of radius
``leg_radius`` around a platform centre must be reachable, so the platform can
spin 360 deg while the arm keeps holding a leg.

* ``serial``  area — union over the three arms, clipped to the base plate
* ``parallel`` area — intersection (all three arms can hold the platform)
* ``PC1 = area / (0.8 * footprint)`` exactly as in the thesis.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

from .geometry import ArmGeometry, CellGeometry, load_cell


def reachable_mask(arm: ArmGeometry, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
    """Vectorised: True where (X, Y) has an IK solution inside J1/J2 limits."""
    dx, dy = X - arm.base_xy[0], Y - arm.base_xy[1]
    r2 = dx * dx + dy * dy
    c2 = (r2 - arm.l1 ** 2 - arm.l2 ** 2) / (2 * arm.l1 * arm.l2)
    inside = np.abs(c2) <= 1.0
    c2 = np.clip(c2, -1.0, 1.0)
    ok = np.zeros_like(X, dtype=bool)
    for sign in (1.0, -1.0):
        q2 = sign * np.arccos(c2)
        th1 = np.arctan2(dy, dx) - np.arctan2(arm.l2 * np.sin(q2), arm.l1 + arm.l2 * np.cos(q2))
        q1 = np.arctan2(np.sin(th1 - arm.base_yaw), np.cos(th1 - arm.base_yaw))
        ok |= (q1 >= arm.j1.lower) & (q1 <= arm.j1.upper) & (q2 >= arm.j2.lower) & (q2 <= arm.j2.upper)
    return inside & ok


@dataclass
class WorkspaceResult:
    step: float
    X: np.ndarray
    Y: np.ndarray
    on_plate: np.ndarray
    reach: dict
    dexterous: dict

    def area(self, mask: np.ndarray) -> float:
        return float(mask.sum()) * self.step ** 2

    @property
    def serial_dexterous(self) -> np.ndarray:
        return np.logical_or.reduce(list(self.dexterous.values())) & self.on_plate

    @property
    def parallel_dexterous(self) -> np.ndarray:
        return np.logical_and.reduce(list(self.dexterous.values())) & self.on_plate


def analyse(cell: CellGeometry, step: float = 0.004, n_orient: int = 24) -> WorkspaceResult:
    R = cell.plate_radius
    xs = np.arange(-R, R + step / 2, step)
    X, Y = np.meshgrid(xs, xs)
    on_plate = X ** 2 + Y ** 2 <= R ** 2
    rho = cell.platform.leg_radius
    reach, dex = {}, {}
    for name, arm in cell.arms.items():
        reach[name] = reachable_mask(arm, X, Y) & on_plate
        m = np.ones_like(X, dtype=bool)
        for k in range(n_orient):
            a = 2 * math.pi * k / n_orient
            m &= reachable_mask(arm, X + rho * math.cos(a), Y + rho * math.sin(a))
        dex[name] = m
    return WorkspaceResult(step, X, Y, on_plate, reach, dex)


def pc1(area: float, plate_radius: float) -> float:
    return area / (0.8 * math.pi * plate_radius ** 2)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="RoboCraft workspace analysis (thesis Sec. 2.1)")
    ap.add_argument("--config", default=None, help="path to cell.yaml")
    ap.add_argument("--step", type=float, default=0.004, help="grid step [m]")
    ap.add_argument("--plot", default=None, help="write a PNG figure to this path")
    args = ap.parse_args(argv)

    cell = load_cell(args.config)
    res = analyse(cell, args.step)
    ser, par = res.area(res.serial_dexterous), res.area(res.parallel_dexterous)
    print("RoboCraft workspace analysis (joint limits included)")
    for n in cell.arm_names:
        print(f"  {n}: reachable {res.area(res.reach[n]) * 1e6:10.0f} mm^2   "
              f"dexterous {res.area(res.dexterous[n] & res.on_plate) * 1e6:10.0f} mm^2")
    print(f"  serial   dexterous (union)        : {ser * 1e6:10.0f} mm^2   PC1 = {pc1(ser, cell.plate_radius):.2f}")
    print(f"  parallel dexterous (intersection) : {par * 1e6:10.0f} mm^2   PC1 = {pc1(par, cell.plate_radius):.2f}")
    print("  thesis reference                  :     383438 mm^2   PC1 = 0.95")

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(7, 7))
        ext = [res.X.min(), res.X.max(), res.Y.min(), res.Y.max()]
        ax.imshow(res.serial_dexterous, origin="lower", extent=ext, cmap="Blues", alpha=0.45)
        ax.imshow(np.ma.masked_where(~res.parallel_dexterous, res.parallel_dexterous),
                  origin="lower", extent=ext, cmap="autumn", alpha=0.7)
        ax.add_patch(plt.Circle((0, 0), cell.plate_radius, fill=False, lw=2))
        for n, arm in cell.arms.items():
            ax.plot(*arm.base_xy, "ks")
            ax.annotate(n, arm.base_xy, textcoords="offset points", xytext=(6, 6))
        ax.set_aspect("equal")
        ax.set_title("Dexterous workspace: blue = serial (union), orange = parallel (all 3 arms)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        fig.savefig(args.plot, dpi=130, bbox_inches="tight")
        print(f"  figure written to {args.plot}")


if __name__ == "__main__":
    main()
