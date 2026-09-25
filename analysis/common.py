"""Shared helpers for the engineering analysis scripts.

Every script in this folder reads the *same* robot data as the ROS 2 model
(``robocraft_ws/src/robocraft_description/config/cell.yaml``), so the
calculations, the simulation and the real robot never disagree.

The scripts only need numpy, matplotlib and PyYAML - ROS is *not* required.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # write PNG files, no window needed
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
CELL_YAML = REPO / "robocraft_ws/src/robocraft_description/config/cell.yaml"
FIG_DIR = REPO / "docs/images/analysis"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# make the ROS-independent kinematics library importable without building the workspace
sys.path.insert(0, str(REPO / "robocraft_ws/src/robocraft_kinematics"))

from robocraft_kinematics.geometry import CellGeometry, load_cell  # noqa: E402

G = 9.81          # gravity [m/s^2]
KGCM = 0.0980665  # 1 kg·cm in N·m (motor datasheets use kg·cm)


def cell() -> CellGeometry:
    """Kinematic model of the cell (arms, platform, limits)."""
    return load_cell(str(CELL_YAML))


def config() -> dict:
    """Raw cell.yaml content (masses, dimensions, limits)."""
    with open(CELL_YAML, encoding="utf-8") as f:
        return yaml.safe_load(f)


def save(fig, name: str) -> Path:
    """Save a figure into docs/images/analysis and print where it went."""
    path = FIG_DIR / name
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  figure -> {path.relative_to(REPO)}")
    return path


def header(title: str) -> None:
    print("\n" + "=" * 72 + f"\n{title}\n" + "=" * 72)
