# Original graduation project (2019)

*Development of a multi degrees of freedom redundant reconfigurable planar parallel
manipulator with 2 DOF planar detachable serial dyads.* İzmir Kâtip Çelebi University,
Mechatronics Engineering, May 2019.
Team: Basheer Al‑Tawil, Saran Sapmaz, Esra Uçar, Adem Candemir. Supervisor: Assist. Prof. Dr. Osman Akın.

This folder keeps the 2019 material unchanged. The modern ROS 2 version is in
[`../robocraft_ws`](../robocraft_ws), and the recalculations are in [`../analysis`](../analysis).

| Folder | Content |
|---|---|
| [`code_appendix/`](code_appendix) | code appendix of the thesis ([PDF](code_appendix/code_appendix.pdf), [Markdown](code_appendix/code_appendix.md)) |
| [`legacy_code/`](legacy_code) | the appendix code as source files ([origin and limitations](legacy_code/CODE_ORIGIN.md)) |
| [`figures/`](figures) | photo of the prototype, CAD, workspace study, H‑bridge schematic |

## Thesis contents at a glance

| Chapter | Topic | Where it lives today |
|---|---|---|
| 2.1 | workspace analysis, PC1 | [`analysis/01_workspace.py`](../analysis/01_workspace.py) |
| 2.2 | D‑H, forward/inverse kinematics, Jacobian | [`analysis/02_kinematics.py`](../analysis/02_kinematics.py), `robocraft_kinematics/scara.py` |
| 2.3–2.4 | dynamics (Lagrange), torque, motor choice | [`analysis/03_dynamics_motor_sizing.py`](../analysis/03_dynamics_motor_sizing.py) |
| 3 | mechanical design: links, bearings, gripper, platform, camera arm | [`docs/01_design.md`](../docs/01_design.md), `robocraft_description` |
| 4.1–4.2 | actuator, potentiometer calibration | [`analysis/06_sensor_calibration.py`](../analysis/06_sensor_calibration.py) |
| 4.3 | H‑bridge, PIC/ATmega, Ziegler–Nichols, I²C | [`analysis/07_motor_control_tuning.py`](../analysis/07_motor_control_tuning.py), `robocraft_hardware` |
| 5.1 | path generation (cubic via point) | [`analysis/05_trajectory_planning.py`](../analysis/05_trajectory_planning.py), `trajectory.py` |
| 5.2–5.4 | pure rotation, square, square with obstacle | scenarios `pure_rotation`, `parallel_square`, `obstacle_square` |
| 5.5 | image processing (OpenCV, green obstacle) | `vision_detector.py` |

<p align="center">
  <img src="figures/robot.jpeg" width="48%" alt="2019 prototype">
  <img src="figures/workspace_analysis.png" width="48%" alt="2019 workspace study">
</p>

> The legacy code was reconstructed from the PDF appendix. It is kept for reference only;
> it contains hardware bugs (see [`docs/04_hardware.md`](../docs/04_hardware.md#43-firmware))
> and must not be run on the robot. Use the ROS 2 workspace instead.
