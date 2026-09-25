# robocraft_description

**What the robot is:** the xacro/URDF model of the whole cell, the Gazebo worlds and the
single configuration file with every dimension.

| Path | Content |
|---|---|
| `config/cell.yaml` | dimensions, masses, joint limits, platform, safety settings (source of truth) |
| `urdf/robocraft_cell.urdf.xacro` | base plate, camera mast, 3 arms, Gazebo plugins |
| `urdf/scara_arm.xacro` | one 4-axis SCARA arm (J1, J2 from the thesis + J3 quill, J4 wrist, gripper) |
| `urdf/robocraft.ros2_control.xacro` | joints and hardware plugin (mock / gazebo / real) |
| `urdf/common.xacro` | colours and inertia macros |
| `models/hex_platform/` | hexagonal platform with 6 passive leg bearings and a laser head |
| `worlds/` | `robocraft_cell.sdf` (parts, bins, cameras) and `robocraft_cell_obstacle.sdf` (+ green obstacle) |

```bash
ros2 launch robocraft_description display.launch.py     # model + joint sliders in RViz
```
