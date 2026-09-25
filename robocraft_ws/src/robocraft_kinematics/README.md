# robocraft_kinematics

**The math**, in plain Python (numpy only, no ROS). It is used by the robot, the unit tests and
the [analysis scripts](../../../analysis).

| Module | Content |
|---|---|
| `geometry.py` | loads `cell.yaml` into dataclasses (arms, limits, platform) |
| `scara.py` | forward/inverse kinematics, Jacobian, manipulability of one arm |
| `parallel.py` | platform inverse/forward kinematics, closure error |
| `trajectory.py` | thesis two-segment cubic, quintic PTP, trapezoidal LIN profile |
| `via_point.py` | thesis via point (two circles) and obstacle detour planning |
| `interference.py` | capsule model of the arms, collision check between arms |
| `workspace.py` | workspace and PC1 analysis (`ros2 run robocraft_kinematics workspace_analysis`) |
| `protocol.py` | Raspberry Pi ↔ ATmega I²C frame codec with CRC‑8 |

```python
from robocraft_kinematics import load_cell, fk, ik, TcpPose
cell = load_cell()
q = ik(cell.arms["arm1"], TcpPose(0.05, -0.10, 0.10, 0.0))
```
