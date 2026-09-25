# robocraft_control

**What the robot does:** the high-level command language, the industrial scenarios, vision and visualisation.

| Module | Content |
|---|---|
| `commander.py` | `CellCommander`: PTP, LIN, pick, place, couple, move_platform, safety checks |
| `backend.py` | backend interface + `DryRunBackend` for offline tests |
| `ros_backend.py` | backend on ROS 2: trajectory/gripper actions, Gazebo grasp, camera |
| `scenarios/library.py` | eight scenarios (sorting, laser cutting, handover rotation, ...) |
| `scenario_runner.py` | command line: `ros2 run robocraft_control scenario <name> [--option value]` |
| `vision_detector.py` | overhead camera → coloured objects in world coordinates |
| `cell_visualizer.py` | RViz markers: platform, laser trail, obstacles |

```bash
ros2 run robocraft_control scenario list
ros2 run robocraft_control scenario laser_cutting_profile --width 0.08 --ros-args -p use_sim_time:=true
```
