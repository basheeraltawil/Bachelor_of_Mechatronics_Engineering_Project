# 5. Developer guide

## 5.1 Code map

```
robocraft_ws/src/
├── robocraft_description/        what the robot IS
│   ├── config/cell.yaml            all dimensions, masses, limits  ← single source of truth
│   ├── urdf/                       xacro model: cell, one arm, ros2_control, colours
│   ├── models/hex_platform/        platform with 6 passive leg bearings
│   └── worlds/                     Gazebo worlds: cell, cell + obstacle
├── robocraft_kinematics/         the MATH (pure Python, no ROS)
│   ├── geometry.py                 reads cell.yaml into dataclasses
│   ├── scara.py                    FK, IK, Jacobian of one arm
│   ├── parallel.py                 platform IK / FK, closure error
│   ├── trajectory.py               thesis cubic, quintic PTP, trapezoid LIN
│   ├── via_point.py                thesis via point + obstacle detour
│   ├── interference.py             collision model between arms
│   ├── workspace.py                workspace analysis
│   └── protocol.py                 I²C frame codec (CRC-8)
├── robocraft_control/            what the robot DOES
│   ├── commander.py                CellCommander: PTP, LIN, pick, place, couple, move_platform
│   ├── backend.py                  Backend interface + DryRunBackend (tests)
│   ├── ros_backend.py              Backend on ROS 2 (actions, topics, Gazebo grasp)
│   ├── scenarios/library.py        the industrial scenarios
│   ├── scenario_runner.py          `ros2 run robocraft_control scenario ...`
│   ├── vision_detector.py          camera → coloured objects in metres
│   └── cell_visualizer.py          RViz markers: platform, laser trail, obstacles
├── robocraft_bringup/            how to START it: launch files + controller configs
├── robocraft_gz_plugins/         Gazebo plugin: verified grasp + platform brake (C++)
├── robocraft_hardware/           real robot: ros2_control I²C driver (C++) + ATmega firmware
└── robocraft_interfaces/         messages: Obstacle, ObstacleArray, CellStatus
```

Dependencies only go downwards: `control → kinematics → description (cell.yaml)`.

## 5.2 Add your own scenario (example)

Scenarios only use the commander's high-level methods. Example: stack all three parts in
front of arm2.

```python
# robocraft_ws/src/robocraft_control/robocraft_control/scenarios/library.py
def run_stack_parts(cmd: CellCommander, arm: str = "arm2") -> None:
    """Collect the parts of the other arms in front of `arm` and stack them."""
    tower = station(cmd, arm, -1)                     # where the stack is built
    for level, source in enumerate(cmd.arms):
        cmd.pick(source, *station(cmd, source, +1), PART_GRASP_Z)
        cmd.place(source, *tower, PART_GRASP_Z + 0.04 * level)
        cmd.home([source])

SCENARIOS["stack_parts"] = run_stack_parts           # register it
```

Then:

1. add it to the dry‑run test (`test/test_scenarios_dry_run.py` runs every registered
   scenario automatically) and run `colcon test --packages-select robocraft_control`;
2. try it without physics: `ros2 launch robocraft_bringup mock.launch.py scenario:=stack_parts`;
3. run it in Gazebo.

This example is deliberately flawed: the dry run stops it before anything moves with
`IKError arm1: (0.098, 0.176) outside reach 0.440 m`, because the tower is out of arm1's reach.
Catching that kind of mistake before it reaches the hardware is what the test is for.

## 5.3 Commander API cheat sheet

| Call | Meaning |
|---|---|
| `cmd.move_joints({"arm1": q})` | PTP in joint space (several arms: synchronised) |
| `cmd.ptp("arm1", TcpPose(x, y, z, yaw))` | PTP to a Cartesian pose |
| `cmd.lin({"arm1": pose1, "arm2": pose2}, speed)` | straight lines, arriving together |
| `cmd.pick(arm, x, y, z)` / `cmd.place(arm, x, y, z)` | full sequences with grasp check |
| `cmd.couple({"arm1": 4, "arm2": 0}, locked=False)` | clamp platform legs |
| `cmd.move_platform([PlatformPose(...), ...], speed, laser=True)` | platform path |
| `cmd.lift(dz)`, `cmd.rotate_platform(dyaw)`, `cmd.translate_platform(x, y)` | shortcuts |
| `cmd.decouple(arms)`, `cmd.home(arms)` | release, park |
| `cmd.backend.detections()` / `obstacles()` | camera results |

## 5.4 Change the robot

* **Dimensions, masses, limits:** edit [`cell.yaml`](../robocraft_ws/src/robocraft_description/config/cell.yaml).
  The URDF, kinematics, tests and analysis scripts all pick up the change.
* **Controller settings:** edit `robocraft_bringup/config/generate_controllers.py` and run it in that
  folder (it writes the mock/real and Gazebo YAML files).
* **New colour for the camera:** add one entry to `CLASSES` in `vision_detector.py`.

## 5.5 Tests and CI

```bash
cd robocraft_ws
colcon build --symlink-install
colcon test && colcon test-result --verbose
python3 ../analysis/run_all.py            # regenerates all analysis figures
```

| Suite | Checks |
|---|---|
| `robocraft_kinematics/test` | thesis formulas, IK round trips, singularity, platform IK/FK, trajectories, protocol |
| `robocraft_control/test` | every scenario in dry run, sorting decisions, part profile, bins, vision colour classes |
| `robocraft_hardware/test` | C++ frame encoding and CRC |

GitHub Actions ([`.github/workflows/ci.yml`](../.github/workflows/ci.yml)) builds the workspace in a
ROS 2 Humble container, runs all tests and the analysis scripts, and compiles the firmware on every push.

## 5.6 Coding conventions

* SI units in code (m, rad, s, kg), with the unit in a comment when it is not obvious.
* Every module starts with a docstring: what it does, the formula or method, where it is used.
* Pure computation (kinematics, planning) stays independent of ROS so it can be tested quickly.
* Constants have a name and a comment (`SAFE_Z = 0.170  # TCP travel height [m] ...`), not magic numbers.

## 5.7 Troubleshooting

| Symptom | Fix |
|---|---|
| Several ROS/Gazebo projects on one PC see each other's topics | `export ROS_DOMAIN_ID=77` and `export IGN_PARTITION=robocraft` |
| `gripper goal rejected` / action timeouts | too much camera traffic: keep `showcase:=false` |
| Controllers do not come up | slow machine: increase the spawn delay in `sim.launch.py` |
| `ros2 control list_controllers` hangs | `ros2 daemon stop` |
| Camera sees nothing | check `/overhead_camera/image_raw`; Gazebo needs OpenGL/EGL, even headless |
