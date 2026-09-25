# robocraft_bringup

**How to start it:** launch files and controller configuration.

| Launch | Starts |
|---|---|
| `sim.launch.py` | Gazebo + ros2_control + camera bridge + vision + RViz (`world:=cell\|obstacle`, `headless:=true`, `scenario:=...`) |
| `mock.launch.py` | ros2_control with ideal mock joints + RViz (no physics) |
| `real.launch.py` | ros2_control with the I²C hardware driver (Raspberry Pi) |

| Config | Content |
|---|---|
| `config/controllers.yaml` | trajectory + gripper controllers, position interface (mock / real) |
| `config/controllers_gazebo.yaml` | same, velocity interface with P + feed-forward (Gazebo) |
| `config/generate_controllers.py` | generates both files: edit this one |
| `config/gz_bridge.yaml` | topics bridged between Gazebo and ROS 2 |
