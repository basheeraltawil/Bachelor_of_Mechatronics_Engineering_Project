# robocraft_gz_plugins

Gazebo Fortress system plugin `RoboCraftGraspPlugin` (C++).

Friction grasping is unreliable in physics engines, so a grasp is modelled as a fixed joint,
**but only if an object is really between the fingers** (within 20 mm of the finger centre),
like a part-present sensor on an industrial gripper.

| Command (`/robocraft/armN/grasp_cmd`) | Effect |
|---|---|
| `grasp` | attach the nearest link named `grasp_*` |
| `grasp_locked` | attach to the platform body instead of the leg = brake engaged |
| `release` | remove the joint |

The state is published on `/robocraft/armN/grasp_state` (`idle`, `holding:<model>/<link>`, `failed:...`).
