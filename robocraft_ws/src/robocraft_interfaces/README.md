# robocraft_interfaces

| Message | Fields |
|---|---|
| `Obstacle` | `label` (colour class), `x`, `y`, `radius` [m], `confidence` |
| `ObstacleArray` | header + list of `Obstacle` (used for `/robocraft/obstacles` and `/robocraft/detections`) |
| `CellStatus` | mode, grasp state per arm, closure error, smallest interference margin, active task |
