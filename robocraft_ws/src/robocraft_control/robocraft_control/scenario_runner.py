"""Run a RoboCraft scenario on the live system.

    ros2 run robocraft_control scenario <name> [--key value ...]
    ros2 run robocraft_control scenario parallel_square --side 0.10 --speed 0.02
    ros2 run robocraft_control scenario list

Common ROS parameters: ``grasp_mode`` (gazebo|simulated), ``speed_override``
(0..1), ``cell_config`` (path to cell.yaml).
"""
from __future__ import annotations

import sys
import threading
import traceback

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from robocraft_kinematics.geometry import load_cell

from .commander import FINGER_OPEN, CellCommander
from .ros_backend import RosBackend
from .scenarios.library import SCENARIOS


def _parse_kwargs(argv):
    kwargs, i = {}, 0
    while i < len(argv):
        key = argv[i]
        if not key.startswith("--") or i + 1 >= len(argv):
            raise SystemExit(f"bad argument '{key}', expected --key value")
        raw = argv[i + 1]
        val: object = raw
        for conv in (int, float):
            try:
                val = conv(raw)
                break
            except ValueError:
                pass
        if raw.lower() in ("true", "false"):
            val = raw.lower() == "true"
        elif "," in raw:
            val = [v for v in raw.split(",") if v]
        kwargs[key[2:].replace("-", "_")] = val
        i += 2
    return kwargs


def main(argv=None) -> int:
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    if not argv or argv[0] in ("-h", "--help", "list"):
        print("available scenarios:\n  " + "\n  ".join(sorted(SCENARIOS)))
        print(__doc__)
        return 0
    name, kwargs = argv[0], _parse_kwargs(argv[1:])
    if name not in SCENARIOS:
        print(f"unknown scenario '{name}'. Available: {', '.join(sorted(SCENARIOS))}")
        return 2

    rclpy.init()
    probe = rclpy.create_node("robocraft_param_probe")
    probe.declare_parameter("cell_config", "")
    cfg = probe.get_parameter("cell_config").value or None
    probe.destroy_node()

    cell = load_cell(cfg)
    backend = RosBackend(cell)
    backend.declare_parameter("speed_override", cell.safety.speed_override)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(backend)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    rc = 0
    try:
        backend.wait_ready()
        cmd = CellCommander(cell, backend, override=float(backend.get_parameter("speed_override").value))
        backend.commander = cmd
        backend.status(f"scenario '{name}' starting {kwargs or ''}")
        backend.set_laser(False)
        for a in cell.arm_names:
            backend.release(a)
            backend.gripper(a, FINGER_OPEN)
        cmd.home()
        SCENARIOS[name](cmd, **kwargs)
        backend.status(f"scenario '{name}' finished")
    except KeyboardInterrupt:
        backend.get_logger().warn("interrupted")
        rc = 130
    except Exception as e:  # noqa: BLE001
        backend.get_logger().error(f"scenario '{name}' aborted: {e}")
        traceback.print_exc()
        rc = 1
    finally:
        executor.shutdown()
        backend.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
