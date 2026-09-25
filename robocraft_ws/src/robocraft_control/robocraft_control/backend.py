"""Backend interface between the cell commander and the "world".

* :class:`DryRunBackend` executes plans instantly (unit tests, offline checks)
* ``robocraft_control.ros_backend.RosBackend`` talks to ros2_control / Gazebo
"""
from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional

import numpy as np

from robocraft_kinematics.parallel import PlatformPose
from robocraft_kinematics.trajectory import JointTrajectory
from robocraft_kinematics.via_point import CircleObstacle

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4")


class ExecutionError(RuntimeError):
    """The robot could not do what was asked (rejected goal, failed grasp, ...)."""


@dataclass
class Detection:
    """An object seen by the camera, in world coordinates [m]."""

    label: str        # colour class, e.g. "red", "green"
    x: float
    y: float
    radius: float


class Backend(abc.ABC):
    """Everything the commander needs from the robot."""

    @staticmethod
    def joint_names(arm: str) -> List[str]:
        return [f"{arm}_{j}" for j in ARM_JOINTS]

    @abc.abstractmethod
    def get_q(self, arm: str) -> np.ndarray: ...

    @abc.abstractmethod
    def execute(self, plans: Mapping[str, JointTrajectory]) -> None:
        """Run all plans with a common start time; block until done."""

    @abc.abstractmethod
    def gripper(self, arm: str, position: float) -> None:
        """Move the fingers (per-finger opening in metres); block until done."""

    @abc.abstractmethod
    def grasp(self, arm: str, locked: bool = False) -> bool:
        """Close the grasp (attach object between fingers). True on success."""

    @abc.abstractmethod
    def release(self, arm: str) -> None: ...

    # ---- optional hooks ---------------------------------------------------
    def set_laser(self, on: bool) -> None:  # noqa: B027 - optional
        pass

    def publish_platform(self, pose: PlatformPose) -> None:  # noqa: B027
        pass

    def status(self, text: str) -> None:  # noqa: B027
        pass

    def platform_ground_truth(self) -> Optional[PlatformPose]:
        return None

    def obstacles(self, timeout: float = 0.0) -> List[CircleObstacle]:
        return []

    def detections(self, timeout: float = 0.0) -> List[Detection]:
        """Everything the camera currently sees (parts and obstacles)."""
        return []

    def sleep(self, seconds: float) -> None:  # noqa: B027
        pass


class DryRunBackend(Backend):
    """Kinematic stand-in: plans 'execute' instantly; records a motion log."""

    def __init__(self, home: Mapping[str, np.ndarray], obstacles: Optional[List[CircleObstacle]] = None,
                 detections: Optional[List[Detection]] = None):
        self.q: Dict[str, np.ndarray] = {a: np.asarray(q, float).copy() for a, q in home.items()}
        self.fingers: Dict[str, float] = {a: 0.022 for a in home}
        self.held: Dict[str, bool] = {a: False for a in home}
        self.log: List[str] = []
        self.plans: List[Dict[str, JointTrajectory]] = []
        self.laser_on = False
        self._obstacles = obstacles or []
        self._detections = detections or []
        self.sim_time = 0.0

    def get_q(self, arm: str) -> np.ndarray:
        return self.q[arm].copy()

    def execute(self, plans: Mapping[str, JointTrajectory]) -> None:
        self.plans.append(dict(plans))
        for arm, traj in plans.items():
            if traj.positions:
                self.q[arm] = np.asarray(traj.positions[-1], float).copy()
        self.sim_time += max((t.duration for t in plans.values()), default=0.0)
        self.log.append("execute " + ",".join(sorted(plans)))

    def gripper(self, arm: str, position: float) -> None:
        self.fingers[arm] = position
        self.log.append(f"gripper {arm} {position:.3f}")

    def grasp(self, arm: str, locked: bool = False) -> bool:
        self.held[arm] = True
        self.log.append(f"grasp {arm}{' locked' if locked else ''}")
        return True

    def release(self, arm: str) -> None:
        self.held[arm] = False
        self.log.append(f"release {arm}")

    def set_laser(self, on: bool) -> None:
        self.laser_on = on
        self.log.append(f"laser {'on' if on else 'off'}")

    def obstacles(self, timeout: float = 0.0) -> List[CircleObstacle]:
        return list(self._obstacles)

    def detections(self, timeout: float = 0.0) -> List[Detection]:
        return list(self._detections)
