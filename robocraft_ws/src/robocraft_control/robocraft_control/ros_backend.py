"""ROS 2 backend: ros2_control trajectory/gripper actions + Gazebo grasp plugin."""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Mapping, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from robocraft_interfaces.msg import CellStatus, ObstacleArray
from robocraft_kinematics.geometry import CellGeometry
from robocraft_kinematics.parallel import PlatformPose
from robocraft_kinematics.trajectory import JointTrajectory
from robocraft_kinematics.via_point import CircleObstacle

from .backend import Backend, ExecutionError


def _duration(t: float) -> Duration:
    sec = int(math.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _wait(fut, timeout: float) -> bool:
    ev = threading.Event()
    fut.add_done_callback(lambda _f: ev.set())
    return ev.wait(timeout)


class RosBackend(Node, Backend):
    """Node that executes commander plans on ros2_control (mock, Gazebo or real)."""

    def __init__(self, cell: CellGeometry):
        Node.__init__(self, "robocraft_cell")
        self.cell = cell
        self.declare_parameter("grasp_mode", "gazebo")     # gazebo | simulated
        self.declare_parameter("start_delay", 0.25)        # common start offset for sync goals [s]
        self.grasp_mode = self.get_parameter("grasp_mode").value
        self.start_delay = float(self.get_parameter("start_delay").value)
        cb = ReentrantCallbackGroup()

        self._js: Dict[str, float] = {}
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_js, 50, callback_group=cb)

        self._traj_ac = {a: ActionClient(self, FollowJointTrajectory, f"/{a}_controller/follow_joint_trajectory",
                                         callback_group=cb) for a in cell.arm_names}
        self._grip_ac = {a: ActionClient(self, GripperCommand, f"/{a}_gripper_controller/gripper_cmd",
                                         callback_group=cb) for a in cell.arm_names}
        self._grasp_pub = {a: self.create_publisher(String, f"/robocraft/{a}/grasp_cmd", 10) for a in cell.arm_names}
        self._grasp_state: Dict[str, Tuple[float, str]] = {}
        for a in cell.arm_names:
            self.create_subscription(String, f"/robocraft/{a}/grasp_state",
                                     lambda m, a=a: self._grasp_state.__setitem__(a, (time.monotonic(), m.data)),
                                     10, callback_group=cb)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._laser_pub = self.create_publisher(Bool, "/robocraft/laser", latched)
        self._pose_pub = self.create_publisher(PoseStamped, "/robocraft/platform/pose_est", 10)
        self._status_pub = self.create_publisher(CellStatus, "/robocraft/status", latched)
        self._gt: Optional[PlatformPose] = None
        self.create_subscription(TFMessage, "/robocraft/world_poses", self._on_world_poses, 10, callback_group=cb)
        self._obstacles: Tuple[float, List[CircleObstacle]] = (0.0, [])
        self.create_subscription(ObstacleArray, "/robocraft/obstacles", self._on_obstacles, 10, callback_group=cb)
        self.commander = None  # set by the scenario runner (for status reporting)
        self._held: Dict[str, str] = {a: "idle" for a in cell.arm_names}

    # ------------------------------------------------------------------ callbacks
    def _on_js(self, msg: JointState) -> None:
        with self._js_lock:
            for n, p in zip(msg.name, msg.position):
                self._js[n] = p

    def _on_world_poses(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id == "hex_platform":
                t, o = tf.transform.translation, tf.transform.rotation
                self._gt = PlatformPose(t.x, t.y, quat_to_yaw(o.x, o.y, o.z, o.w), t.z)
                return

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        self._obstacles = (time.monotonic(), [CircleObstacle(o.x, o.y, o.radius) for o in msg.obstacles])

    # ------------------------------------------------------------------ startup
    def wait_ready(self, timeout: float = 60.0) -> None:
        deadline = time.monotonic() + timeout
        names = [n for a in self.cell.arm_names for n in self.joint_names(a)]
        while time.monotonic() < deadline:
            with self._js_lock:
                if all(n in self._js for n in names):
                    break
            time.sleep(0.1)
        else:
            raise ExecutionError("no /joint_states for all arms (are the controllers running?)")
        for ac in list(self._traj_ac.values()) + list(self._grip_ac.values()):
            if not ac.wait_for_server(timeout_sec=max(1.0, deadline - time.monotonic())):
                raise ExecutionError(f"action server {ac._action_name} not available")
        self.get_logger().info("RoboCraft cell ready")

    # ------------------------------------------------------------------ Backend API
    def get_q(self, arm: str) -> np.ndarray:
        with self._js_lock:
            return np.array([self._js[n] for n in self.joint_names(arm)])

    def _goal(self, traj: JointTrajectory, stamp) -> FollowJointTrajectory.Goal:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(traj.joint_names)
        g.trajectory.header.stamp = stamp
        for t, q, qd in zip(traj.times, traj.positions, traj.velocities):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [float(v) for v in qd]
            pt.time_from_start = _duration(t)
            g.trajectory.points.append(pt)
        return g

    def execute(self, plans: Mapping[str, JointTrajectory]) -> None:
        self.execute_with_platform(plans, None)

    def execute_with_platform(self, plans: Mapping[str, JointTrajectory],
                              platform_path: Optional[List[Tuple[float, PlatformPose]]]) -> None:
        # one common start stamp -> the arms of a parallel move start in the same control cycle
        stamp = (self.get_clock().now() + rclpy.duration.Duration(seconds=self.start_delay)).to_msg()
        handles = {}
        for arm, traj in plans.items():
            fut = self._traj_ac[arm].send_goal_async(self._goal(traj, stamp))
            if not _wait(fut, 15.0) or not fut.result().accepted:
                raise ExecutionError(f"{arm}: trajectory goal rejected")
            handles[arm] = fut.result()
        if platform_path:
            threading.Thread(target=self._stream_platform, args=(platform_path,), daemon=True).start()
        T = max(t.duration for t in plans.values())
        for arm, h in handles.items():
            rf = h.get_result_async()
            if not _wait(rf, T * 3 + 10.0):
                raise ExecutionError(f"{arm}: trajectory timed out")
            res = rf.result().result
            if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise ExecutionError(f"{arm}: trajectory failed ({res.error_code}) {res.error_string}")

    def _stream_platform(self, path: List[Tuple[float, PlatformPose]]) -> None:
        t0 = time.monotonic() + self.start_delay
        for t, p in path:
            dt = t0 + t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            self.publish_platform(p)

    def gripper(self, arm: str, position: float) -> None:
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = 20.0
        fut = self._grip_ac[arm].send_goal_async(g)
        if not _wait(fut, 15.0):
            raise ExecutionError(f"{arm}: no answer from the gripper controller")
        if not fut.result().accepted:
            raise ExecutionError(f"{arm}: gripper goal rejected")
        _wait(fut.result().get_result_async(), 10.0)

    def grasp(self, arm: str, locked: bool = False) -> bool:
        if self.grasp_mode != "gazebo":
            self._held[arm] = "holding:simulated" + (":locked" if locked else "")
            return True
        t_cmd = time.monotonic()
        self._grasp_pub[arm].publish(String(data="grasp_locked" if locked else "grasp"))
        deadline = t_cmd + 3.0
        while time.monotonic() < deadline:
            ts, st = self._grasp_state.get(arm, (0.0, ""))
            if ts > t_cmd and (st.startswith("holding") or st.startswith("failed")):
                self._held[arm] = st
                ok = st.startswith("holding")
                (self.get_logger().info if ok else self.get_logger().error)(f"{arm}: {st}")
                return ok
            time.sleep(0.02)
        self.get_logger().error(f"{arm}: no answer from grasp plugin")
        return False

    def release(self, arm: str) -> None:
        self._held[arm] = "idle"
        if self.grasp_mode == "gazebo":
            self._grasp_pub[arm].publish(String(data="release"))
            time.sleep(0.1)

    def set_laser(self, on: bool) -> None:
        self._laser_pub.publish(Bool(data=on))

    def publish_platform(self, p: PlatformPose) -> None:
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y = p.x, p.y
        # model origin = underside of the plate: 10 mm base plate + current lift
        m.pose.position.z = 0.010 + max(0.0, p.z - self.cell.platform.grip_height)
        x, y, z, w = yaw_to_quat(p.yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = x, y, z, w
        self._pose_pub.publish(m)

    def status(self, text: str) -> None:
        self.get_logger().info(text)
        m = CellStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.arms = list(self.cell.arm_names)
        m.grasp_states = [self._held[a] for a in self.cell.arm_names]
        m.active_task = text
        if self.commander is not None:
            c = self.commander
            m.mode = c.mode if c.grips else "serial"
            m.min_interference_margin = float(c.last_margin) if math.isfinite(c.last_margin) else -1.0
            if len(c.grips) >= 2:
                from robocraft_kinematics.parallel import closure_error
                try:
                    m.closure_error = closure_error(self.cell, {a: self.get_q(a) for a in c.grips}, c.grips)
                except Exception:  # noqa: BLE001
                    m.closure_error = -1.0
        self._status_pub.publish(m)

    def platform_ground_truth(self) -> Optional[PlatformPose]:
        deadline = time.monotonic() + (2.0 if self.grasp_mode == "gazebo" else 0.0)
        while self._gt is None and time.monotonic() < deadline:
            time.sleep(0.05)
        return self._gt

    def obstacles(self, timeout: float = 0.0) -> List[CircleObstacle]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self._obstacles[0] == 0.0:
            time.sleep(0.1)
        return list(self._obstacles[1])

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)
