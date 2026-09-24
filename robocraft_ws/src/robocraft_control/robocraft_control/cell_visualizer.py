"""RViz visualisation of the hex platform, the laser trail and detected obstacles.

Subscribes to the platform pose (``pose_topic``: ground truth from Gazebo or
the commander's estimate in mock mode) and publishes

* ``/robocraft/markers`` (MarkerArray): platform, legs, laser trail, obstacles,
  dexterous-workspace outline
* TF ``world -> hex_platform``
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from robocraft_interfaces.msg import ObstacleArray
from robocraft_kinematics.geometry import load_cell


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class CellVisualizer(Node):
    def __init__(self):
        super().__init__("robocraft_visualizer")
        # /robocraft/world_poses (Gazebo ground truth, TFMessage) or
        # /robocraft/platform/pose_est (commander estimate, PoseStamped; mock / real hardware)
        self.declare_parameter("pose_topic", "/robocraft/world_poses")
        self.declare_parameter("cell_config", "")
        self.cell = load_cell(self.get_parameter("cell_config").value or None)
        topic = self.get_parameter("pose_topic").value
        if topic.endswith("world_poses"):
            self.create_subscription(TFMessage, topic, self._on_world_poses, 10)
        else:
            self.create_subscription(PoseStamped, topic, lambda m: self._on_pose(m.pose), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/robocraft/laser", self._on_laser, latched)
        self.create_subscription(ObstacleArray, "/robocraft/obstacles", self._on_obstacles, 10)
        self.pub = self.create_publisher(MarkerArray, "/robocraft/markers", 10)
        self.tf = TransformBroadcaster(self)
        self.laser = False
        self.trail = []
        self.pose = None
        p = self.cell.platform
        yaw = p.initial_yaw
        self._on_pose_xyz(0.0, 0.0, 0.010, yaw)   # sensible default until a pose arrives
        self.create_timer(0.1, self._publish)

    # ------------------------------------------------------------------
    def _on_laser(self, msg: Bool) -> None:
        if msg.data and not self.laser:
            self.trail.append(None)  # pen-up separator between strokes
        self.laser = msg.data

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        self.obstacles = msg.obstacles

    def _on_world_poses(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id == "hex_platform":
                t, o = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
                self._on_pose_xyz(t.x, t.y, t.z, yaw)
                return

    def _on_pose(self, pose: Pose) -> None:
        o = pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        self._on_pose_xyz(pose.position.x, pose.position.y, pose.position.z, yaw)

    def _on_pose_xyz(self, x, y, z, yaw) -> None:
        self.pose = (x, y, z, yaw)
        if self.laser:
            if not self.trail or self.trail[-1] is None or math.dist(self.trail[-1], (x, y)) > 0.0005:
                self.trail.append((x, y))

    # ------------------------------------------------------------------
    def _publish(self) -> None:
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "world", "hex_platform"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)

        arr = MarkerArray()
        p = self.cell.platform

        def mk(ns, mid, mtype, frame="hex_platform"):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, now
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        hexm = mk("platform", 0, Marker.LINE_STRIP)
        hexm.scale.x = 0.006
        hexm.color = _color(0.2, 0.45, 0.85)
        for k in range(7):
            a = k * math.pi / 3
            hexm.points.append(Point(x=p.plate_radius * math.cos(a), y=p.plate_radius * math.sin(a), z=0.010))
        arr.markers.append(hexm)
        for k in range(6):
            leg = mk("legs", k, Marker.CYLINDER)
            a = k * math.pi / 3
            leg.pose.position.x, leg.pose.position.y, leg.pose.position.z = \
                p.leg_radius * math.cos(a), p.leg_radius * math.sin(a), 0.050
            leg.scale.x = leg.scale.y = 0.012
            leg.scale.z = 0.080
            leg.color = _color(0.5, 0.5, 0.55)
            arr.markers.append(leg)
        head = mk("platform", 1, Marker.CYLINDER)
        head.pose.position.z = 0.080
        head.scale.x = head.scale.y = 0.02
        head.scale.z = 0.03
        head.color = _color(1.0, 0.1, 0.1) if self.laser else _color(0.5, 0.1, 0.1)
        arr.markers.append(head)

        trail = mk("laser_trail", 0, Marker.LINE_LIST, frame="world")
        trail.scale.x = 0.003
        trail.color = _color(1.0, 0.15, 0.0)
        for a, b in zip(self.trail[:-1], self.trail[1:]):
            if a is None or b is None:
                continue
            trail.points += [Point(x=a[0], y=a[1], z=0.0115), Point(x=b[0], y=b[1], z=0.0115)]
        arr.markers.append(trail)

        for i, ob in enumerate(getattr(self, "obstacles", [])):
            c = mk("obstacles", i, Marker.CYLINDER, frame="world")
            c.pose.position.x, c.pose.position.y, c.pose.position.z = ob.x, ob.y, 0.015
            c.scale.x = c.scale.y = 2 * ob.radius
            c.scale.z = 0.01
            c.color = _color(0.1, 0.9, 0.1, 0.6)
            arr.markers.append(c)

        ring = mk("plate", 0, Marker.LINE_STRIP, frame="world")
        ring.scale.x = 0.003
        ring.color = _color(0.2, 0.2, 0.2, 0.6)
        for k in range(73):
            a = k * 2 * math.pi / 72
            ring.points.append(Point(x=self.cell.plate_radius * math.cos(a), y=self.cell.plate_radius * math.sin(a), z=0.011))
        arr.markers.append(ring)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CellVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
