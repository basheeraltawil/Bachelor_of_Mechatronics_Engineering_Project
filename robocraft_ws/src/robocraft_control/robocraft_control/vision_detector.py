"""Colour-based object detection with the overhead camera (thesis Section 5.5).

The thesis detected a green obstacle with OpenCV. This node keeps that
pipeline and generalises it to several colour classes, so the same camera can
find obstacles *and* parts to be sorted:

    frame -> HSV -> inRange(low, high)          thesis: "dusuk" / "yuksek" limits
          -> medianBlur                         thesis: "median" (15 px on the thesis camera)
          -> contours -> size filter            (added)
          -> pixel ray  x  plate plane          (added: CameraInfo + TF -> metres)

Outputs
    /robocraft/detections   ObstacleArray, every class (label = class name)
    /robocraft/obstacles    ObstacleArray, only classes marked as obstacle
    /robocraft/vision/debug annotated image (contours + labels)

Adding a colour means adding one entry to ``CLASSES``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from robocraft_interfaces.msg import Obstacle, ObstacleArray


@dataclass(frozen=True)
class ColorClass:
    """One thing the camera should find."""

    hsv_ranges: Sequence[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]  # OpenCV H 0..180, S/V 0..255
    height: float            # height of the object's top surface above the floor [m]
    min_area: int            # blob size window in pixels: rejects small look-alikes
    max_area: int
    obstacle: bool = False   # also publish on /robocraft/obstacles
    bgr: Tuple[int, int, int] = (255, 255, 255)  # colour used in the debug image


# Ranges measured on the rendered Gazebo scene (hue histogram of a camera frame):
#   parts: red H 0-4, yellow H 30-34, blue H 105-114, all with S > 110
#   look-alikes: gold columns H 15-24 (excluded by hue), blue potentiometer caps
#   on the arms ~200 px (excluded by the 300 px minimum; a cube is ~450 px).
CLASSES: Dict[str, ColorClass] = {
    "green": ColorClass([((45, 80, 50), (80, 255, 255))], 0.018, 150, 20000, obstacle=True, bgr=(0, 200, 0)),
    "red": ColorClass([((0, 110, 70), (8, 255, 255)), ((172, 110, 70), (180, 255, 255))], 0.050, 300, 2500,
                      bgr=(0, 0, 255)),
    "blue": ColorClass([((100, 100, 50), (125, 255, 255))], 0.050, 300, 2500, bgr=(255, 80, 0)),
    "yellow": ColorClass([((28, 110, 80), (36, 255, 255))], 0.050, 300, 2500, bgr=(0, 220, 255)),
}


def find_blobs(bgr: np.ndarray, cls: ColorClass, median_ksize: int = 5) -> List[Tuple[float, float, float, float]]:
    """Blobs of one colour class: list of (u, v, radius_px, area_px). Pure OpenCV.

    The median filter removes pixel noise but also rounds the corners of the
    blob, so half the kernel size is added back to the radius. Obstacles are
    therefore never reported smaller than they are. The kernel must be small
    compared with the object: here a 50 mm block is only ~28 px wide.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in cls.hsv_ranges:
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    mask = cv2.medianBlur(mask, median_ksize)          # thesis: median filter against salt-and-pepper noise
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if cls.min_area <= area <= cls.max_area:
            (u, v), r = cv2.minEnclosingCircle(c)
            blobs.append((u, v, r + median_ksize // 2, area))
    return blobs


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class VisionDetector(Node):
    def __init__(self):
        super().__init__("robocraft_vision")
        self.declare_parameter("image_topic", "/overhead_camera/image_raw")
        self.declare_parameter("info_topic", "/overhead_camera/camera_info")
        self.declare_parameter("camera_frame", "overhead_camera_optical")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("median_ksize", 5)
        self.bridge = CvBridge()
        self.K_inv = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value, self._on_info, 10)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._on_image, 5)
        self.pub_all = self.create_publisher(ObstacleArray, "/robocraft/detections", 10)
        self.pub_obstacles = self.create_publisher(ObstacleArray, "/robocraft/obstacles", 10)
        self.pub_debug = self.create_publisher(Image, "/robocraft/vision/debug", 5)
        self._warned = False

    def _on_info(self, msg: CameraInfo) -> None:
        self.K_inv = np.linalg.inv(np.array(msg.k, dtype=float).reshape(3, 3))

    def _camera_pose(self):
        """Rotation and position of the camera optical frame in the world, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("world_frame").value,
                                                 self.get_parameter("camera_frame").value, Time())
        except Exception as e:  # noqa: BLE001 - TF not ready yet
            if not self._warned:
                self.get_logger().warn(f"waiting for camera TF: {e}")
                self._warned = True
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        return quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def _pixel_to_world(self, u: float, v: float, R: np.ndarray, t: np.ndarray, z: float):
        """Intersect the camera ray through pixel (u, v) with the plane at height z."""
        ray = R @ (self.K_inv @ np.array([u, v, 1.0]))
        if abs(ray[2]) < 1e-9:
            return None
        return t + ((z - t[2]) / ray[2]) * ray

    def _on_image(self, msg: Image) -> None:
        if self.K_inv is None:
            return
        pose = self._camera_pose()
        if pose is None:
            return
        R, t = pose
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        debug = bgr.copy()
        out_all, out_obs = ObstacleArray(), ObstacleArray()
        for arr in (out_all, out_obs):
            arr.header.stamp = msg.header.stamp
            arr.header.frame_id = self.get_parameter("world_frame").value

        for label, cls in CLASSES.items():
            for u, v, r, area in find_blobs(bgr, cls, int(self.get_parameter("median_ksize").value)):
                c = self._pixel_to_world(u, v, R, t, cls.height)
                edge = self._pixel_to_world(u + r, v, R, t, cls.height)
                if c is None or edge is None:
                    continue
                det = Obstacle(label=label, x=float(c[0]), y=float(c[1]),
                               radius=float(math.hypot(edge[0] - c[0], edge[1] - c[1])),
                               confidence=float(min(1.0, area / (math.pi * r * r + 1e-9))))
                out_all.obstacles.append(det)
                if cls.obstacle:
                    out_obs.obstacles.append(det)
                cv2.circle(debug, (int(u), int(v)), int(r) + 3, cls.bgr, 2)
                cv2.putText(debug, label, (int(u + r + 4), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, cls.bgr, 1)

        self.pub_all.publish(out_all)
        self.pub_obstacles.publish(out_obs)
        dbg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        dbg.header = msg.header
        self.pub_debug.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
