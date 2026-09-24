"""Green-obstacle detection with the overhead camera (thesis Section 5.5).

Pipeline (thesis steps kept, names translated from the original Turkish code):
    frame -> HSV (cvtColor) -> inRange(low, high)  ("dusuk", "yuksek")
          -> bitwise_and -> medianBlur(15)         ("son_resim", "median")
          -> goodFeaturesToTrack + Canny           ("koseler", "kenarlar")   [debug image]
Added for the ROS version:
          -> contours -> minEnclosingCircle -> ray/plane intersection with the
             plate (camera intrinsics from CameraInfo + TF) -> world x, y, r

Publishes ``/robocraft/obstacles`` (robocraft_interfaces/ObstacleArray) and a
debug image ``/robocraft/vision/debug``.
"""
from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from robocraft_interfaces.msg import Obstacle, ObstacleArray


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def detect_green(bgr: np.ndarray, low, high, median_ksize: int = 15, min_area: float = 80.0):
    """Return (blobs[(u, v, r_px, area)], mask, debug_bgr) — pure OpenCV, testable."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    son_resim = cv2.bitwise_and(bgr, bgr, mask=mask)
    median = cv2.medianBlur(son_resim, median_ksize)
    gri = cv2.cvtColor(median, cv2.COLOR_BGR2GRAY)
    debug = bgr.copy()
    koseler = cv2.goodFeaturesToTrack(np.float32(gri), 50, 0.01, 5)
    if koseler is not None:
        for kose in koseler.astype(int):
            cx, cy = kose.ravel()
            cv2.circle(debug, (int(cx), int(cy)), 3, (0, 0, 255), -1)
    kenarlar = cv2.Canny(median, 50, 100)
    debug[kenarlar > 0] = (255, 0, 255)
    _, bin_mask = cv2.threshold(gri, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        (u, v), r = cv2.minEnclosingCircle(c)
        blobs.append((u, v, r, area))
        cv2.circle(debug, (int(u), int(v)), int(r), (0, 255, 0), 2)
    return blobs, bin_mask, debug


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("robocraft_obstacle_detector")
        # OpenCV HSV ranges (H 0..180, S/V 0..255). The thesis used H 60..70.
        self.declare_parameter("hsv_low", [45, 80, 50])
        self.declare_parameter("hsv_high", [80, 255, 255])
        self.declare_parameter("median_ksize", 15)
        self.declare_parameter("min_area_px", 80.0)
        self.declare_parameter("plane_z", 0.018)          # height of the obstacle top [m]
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("image_topic", "/overhead_camera/image_raw")
        self.declare_parameter("info_topic", "/overhead_camera/camera_info")
        self.declare_parameter("camera_frame", "overhead_camera_optical")
        self.bridge = CvBridge()
        self.K = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value, self._on_info, 10)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self._on_image, 5)
        self.pub = self.create_publisher(ObstacleArray, "/robocraft/obstacles", 10)
        self.dbg = self.create_publisher(Image, "/robocraft/vision/debug", 5)
        self._warned = False

    def _on_info(self, msg: CameraInfo) -> None:
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _pixel_to_world(self, u: float, v: float, R: np.ndarray, t: np.ndarray, z: float):
        ray_c = np.linalg.inv(self.K) @ np.array([u, v, 1.0])
        ray_w = R @ ray_c
        if abs(ray_w[2]) < 1e-9:
            return None
        s = (z - t[2]) / ray_w[2]
        return t + s * ray_w

    def _on_image(self, msg: Image) -> None:
        if self.K is None:
            return
        cam = self.get_parameter("camera_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(self.get_parameter("world_frame").value, cam, Time())
        except Exception as e:  # noqa: BLE001
            if not self._warned:
                self.get_logger().warn(f"waiting for TF {cam}: {e}")
                self._warned = True
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.x, tr.y, tr.z])
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blobs, _, debug = detect_green(bgr, self.get_parameter("hsv_low").value,
                                       self.get_parameter("hsv_high").value,
                                       int(self.get_parameter("median_ksize").value),
                                       float(self.get_parameter("min_area_px").value))
        z = float(self.get_parameter("plane_z").value)
        out = ObstacleArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.get_parameter("world_frame").value
        for u, v, r, area in blobs:
            c = self._pixel_to_world(u, v, R, t, z)
            e = self._pixel_to_world(u + r, v, R, t, z)
            if c is None or e is None:
                continue
            ob = Obstacle(label="green", x=float(c[0]), y=float(c[1]),
                          radius=float(math.hypot(e[0] - c[0], e[1] - c[1])),
                          confidence=float(min(1.0, area / (math.pi * r * r + 1e-9))))
            out.obstacles.append(ob)
        self.pub.publish(out)
        dbg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        dbg.header = msg.header
        self.dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
