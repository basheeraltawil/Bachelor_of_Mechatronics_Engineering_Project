import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from robocraft_control.obstacle_detector import detect_green  # noqa: E402


def test_detects_green_square_on_white_plate():
    img = np.full((400, 400, 3), 235, np.uint8)
    cv2.rectangle(img, (180, 250), (230, 300), (30, 200, 30), -1)     # green (BGR)
    cv2.circle(img, (100, 100), 20, (30, 30, 200), -1)                # red part: ignored
    blobs, _, _ = detect_green(img, [45, 80, 50], [80, 255, 255])
    assert len(blobs) == 1
    u, v, r, _ = blobs[0]
    assert abs(u - 205) < 3 and abs(v - 275) < 3 and 30 < r < 40
