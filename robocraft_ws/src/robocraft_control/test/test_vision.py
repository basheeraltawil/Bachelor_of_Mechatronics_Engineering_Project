"""The colour classes find the right parts and ignore look-alikes."""
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from robocraft_control.vision_detector import CLASSES, find_blobs  # noqa: E402


def scene():
    img = np.full((400, 400, 3), 235, np.uint8)                        # white plate
    cv2.rectangle(img, (180, 250), (230, 300), (30, 200, 30), -1)       # green obstacle
    cv2.rectangle(img, (40, 40), (64, 64), (25, 25, 215), -1)           # red part (24 px)
    cv2.rectangle(img, (300, 60), (324, 84), (215, 50, 25), -1)         # blue part
    cv2.rectangle(img, (60, 300), (84, 324), (40, 215, 215), -1)        # yellow part
    cv2.rectangle(img, (300, 300), (360, 380), (76, 148, 184), -1)      # gold column top: must be ignored
    cv2.circle(img, (200, 60), 7, (240, 80, 30), -1)                    # small blue pot cap: ignored
    return img


@pytest.mark.parametrize("label,centre", [("green", (205, 275)), ("red", (52, 52)),
                                          ("blue", (312, 72)), ("yellow", (72, 312))])
def test_each_class_found_once(label, centre):
    blobs = find_blobs(scene(), CLASSES[label])
    assert len(blobs) == 1, f"{label}: {blobs}"
    u, v, _, _ = blobs[0]
    assert abs(u - centre[0]) < 3 and abs(v - centre[1]) < 3


def test_obstacle_size_is_not_underestimated():
    """The planner keeps its margin from this radius, so it must cover the whole object."""
    img = np.full((400, 400, 3), 235, np.uint8)
    cv2.rectangle(img, (180, 180), (220, 220), (30, 200, 30), -1)      # 41 x 41 px green square
    (_, _, r, _), = find_blobs(img, CLASSES["green"])
    assert r >= 40 / 2 * 2 ** 0.5          # >= half diagonal between pixel centres (28.3 px)
