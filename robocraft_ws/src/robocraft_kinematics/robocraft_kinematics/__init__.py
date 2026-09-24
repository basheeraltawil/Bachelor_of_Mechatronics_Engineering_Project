"""RoboCraft kinematics: pure-Python (numpy) library, no ROS dependency.

Serial SCARA kinematics, parallel platform kinematics, thesis trajectory
generation, interference checking, workspace analysis and the I2C protocol.
"""
from .geometry import ArmGeometry, CellGeometry, JointLimit, PlatformGeometry, build_cell, load_cell, wrap_angle
from .parallel import PlatformPose, closure_error, leg_world_xy, nearest_leg, platform_fk, platform_ik
from .scara import IKError, TcpPose, elbow_sign, fk, fk_planar, ik, ik_planar_solutions, jacobian, manipulability

__all__ = [
    "ArmGeometry", "CellGeometry", "JointLimit", "PlatformGeometry", "build_cell", "load_cell", "wrap_angle",
    "PlatformPose", "closure_error", "leg_world_xy", "nearest_leg", "platform_fk", "platform_ik",
    "IKError", "TcpPose", "elbow_sign", "fk", "fk_planar", "ik", "ik_planar_solutions", "jacobian",
    "manipulability",
]
