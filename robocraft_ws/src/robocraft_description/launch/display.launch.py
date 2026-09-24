"""Inspect the RoboCraft model in RViz with joint sliders.

    ros2 launch robocraft_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory("robocraft_description")
    urdf = ParameterValue(Command([FindExecutable(name="xacro"), " ",
                                   os.path.join(desc, "urdf", "robocraft_cell.urdf.xacro"), " hardware:=mock"]),
                          value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "robocraft.rviz")]),
    ])
