"""Real RoboCraft cell: Raspberry Pi (ros2_control) -> 3x ATmega over I2C.

    ros2 launch robocraft_bringup real.launch.py i2c_device:=/dev/i2c-1 i2c_addresses:=8,9,10

Commission the hardware first (see robocraft_hardware/README and the firmware).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mock = os.path.join(get_package_share_directory("robocraft_bringup"), "launch", "mock.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-1"),
        DeclareLaunchArgument("i2c_addresses", default_value="8,9,10"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("speed_override", default_value="0.3"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mock), launch_arguments={
            "hardware": "real",
            "i2c_device": LaunchConfiguration("i2c_device"),
            "i2c_addresses": LaunchConfiguration("i2c_addresses"),
            "rviz": LaunchConfiguration("rviz"),
            "speed_override": LaunchConfiguration("speed_override"),
        }.items()),
    ])
