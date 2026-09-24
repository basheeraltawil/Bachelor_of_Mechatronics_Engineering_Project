"""RoboCraft cell with ros2_control mock hardware (no physics) + RViz.

Fast way to try every scenario without Gazebo:
    ros2 launch robocraft_bringup mock.launch.py
    ros2 run robocraft_control scenario pure_rotation --ros-args -p grasp_mode:=simulated

hardware:=real uses the Raspberry Pi -> ATmega I2C hardware interface instead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARMS = ("arm1", "arm2", "arm3")


def generate_launch_description():
    desc = get_package_share_directory("robocraft_description")
    controllers = os.path.join(get_package_share_directory("robocraft_bringup"), "config", "controllers.yaml")
    hardware = LaunchConfiguration("hardware")
    scenario = LaunchConfiguration("scenario")

    args = [
        DeclareLaunchArgument("hardware", default_value="mock", description="mock | real"),
        DeclareLaunchArgument("i2c_device", default_value="/dev/i2c-1"),
        DeclareLaunchArgument("i2c_addresses", default_value="8,9,10"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scenario", default_value=""),
        DeclareLaunchArgument("scenario_args", default_value=""),
        DeclareLaunchArgument("speed_override", default_value="0.6"),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("robocraft_description"), "urdf", "robocraft_cell.urdf.xacro"]),
        " hardware:=", hardware,
        " i2c_device:=", LaunchConfiguration("i2c_device"),
        " i2c_addresses:=", LaunchConfiguration("i2c_addresses")]), value_type=str)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description}])
    cm = Node(package="controller_manager", executable="ros2_control_node", output="screen",
              parameters=[{"robot_description": robot_description}, controllers])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager"])

    ctrl = [spawner("joint_state_broadcaster")] + \
           [spawner(f"{a}_controller") for a in ARMS] + [spawner(f"{a}_gripper_controller") for a in ARMS]

    visualizer = Node(package="robocraft_control", executable="cell_visualizer", output="screen",
                      parameters=[{"pose_topic": "/robocraft/platform/pose_est"}])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", os.path.join(desc, "rviz", "robocraft.rviz")],
                condition=IfCondition(LaunchConfiguration("rviz")))

    grasp_mode = PythonExpression(["'simulated'"])
    run_scenario = TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=[PythonExpression(["'ros2 run robocraft_control scenario ' + '", scenario, "' + ' ' + '",
                               LaunchConfiguration("scenario_args"),
                               "' + ' --ros-args -p grasp_mode:=", grasp_mode, " -p speed_override:=",
                               LaunchConfiguration("speed_override"), "'"])],
        shell=True, output="screen")],
        condition=UnlessCondition(PythonExpression(["'", scenario, "' == ''"])))

    return LaunchDescription(args + [rsp, cm, *ctrl, visualizer, rviz, run_scenario])
