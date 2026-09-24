"""RoboCraft cell in Gazebo Fortress.

    ros2 launch robocraft_bringup sim.launch.py                       # GUI + RViz
    ros2 launch robocraft_bringup sim.launch.py world:=obstacle       # with green obstacle + vision
    ros2 launch robocraft_bringup sim.launch.py headless:=true rviz:=false
    ros2 launch robocraft_bringup sim.launch.py scenario:=parallel_square
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARMS = ("arm1", "arm2", "arm3")


def generate_launch_description():
    desc = get_package_share_directory("robocraft_description")
    bringup = get_package_share_directory("robocraft_bringup")
    gz_plugins_lib = os.path.join(get_package_share_directory("robocraft_gz_plugins"), "..", "..", "lib")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    scenario = LaunchConfiguration("scenario")
    override = LaunchConfiguration("speed_override")
    vision = LaunchConfiguration("vision")

    args = [
        DeclareLaunchArgument("world", default_value="cell", description="cell | obstacle"),
        DeclareLaunchArgument("headless", default_value="false", description="run Gazebo without GUI"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true", description="start the green-obstacle detector"),
        DeclareLaunchArgument("scenario", default_value="", description="scenario to start automatically"),
        DeclareLaunchArgument("scenario_args", default_value="", description="e.g. '--side 0.1 --speed 0.02'"),
        DeclareLaunchArgument("speed_override", default_value="0.6", description="industrial override 0..1"),
        DeclareLaunchArgument("showcase", default_value="false",
                              description="bridge the oblique documentation camera (/showcase_camera/image_raw)"),
    ]

    controllers = os.path.join(bringup, "config", "controllers_gazebo.yaml")
    world_file = PythonExpression([
        "'", os.path.join(desc, "worlds", "robocraft_cell"), "' + ('_obstacle' if '", world,
        "' == 'obstacle' else '') + '.sdf'"])

    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("robocraft_description"), "urdf", "robocraft_cell.urdf.xacro"]),
        " hardware:=gazebo controllers_file:=", controllers]), value_type=str)

    env = [
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH",
                               os.path.join(desc, "models") + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                               os.path.abspath(gz_plugins_lib) + ":" + os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")),
    ]

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": PythonExpression(["'-r -v 2 ' + ('-s ' if '", headless, "' == 'true' else '') + '",
                                                       world_file, "'"])}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "robocraft_cell", "-topic", "robot_description", "-z", "0.0"])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  parameters=[{"config_file": os.path.join(bringup, "config", "gz_bridge.yaml"),
                               "use_sim_time": True}])

    def spawner(name, extra=()):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager", "/controller_manager",
                               "--controller-manager-timeout", "60", *extra])

    ctrl = [spawner("joint_state_broadcaster")] + \
           [spawner(f"{a}_controller") for a in ARMS] + [spawner(f"{a}_gripper_controller") for a in ARMS]
    start_controllers = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=ctrl))

    showcase_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="showcase_bridge",
                           arguments=["/showcase_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"],
                           remappings=[("/showcase_camera/image", "/showcase_camera/image_raw")],
                           condition=IfCondition(LaunchConfiguration("showcase")))

    visualizer = Node(package="robocraft_control", executable="cell_visualizer", output="screen",
                      parameters=[{"use_sim_time": True, "pose_topic": "/robocraft/world_poses"}])
    detector = Node(package="robocraft_control", executable="obstacle_detector", output="screen",
                    parameters=[{"use_sim_time": True}], condition=IfCondition(vision))
    rviz_node = Node(package="rviz2", executable="rviz2", output="log",
                     arguments=["-d", os.path.join(desc, "rviz", "robocraft.rviz")],
                     parameters=[{"use_sim_time": True}], condition=IfCondition(rviz))

    run_scenario = TimerAction(period=12.0, actions=[ExecuteProcess(
        cmd=[PythonExpression(["'ros2 run robocraft_control scenario ' + '", scenario, "' + ' ' + '",
                               LaunchConfiguration("scenario_args"),
                               "' + ' --ros-args -p use_sim_time:=true -p grasp_mode:=gazebo -p speed_override:=",
                               override, "'"])],
        shell=True, output="screen")],
        condition=UnlessCondition(PythonExpression(["'", scenario, "' == ''"])))

    # give robot_state_publisher time to come up: gz_ros2_control fetches the
    # URDF from it and can hang if it asks too early
    spawn_later = TimerAction(period=3.0, actions=[spawn])

    return LaunchDescription(args + env + [gz, rsp, spawn_later, bridge, start_controllers,
                                           showcase_bridge, visualizer, detector, rviz_node, run_scenario])
