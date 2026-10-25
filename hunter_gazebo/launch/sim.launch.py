"""Spawn the HUNTeR replica in Gazebo (gz sim Harmonic) with ros2_control and
the four-wheel-steering controller.

  ros2 launch hunter_gazebo sim.launch.py
  ros2 launch hunter_gazebo sim.launch.py world:=offroad.sdf payload:=rws steering_mode:=crab
  ros2 launch hunter_gazebo sim.launch.py headless:=true rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("hunter_description")
    gz_share = get_package_share_directory("hunter_gazebo")
    ctrl_share = get_package_share_directory("hunter_control")

    args = [
        DeclareLaunchArgument("world", default_value="flat.sdf", description="world file in hunter_gazebo/worlds or absolute path"),
        DeclareLaunchArgument("payload", default_value="none", description="none | cargo | rws | uav_pad"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("visual_detail", default_value="mesh", description="mesh | primitive"),
        DeclareLaunchArgument("steering_mode", default_value="counter", description="front | rear | counter | crab | adaptive | free"),
        DeclareLaunchArgument("linkage", default_value="", description="override steering linkage model: trapezoid | ideal"),
        DeclareLaunchArgument("headless", default_value="false", description="run the gz server only"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.15", description="spawn height of base_footprint above ground"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_stamped_vel", default_value="false", description="cmd_vel as TwistStamped"),
    ]
    lc = LaunchConfiguration

    controllers_file = os.path.join(ctrl_share, "config", "controllers.yaml")
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", os.path.join(desc_share, "urdf", "hunter.urdf.xacro"),
        " sim_mode:=gz",
        " controllers_file:=", controllers_file,
        " payload:=", lc("payload"),
        " sensors:=", lc("sensors"),
        " visual_detail:=", lc("visual_detail"),
    ]), value_type=str)

    # Let gz resolve package://hunter_description/... mesh URIs.
    resource_env = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(desc_share))
    resource_env_models = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(gz_share, "models"))

    world = PathJoinSubstitution([gz_share, "worlds", lc("world")])
    gz_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(lc("headless")))
    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(lc("headless")))

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-name", "hunter", "-topic", "robot_description",
                   "-x", lc("x"), "-y", lc("y"), "-z", lc("z"), "-Y", lc("yaw")])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": os.path.join(gz_share, "config", "gz_bridge.yaml"), "use_sim_time": True}])

    # One spawner for all controllers: parallel spawners serialise on a lock.
    controllers = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "steering_position_controller", "wheel_velocity_controller",
                   "--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": True}])
    start_controllers = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers]))

    four_ws = Node(
        package="hunter_control", executable="four_ws_controller", output="screen",
        parameters=[{"use_sim_time": True,
                     "steering_mode": lc("steering_mode"),
                     "linkage": lc("linkage"),
                     "use_stamped_vel": ParameterValue(lc("use_stamped_vel"), value_type=bool)}])

    linkage_viz = Node(
        package="hunter_control", executable="suspension_linkage_publisher", output="screen",
        parameters=[{"use_sim_time": True}])

    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=["-d", os.path.join(desc_share, "rviz", "hunter.rviz")],
        parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")))

    return LaunchDescription(args + [
        resource_env, resource_env_models, gz_server_only, gz_with_gui, rsp, spawn, bridge,
        start_controllers, TimerAction(period=3.0, actions=[four_ws, linkage_viz]), rviz,
    ])
