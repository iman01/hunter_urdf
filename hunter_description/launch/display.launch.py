"""View the HUNTeR replica in RViz with joint sliders (no simulation).

  ros2 launch hunter_description display.launch.py
  ros2 launch hunter_description display.launch.py variant:=2023 payload:=rws
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("hunter_description")
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("variant", default_value="2020", description="2020 | 2023"),
        DeclareLaunchArgument("payload", default_value="none", description="none | cargo | rws | uav_pad"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("visual_detail", default_value="mesh", description="mesh | primitive"),
        DeclareLaunchArgument("gui", default_value="true", description="joint_state_publisher_gui sliders"),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]
    description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", os.path.join(share, "urdf", "hunter.urdf.xacro"),
        " variant:=", lc("variant"), " payload:=", lc("payload"),
        " sensors:=", lc("sensors"), " visual_detail:=", lc("visual_detail"),
    ]), value_type=str)

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(lc("gui"))),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(lc("gui"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "hunter.rviz")],
             condition=IfCondition(lc("rviz")), output="log"),
    ]
    try:
        # Physical links, Panhard rods, coil-overs and tie rods (RViz markers).
        get_package_share_directory("hunter_control")
        nodes.append(Node(package="hunter_control", executable="suspension_linkage_publisher", output="screen"))
    except PackageNotFoundError:
        pass
    return LaunchDescription(args + nodes)
