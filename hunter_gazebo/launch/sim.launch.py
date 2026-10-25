"""Spawn the HUNTeR replica in Gazebo (gz sim Harmonic) with ros2_control,
the four-wheel-steering controller and the sensor bridge.

  ros2 launch hunter_gazebo sim.launch.py
  ros2 launch hunter_gazebo sim.launch.py world:=offroad.sdf variant:=2023 payload:=rws
  ros2 launch hunter_gazebo sim.launch.py headless:=true rviz:=false steering_mode:=crab

Drive it:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard        (cmd_vel, Twist)
  ros2 topic pub /four_ws_controller/steering_mode std_msgs/String "data: crab"
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESC = get_package_share_directory("hunter_description")
GZ = get_package_share_directory("hunter_gazebo")
CTRL = get_package_share_directory("hunter_control")


def _truthy(v):
    return str(v).lower() in ("true", "1", "yes")


def _bridge_and_controllers(context):
    """Bridge only the sensors present in this configuration and spawn the
    optional PTZ / RWS position controllers."""
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    variant, payload = get("variant"), get("payload")
    sensors, cameras = _truthy(get("sensors")), _truthy(get("camera_sensors"))
    veh = yaml.safe_load(open(os.path.join(DESC, "config", "vehicle.yaml")))

    def entry(ros, gz, ros_t, gz_t):
        return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_t, "gz_type_name": gz_t,
                "direction": "GZ_TO_ROS"}

    bridge = yaml.safe_load(open(os.path.join(GZ, "config", "gz_bridge.yaml")))
    cams = []
    if sensors:
        for s in veh["sensors"]:
            if variant not in s["variants"]:
                continue
            n = s["name"]
            if s["type"] == "lidar":
                bridge.append(entry(f"/{n}/points", f"/{n}/scan/points", "sensor_msgs/msg/PointCloud2",
                                    "gz.msgs.PointCloudPacked"))
            elif s["type"] == "imu":
                bridge.append(entry(f"/{n}/data", f"/{n}/data", "sensor_msgs/msg/Imu", "gz.msgs.IMU"))
            elif s["type"] == "navsat":
                bridge.append(entry(f"/{n}/fix", f"/{n}/fix", "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat"))
            elif s["type"] == "camera":
                cams.append(n)
            elif s["type"] == "stereo":
                cams += [f"{n}_left", f"{n}_right"]
        if variant in veh["ptz"]["variants"] and payload != "rws":
            cams.append("ptz_camera")
        if payload == "rws":
            cams.append("rws_camera")
    if cameras:
        for c in cams:
            bridge.append(entry(f"/{c}/image_raw", f"/{c}/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"))
            bridge.append(entry(f"/{c}/camera_info", f"/{c}/camera_info", "sensor_msgs/msg/CameraInfo",
                                "gz.msgs.CameraInfo"))
    fd, path = tempfile.mkstemp(prefix="hunter_bridge_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(bridge, f, sort_keys=False)

    actions = [Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                    parameters=[{"config_file": path, "use_sim_time": True}])]
    extra = []
    if sensors and variant in veh["ptz"]["variants"] and payload != "rws":
        extra.append("ptz_controller")
    if payload == "rws":
        extra.append("rws_controller")
    if extra:
        actions.append(TimerAction(period=8.0, actions=[Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=extra + ["--controller-manager", "/controller_manager", "--controller-manager-timeout", "120"],
            parameters=[{"use_sim_time": True}])]))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("world", default_value="flat.sdf", description="world file in hunter_gazebo/worlds"),
        DeclareLaunchArgument("variant", default_value="2020", description="2020 | 2023"),
        DeclareLaunchArgument("payload", default_value="none", description="none | cargo | rws | uav_pad"),
        DeclareLaunchArgument("cargo_mass", default_value="0", description="kg (payload:=cargo), 0 = default"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("camera_sensors", default_value="true"),
        DeclareLaunchArgument("lidar_resolution", default_value="0.5", description="fraction of native lidar samples"),
        DeclareLaunchArgument("visual_detail", default_value="mesh", description="mesh | primitive"),
        DeclareLaunchArgument("steering_mode", default_value="counter",
                              description="front | rear | counter | crab | adaptive | free"),
        DeclareLaunchArgument("linkage", default_value="", description="override steering linkage: trapezoid | ideal"),
        DeclareLaunchArgument("headless", default_value="false", description="run the gz server only"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.15", description="spawn height of base_footprint"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_stamped_vel", default_value="false", description="cmd_vel as TwistStamped"),
    ]
    lc = LaunchConfiguration

    controllers_file = os.path.join(CTRL, "config", "controllers.yaml")
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", os.path.join(DESC, "urdf", "hunter.urdf.xacro"),
        " sim_mode:=gz controllers_file:=", controllers_file,
        " variant:=", lc("variant"), " payload:=", lc("payload"), " cargo_mass:=", lc("cargo_mass"),
        " sensors:=", lc("sensors"), " camera_sensors:=", lc("camera_sensors"),
        " lidar_resolution:=", lc("lidar_resolution"), " visual_detail:=", lc("visual_detail"),
    ]), value_type=str)

    # Let gz resolve package://hunter_description/... mesh URIs.
    resource_env = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(DESC))
    resource_env_models = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(GZ, "models"))

    world = PathJoinSubstitution([GZ, "worlds", lc("world")])
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    gz_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r -s -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(lc("headless")))
    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(lc("headless")))

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "hunter", "-topic", "robot_description",
                            "-x", lc("x"), "-y", lc("y"), "-z", lc("z"), "-Y", lc("yaw")])

    # One spawner for all drive controllers: parallel spawners serialise on a lock.
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
    linkage_viz = Node(package="hunter_control", executable="suspension_linkage_publisher", output="screen",
                       parameters=[{"use_sim_time": True}])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", os.path.join(DESC, "rviz", "hunter.rviz")],
                parameters=[{"use_sim_time": True}], condition=IfCondition(lc("rviz")))

    return LaunchDescription(args + [
        resource_env, resource_env_models, gz_server_only, gz_with_gui, rsp, spawn,
        OpaqueFunction(function=_bridge_and_controllers), start_controllers,
        TimerAction(period=3.0, actions=[four_ws, linkage_viz]), rviz,
    ])
