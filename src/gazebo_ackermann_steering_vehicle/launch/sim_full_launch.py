"""
Full simulation launch — 1 lệnh chạy tất cả nodes.
Usage:
  ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py
  ros2 launch gazebo_ackermann_steering_vehicle sim_full_launch.py map:=2
"""
import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

MAP_CONFIGS = {
    "1": {"world": "lane_track.sdf", "x": "0.0",  "y": "1.0",    "Y": "0.0"},
    "2": {"world": "track_test.sdf", "x": "0.0",  "y": "-2.666", "Y": "0.0"},
    "3": {"world": "lane_change.sdf", "x": "0.54",  "y": "0.75",    "Y": "0.0"},
    "4": {"world": "obstacle_track.sdf", "x": "1.5", "y": "1.0", "Y": "0.0"},
}

def load_robot_description(xacro_path, params_path):
    with open(params_path) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    return xacro.process_file(xacro_path,
        mappings={k: str(v) for k, v in params.items()}).toxml()

def generate_launch_description():
    pkg = get_package_share_directory("gazebo_ackermann_steering_vehicle")
    ws  = os.path.expanduser("~/main/1_projects/1_autonomous_car_research/ros2_ws")

    # Resolve map id
    map_id_val = "1"
    for arg in sys.argv:
        if arg.startswith("map:="):
            map_id_val = arg.split(":=")[1]
    if map_id_val not in MAP_CONFIGS:
        raise ValueError(f"map={map_id_val} khong hop le. Dung: {list(MAP_CONFIGS.keys())}")

    cfg        = MAP_CONFIGS[map_id_val]
    world_path = os.path.join(pkg, "worlds", cfg["world"])

    xacro_path  = os.path.join(pkg, "model",  "vehicle.xacro")
    params_path = os.path.join(pkg, "config", "parameters.yaml")
    bridge_path = os.path.join(pkg, "config", "ros_gz_bridge.yaml")
    perc_yaml   = os.path.join(ws,  "src/perception_pkg/config/perception.yaml")
    nmpc_yaml   = os.path.join(ws,  "src/mpc_pkg/config/nmpc.yaml")

    robot_desc = load_robot_description(xacro_path, params_path)

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 2 {world_path}",
                          "on_exit_shutdown": "true"}.items())

    # Spawn
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "ackermann_steering_vehicle",
                   "-string", robot_desc,
                   "-x", cfg["x"], "-y", cfg["y"], "-z", "0.05",
                   "-R", "0.0", "-P", "0.0", "-Y", cfg["Y"],
                   "-allow_renaming", "false"],
        output="screen")

    robot_state_pub = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
        output="screen")

    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_path}"],
        output="screen")

    vehicle_ctrl = Node(
        package="gazebo_ackermann_steering_vehicle", executable="vehicle_controller",
        parameters=[params_path], output="screen")

    # ros2_control
    joint_state_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"], output="screen")
    fwd_vel_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "forward_velocity_controller"], output="screen")
    fwd_pos_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "forward_position_controller"], output="screen")

    # ROS2 nodes (delay de doi Gazebo + controllers san sang)
    perception = TimerAction(period=5.0, actions=[
        Node(package="perception_pkg", executable="perception_node",
             arguments=["--ros-args", "--params-file", perc_yaml],
             output="screen")])

    ekf = TimerAction(period=6.0, actions=[
        Node(package="ekf_pkg", executable="ekf_node", output="screen")])

    nmpc = TimerAction(period=7.0, actions=[
        Node(package="mpc_pkg", executable="mpc_node",
             arguments=["--ros-args", "--params-file", nmpc_yaml],
             output="screen")])

    visualizer = TimerAction(period=6.5, actions=[
        Node(package="perception_pkg", executable="visualizer_node",
             output="screen")])

    grid_viewer = TimerAction(period=7.5, actions=[
        Node(package="perception_pkg", executable="grid_viewer",
             output="screen")])

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="1",
                              description="Map ID: 1=lane_track 2=track_test"),
        gazebo,
        spawn,
        robot_state_pub,
        gz_bridge,
        vehicle_ctrl,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_ctrl,
                                           on_exit=[fwd_vel_ctrl, fwd_pos_ctrl])),
        perception,
        ekf,
        visualizer,
        nmpc,
        grid_viewer,
    ])
