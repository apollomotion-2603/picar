import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()})
    return robot_description.toxml()


def start_vehicle_control():
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'joint_state_broadcaster'],
        output='screen')
    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'forward_velocity_controller'],
        output='screen')
    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen')
    return joint_state_controller, forward_velocity_controller, forward_position_controller


def generate_launch_description():
    package_name = "gazebo_ackermann_steering_vehicle"
    package_path = get_package_share_directory(package_name)

    robot_description_path = os.path.join(package_path, 'model', 'vehicle.xacro')
    gz_bridge_params_path  = os.path.join(package_path, 'config', 'ros_gz_bridge.yaml')
    vehicle_params_path    = os.path.join(package_path, 'config', 'parameters.yaml')
    world_path             = os.path.join(package_path, 'worlds', 'track_test.sdf')

    robot_description = load_robot_description(robot_description_path, vehicle_params_path)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',
            'on_exit_shutdown': 'true'
        }.items())

    robot_name = "ackermann_steering_vehicle"

    # ── Spawn: bottom straight centerline, heading East (yaw=0) ──
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', '0.0',
            '-y', '-2.666',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-allow_renaming', 'false'],
        output='screen')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output='screen')

    joint_state, forward_velocity, forward_position = start_vehicle_control()

    vehicle_controller_node = Node(
        package='gazebo_ackermann_steering_vehicle',
        executable='vehicle_controller',
        parameters=[vehicle_params_path],
        output='screen')

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[joint_state])),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state,
                on_exit=[forward_velocity, forward_position])),
        gazebo_launch,
        spawn_node,
        robot_state_publisher_node,
        vehicle_controller_node,
        gz_bridge_node,
    ])
