import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, ExecuteProcess, TimerAction,
                            SetEnvironmentVariable)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path, extra_mappings={}):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.

    @param robot_description_path: Path to the robot's Xacro file.
    @param vehicle_params_path: Path to the YAML file containing the vehicle parameters.
    @return: A string containing the robot's URDF XML description.
    """
    # Load parameters from YAML file
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    mappings = {key: str(value) for key, value in vehicle_params.items()}
    mappings.update(extra_mappings)

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(
        robot_description_path,
        mappings=mappings)

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.

    @return: A tuple containing ExecuteProcess actions for the joint state, forward velocity, 
             and forward position controllers.
    """
    joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'start', 'joint_state_broadcaster'],
        output='screen')

    forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'start', 'forward_velocity_controller'],
        output='screen')

    forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'forward_position_controller'],
        output='screen')

    return (joint_state_controller,
            forward_velocity_controller,
            forward_position_controller)


def generate_launch_description():
    # Define the robot's and package name (cần trước để tính path)
    robot_name = "gazebo_ackermann_steering_vehicle"
    robot_pkg_path = get_package_share_directory(robot_name)

    # Tính đường dẫn model và world mặc định
    model_path = os.path.join(robot_pkg_path, 'model')
    default_world = os.path.join(robot_pkg_path, 'worlds', 'track.world')

    # Thiết lập GAZEBO_MODEL_PATH để Gazebo tìm model://track
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    full_model_path = model_path + ':' + existing_model_path if existing_model_path else model_path
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', full_model_path)

    # Define a launch argument for the world file, defaulting to track.world
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Specify the world file for Gazebo (e.g., track.world)')

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                  description='Initial X position')

    y_arg = DeclareLaunchArgument('y', default_value='-6.5',
                                  description='Initial Y position') # -9.0 khi dùng track.world để đặt xe ở vạch xuất phát

    z_arg = DeclareLaunchArgument('z', default_value='0.065',
                                  description='Initial Z position')

    roll_arg = DeclareLaunchArgument('R', default_value='0.0',
                                     description='Initial Roll')

    pitch_arg = DeclareLaunchArgument('P', default_value='0.0',
                                      description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0',
                                    description='Initial Yaw')

    # Retrieve launch configurations
    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Set paths to Xacro model and configuration files
    robot_description_path = os.path.join(robot_pkg_path, 'model',
                                          'vehicle.xacro')

    vehicle_params_path = os.path.join(robot_pkg_path, 'config',
                                       'params.yaml')
    # Load URDF
    gz_ros2_control_config = os.path.join(robot_pkg_path, 'config', 'gz_ros2_control.yaml')
    robot_description = load_robot_description(
        robot_description_path,
        vehicle_params_path,
        {'gz_ros2_control_config': gz_ros2_control_config})

    # Include the Gazebo Classic launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items())

    # Create node to publish the robot state
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description,
                                          'use_sim_time': True}],
                                      output='screen')

    # Create node to spawn robot model in the Gazebo world
    spawn_model_gazebo_node = Node(package='gazebo_ros',
                                   executable='spawn_entity.py',
                                   arguments=['-entity', robot_name,
                                              '-topic', 'robot_description',
                                              '-x', x,
                                              '-y', y,
                                              '-z', z,
                                              '-R', roll,
                                              '-P', pitch,
                                              '-Y', yaw],
                                   output='screen')

    # Load controllers after spawn is complete
    (joint_state_controller,
     forward_velocity_controller,
     forward_position_controller) = start_vehicle_control()

    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_model_gazebo_node,
            on_exit=[TimerAction(
                period=3.0,
                actions=[joint_state_controller,
                         forward_velocity_controller,
                         forward_position_controller])]))

    vehicle_controller_node = Node(package=robot_name,
                                   executable='vehicle_controller',
                                   parameters=[vehicle_params_path],
                                   output='screen')

    drive_model_node = Node(package=robot_name,
                            executable='drive_model',
                            parameters=[vehicle_params_path],
                            output='screen')


    odom_filter_node = Node(package=robot_name,
                            executable='odom_filter',
                            output='screen')

    keyboard_controller_node = Node(package=robot_name,
                                    executable='keyboard_controller',
                                    output='screen')

    # Create the launch description
    launch_description = LaunchDescription([set_gazebo_model_path,
                                            world_arg,
                                            x_arg,
                                            y_arg,
                                            z_arg,
                                            roll_arg,
                                            pitch_arg,
                                            yaw_arg,
                                            gazebo_launch,
                                            robot_state_publisher_node,
                                            spawn_model_gazebo_node,
                                            load_controllers,
                                            vehicle_controller_node,
                                            drive_model_node,  # not used in keyboard mode
                                            odom_filter_node,
                                            keyboard_controller_node
                                            ])

    return launch_description