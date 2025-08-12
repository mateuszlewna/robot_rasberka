#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from xacro import process_file

def generate_launch_description():
    pkg_name = 'robot'
    
    # Ścieżka do pliku URDF
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'description', 'robot.urdf.xacro')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(pkg_name), 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    # Argument 'use_sim_time'
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/centralized clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_command = Command(['xacro ', urdf_path])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_params_file_cmd,

        # Węzeł 1: Odometria
        Node(
            package=pkg_name,
            executable='odometry_publisher',
            name='robot_odometry_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        #Węzeł 1: Odometria z IMU
        # Node(
        #     package=pkg_name,
        #     executable='odometry_IMU',
        #     name='robot_odometry_imu_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),

        # Węzeł 2: Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_command},
                {'use_sim_time': use_sim_time}
            ],
        ),

        # Węzeł 3: Lidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 256000},
                {'frame_id': 'laser_frame'},
                {'scan_mode': 'Standard'},
                {'use_sim_time': use_sim_time},
            ]
        ),

        # Węzeł 4: TF2 Buffer Server
        Node(
            package='tf2_ros',
            executable='buffer_server',
            name='tf2_buffer_server',
            output='screen',
            parameters=[
                {'buffer_size': 10.0},  # Set buffer size to 10 seconds
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Węzeł 5: Motor controller

        #  Node(
        #     package='robot',
        #     executable='motor_controller_node', 
        #     name='motor_controller',
        #     output='screen'
        # ),

    ])