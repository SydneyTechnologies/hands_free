import os
import xacro

from serial.tools import list_ports
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def getSerialPort():
    """
    Get the serial port for the LiDAR device. The function searches through all available ports and returns
    the one that contains 'USB' in the device name.
    """
    ports = list_ports.comports()
    for port in ports:
        if "USB" in port.device: 
            return port.device
    return None

# Determine the serial port for LiDAR
lidar_serial_port = getSerialPort()
lidar_serial_port = str(lidar_serial_port) if lidar_serial_port else "/dev/ttyUSB0"

def generate_launch_description():
    package_name = "hands_free"

    # Declare whether to use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Load robot description (URDF/XACRO)
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Create the node for robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params]
    )

    # Create the node for joy controller
    joy_params = os.path.join(pkg_path, 'config', 'joystick.yaml')
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params]
    )

    # Create the node for teleop_twist_joy
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='both',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    # Create the node for twist_mux
    twist_mux_params = os.path.join(pkg_path, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params]
    )

    # Create the node for state_publisher
    state_publisher_node = Node(
        package='hands_free',
        executable='state_publisher',
        output='both'
    )

    # Create the node for motion_controller
    motion_controller_node = Node(
        package='hands_free',
        executable='controller',
        name='controller_node'
    )

    # Create the node for LiDAR
    lidar_params = [
        {"port": lidar_serial_port},
        {"frame_id": "lidar_link"}
    ]
    lidar_node = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        parameters=lidar_params
    )

    # Create the node for RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
    'slam_params_file',
    default_value=os.path.join(get_package_share_directory(package_name),
                                'config', 'mapper_params_online_async.yaml'),)




    # TimerAction to introduce a delay after launching state_publisher_node
    delay_after_state_publisher = TimerAction(
        period=3.0,  # Delay period in seconds (adjust as needed)
        actions=[
            # Declare use of simulation time
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use sim time if true'
            ),

            declare_slam_params_file_cmd,


            # Launch other nodes after the delay
            rviz_node,
            joy_node,
            teleop_twist_joy_node,
            motion_controller_node,
            twist_mux_node,
            node_robot_state_publisher,
            lidar_node,
            
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={"slam_params_file": LaunchConfiguration('slam_params_file')}.items()),

        ]
    )

    # LaunchDescription object includes the state_publisher_node followed by the delay
    return LaunchDescription([
        state_publisher_node,
        delay_after_state_publisher
    ])
