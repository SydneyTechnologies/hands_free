import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = "hands_free"

    # Paths to configuration files
    joy_params = os.path.join(get_package_share_directory(package_name), 'config', 'joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # Define the state publisher node
    state_publisher_node = Node(
        package='hands_free',
        executable='state_publisher',
        output='both'
    )

    # Define a timer action to introduce a delay after launching state publisher node
    delay_after_state_publisher = TimerAction(
        period=3.0,  # Delay period in seconds (adjust as needed)
        actions=[
            # Define other nodes to start after the delay
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[joy_params]
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_node',
                output='both',
                parameters=[joy_params],
                remappings=[
                    ('/cmd_vel', '/cmd_vel_joy'),
                ]
            ),
            Node(
                package='twist_mux',
                executable='twist_mux',
                parameters=[twist_mux_params]
            ),
            Node(
                package='hands_free',
                executable='controller',
                name='controller_node',
                output='screen'
            )
        ]
    )

    # Launch description including the state publisher node and the delay
    return LaunchDescription([
        state_publisher_node,
        delay_after_state_publisher
    ])
