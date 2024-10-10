# controller_turtle.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Define the relative path to the YAML configuration
    config_file = os.path.join(
        FindPackageShare('RS_Package').find('RS_Package'),
        '../config',  # Relative path inside the package
        'ps3.config.yaml'  # The YAML file
    )

    return LaunchDescription([
        # =========================================================
        # Launch the Turtlesim node (for testing purposes)
        # =========================================================
        Node(
            package='turtlesim',          # ROS package that contains the node
            executable='turtlesim_node',  # The executable to run
            name='turtlesim',             # The name to assign to the node
            output='screen'               # Logs outputs to the screen
        ),

        # =========================================================
        # Launch the joy_node to read PS3 controller inputs
        # =========================================================
        Node(
            package='joy',                # ROS package for the joystick node
            executable='joy_node',        # The executable to run
            name='joy_node',              # The name to assign to the node
            parameters=[                  # Sets parameters for the node
                {'dev': '/dev/input/js0'} # Specify the joystick device (e.g., /dev/input/js0)
            ],
            output='screen'               # Logs outputs to the screen
        ),

        # =========================================================
        # Launch the teleop_twist_joy_node to convert joystick inputs to Twist messages
        # =========================================================
        Node(
            package='teleop_twist_joy',   # ROS package that contains the node
            executable='teleop_twist_joy_node', # The executable to run
            name='teleop_twist_joy_node', # The name to assign to the node
            output='screen',              # Logs outputs to the screen
            parameters=[config_file],     # Use relative path to the PS3 controller configuration file
            remappings=[                  # Remaps topics for communication between nodes
                ('/cmd_vel', '/turtle1/cmd_vel')  # Remap /cmd_vel to /turtle1/cmd_vel
            ]
        )
    ])


