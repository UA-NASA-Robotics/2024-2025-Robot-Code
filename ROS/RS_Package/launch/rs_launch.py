from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Get the dir path to the config file
    config = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',
        'config',
        'controller.config.yaml'
    )
    config = os.path.normpath(config)

    return LaunchDescription([
        # =========================================================
        # Launch the Turtlesim node (for testing purposes)
        # =========================================================
        Node(
            package='turtlesim',          # ROS package that contains the node
            executable='turtlesim_node',  # The executable to run
            name='turtlesim_node',        # The name to assign to the node
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
        # Launch the teleop_nodeto convert joystick inputs to Twist messages
        # =========================================================
        Node(
            package='teleop_twist_joy',   # ROS package that contains the node
            executable='teleop_node',     # The correct executable name
            name='teleop_node',           # The name to assign to the node
            output='screen',              # Logs outputs to the screen
            parameters=[config],          # Manually define controller parameters
            remappings=[                  # Remap topic for turtlesim
                ('/cmd_vel', '/turtle1/cmd_vel')    
            ]
        ),
    ])

# TODO: make sure the turtlesim remapping doesn't interfere with normal run flow.