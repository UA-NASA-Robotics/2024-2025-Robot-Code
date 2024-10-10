from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
        # Launch the teleop_node (instead of teleop_twist_joy_node) to convert joystick inputs to Twist messages
        # =========================================================
        Node(
            package='teleop_twist_joy',   # ROS package that contains the node
            executable='teleop_node',     # The correct executable name
            name='teleop_node',           # The name to assign to the node
            output='screen',              # Logs outputs to the screen
            parameters=[                  # Manually define PS3 controller parameters
                {
                    'axis_linear': 1,            # Axis mapping for linear velocity (e.g., left stick up/down)
                    'axis_angular': 0,           # Axis mapping for angular velocity (e.g., left stick left/right)
                    'scale_linear': 2.0,         # Linear scaling factor
                    'scale_angular': 1.0,        # Angular scaling factor
                    'enable_button': 5,          # Button that enables movement (e.g., R1 button)
                    'enable_turbo_button': 4,    # Button for turbo mode (e.g., R2 button)
                    'scale_linear_turbo': 3.0,   # Turbo scaling factor for linear velocity
                    'scale_angular_turbo': 2.0   # Turbo scaling factor for angular velocity
                }
            ],
            remappings=[                  # Remaps topics for communication between nodes
                ('/cmd_vel', '/turtle1/cmd_vel')  # Remap /cmd_vel to /turtle1/cmd_vel
            ]
        )
    ])
