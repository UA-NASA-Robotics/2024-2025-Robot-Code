from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    use_turtlesim = LaunchConfiguration('use_turtlesim')

    return LaunchDescription([

        # Declare the 'use_turtlesim' argument
        DeclareLaunchArgument(
            'use_turtlesim',
            default_value='false',
            description='Set to "true" to include Turtlesim node'
        ),

        # =========================================================
        # Launch the Turtlesim node (for testing purposes)
        # =========================================================
        Node(
            package='turtlesim',          # ROS package that contains the node
            executable='turtlesim_node',  # The executable to run
            name='turtlesim_node',        # The name to assign to the node
            output='screen',              # Logs outputs to the screen
            condition=IfCondition(use_turtlesim)
        ),

        # =========================================================
        # Launch the joy_node to read controller inputs
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
        # ========I=================================================
        Node(
            package='teleop_twist_joy',   # ROS package that contains the node
            executable='teleop_node',     # The executable name
            name='teleop_node',           # The name to assign to the node
            output='screen',              # Logs outputs to the screen
            parameters=[config]           # Manually define controller parameters
        ),

        # Relays cmd_vel to topic turtlesim subscribes to.
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_to_turtle1_cmd_vel',
            arguments=['/cmd_vel', '/turtle1/cmd_vel'],
            output='screen',
            condition=IfCondition(use_turtlesim)
        ),

        # Relays cmd_vel to topic Queue Package subscribes to.
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_to_rs_cmd_vel',
            arguments=['/cmd_vel', '/rs/cmd_vel']
        ),
    ])