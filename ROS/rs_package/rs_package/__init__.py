# Necessary for setuptools build (including duplicate folder name)

# If you have package-wide variables or need to run initialization 
# code when the package is imported, you can include that code in __init__.py.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from abc import ABC, abstractmethod

class JoysticksBase(Node, ABC):
    def __init__(self, node_name, parameters=[]):
        super().__init__(node_name)

        # Declare and get parameters
        self.params = {}
        for param_name, default_value in parameters:
            self.declare_parameter(param_name, default_value)
            self.params[param_name] = self.get_parameter(param_name).value

        # Subscribers & Publishers
        self.joy_subscribe = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.cmd_vel_publish = self.create_publisher(
            Twist,
            '/rs/cmd_vel',  # Publish to /rs/cmd_vel as per your request
            10
        )

        self.get_logger().info(f'{node_name} initialized with parameters: {self.params}')

    #gets called everytime a message is received from the /joy topic
    def joy_callback(self, msg):
        twist_msg = self.process_joystick_input(msg)
        if twist_msg:
            self.cmd_vel_publish.publish(twist_msg)

    @abstractmethod
    def process_joystick_input(self, joy_msg):
        """Process joystick input and return a Twist message."""
        pass


class JoystickTracks(JoysticksBase):
    def __init__(self):
        parameters = [
            ('left_axis_index', 1),
            ('right_axis_index', 4),
            ('scale_linear', 1.0),
            ('scale_angular', 1.0),
        ]
        super().__init__('control_scheme_joystick_tracks', parameters)

    def process_joystick_input(self, msg):
        twist_msg = Twist()

        # Get parameters from self.params
        left_axis_index = self.params['left_axis_index']
        right_axis_index = self.params['right_axis_index']
        scale_linear = self.params['scale_linear']
        scale_angular = self.params['scale_angular']

        # Get L/R track inputs
        try:
            left = msg.axes[left_axis_index] * scale_linear
            right = msg.axes[right_axis_index] * scale_linear
        except IndexError as e:
            self.get_logger().error(f'Axis index out of range: {e}')
            return None

        # Compute linear & angular velocity
        twist_msg.linear.x = (left + right) / 2.0
        twist_msg.angular.z = (right - left) / 2.0 * scale_angular

        return twist_msg


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTracks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
