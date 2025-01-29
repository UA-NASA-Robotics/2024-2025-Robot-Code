import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from interfaces.msg import TwistPlus

# Node for taking input from a controller and interpreting it
# input is from sensor_msgs/msg/Joy
# output is of interfaces/msg/TwistPlus
class ControllerInterpreter(Node):
    def __init__(self):
        super().__init__('skid_steer')

        # Each element corresponds to an input from the controller
        # Split into axes and buttons
        # Each element has a comment with which button (or analog control) it represents
        # In order to change a mapping, the string is moved around.
        # For example, axes_map[1] represents the Left Stick Y
        # If 'left_drive' is in axes_map[1], then Left Stick Y controls the left drive
        axes_map = [
            '',                     # Left Stick X
            'left_drive',           # Left Stick Y
            '',                     # Right Stick X
            'right_drive',          # Right Stick Y
            'actuator_arm_down',    # Left Trigger (L2)
            'actuator_pitch_down',  # Right Trigger (R2)
        ]
        button_map = [
            'autonomy_enable',      # A (X)
            '',                     # B (◯)
            '',                     # X (□)
            '',                     # Y (△)
            '',                     # Menu
            '',                     # Home
            '',                     # Start
            '',                     # Left Stick
            '',                     # Right Stick
            'actuator_arm_up',      # Left Bumper (L1)
            'actuator_pitch_up',    # Right Bumper (R1)
            '',                     # D-Pad Up
            '',                     # D-Pad Down
            '',                     # D-Pad Left
            '',                     # D-Pad Right
        ]

        # Don't send many null packets (reduce bandwidth usage)
        self.null_sent = False

        self.declare_parameter('wheel_radius', 10.0)
        self.declare_parameter('wheel_separation', 10.0)
        self.declare_parameter('axes', axes_map.copy())
        self.declare_parameter('buttons', button_map.copy())

        self.joystick_subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(TwistPlus, 'twist_plus', 10)

    # On a new controller input, send a TwistPlus packet
    def listener_callback(self, msg: Joy):
        # Get parameters (live updating)
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        axes_parameters = self.get_parameter('axes').get_parameter_value().string_array_value
        button_parameters = self.get_parameter('buttons').get_parameter_value().string_array_value

        output = TwistPlus()

        # Set both parts of TwistPlus message
        self.set_twist(msg, output, axes_parameters, wheel_radius, wheel_separation)
        self.set_buttons(msg, output, button_parameters, axes_parameters)

        # If the output packet is all 0's
        if self.test_null(output):
            # If a 0's packet was already sent
            if self.null_sent:
                # stop
                return
            # Set null_sent to true
            # Send the packet of 0's
            self.null_sent = True
        else:
            # The packet is not all 0's
            self.null_sent = False

        # Send the packet
        self.cmd_vel_publisher.publish(output)

    # Prepares Twist part of TwistPlus
    def set_twist(self, input: Joy, output: TwistPlus, axes: list[str],
                  wheel_radius: float, wheel_separation: float) -> TwistPlus:

        # Get the percents from the joy topic
        left_percent = input.axes[axes.index('left_drive')]
        right_percent = input.axes[axes.index('right_drive')]
        
        # Formula was derrived from the opposite (take linear and angular to left and right percent)
        if abs(left_percent) >= 0.05 or abs(right_percent) >= 0.05:
            setattr(output.buttons, 'button_weels_ismoving', True)
            output.linear.x = (left_percent + right_percent) * wheel_radius / 2
            output.angular.z = 2 * (output.linear.x - left_percent * wheel_radius) / wheel_separation

        # Set everything else to 0's
        output.linear.y, output.linear.z = 0.0, 0.0
        output.angular.x, output.angular.y = 0.0, 0.0

        # Return the output
        return output

    # Prepares Button part of TwistPlus
    def set_buttons(self, input: Joy, output: TwistPlus, 
                    buttons: list[str], axes: list[str]) -> TwistPlus:
        for item in [item for item in buttons + axes if item != '' if 'button_' + item in dir(output.buttons)]:
            full_name = 'button_' + item
            if item in buttons:
                setattr(output.buttons, full_name,
                        input.buttons[buttons.index(item)] == 1)
            elif item in axes:
                setattr(output.buttons, full_name,
                        abs(input.axes[axes.index(item)]) >= 0.5)

        return output
    
    # Goes through every element in the Joy topic (other than timing)
    # Returns False if something is not 0, otherwise returns true
    def test_null(self, msg: TwistPlus) -> bool:
        for item in [item[1:] for item in msg.buttons.__slots__ if 'button' in item]:
            if getattr(msg.buttons, item):
                return False
        return True

def main(args=None):
    rclpy.init(args=args)

    controller_interpreter = ControllerInterpreter()

    rclpy.spin(controller_interpreter)

    controller_interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
