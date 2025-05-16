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
        self.output = TwistPlus()

        # Each element corresponds to an input from the controller
        # Split into axes and buttons
        # Each element has a comment with which button (or analog control) it represents
        # In order to change a mapping, the string is moved around.
        # For example, axes_map[1] represents the Left Stick Y
        # If 'left_drive' is in axes_map[1], then Left Stick Y controls the left drive
        axes_map = [
            '',                 # Left Stick X
            'left_drive',       # Left Stick Y
            '',                 # Right Stick X
            'right_drive',      # Right Stick Y
            'actuator_arm_down',                 # Left Trigger (L2)
            'actuator_pitch_down',                 # Right Trigger (R2)
        ]
        button_map = [
            'actuator_dig_cycle',       # A (X)
            'actuator_dump_cycle',      # B (◯)
            '',                         # X (□)
            '',                         # Y (△)
            'actuator_arm_stop',                         # Menu
            '',                         # Home
            '',                         # Start
            '',                         # Left Stick
            '',                         # Right Stick
            'actuator_arm_up',                         # Left Bumper (L1)
            'actuator_pitch_up',                         # Right Bumper (R1)
            'wheel_nav_north',                         # D-Pad Up
            'wheel_nav_south',                         # D-Pad Down
            'wheel_nav_west',                         # D-Pad Left
            'wheel_nav_east',                         # D-Pad Right
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
        self.create_timer(0.05, self.timer_callback)

    # On a new controller input, send a TwistPlus packet
    def listener_callback(self, msg: Joy):
        # Get parameters (live updating)
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        axes_parameters = self.get_parameter('axes').get_parameter_value().string_array_value
        button_parameters = self.get_parameter('buttons').get_parameter_value().string_array_value

        # Set both parts of TwistPlus message
        self.set_twist(msg, self.output, axes_parameters, wheel_radius, wheel_separation)
        self.set_buttons(msg, self.output, button_parameters)

        # # If the output packet is all 0's
        # if self.test_null(output):
        #     # If a 0's packet was already sent
        #     if self.null_sent:
        #         # stop
        #         return
        #     # Set null_sent to true
        #     # Send the packet of 0's
        #     self.null_sent = True
        # else:
        #     # The packet is not all 0's
        #     self.null_sent = False

        # Send the packet
        #self.cmd_vel_publisher.publish(output)

    def timer_callback(self):
        self.cmd_vel_publisher.publish(self.output)

    # Prepares Twist part of TwistPlus
    def set_twist(self, input: Joy, output: TwistPlus, axes: list[str],
                  wheel_radius: float, wheel_separation: float) -> TwistPlus:

        # Get the percents from the joy topic
        left_percent = input.axes[axes.index('left_drive')]
        right_percent = input.axes[axes.index('right_drive')]
        
        # Formula was derrived from the opposite (take linear and angular to left and right percent)
        output.linear.x = (left_percent + right_percent) * wheel_radius / 2
        output.angular.z = 2 * (output.linear.x - left_percent * wheel_radius) / wheel_separation

        # Set everything else to 0's
        output.linear.y, output.linear.z = 0.0, 0.0
        output.angular.x, output.angular.y = 0.0, 0.0

        if output.linear.x != 0.0 or output.angular.z != 0.0:
            output.buttons.button_wheel_ismoving = True
        else:
            output.buttons.button_wheel_ismoving = False

        actuator_pitch_down_bool = input.axes[axes.index('actuator_pitch_down')]
        actuator_arm_down_bool = input.axes[axes.index('actuator_arm_down')]

        #self.get_logger().info(str(actuator_arm_down_bool))

        if actuator_arm_down_bool <= -0.5:
            output.buttons.button_actuator_arm_down = True
        else:
            output.buttons.button_actuator_arm_down = False

        if actuator_pitch_down_bool <= -0.5:
            output.buttons.button_actuator_pitch_down = True
        else:
            output.buttons.button_actuator_pitch_down = False 

        # Return the output
        return output

    # Prepares Button part of TwistPlus
    def set_buttons(self, input: Joy, output: TwistPlus, 
                    buttons: list[str]) -> TwistPlus:
        # Loop through all button mappings
        for button_name in buttons:
            if button_name:  # Skip empty mappings
                # Construct the full variable name
                full_button_name = f'button_{button_name}'
                # Set the button state using dynamic attribute access
                setattr(output.buttons, full_button_name, 
                        input.buttons[buttons.index(button_name)] == 1)
        return output
    
    # Goes through every element in the Joy topic (other than timing)
    # Returns False if something is not 0, otherwise returns true
    def test_null(self, msg: TwistPlus) -> bool:
        for i in [msg.linear, msg.angular]:
            for j in [i.x, i.y, i.z]:
                if abs(j) >= 0.05:
                    return False
        # Check all button attributes
        for attr in dir(msg.buttons):
            if attr.startswith('button_'):  # Only check button states
                if getattr(msg.buttons, attr):
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
