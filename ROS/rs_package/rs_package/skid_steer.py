import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from interfaces.msg import TwistPlus

class ControllerInterpreter(Node):
    def __init__(self):
        super().__init__('skid_steer')

        axis_map = [
            '',                 # Left Stick X
            'left_drive',       # Left Stick Y
            '',                 # Right Stick X
            'right_drive',      # Right Stick Y
            '',                 # Left Trigger (L2)
            '',                 # Right Trigger (R2)
        ]
        button_map = [
            'autonomy_enable',  # A (X)
            '',                 # B (◯)
            '',                 # X (□)
            '',                 # Y (△)
            '',                 # Menu
            '',                 # Home
            '',                 # Start
            '',                 # Left Stick
            '',                 # Right Stick
            '',                 # Left Bumper (L1)
            '',                 # Right Bumper (R1)
            '',                 # D-Pad Up
            '',                 # D-Pad Down
            '',                 # D-Pad Left
            '',                 # D-Pad Right
        ]

        self.null_sent = False

        self.declare_parameter('wheel_radius', 10.0)
        self.declare_parameter('wheel_separation', 10.0)
        self.declare_parameter('axis', axis_map.copy())
        self.declare_parameter('buttons', button_map.copy())

        self.joystick_subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(TwistPlus, 'twist_plus', 10)

    def listener_callback(self, msg: Joy):
        if self.test_null(msg):
            if self.null_sent:
                return
            self.null_sent = True
        else:
            self.null_sent = False
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        axis_parameters = self.get_parameter('axis').get_parameter_value().string_array_value
        button_parameters = self.get_parameter('buttons').get_parameter_value().string_array_value

        output = TwistPlus()

        self.set_twist(msg, output, axis_parameters, wheel_radius, wheel_separation)
        self.set_buttons(msg, output, button_parameters)

        self.cmd_vel_publisher.publish(output)

    def set_twist(self, input: Joy, output: TwistPlus, axis: list[str],
                  radius: float, separation: float) -> TwistPlus:

        left_percent = input.axes[axis.index('left_drive')]
        right_percent = input.axes[axis.index('right_drive')]
        
        output.linear.x = (left_percent + right_percent) * radius / 2
        output.angular.z = 2 * (output.linear.x - left_percent * radius) / separation

        output.linear.y, output.linear.z = 0.0, 0.0
        output.angular.x, output.angular.y = 0.0, 0.0

        return output

    def set_buttons(self, input: Joy, output: TwistPlus, 
                    buttons: list[str]) -> TwistPlus:
        
        output.buttons.autonomy_enable = input.buttons[buttons.index('autonomy_enable')] == 1

        return output
    
    def test_null(self, msg: Joy) -> bool:
        tmp = [msg.axes[1], msg.axes[3]]+ list(msg.buttons)

        print(tmp)

        for i in tmp:
            if abs(i) >= 0.01:
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
