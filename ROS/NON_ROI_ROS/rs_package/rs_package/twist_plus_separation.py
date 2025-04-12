import rclpy
from rclpy.node import Node

from interfaces.msg import TwistPlus
from geometry_msgs.msg import Twist
from interfaces.msg import Buttons

class SeparateTwistPlus(Node):
    def __init__(self):
        super().__init__('twistplus_separation')

        self.twistplus_subscription = self.create_subscription(
            TwistPlus,
            'twist_plus',
            self.listener_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10
        )

        self.buttons_publisher = self.create_publisher(
            Buttons, 'buttons', 10
        )

    def listener_callback(self, msg: TwistPlus):
        twist_output = Twist()
        button_output = Buttons()

        twist_output.linear.x = msg.linear.x
        twist_output.angular.z = msg.angular.z

        for attr in dir(msg.buttons):
            if attr.startswith("button_"):
                setattr(button_output, attr, getattr(msg.buttons, attr))

        self.cmd_vel_publisher.publish(twist_output)
        self.buttons_publisher.publish(button_output)

def main(args=None):
    rclpy.init(args=args)

    separation = SeparateTwistPlus()

    rclpy.spin(separation)

    separation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()