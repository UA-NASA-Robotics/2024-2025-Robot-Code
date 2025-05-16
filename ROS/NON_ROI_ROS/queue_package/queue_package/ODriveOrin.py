import rclpy
from rclpy.node import Node
import queue
from std_msgs.msg import String
from interfaces.msg import TwistPlus
#from interfaces.srv import ODriveSetVelocity
from roi_ros.srv import ODriveSetVelocity
from roi_ros.srv import ActuatorSetVelocity
import time
import odrive
from odrive.enums import AxisState
from odrive.utils import request_state

class ODriveOrin(Node):
    """!
    @brief A ROS2 node that controls the front right motor.
    
    This node servers requests from the control node on the jetson to control the front right motor over usb
    """

    def __init__(self):
        super().__init__('odrive_orin')
        self.srv = self.create_service(ODriveSetVelocity, '/output', self.set_Odrive_velocity)
        
        self.odrive0 = odrive.find_sync()
        request_state(self.odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

        self.odrv0.axis0.controller.config.input_mode = 2
        self.odrv0.axis0.controller.config.control_mode = 2

        self.get_logger().info("Hello World")


    def set_Odrive_velocity(self, request, response):
        response.successs = True
        self.get_logger().info('Velocity set to: %d' % (request.velocity))
        self.odrv0.axis0.controller.input_vel = request.velocity

        return response


def main():
    rclpy.init()

    odrive_orin = ODriveOrin()

    rclpy.spin(odrive_orin)

    rclpy.shutdown()


if __name__ == '__main__':
    main()