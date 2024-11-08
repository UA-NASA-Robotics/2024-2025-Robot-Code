import rclpy
from rclpy.node import Node
#from interfaces.msg import TwistPlus   #From homebrew ROS package to be implemented by Jon 
#from topic_tools.srv import switch_mux #Service to actually change the mux imput

class MuxControl(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('mux_control')

        #create subscriber listening to output from rs_package
        self.mux_subscriber = self.create_subscription(TwistPlus, '/rs/cmd_vel', self.callbackChangeMux, 10)
        
        #create a client to execute the switch_mux service
        self.cli = self.create_client(switch_mux, 'switch_mux') 
        while not self.cli.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info('Service not available, waiting...') 
        self.req = switch_mux.Request()

    #create a function that makes a request to switch to the /rs/cmd_vel 
    #when a message over /rs/cmd_vel is recieved
    def callbackChangeMux(self, topic):
       mux_control.send_request('/rs/cmd_vel')
        


def main(args=None):
    rclpy.init(args=args)
    mux_control = MuxControl()
    rclpy.spin(mux_control)
    mux_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()