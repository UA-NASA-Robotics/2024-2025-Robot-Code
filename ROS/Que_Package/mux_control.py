import rclpy
from rclpy.node import Node
#from interfaces.msg import TwistPlus   #From homebrew ROS package to be implemented by Jon 
#from topic_tools.srv import switch_mux #Service to actually change the mux imput

class MuxControl(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('mux_control')

        #create subscriber listening to output from RS_package
        
        #create a client to execute the switch_mux service

    #create a function that makes a request to switch to the /rs/cmd_vel 
    #when a message over /rs/cmd_vel is recieved

def main(args=None):
    rclpy.init(args=args)
    mux_control = MuxControl()
    rclpy.spin(mux_control)
    mux_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()