import rclpy
from rclpy.node import Node

class MuxControl(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('mux_control')

def main(args=None):
    rclpy.init(args=args)
    mux_control = MuxControl()
    rclpy.spin(mux_control)
    mux_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()