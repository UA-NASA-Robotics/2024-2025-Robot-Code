import rclpy
from rclpy.node import Node

class RequestHandler(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('request_handler')

def main(args=None):
    rclpy.init(args=args)
    request_handler = RequestHandler()
    rclpy.spin(request_handler)
    request_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()