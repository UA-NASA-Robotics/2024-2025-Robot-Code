import rclpy
from rclpy.node import Node

class Feedback_Processor(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('control_node')

def main(args=None):
    rclpy.init(args=args)
    feedback_processor = Feedback_Processor()
    rclpy.spin(feedback_processor)
    feedback_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()