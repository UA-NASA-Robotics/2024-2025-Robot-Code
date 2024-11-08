import rclpy
from rclpy.node import Node

class FifoQueue(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('fifo_queue')

def main(args=None):
    rclpy.init(args=args)
    fifo_queue = FifoQueue()
    rclpy.spin(fifo_queue)
    fifo_queue.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()