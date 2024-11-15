"""
WE'RE THINKING OF DELETING THIS NODE
"""
import rclpy
from rclpy.node import Node
import queue

class FifoQueue(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('fifo_queue')
        self.queue = queue.Queue()

    def enqueue(self, item):
        self.queue.put(item)
        # self.get_logger().info(f'Item {item} enqueued')

    def dequeue(self):
        if not self.queue.empty():
            item = self.queue.get()
            # self.get_logger().info(f'Item {item} dequeued')
            return item
        else:
            # self.get_logger().info('Queue is empty')
            return None

def main(args=None):
    rclpy.init(args=args)
    fifo_queue = FifoQueue()
    rclpy.spin(fifo_queue)

    fifo_queue.enqueue('something')
    fifo_queue.enqueue('something else')

    fifo_queue.dequeue()
    fifo_queue.dequeue()

    fifo_queue.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()