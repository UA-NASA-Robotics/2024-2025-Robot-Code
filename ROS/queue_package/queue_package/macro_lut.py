import rclpy
from rclpy.node import Node

class MacroLut(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('macro_lut')

def main(args=None):
    rclpy.init(args=args)
    macro_lut = MacroLut()
    rclpy.spin(macro_lut)
    macro_lut.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()