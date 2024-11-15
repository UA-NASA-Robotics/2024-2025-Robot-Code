"""
Macro Look-up Table (LUT) looks at the ROS/interfaces/msg/Buttons.msg and
checks if it should run that macro based on the messages.
Conflicting diagram:
Control
    ├── Actuators
    │   ├── arm
    |   └── pitch
    └── Wheels

For example, `arm` conflicts with `Actuators` and `Control`, but not `pitch`.
"""

import rclpy
from rclpy.node import Node

class MacroLut(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('macro_lut')

        self.isControlRunning = False
        self.isActuatorsRunning = False
        self.isWheelsRunning = False
        self.isArmRunning = False
        self.isPitchRunning = False

def main(args=None):
    rclpy.init(args=args)
    macro_lut = MacroLut()
    rclpy.spin(macro_lut)

    macro_lut.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()