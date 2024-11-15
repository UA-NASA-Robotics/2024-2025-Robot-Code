"""
This node is used to keep track of all running macros and detect if there's any conflicts going on after a request is recieved.
"""

import rclpy
from rclpy.node import Node

"""
@TODO
This class has a tree with different macro dependencies

Control
    ├── Actuators
    │   ├── arm
    |   └── pitch
    └── Wheels
"""
class conflictTree:
    def __init__(self):
        return 0


class MacroConflicts(Node):
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('macro_conflicts')

        self.isControlRunning = False
        self.isActuatorsRunning = False
        self.isWheelsRunning = False
        self.isArmRunning = False
        self.isPitchRunning = False

    self.MuxInput = self.create_subscription(
            TwistPlus,
            '/MuxOutput',
            10)

    self.publisher = self.create_publisher(
        TwistPlus,
        '/MacroConflicts',
        10)

    # Checks if input is conflicting with anything
    def isConflict(macro):
        return 0

def main(args=None):
    rclpy.init(args=args)
    macroConflicts = MacroConflicts()
    rclpy.spin(macroConflicts)

    # do something with the node

    macroConflicts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()