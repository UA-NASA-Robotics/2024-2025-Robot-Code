"""
This node is used to keep track of all running macros and detect if there's any conflicts going on after a request is recieved.
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import TwistPlus
import json
from rcl_interfaces.msg import ParameterDescriptor

class TreeLeaf:
    def __init__(self, name: str):
        self.name = name
        self.children = []
        self.is_active = False  # Track if this macro is currently running

    def add_child(self, child_node):
        self.children.append(child_node)

    def find_node(self, name):
        if self.name == name:
            return self
        
        for child in self.children:
            node = child.find_node(name)
            if node is not None:
                return node
        return None
    
    def get_path_to_root(self, name):
        """Get the path from the named node to the root"""
        if self.name == name:
            return [self]
        
        for child in self.children:
            path = child.get_path_to_root(name)
            if path is not None:
                return [self] + path
        return None

    def __str__(self):
        return self.name

    @classmethod
    def from_dict(cls, tree_dict):
        """Create a TreeNode from a dictionary representation"""
        node = cls(tree_dict['name'])
        for child in tree_dict['children']:
            node.add_child(cls.from_dict(child))
        return node

"""
@TODO
This class has a tree with different macro dependencies

Control
    ├── Actuators
    │   ├── arm
    |   └── pitch
    └── Wheels
"""
class MacroConflicts(Node):
    def __init__(self):
        super().__init__('macro_conflicts')

        # Declare the parameter with a default value
        self.declare_parameter(
            'macro_tree',
            '{"name": "control", "children": [{"name": "actuator", "children": [{"name": "arm", "children": []}, {"name": "pitch", "children": []}]}, {"name": "wheel", "children": []}]}',
            ParameterDescriptor(description='JSON string representing the macro dependency tree')
        )

        # Get and parse the parameter
        tree_json = self.get_parameter('macro_tree').value
        tree_dict = json.loads(tree_json)
        
        # Build the tree from the JSON
        self.macro_tree = TreeLeaf.from_dict(tree_dict)
        
        # Print the tree structure
        self.get_logger().info(f"Initialized macro tree:\n{self.macro_tree}")

        self.active_macros = {}

        self.input = self.create_subscription(
            TwistPlus,
            '/input',
            self.input_callback,
            10)

        self.output = self.create_publisher(
            TwistPlus,
            '/output',
            10)
        
    def input_callback(self, msg: TwistPlus):
        input_enabled = [name[1:] for name in msg.buttons.__slots__ if getattr(msg.buttons, name)]

        macros = {}

        for input in input_enabled:
            macros[input] = self.get_macro_path(input)

        # for key in macros.keys():
        #     print(f'{key}: {[str(node) for node in macros[key]]}')

        if self.check_input_conflicts(macros):
            self.get_logger().error("Input conflicts detected")
            return
        
        for key in self.active_macros.keys():
            print(f'{key}: {[str(node) for node in self.active_macros[key]]}')
        self.check_macro_conflicts(macros)
        for key in self.active_macros.keys():
            print(f'\t{key}: {[str(node) for node in self.active_macros[key]]}')

    def get_macro_path(self, macro_name: str) -> list[TreeLeaf]:
        macro_path = macro_name.split('_')
        for part in macro_path[::-1]:
            node = self.macro_tree.find_node(part)
            if node is not None:
                return self.macro_tree.get_path_to_root(node.name)
        return None
    
    def check_input_conflicts(self, macros: dict[str, list[TreeLeaf]]):
        rows = [macros[macro] for macro in macros.keys()]

        for index in range(len(rows)):
            if rows[index] in rows[index+1:]:
                return True
        
        for macro in macros.keys():
            for other_macro in macros.keys():
                if macro != other_macro:
                    index = min(len(macros[macro]), len(macros[other_macro]))
                    if macros[macro][:index] == macros[other_macro][:index]:
                        return True
        return False

    def check_macro_conflicts(self, request_macros: dict[str, list[TreeLeaf]]):
        request_macros_paths = list(request_macros.values())
        active_macros = self.active_macros
        keys_to_pop = []
        
        for running_key in active_macros.keys():
            if active_macros[running_key] in request_macros_paths:
                keys_to_pop.append(running_key)

        for key in keys_to_pop:
            active_macros.pop(key, None)

        for key in request_macros.keys():
            active_macros[key] = request_macros[key]
        
        self.active_macros = active_macros

def find_key(dictionary: dict[str, list[TreeLeaf]], item: list[TreeLeaf]) -> str | None:
    for key in dictionary.keys():
        if dictionary[key] == item:
            return key
    return None

def main(args=None):
    rclpy.init(args=args)
    macroConflicts = MacroConflicts()
    rclpy.spin(macroConflicts)

    # do something with the node

    macroConflicts.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
