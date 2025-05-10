import rclpy
from rclpy.node import Node

from interfaces.msg import TwistPlus

class RequestHandler(Node):
    """!
    @brief A ROS2 node that handles switching between two input sources based on specific conditions.
    
    This node subscribes to two input topics and publishes to one output topic. It manages the switching
    between inputs based on an autonomy button press and movement detection.
    """
    
    def __init__(self):
        """!
        @brief Initialize the RequestHandler node.
        
        Creates subscribers for two input topics and a publisher for the output topic.
        Initializes with input1 as the active input source.
        """
        super().__init__('request_handler')
        
        # Create subscribers for the two input topics
        self.sub1 = self.create_subscription(
            TwistPlus,
            '/input1',
            self.input1_callback,
            10)
        
        self.sub2 = self.create_subscription(
            TwistPlus,
            '/input2',
            self.input2_callback,
            10)
            
        # Create publisher for the output topic
        self.publisher = self.create_publisher(
            TwistPlus,
            '/output',
            10)
            
        # Variable to track which input to use (1 or 2)
        self.active_input = 1
        
    def is_movement_detected(self, msg):
        """!
        @brief Check if there is any significant movement in the TwistPlus message.
        
        @param msg The TwistPlus message to check for movement
        @return True if any linear or angular velocity component exceeds 5% of full range, False otherwise
        """
        # print('hi' if getattr(msg.buttons, '_button_control_autonomy_enable'[1:]) else "bye")
        buttons = [button[1:] for button in msg.buttons.__slots__ if button.__contains__('button')]
        return any([getattr(msg.buttons, button_name) for button_name in buttons])

    def input1_callback(self, msg):
        """!
        @brief Callback function for the first input topic.
        
        Handles messages from input1. If the autonomy button is pressed, switches control
        to input2. If movement is detected while input2 is active, switches back to input1.
        
        @param msg The TwistPlus message received from input1
        """
        # If we're on input2 and detect movement on input1, switch back to input1
        if self.active_input == 2 and self.is_movement_detected(msg):
            self.active_input = 1

        # If autonomy button is pressed, switch to input2
        if msg.buttons.button_control_autonomy_enable:
            self.active_input = 2
        
        # If we're on input1, publish the message
        if self.active_input == 1:
            self.publisher.publish(msg)
            
    def input2_callback(self, msg):
        """!
        @brief Callback function for the second input topic.
        
        Handles messages from input2. Publishes messages when input2 is active.
        
        @param msg The TwistPlus message received from input2
        """
        # If we're on input2, publish the message
        if self.active_input == 2:
            self.publisher.publish(msg)

def main(args=None):
    """!
    @brief Main function to initialize and run the RequestHandler node.
    
    @param args Command line arguments (optional)
    """
    rclpy.init(args=args)
    request_handler = RequestHandler()
    rclpy.spin(request_handler)
    request_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()