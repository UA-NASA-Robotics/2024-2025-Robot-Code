import rclpy
from rclpy.node import Node
import queue
from std_msgs.msg import String
from interfaces.msg import TwistPlus
#from interfaces.srv import ODriveSetVelocity
from roi_ros.srv import ODriveSetVelocity

distance_between_wheels = 0.5 #meters

#What control_node should do:
    #Listen to the output topic from mux_node which is a Twist_Plus() type message
    #Twist_Plus includes:
        #a linear vector with fields x, y, z
        #an angular vector with fields x, y, z
        #an array of booleans that represent controller button presses
            #button array index to controller button list:
                #0 - A/Cross
                #1 - B/Circle
                #2 - X/Triangle
                #3 - Y/Square
                #4 - L1
                #5 - R1
                #6 - Select
                #7 - Start
                #8 - Home/P3
                #9 - Left Stick Press/L3
                #10 - Right Stick Press/R3
                #etc.
    #Interperet the Twist_Plus to make a service request to the MCU package
        #On startup call the "Set Mode" service
        #Check to see if a button/macro is pressed/called
            #If it is the "cancel" button, clear the que and send a "stop" command
            #otherwise do the macro
                #continue to velocity translations unless macro says to wait
        #otherwise translate the linear.x (forwards velocity from -1 to 1?) and angular.z 
        #(rotational velocity from w_min to w_max) into Left and Right motor RPMs/desired velocities

    #Also listens to incomming topics from MCU
    #namely, 'current_state', 'node_name', 'node_health'
    #and interperet/use that information
        #unknown what to do with that info at this time. ~Ethan

class ControlNode(Node):
    """
    Node for taking incoming requests and sending it to each part
    """
    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('control_node')

        self.mux_subscriber = self.create_subscription(TwistPlus, '/input/twist_plus', self.twist_callback, 10) # Rename 'macro_twist_plus in launch file
        #self.active_macros = self.create_subscription(ActiveMacros, 'active_macros', self.macro_callback, 10)

        self.oDrive1= self.create_client(ODriveSetVelocity, '/oDrive1')
        self.oDrive2 = self.create_client(ODriveSetVelocity, '/oDrive2')
        self.oDrive3 = self.create_client(ODriveSetVelocity, '/oDrive3')
        self.oDrive4 = self.create_client(ODriveSetVelocity, '/oDrive4')

        
        self.forwardVelocity = None
        self.angularVelocity = None
        self.buttonArray = None
        self.requestArray = [None, None, None, None, None]
        self.theQue = queue.Queue()
        self.left_wheel_speed = None
        self.right_wheel_speed = None
    
    def calculateRPM(self):
        """
        source `ros.org`_.

        Calculate the RPM of the left and right wheels based on the linear and angular velocities
        .. _ros.org: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot
        """
        leftVelocity = self.forwardVelocity - (self.angularVelocity * distance_between_wheels / 2)/60
        rightVelocity = self.forwardVelocity + (self.angularVelocity * distance_between_wheels / 2)/60
        return leftVelocity, rightVelocity
    
    # Sends service request to all 4 wheel servers
    def request_set_velocity(self):
        """
        Sends request to wheels based on control
        """
        print("HELLO")
        left_message = ODriveSetVelocity.Request()
        right_message = ODriveSetVelocity.Request()

        # Set Left side speed and torque
        
        left_message.velocity = self.left_wheel_speed
        left_message.torque_feedforward = 0.0

        # Set Right side speed and torque
        right_message.velocity = self.right_wheel_speed
        right_message.torque_feedforward = 0.0

        # Send request to server, and don't hold up waiting for response
        #self.get_logger().info(str(left_message))
        left_future1 = self.oDrive1.call_async(left_message)
        #self.get_logger().info("Service Success")
        left_future2 = self.oDrive3.call_async(left_message)

        right_future1 = self.oDrive2.call_async(right_message)
        right_future2 = self.oDrive4.call_async(right_message)

    def macro_callback(self, msg):
        """
        Interpret the Twist_Plus to make a service request to the MCU package

        On startup, call the "Set Mode" service – this will be a macro
        Check to see if a button/macro is pressed/called

            If false, "cancel" the button, clear the queue and send a "stop" command
            Otherwise, call the macro
        Also, translare the linear and angular portions to left/right side commands
        """

        #if any controller button is pressed
        if 1.0 in self.buttonArray:
            #do the action with priority in order

            #TODO: Cancel all actions button (probably B/O button)
            if buttonArray[0] == 1:
                #clear que
                send_state_request(self, 0, 0, -1, -1, -1) #set motor RPMs to 0, actuators to idle, instantly
                return  #stop processing code early
            
            #Macro Name
            elif buttonArray[1] == 1:
                #TODO: Add macro steps to que
                pass

            #Macro Name
            elif buttonArray[2] == 1:
                #TODO: Add macro steps to que
                pass

            #Macro Name
            elif buttonArray[3] == 1:
                #TODO: Add macro steps to que
                pass
            
            #Macro Name
            elif buttonArray[4] == 1:
                #TODO: Add macro steps to que
                pass
            
            #Macro Name
            elif buttonArray[5] == 1:
                #TODO: Add macro steps to que
                pass 
            
            #Macro Name
            elif buttonArray[6] == 1:
                #TODO: Add macro steps to que
                pass

            #Macro Name
            elif buttonArray[7] == 1:
                pass
                                
            #Setup Pin Mode -- P3 button / Home button
            elif buttonArray[8] == 1:
                send_mode_request(self) #send directly, bypassing que
                return  #stop early
            
            #Macro Name
            elif buttonArray[9] == 1:
                #TODO: Add macro steps to que
                pass
            
            #Macro Name
            elif buttonArray[10] == 1:
                #TODO: Add macro steps to que
                pass

        #pack all the information into
        # What is "left" in this context? ~Elia
        self.requestArray = [left]
        self.theQue.put(self.requestArray)

        #if the que is not empty, make request
        #if not self.theQue.empty():
        #    send_state_request(self, self.theQue.get())

        self.left_wheel_speed, self.right_wheel_speed = self.calculateRPM()
        self.request_set_velocity(self.left_wheel_speed, self.right_wheel_speed)

        return 0
    
    def twist_callback(self, msg):
        """
        Main update loop
        """
        self.buttonArray = msg.buttons
        self.forwardVelocity = msg.linear.x
        self.angularVelocity = msg.angular.z
        self.left_wheel_speed, self.right_wheel_speed = self.calculateRPM()
        self.request_set_velocity()


#standard node main function
def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
