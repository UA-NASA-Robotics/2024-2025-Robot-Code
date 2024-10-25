import rclpy
from rclpy.node import Node
import queue
from std_msgs.msg import String
from mcu.srv import Set_Mode 
from mcu.srv import Set_State 
from rs.msg import TwistPlus

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

    #Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__('control_node')
        self.subscriber_ = self.create_subscription(TwistPlus, 'output topic from mux_node', self.callback_Input, 10)
        self.subscriber_ = self.create_subscription(String, 'output topic MCU - current_state', self.callback_State, 10)
        self.subscriber_ = self.create_subscription(String, 'output topic MCU - node_name', self.callback_Name, 10)
        self.subscriber_ = self.create_subscription(String, 'output topic MCU - node_health', self.callback_Health, 10)
        
        self.cli = self.create_client(Set_State, 'set_state')                   #create client for Set_State service
        while not self.cli.wait_for_service(timeout_sec=1.0):                   #wait until service is available
            self.get_logger().info('service not available, waiting again...')
        self.req1 = Set_State.Request()                                         #create request object
        
        self.cli = self.create_client(Set_Mode, 'set_mode')                     #do again for Set_Mode service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = Set_Mode.Request()
        
        self.setupPinModeCode = [0]     #Define the setup code as an array of values to pass into in the set_mode service
        self.forwardVelocity = None
        self.angularVelocity = None
        self.buttonArray = None
        self.requestArray = [None, None, None, None, None]
        self.theQue = queue.Queue()
    
    #Interperet the Twist_Plus to make a service request to the MCU package
        #On startup call the "Set Mode" service -- This will be a macro
        #Check to see if a button/macro is pressed/called
            #If it is the "cancel" button, clear the que and send a "stop" command
            #otherwise do the macro
                #continue to velocity translations unless macro says to wait
        #otherwise translate the linear.x (forwards velocity from -1 to 1?) and angular.z 
        #(rotational velocity from w_min to w_max) into Left and Right motor RPMs/desired velocities    
    def callback_Input(self, msg):
        
        #update self variables
        self.buttonArray = msg.buttons
        self.forwardVelocity = msg.linear.x
        self.angularVelocity = msg.angular.z

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
        self.requestArray = [left]
        self.theQue.put(self.requestArray)

        #if the que is not empty, make request
        if not self.theQue.empty():
            send_state_request(self, self.theQue.get())

        return 0

    #Create request method for State
    def send_state_request(self, requestInfo):   #can be more parameters, names should match expected in the services
        self.req1.leftRPM = requestInfo[0]
        self.req1.rightRPM = requestInfo[1]
        self.req1.bottomActuatorPosition = requestInfo[2]
        self.req1.topActuatorPosition = requestInfo[3]
        self.req1.time = requestInfo[5]
        return self.cli.call_async(self.req1)

    #Create request method for Set_Mode
    def send_mode_request(self):    #can be more parameters, names should match expected in the services
        self.req2.pinModeCode = self.setupPinModeCode
        return self.cli.call_async(self.req2)

    #and interperet/use that information
        #unknown what to do with that info at this time. ~Ethan
    def callback_State(self):
        return 0
    
    #and interperet/use that information
        #unknown what to do with that info at this time. ~Ethan
    def callback_Name(self):
        return 0
    
    #and interperet/use that information
        #unknown what to do with that info at this time. ~Ethan
    def callback_Health(self):
        return 0

#standard node main function
def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
