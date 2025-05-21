import rclpy
from rclpy.node import Node
import queue
import rclpy.timer
import rclpy.waitable
from std_msgs.msg import String
from interfaces.msg import TwistPlus

from keyboard_msgs.msg import Key

# from interfaces.srv import ODriveSetVelocity
from roi_ros.srv import ODriveSetVelocity
from roi_ros.srv import ActuatorSetVelocity
import time, threading

distance_between_wheels = 10  # meters

# What control_node should do:
# Listen to the output topic from mux_node which is a Twist_Plus() type message
# Twist_Plus includes:
# a linear vector with fields x, y, z
# an angular vector with fields x, y, z
# an array of booleans that represent controller button presses
# button array index to controller button list:
# 0 - A/Cross
# 1 - B/Circle
# 2 - X/Triangle
# 3 - Y/Square
# 4 - L1
# 5 - R1
# 6 - Select
# 7 - Start
# 8 - Home/P3
# 9 - Left Stick Press/L3
# 10 - Right Stick Press/R3
# etc.
# Interperet the Twist_Plus to make a service request to the MCU package
# On startup call the "Set Mode" service
# Check to see if a button/macro is pressed/called
# If it is the "cancel" button, clear the que and send a "stop" command
# otherwise do the macro
# continue to velocity translations unless macro says to wait
# otherwise translate the linear.x (forwards velocity from -1 to 1?) and angular.z
# (rotational velocity from w_min to w_max) into Left and Right motor RPMs/desired velocities

# Also listens to incomming topics from MCU
# namely, 'current_state', 'node_name', 'node_health'
# and interperet/use that information
# unknown what to do with that info at this time. ~Ethan


class ControlNode(Node):
    """
    Node for taking incoming requests and sending it to each part
    """

    # Initialize node and crease subscribers and clients
    def __init__(self):
        super().__init__("control_node")

        self.mux_subscriber = self.create_subscription(
            TwistPlus, "/input/twist_plus", self.twist_callback, 10
        )  # Rename 'macro_twist_plus in launch file
        # self.active_macros = self.create_subscription(ActiveMacros, 'active_macros', self.macro_callback, 10)

        self.keyboard_sub = self.create_subscription(Key, "keydown", self.keyboard_callback, 10)
        self.is_keyboard_movement = False

        self.oDrive1 = self.create_client(ODriveSetVelocity, "/oDrive1")
        self.oDrive2 = self.create_client(ODriveSetVelocity, "/oDrive2")
        self.oDrive3 = self.create_client(ODriveSetVelocity, "/oDrive3")
        self.oDrive4 = self.create_client(ODriveSetVelocity, "/oDrive4")

        self.actuator1 = self.create_client(ActuatorSetVelocity, "/actuator1")
        self.actuator2 = self.create_client(ActuatorSetVelocity, "/actuator2")

        self.forwardVelocity = None
        self.angularVelocity = None
        self.buttonArray = None
        self.requestArray = [None, None, None, None, None]
        self.theQue = queue.Queue()
        self.left_wheel_speed = None
        self.right_wheel_speed = None

        self.actuatorMessage1 = ActuatorSetVelocity.Request()
        self.actuatorMessage2 = ActuatorSetVelocity.Request()

        self.actuatorMessage1.sub_device_id = 0
        self.actuatorMessage2.sub_device_id = 1

        self.act1_update = 1
        self.act2_update = 1

        self.digMacroThread = threading.Thread(target=self.digMacro, daemon=True)
        self.dumpMacroThread = threading.Thread(target=self.dumpMacro, daemon=True)
        self.nNavThread = threading.Thread(target=self.nav_north, daemon=True)
        self.sNavThread = threading.Thread(target=self.nav_south, daemon=True)
        self.eNavThread = threading.Thread(target=self.nav_east, daemon=True)
        self.wNavThread = threading.Thread(target=self.nav_west, daemon=True)
        self.threeDigMacroThread = threading.Thread(target=self.three_dig_macro, daemon=True)
        self.keyboard_macro = threading.Thread(target=self.keyboard_run_macro, daemon=True)

    def calculateRPM(self):
        """
        source `ros.org`_.

        Calculate the RPM of the left and right wheels based on the linear and angular velocities
        .. _ros.org: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#differential-drive-robot
        """
        leftVelocity = self.forwardVelocity - (self.angularVelocity * distance_between_wheels / 2)
        rightVelocity = self.forwardVelocity + (self.angularVelocity * distance_between_wheels / 2)
        return leftVelocity, rightVelocity

    def keyboard_callback(self, msg):

        self.get_logger().info("Hello World")
        keyboard_code = msg.code

        wheel_speeds = [5.0, 5.0]

        key_translation_linear = {97: 1, 115: 2, 100: 3, 102: 4} # Time in seconds for moving forward
        key_translation_angular = {106: 1, 107: 2, 108: 3, 59: 4, 117: -1, 105: -2, 111: -3, 112: -4} # Time in seconds for rotating, negative counterclockwise

        distance_time = key_translation_linear.get(keyboard_code, 0)
        turn_time = key_translation_angular.get(keyboard_code, 0)

        if distance_time + turn_time == 0:
            return
        self.is_keyboard_movement = True

        if turn_time != 0:
            if turn_time > 0:
                wheel_speeds[1] *= -1
            else:
                wheel_speeds[0] *= -1
                turn_time *= -1
        
        self.left_wheel_speed = wheel_speeds[0]
        self.right_wheel_speed = wheel_speeds[1]

        self.keyboard_time = distance_time + turn_time

        if distance_time + turn_time > 0:
            if not self.keyboard_macro.is_alive():
                self.keyboard_macro = threading.Thread(target=self.keyboard_run_macro, daemon=True)
                self.keyboard_macro.start()
        
    def keyboard_run_macro(self):

        self.get_logger().info(str(self.keyboard_time))

        if not self.cancelSleep(self.keyboard_time):
            self.is_keyboard_movement = False
            return
        
        self.is_keyboard_movement = False
        
    # Sends service request to all 4 wheel servers
    def request_set_velocity(self):
        """
        Sends request to wheels based on control
        """

        left_message = ODriveSetVelocity.Request()
        right_message = ODriveSetVelocity.Request()

        # Set Left side speed and torque

        left_message.velocity = self.left_wheel_speed * 5 * -1
        left_message.torque_feedforward = 0.0

        # Set Right side speed and torque
        right_message.velocity = self.right_wheel_speed * 5
        right_message.torque_feedforward = 0.0

        # Send request to server, and don't hold up waiting for response
        # self.get_logger().info(str(left_message))
        left_future1 = self.oDrive1.call_async(left_message)
        right_future1 = self.oDrive2.call_async(right_message)

        right_message.velocity = self.right_wheel_speed * 5 * -1
        left_message.velocity = self.left_wheel_speed * 5 * -1

        right_future2 = self.oDrive4.call_async(right_message)
        left_future2 = self.oDrive3.call_async(left_message)

    def macro_callback(self, msg):
        return 0

    def twist_callback(self, msg):
        """
        Main update loop
        """
        self.buttonArray = msg.buttons
        self.forwardVelocity = msg.linear.x
        self.angularVelocity = msg.angular.z

        if not self.is_keyboard_movement and not self.digMacroThread.is_alive() and not self.dumpMacroThread.is_alive() and not self.nNavThread.is_alive() and not self.sNavThread.is_alive() and not self.wNavThread.is_alive() and not self.eNavThread.is_alive() and not self.threeDigMacroThread.is_alive():
            self.left_wheel_speed, self.right_wheel_speed = self.calculateRPM()
            self.request_set_velocity()

        if self.buttonArray.button_actuator_dig_cycle == 1:
            if not self.threeDigMacroThread.is_alive():
                self.threeDigMacroThread = threading.Thread(target=self.three_dig_macro, daemon=True)
                self.threeDigMacroThread.start()

        if self.buttonArray.button_actuator_dump_cycle == 1:
            if not self.dumpMacroThread.is_alive():
                self.dumpMacroThread = threading.Thread(target=self.dumpMacro, daemon=True)
                self.dumpMacroThread.start()

        if self.buttonArray.button_wheel_nav_north == 1:
            if not self.dumpMacroThread.is_alive():
                self.dumpMacroThread = threading.Thread(target=self.nav_north, daemon=True)
                self.dumpMacroThread.start()

        if self.buttonArray.button_wheel_nav_east == 1:
            if not self.dumpMacroThread.is_alive():
                self.dumpMacroThread = threading.Thread(target=self.nav_east, daemon=True)
                self.dumpMacroThread.start()

        if self.buttonArray.button_wheel_nav_south == 1:
            if not self.dumpMacroThread.is_alive():
                self.dumpMacroThread = threading.Thread(target=self.nav_south, daemon=True)
                self.dumpMacroThread.start()
        
        if self.buttonArray.button_wheel_nav_west == 1:
            if not self.dumpMacroThread.is_alive():
                self.dumpMacroThread = threading.Thread(target=self.nav_west, daemon=True)
                self.dumpMacroThread.start()

        if self.buttonArray.button_actuator_arm_up == 1:
            if self.actuatorMessage1.velocity != 100.0:
                self.act1_update = 1
                self.actuatorMessage1.velocity = 100.0

        elif self.buttonArray.button_actuator_arm_down == 1:
            if self.actuatorMessage1.velocity != -100.0:
                self.act1_update = 1
            self.actuatorMessage1.velocity = -100.0

        else:
            if self.actuatorMessage1.velocity != 0:
                self.act1_update = 1
            self.actuatorMessage1.velocity = 0.0

        if self.buttonArray.button_actuator_pitch_up == 1:
            if self.actuatorMessage2.velocity != 100.0:
                self.act2_update = 1
            self.actuatorMessage2.velocity = 100.0

        elif self.buttonArray.button_actuator_pitch_down == 1:
            if self.actuatorMessage2.velocity != -100.0:
                self.act2_update = 1
            self.actuatorMessage2.velocity = -100.0

        else:
            if self.actuatorMessage2.velocity != 0:
                self.act2_update = 1
            self.actuatorMessage2.velocity = 0.0

        if not self.digMacroThread.is_alive() and not self.dumpMacroThread.is_alive() and not self.threeDigMacroThread.is_alive():

            if self.act1_update == 1:
                act1_result = self.actuator1.call_async(self.actuatorMessage1)
                self.get_logger().info(str(self.actuatorMessage1) + " was message to actuator1")
                self.act1_update = 0

            if self.act2_update == 1:
                act2_result = self.actuator2.call_async(self.actuatorMessage2)
                self.get_logger().info(str(self.actuatorMessage2) + " was message to actuator2")
                self.act2_update = 0

    def digMacro(self, depth=0):
        """Dig Macro. Actuator 1 goes down, act 2 goes up, then slightly down.
        then wheels forward for x secs."""

        # NOTE: positive is up for both actuators, negative is down
        # NOTE: actuator1 is pitch and 2 is arm

        #
        ## home arm all the way up put bucket perpendicular to ground
        #
        self.act1(-100.0)
        self.act2(100.0)

        self.get_logger().info("started dig macro")

        if not self.cancelSleep(1):
            return

        #
        ## Plunge pitch into ground by lowering arm
        #
        self.act2(-100.0)

        if not self.cancelSleep(3):
            return
        
        #
        ## Bring arm up and lessen the pitch's angle of attack
        #
        self.act1(100.0)
        self.act2(100.0)

        if not self.cancelSleep(0.1 + depth * 0.1):
            return 
        
        self.act2(0)

        if not self.cancelSleep(0.35 + depth * 0.2):
            return

        #
        ## Move and dig
        #
        self.act1(0.0)

        self.left_wheel_speed, self.right_wheel_speed = 7.0, 7.0
        self.request_set_velocity()
        self.request_set_velocity()

        self.get_logger().info("stopping act, strating wheel")

        if not self.cancelSleep(3):
            return
        
        #
        ## Begin raising pitch for movement to dump zone, stop moving
        #
        self.act1(100.0)

        if not self.cancelSleep(2):
            return

        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()

        self.get_logger().info("stop wheel")

        #
        ## Raising arm
        #
        self.act2(100.0)

        if not self.cancelSleep(5):
            return

        self.get_logger().info("dig cycle done")

        self.dumpMacro()

    def three_dig_macro(self):
        self.digMacro(2)
        self.digMacro(2)
        self.digMacro(1)

    def dumpMacro(self):
        """Dump Macro. Wheels forward, then act 1 does 100, then back and forth, then wheels back."""

        self.left_wheel_speed, self.right_wheel_speed = 7.0, 7.0
        self.request_set_velocity()

        self.get_logger().info("started dump macro")

        if not self.cancelSleep(3):
            return

        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()

        self.get_logger().info("stop wheel")

        self.actuatorMessage1.velocity = -100.0
        act1_result = self.actuator1.call_async(self.actuatorMessage1)
        act1_result = self.actuator1.call_async(self.actuatorMessage1)

        if not self.cancelSleep(3):
            return

        self.get_logger().info("actuator up")

        self.actuatorMessage1.velocity = 100.0
        act1_result = self.actuator1.call_async(self.actuatorMessage1)
        act1_result = self.actuator1.call_async(self.actuatorMessage1)

        if not self.cancelSleep(1):
            return

        self.get_logger().info("actuator down")

        self.actuatorMessage1.velocity = -100.0

        act1_result = self.actuator1.call_async(self.actuatorMessage1)
        act1_result = self.actuator1.call_async(self.actuatorMessage1)

        if not self.cancelSleep(2):
            return

        self.get_logger().info("back up")
        self.actuatorMessage1.velocity = 0.0
        act1_result = self.actuator1.call_async(self.actuatorMessage1)
        act1_result = self.actuator1.call_async(self.actuatorMessage1)

        self.left_wheel_speed, self.right_wheel_speed = -7.0, -7.0
        self.request_set_velocity()
        self.request_set_velocity()
        self.get_logger().info("wheels back")
        if not self.cancelSleep(6.3):
            return
        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()
        self.get_logger().info("stop wheel")

    def nav_north(self):
        """Literal shot in the dark"""

        self.left_wheel_speed, self.right_wheel_speed = 5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        if not self.cancelSleep(45):
            return
        

        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()

        self.get_logger().info("done, welcome to the dig zone?")

    def nav_east(self):
        """Literal shot in the dark"""

        self.left_wheel_speed, self.right_wheel_speed = -5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        # turning
        if not self.cancelSleep(3.5):
            return


        self.left_wheel_speed, self.right_wheel_speed = 5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        if not self.cancelSleep(45):
            return


        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()
        
        self.get_logger().info("done, welcome to the dig zone?")

    def nav_west(self):
        """Literal shot in the dark"""

        self.left_wheel_speed, self.right_wheel_speed = 5.0, -5.0
        self.request_set_velocity()
        self.request_set_velocity()

        # turning
        if not self.cancelSleep(3.5):
            return


        self.left_wheel_speed, self.right_wheel_speed = 5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        if not self.cancelSleep(45):
            return


        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()
        
        self.get_logger().info("done, welcome to the dig zone?")


    def nav_south(self):
        """Literal shot in the dark"""

        self.left_wheel_speed, self.right_wheel_speed = -5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        # turning
        if not self.cancelSleep(7):
            return


        self.left_wheel_speed, self.right_wheel_speed = 5.0, 5.0
        self.request_set_velocity()
        self.request_set_velocity()

        if not self.cancelSleep(45):
            return


        self.left_wheel_speed, self.right_wheel_speed = 0.0, 0.0
        self.request_set_velocity()
        self.request_set_velocity()
        
        self.get_logger().info("done, welcome to the dig zone?")

    def cancelSleep(self, seconds):
        """
        Sleep for a given number of seconds, but check if the cancel button is pressed
        If it is, return early
        """
        self.get_logger().info(str(self.forwardVelocity))
        startTime = time.time()
        while rclpy.ok() and self.forwardVelocity == 0 and self.angularVelocity == 0:
            if time.time() - startTime > seconds:
                return True
            self.request_set_velocity()
            #self.actuator1.call_async(self.actuatorMessage1)
            #self.actuator2.call_async(self.actuatorMessage2)
            time.sleep(0.1)

        return False

    def act1(self, velocity = 0.0):
        self.actuatorMessage1.velocity = float(velocity)
        self.actuator1.call_async(self.actuatorMessage1)
        self.actuator1.call_async(self.actuatorMessage1)

    def act2(self, velocity = 0.0):
        self.actuatorMessage2.velocity = float(velocity)
        self.actuator2.call_async(self.actuatorMessage2)
        self.actuator2.call_async(self.actuatorMessage2)
        

# standard node main function
def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
