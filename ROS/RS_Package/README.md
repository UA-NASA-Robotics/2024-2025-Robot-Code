# Remote Station Package
The Remote Station Package will take in input from a controller and then convert those inputs into an angular and linear velocity.

## Topics
### `RS/twist_plus`
**From:** RS/RS_Node,
**Contains:** This topic contains the desired linear and angular velocity, as well as the buttons/macros.

>Note: This needs to be filled out with the topics coming from MCU package

## Nodes
### RS Node
**Input:** Values from a Joystick
**Output:** `RS/twist_plus`  
**Abstract:** This node inputs values using the JOY package, this then is converted into `RS/twist_plus`. The math is fairly simple and can be found in the [ROS documentation](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html) under the differential drive section. 
