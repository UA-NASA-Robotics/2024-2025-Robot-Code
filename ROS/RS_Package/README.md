# Remote Station Package
The Remote Station (RS) Package will take in input from a controller and then convert those inputs into an angular and linear velocity as well as button/macro inputs into the `RS/RS/twist_plus` topic. This package relies heavily on the [ROS Joy Package](https://wiki.ros.org/joy).

## Topics
### `RS/RS/twist_plus`
**Contains:** This topic contains the desired linear and angular velocity, as well as the buttons/macros.

## Nodes
### RS Node
**Input:** Values from a Joystick
**Output:** `RS/RS/twist_plus`  
**Abstract:** This node inputs values using the JOY package, this then is converted into `RS/twist_plus`. The math is fairly simple and can be found in the [ROS documentation](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html) under the differential drive section. 
