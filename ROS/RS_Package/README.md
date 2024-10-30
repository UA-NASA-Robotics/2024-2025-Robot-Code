# Remote Station Package
The Remote Station Package will take in input from a controller

## Topics
### `RS/twist_plus`, `Auto/twist_plus`, `Control/twist_plus`
**From:** RS/RS_Node, Auto/ , Queue/MUX_Node  
**Contains:** This topic contains the desired linear and angular velocity, as well as the buttons/macros. The RS comes from the RS Package, the Auto comes from the Autonomous Package, and Control is the twist coming from the MUX Node (i.e. which ever twist is the one we're listening to).

>Note: This needs to be filled out with the topics coming from MCU package

## Nodes
### MUX_Node
**Input:** `RS/twist_plus`, `Auto/twist_plus`  
**Output:** `Control/twist_plus`  
**Abstract:** The MUX node will choose whether we are listening from inputs from autonomous or RS. It will switch to the RS if any non-zero value is given, and only switch to auto if a command is given from the controller.

### Control_Node
**Input:** `Control/twist_plus`  
**Output:**  `Control/Set_Torque`, `Control/Set_Velocity`, `Control/Absolute_Position`, `Control/Relative_Position`  
**Abstract:** The Control Node will take macros/buttons/joystick-values from the twist_plus topic and break that down into discrete movements. These movements/macros will enter a queue where each movement is passed onto the MCU Package. The queue will be cleared/interrupted from any updated to the twist_plus topic. If no macros are selected, it will simply take the desired velocities/desire_position/torque and pass that to MCU Package.  
