# Queue Package
The Queue Package takes in the topic `RS/twist_plus` from the RS Package and `Auto/twist_plus` from the Autonomous package. It then uses the `Queue/MUX_Node` to determine which topic should be pushed onto the `Queue/Control_Node`. The `Queue/Control` will prep the topics for the MCU Package, breaking down any buttons/macros into discrete actions

## Topics
### `RS/RS/twist_plus`, `Auto//twist_plus`, `Control/MUX/twist_plus`
**Contains:** This topic contains the desired linear and angular velocity, as well as the buttons/macros. The RS comes from the RS Package, the Auto comes from the Autonomous Package, and Control is the twist coming from the Queue/MUX_Node (this contains the twist_plus we are listening to).

### `MCU/ROI_Virtualization/pos_Out`
**Contains:** The position returned to the MCU Package from the O-Drive.

### `MCU/ROI_Virtualization/vel_Out`
**Contains:** The velocity of the motor returned to the MCU Package from the O-Drive.

### `MCU/ROI_Virtualization/torque_est

## Nodes
### MUX_Node
**Input:** `RS/twist_plus`, `Auto/twist_plus`  
**Output:** `Control/twist_plus`  
**Abstract:** The MUX node will choose whether we are listening from inputs from autonomous or RS. It will switch to the RS if any non-zero value is given, and only switch to auto if a command is given from the controller.

### Control_Node
**Input:** `Control/twist_plus`  
**Output:**  `Control/set_torque`, `Control/set_velocity`, `Control/abs_position`, `Control/rel_position`  
**Abstract:** The Control Node will take macros/buttons/joystick-values from the twist_plus topic and break that down into discrete movements. These movements/macros will enter a queue where each movement is passed onto the MCU Package. The queue will be cleared/interrupted from any updated to the twist_plus topic. If no macros are selected, it will simply take the desired velocities/desire_position/torque and pass that to MCU Package.  
