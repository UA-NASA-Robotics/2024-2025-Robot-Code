# Queue Package
The Queue Package takes in the topic `RS/twist_plus` from the RS Package and `Auto/twist_plus` from the Autonomous package. It then uses the **Priority Node** to determine which topic should be pushed onto the **Control Queue Node**. The **Control Queue Node** will prep the topics for the MCU Package, breaking down any buttons/macros into discrete movements.

## Topics
#### `RS/twist_plus`, `Auto/twist_plus`, `Control/twist_plus` 
**Contains:** This topic contains the desired linear and angular velocity, as well as the buttons/macros. The RS comes from the RS Package, the Auto comes from the Autonomous Package, and Control is the twist coming from the MUX Node (i.e. which ever twist is the one we're listening to).

>Note: This needs to be filled out with the topics coming from MCU package

## Nodes
### MUX Node
**Input:** `RS/twist_plus`, `Auto/twist_plus`  
**Output:** `Control/twist_plus`  
**Abstract:** The MUX node will choose whether we are listening from inputs from autonomous or RS. It will switch to the RS if any non-zero value is given, and only switch to auto if a command is given from the controller.

### Control Node
**Input:** `Control/twist_plus`  
**Output:**  
**Abstract:** The Control Node will take macros/buttons/joystick-values from the twist_plus topic and break that down into discrete movements. These movements/macros will enter a queue where each movement is passed onto the MCU Package. The queue will be clear/interrupted from any updated to the twist_plus topic. If no macros are selected, it will simply take the desired velocities and pass that to MCU Package.
