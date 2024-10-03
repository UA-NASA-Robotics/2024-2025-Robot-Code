# What is this
This folder contains code that can be referenced for development on the Arduinos that control the ODrive that controls the motors attached to the wheels. 

This package was made with the VSCode extension PlatformIO - this helps with building and uploading to a micro-controller

At the moment, the arduino will take in data from the serial connection to the computer, interpret the command and send the according command to the ODrive. 

## Commands
Commands will be a letter followed by a number such as `p50` or `v 5`

Possible letter commands are 
- `p`
    - Position
    - Go to a specific rotation number (p 1 spins once from startup)
- `v`
    - Velocity
    - Set to specific rotations per second (v 50 is 50 rotations per second)
- `t`
    - Torque
    - Spin the motor at a torque and will increase the speed of the rotor or decrease in order to maintain a specific torque output