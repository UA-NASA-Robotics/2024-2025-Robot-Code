# 2024-2025 Robot Code
This repo contains the code for neccessary for the UA NASA Robotics' 2024-2025 robot's operation, meaning: all code run on the SBC (single board computers/jetson), all code run on the MCU's (arduinos), and all code run on the remote station (RS).

### File Structure
There are three main folder at top level: ROS, Docs, and Other. ROS holds the packages that use ROS to run (including the ROI program), Docs hold documents related to the operation of the Robot as well as example used while writing code, and Other holds miscellanous files.

### ROI
The ROI repo is installed as a submodule. When cloning the repository, run `git submodule init` and `git submodule update` to clone the latest ROI code into your workspace.

## ROS Diagram

![Image Didn't Load :(](./Docs/ROS_Flowchart.png "ROS Diagram")

Above is a Diagram created using rqt showing the flow of data through our architecture. 
> **NOTE:** The diagram does not include the services

1. **ROI Package:** This pacakge sends/receives data to/from the MCU's using ROI. This is what allows us to control the motors.
2. **Queue Package:** This package determines whether our movements are based off of autonomous inputs or human/RS inputs. It also holds a qeue for any multi movement commands. For example if someone was to press a button to lower the bucket and move forward, that sequence would be stored here.
3. **Autonomous:** This package as the name implies is the autonomy. Note that the actual package will have more than one node.
4. **RS Package:** The RS (or Remote Station) Package takes input from a user and parses them into the ROS dataflow. This will allow human control.