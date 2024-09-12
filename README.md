# 2024-2025 Robot Code
This repo contains the code for neccessary for the UA NASA Robotics' 2024-2025 robot's operation, meaning: all code run on the SBC (single board computers/jetson), all code run on the MCU's (arduinos), and all code run on the remote station (RS).

### File Structure
There are two main folder at top level: ROS, and Other. ROS holds the packages and various other ROS focus programs, so stuff running on the SBC and RS. The other 'Other' is used for code not pretaining to ROS, but still critical to robot operation. An example would be the code running on the arduinos. An example of what would *not* go here is code for the test robot, or website.  