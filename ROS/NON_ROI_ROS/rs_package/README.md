# Remote Station Package (Human Control Input)

__Overall:__ for our competition we'd like to implement both human input (PS3 Controller) and autonomous driving to move the robot. "Remote Station" refers to the fact, that the PS3 controller will not be attached to the robot, but rather a laptop.

#### The RS Package is built from pre-written ROS packages so far, such as `joy` and `teleop_twist_joy`.

```
sudo apt update
sudo apt install ros-humble-topic-tools
```
### Running rs_package
Once built open up a terminal and type the following

```ros2 launch rs_package rs_launch.xml```  => runs joy, teleop_twist_joy

```ros2 launch rs_package rs_launch.xml use_turtlesim:=1``` => runs joyt, teleop_twist_joy, and turtlesim for testing

### Build Code: 

Launch files will look for the controller configuration file & run these pre-written packages to create the topics we will want to subscribe to in the Queue package.

#### Related build code is in `./setup.py` `./config/..` & `./launch/..` 

#### `./setup.py` => Package the folder so ROS & Python know where to look for things. 

#### `./launch/..` => Launch files (runs nodes) with parameters and configs.

#### `./config/..` => Config files for YAML, such as for the Joy controller config.

## Desired Movement (RS Package Output):

    ```C++
    // joy, published by RS package only
    struct Joy {
        std::vector<float> axes;        // Not used past priority node.
        std::vector<int32_t> buttons;   // Used throughout queue package.
    };
    
    // cmd_vel, published by RS & Autonomy
    struct Twist {
        Vector3 linear;  // Linear velocity in the x, y, and z directions
        Vector3 angular;  // Angular velocity (rotary) around the x, y, and z axes
    };
    ```

## ROS Community Packages:

#### `/joy` - Takes input from USB Joystick.

- **Description**: Publishes `sensor_msgs/joy` => the current state of the joystick buttons and axes.

    ```C++
    struct Joy {
        std::vector<float> axes;  // Positions of the joystick axes (e.g., sticks)
        std::vector<int32_t> buttons;  // States of the joystick buttons (pressed = 1, not pressed = 0)
    };
    ```

- **Run Command**: `ros2 run joy joy_node`

- **Key Fields**:
    - `axes`: List of floating-point values representing the positions of the joystick axes (e.g., left stick, right stick).

    ```bas    axes:
    0.0   # left stick,   left is positive
    0.0   # left stick,   up is positive
    1.0   # left trigger  depression
    0.0   # right stick,  left is positive
    0.0   # right stick,  up is positve
    1.0   # right trigger depression
    0.0   # d-pad horiz., left is positive
    0.0   # d-pad vert.,  up is positve
    ```

    - `buttons`: List of integers representing the state of the buttons (pressed = 1, not pressed = 0).

    ```bash
    buttons:
    0   # A/cross button 
    0   # B/circle button
    0   # X/square button
    0   # Y/triangle button
    0   # Left/R1 button
    0   # Right/R1 button
    0   # Select Button
    0   # Start Button
    0   # P3/Home Button
    0   # Left Stick/L3 Button
    0   # Right Stick/R3 Button
    ```

#### `/cmd_vel` (from `teleop_twist_joy_node`)

- **Message Type**: `geometry_msgs/Twist`
- **Description**: Publishes velocity commands for controlling the robot’s movement. This message contains linear and angular velocities in the X, Y, and Z directions.
- **Key Fields**:
    - `linear`: Represents the linear velocity of the robot in the X, Y, and Z directions.
    - `angular`: Represents the angular velocity (rotation) of the robot around the X, Y, and Z axes.
- **Remapped Topic**: The `/cmd_vel` topic is remapped to `/turtle1/cmd_vel` in the launch file, so Turtlesim listens for velocity commands on `/turtle1/cmd_vel`.

- **Each value can be scaled in the `/config/ps3.config.yaml` file.**

- **Example Data (cmd_vel):**

    ```yaml
    linear: 
      x: 2.0  # Move forward at a speed of 2.0 units per second
      y: 0.0  # Not Used
      z: 0.0  # Not Used
    angular: 
      roll: 0.0  # Not Used
      pitch: 0.0  # Not Used
      yaw: 1.0  # Rotate around Z-axis at 1 radian (counter-clockwise) per second
    ```

- **Example Data Structures**:

    ```C++
    struct Twist {
        Vector3 linear;   // Linear velocity in the x, y, and z directions
        Vector3 angular;  // Angular velocity (rotation) around the x, y, and z axes
    };

    struct Vector3 {
        float x;
        float y;
        float z;
    };

    Twist twist_message;
    twist_message.linear.x = 2.0;  // Move forward at 2 m/s
    twist_message.linear.y = 0.0;  // No movement sideways
    twist_message.linear.z = 0.0;  // No vertical movement

    twist_message.angular.roll = 0.0;  // No roll
    twist_message.angular.pitch = 0.0;  // No pitch
    twist_message.angular.yaw = 1.0;  // Turn right at 1 rad/s
    ```

### Angular Motion: Roll, Pitch, and Yaw

In 3D space, an object can rotate around three different axes:

- **Roll (X-axis rotation)**:  
  Rotation around the X-axis, which runs from front to back of the robot.  
  Imagine the robot tilting side-to-side, similar to how an airplane rolls when one wing dips down and the other rises.  
  
  In most ground-based robots, roll is typically not used.

- **Pitch (Y-axis rotation)**:  
  Rotation around the Y-axis, which runs from left to right of the robot.  
  This is like nodding your head up and down or the robot tilting forward and backward.  
  
  Like roll, pitch is not used for movement in most ground-based robots.

- **Yaw (Z-axis rotation)**:  
  Rotation around the Z-axis, which runs vertically through the robot.  
  Yaw is responsible for turning the robot left or right (rotation around its own center, like turning in place).  
  
  This is commonly used in robots to control turning or steering.

---

### Linear Motion (X, Y, Z)

Linear movement refers to motion along the three axes:

- **X (Forward/Backward)**: Movement forward and backward (along the X-axis).
- **Y (Side-to-side)**: Lateral movement left and right (along the Y-axis, rarely used in wheeled robots).
- **Z (Up/Down)**: Vertical movement (along the Z-axis, also rarely used in wheeled robots).


## Nodes Used

### 1. Turtlesim Node (for testing)

- **Package**: `turtlesim`
- **Executable**: `turtlesim_node`
- **Purpose**: The `turtlesim_node` is used for testing purposes to visualize the movement of a turtle in a 2D space. This node subscribes to velocity commands and moves the turtle accordingly.
- **Output**: The turtle’s movement is displayed on the screen. It subscribes to the topic `/turtle1/cmd_vel`, which is remapped from `/cmd_vel` by the `teleop_twist_joy_node`.

### 2. `joy_node`

- **Package**: `joy`
- **Executable**: `joy_node`
- **Purpose**: The `joy_node` is responsible for reading inputs from a joystick device (in this case, a PS3 controller) and publishing those inputs as ROS messages. It publishes on the topic `/joy`.
- **Parameters**:
    - `dev`: Specifies the device path of the joystick (e.g., `/dev/input/js0`).
- **Output**: The `joy_node` publishes a `sensor_msgs/Joy` message that contains the current state of the joystick’s axes and buttons. This message can be used by other nodes to control robot movement or other actions.

### 3. `teleop_twist_joy_node`

- **Package**: `teleop_twist_joy`
- **Executable**: `teleop_twist_joy_node`
- **Purpose**: The `teleop_twist_joy_node` converts joystick input into `Twist` messages. `Twist` messages are used to represent velocity commands for robots.
- **Parameters**:
    - `config_file`: A YAML file that defines how joystick inputs map to robot movement. The `controller.config.yaml` file contains configuration for the controller.
- **Output**: The node publishes `geometry_msgs/Twist` messages, which describe the linear and angular velocity of the robot (in this case, the turtle). These messages are sent to `/cmd_vel` to control the movement of the turtle in Turtlesim.