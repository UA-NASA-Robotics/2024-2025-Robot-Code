# RS Package README

## Inputs/Outputs

### Published Topics

#### `/joy` (from `joy_node`) - Sensor Message

- **Message Type**: `sensor_msgs/Joy`
- **Description**: Publishes the current state of the joystick buttons and axes.
- **Key Fields**:
    - `axes`: List of floating-point values representing the positions of the joystick axes (e.g., left stick, right stick).
    - `buttons`: List of integers representing the state of the buttons (pressed = 1, not pressed = 0).

- **Example Data**:

    ```yaml
    axes: [0.0, 1.0, -0.5]  # Example for a joystick pushed fully forward and partially to the right
    buttons: [0, 0, 0, 0, 1]  # Example where the R1 button is pressed
    ```

- **Example Data Structures**:

    ```C++
    struct Joy {
        std::vector<float> axes;  // Positions of the joystick axes (e.g., sticks)
        std::vector<int32_t> buttons;  // States of the joystick buttons (pressed = 1, not pressed = 0)
    };
    ```

#### `/cmd_vel` (from `teleop_twist_joy_node`) - Geometry Message

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
      x: 0.0  # Not Used
      y: 0.0  # Not Used
      z: 1.0  # Rotate around Z-axis at 1 radian (counter-clockwise) per second
    ```

- **Example Data Structures**:

    ```C++
    struct Twist {
        Vector3 linear;  // Linear velocity in the x, y, and z directions
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

    twist_message.angular.x = 0.0;  // No roll
    twist_message.angular.y = 0.0;  // No pitch
    twist_message.angular.z = 1.0;  // Turn right at 1 rad/s
    ```

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
    - `config_file`: A YAML file that defines how joystick inputs map to robot movement. The `ps3.config.yaml` file contains configuration specific to the PS3 controller.
- **Remappings**:
    - `/cmd_vel`: Remapped to `/turtle1/cmd_vel` to ensure the velocity commands reach the Turtlesim node.
- **Output**: The node publishes `geometry_msgs/Twist` messages, which describe the linear and angular velocity of the robot (in this case, the turtle). These messages are sent to `/turtle1/cmd_vel` to control the movement of the turtle in Turtlesim.
