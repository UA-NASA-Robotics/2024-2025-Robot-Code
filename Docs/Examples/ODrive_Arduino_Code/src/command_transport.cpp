#include "command_transport.hpp"

// Add lines to both maps for each new command
std::map<char, void (ODriveUART::*)(float)> function_map = {
  {'p', &ODriveUART::setPosition},
  {'v', &ODriveUART::setVelocity},
  {'t', &ODriveUART::setTorque}
};

// If keys don't line up, look at blame
std::map<char, std::vector<int>> parameter_map = {
  {'p', {3, INPUT_MODE_TRAP_TRAJ}},
  {'v', {2, INPUT_MODE_VEL_RAMP}},
  {'t', {1, INPUT_MODE_TORQUE_RAMP}}
};

// Change for actual implementation
bool command_available() {
  return Serial.available() > 0;
}

// Change for actual implementation
String get_command() {
  String input = Serial.readStringUntil('\0');

  return input;
}

void send_command_to_odrive(ODriveUART odrive, String command) {
  // Start off by setting the funtion to nothing along with all parameters
  void (ODriveUART::*controlFunction)(float) = nullptr;
  int control_mode = -1;
  int input_mode = -1;

  // bool motorCommand = false;

  // Get the key from the command
  char key = command[0];

  // If the key is in the map
  if (function_map.find(key) != function_map.end()) {
    // Set the function and parameters
    controlFunction = function_map[key];
    control_mode = parameter_map[key][0];
    input_mode = parameter_map[key][1];
    float value = command.substring(1).toFloat();

    // motorCommand = true;

    // Set the parameters on the ODrive
    odrive.setParameter("axis0.controller.config.control_mode", control_mode);
    odrive.setParameter("axis0.controller.config.input_mode", input_mode);

    // Call the function with the data value
    // Possibly add a possiblility for two parameters
    (odrive.*controlFunction)(value);
    Serial.println("\tSetting " + String(control_mode) + " to " + String(value));
  } else {
    // Command is not in the map
    // Print error
    Serial.println("Command not found");
  }
}