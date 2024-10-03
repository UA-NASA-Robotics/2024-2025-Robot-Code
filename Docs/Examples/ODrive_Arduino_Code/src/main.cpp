#include <Arduino.h>

#include <ODriveUART.h>
#include <SoftwareSerial.h>

#include "odrive.hpp"
#include "command_transport.hpp"

// Set pins to 8 and 9 for SoftwareSerial to ODrive
// Also define the baudrate
SoftwareSerial odrive_serial(8, 9);
unsigned long baudrate = 19200;

// Create an ODrive instance
ODriveUART odrive(odrive_serial);

void setup() {
  // Start serial to computer
  Serial.begin(115200);

  // Start serial to ODrive
  odrive_serial.begin(baudrate);

  // Wait for serial to start
  while (!Serial) {}

  delay(10);

  // Setup the ODrive
  odrive_setup(odrive);
}

void loop() {
  // Check if a command is available
  if (command_available()) {
    // Get the command
    String command = get_command();
    Serial.println("Received: " + command);

    // Send the command to the ODrive
    send_command_to_odrive(odrive, command);
  }
}