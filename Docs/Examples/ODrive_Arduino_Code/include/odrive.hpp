#pragma once

#include <Arduino.h>

#include <ODriveUART.h>
#include <SoftwareSerial.h>

// Sets up the ODrive
// Shouldn't need to change use this
// Other than startup
void odrive_setup(ODriveUART);