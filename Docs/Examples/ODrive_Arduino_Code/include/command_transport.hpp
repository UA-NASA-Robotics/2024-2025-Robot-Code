#pragma once

#include <Arduino.h>

#include <map>
#include <vector>

#include <ODriveUART.h>
#include <SoftwareSerial.h>

/*
 *  These maps will contain a character identifier
 *  This character will point to a function for the first map
 *  and a vector of integers for the second map
 *  Defined in the source file
 */
extern std::map<char, void (ODriveUART::*)(float)> function_map;
extern std::map<char, std::vector<int>> parameter_map;

// Thse are the functions that are needed for making the program modular

// Tests if a command is available
// returns true if a command is available
bool command_available();

// Gets the command from the serial port
// return value will contain a character inside the maps above
//      it will also contain the parameters for the function
String get_command();

// Sends a command to the odrive
// Shoudln't need to be changed at all
// If a command is added, add it to the maps above
void send_command_to_odrive(ODriveUART odrive, String command);