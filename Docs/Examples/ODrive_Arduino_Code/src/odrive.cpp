
#include "odrive.hpp"

void odrive_setup(ODriveUART odrive) {
    Serial.println("Waiting for ODrive to start...");
    // while (odrive.getState() == AXIS_STATE_UNDEFINED) {
    //     delay(100);
    // }

    // Serial.println("ODrive found");

    // Serial.print("DC voltage: ");
    // Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

    // Serial.println("Enabling closed loop control...");
    // while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    //     odrive.clearErrors();
    //     odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    //     delay(100);
    // }

    Serial.println("ODrive ready");
}