#pragma once

/// @file   AC_DroneShowLED_Notify.h
/// @brief  Drone show LED that sends its output to ArduPilot's AP_Notify library.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"
#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * RGB LED implementation that sends its output to ArduPilot's AP_Notify library.
 * Users should set NTF_LED_OVERRIDE to "Scripting" to see the colors sent here
 * on the notification LED of the drone.
 */
class DroneShowLED_Notify : public DroneShowLED {
public:
    DroneShowLED_Notify() : DroneShowLED() {};

protected:
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;
    bool supports_white_channel() override { return false; }
};
