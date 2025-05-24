#pragma once

/// @file   AC_DroneShowLED_MAVLink.h
/// @brief  Drone show LED that sends its output via MAVLink messages to an external component.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"
#include "include/mavlink/v2.0/mavlink_types.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED as a
 * DEBUG_VECT message on a MAVLink channel.
 */
class DroneShowLED_MAVLink : public DroneShowLED {
public:
    DroneShowLED_MAVLink(uint8_t instance = 0);

    /* Do not allow copies */
    DroneShowLED_MAVLink(const DroneShowLED_MAVLink &other) = delete;
    DroneShowLED_MAVLink &operator=(const DroneShowLED_MAVLink&) = delete;

protected:
    bool init(void) override;
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;

private:
    mavlink_channel_t _chan;
    uint8_t _instance;
};
