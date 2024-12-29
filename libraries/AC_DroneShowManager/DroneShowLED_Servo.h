#pragma once

/// @file   AC_DroneShowLED_RGB.h
/// @brief  Drone show LED that sends its output to servo channels.

#include <AP_HAL/utility/OwnPtr.h>
#include "DroneShowLED.h"


class DroneShowLED_Servo : public DroneShowLED {

public:
    DroneShowLED_Servo(
        uint8_t red_channel, uint8_t green_channel,
        uint8_t blue_channel, uint8_t white_channel = 255,
        bool inverted = false, bool use_servo_limits = false
    );
    ~DroneShowLED_Servo() {};

    /* Do not allow copies */
    DroneShowLED_Servo(const DroneShowLED_Servo &other) = delete;
    DroneShowLED_Servo &operator=(const DroneShowLED_Servo&) = delete;

protected:
    bool init(void) override;
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;
    bool supports_white_channel() override { return _white_channel < 32; }

private:
    uint8_t _red_channel;
    uint8_t _green_channel;
    uint8_t _blue_channel;
    uint8_t _white_channel;
    bool _inverted;
    bool _use_servo_limits;  // Whether to use servo min/max limits
};
