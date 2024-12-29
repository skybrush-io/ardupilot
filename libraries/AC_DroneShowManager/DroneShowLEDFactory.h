#pragma once

/// @file   AC_DroneShowLEDFactory.h
/// @brief  LED factory class that creates LED instances for the drone show manager module

#include "DroneShowLED.h"

// Supported LED types for the drone show manager
enum DroneShowLEDType {
    // No LED light output
    DroneShowLEDType_None = 0,

    // LED light color is sent in a DEBUG_VECT to a MAVLink channel
    DroneShowLEDType_MAVLink = 1,

    // LED light is to be forwarded to a NeoPixel LED strip
    DroneShowLEDType_NeoPixel = 2,

    // LED light is to be forwarded to a ProfiLED LED strip
    DroneShowLEDType_ProfiLED = 3,

    // Debug output; LED light RGB codes are sent as MAVLink STATUSTEXT messages
    DroneShowLEDType_Debug = 4,

    // LED light color is to be forwarded to the SITL simulator
    DroneShowLEDType_SITL = 5,

    // LED light is attached to servo channels
    DroneShowLEDType_Servo = 6,

    // LED light is driven over an I2C bus with 3 bytes per transfer (RGB)
    DroneShowLEDType_I2C = 7,

    // LED light is attached to servo channels with inverted polarity
    DroneShowLEDType_InvertedServo = 8,

    // WGDrones LED
    DroneShowLEDType_WGDrones = 9,

    // LED light is to be forwarded to a NeoPixel RGBW LED strip
    DroneShowLEDType_NeoPixel_RGBW = 10,

    // LED light is driven over an I2C bus with 4 bytes per transfer (RGBW)
    DroneShowLEDType_I2C_RGBW = 11,

    // LED light is driven by ArduPilot's AP_Notify framework
    DroneShowLEDType_Notify = 12,

    // LED light is attached to servo channels and respects configured min/max limits
    DroneShowLEDType_ServoWithLimits = 13,
};

class DroneShowLEDFactory
{
public:
    DroneShowLEDFactory();

    /* Do not allow copies */
    DroneShowLEDFactory(const DroneShowLEDFactory &other) = delete;
    DroneShowLEDFactory &operator=(const DroneShowLEDFactory&) = delete;

    /**
     * Creates a new DroneShowLED instance, given the LED type, the channel
     * index (if the LED type support multiple channels), and the number of
     * LEDs on this channel (for NeoPixel or ProfiLED strips).
     */
    DroneShowLED* new_rgb_led_by_type(
        DroneShowLEDType type, uint8_t channel, uint8_t num_leds
    );
};
