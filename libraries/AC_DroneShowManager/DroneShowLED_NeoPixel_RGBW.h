#pragma once

/// @file   AC_DroneShowLED_NeoPixel_RGBW.h
/// @brief  Drone show LED that sends its output to a NeoPixel RGBW LED strip.

#include <AP_Common/AP_Common.h>

#include "DroneShowLED.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED to a
 * NeoPixel RGBW LED strip.
 *
 * The implementation assumes that the wire ordering of the components is RGBW.
 */
class DroneShowLED_NeoPixel_RGBW : public DroneShowLED {
public:
    DroneShowLED_NeoPixel_RGBW(uint8_t chan = 1, uint8_t num_leds = 16);

    /* Do not allow copies */
    DroneShowLED_NeoPixel_RGBW(const DroneShowLED_NeoPixel_RGBW &other) = delete;
    DroneShowLED_NeoPixel_RGBW &operator=(const DroneShowLED_NeoPixel_RGBW&) = delete;

protected:
    bool init(void) override;
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;
    bool supports_white_channel() override { return true; }

private:
    uint8_t _chan;
    uint8_t _num_leds;
    uint8_t _virtual_num_leds;
};
