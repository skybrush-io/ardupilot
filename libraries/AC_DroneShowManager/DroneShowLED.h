#pragma once

/// @file   AC_DroneShowLED.h
/// @brief  Abstract LED that is used as a color channel output in a drone show

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class DroneShowLED
{
private:
    /**
     * Current gamma correction exponent of the LED.
     */
    float _gamma;

    /**
     * Current minimum brightness threshold of the LED as a fractional number.
     */
    float _min_brightness;

    /**
     * Current minimum brightness threshold of the LED, scaled to the 0-255 range.
     */
    uint8_t _min_brightness_scaled;

    /**
     * Gamma correction lookup table that maps uncorrected RGB components to
     * gamma-corrected values.
     */
    uint8_t _gamma_lookup_table[256];

    /**
     * Last RGB components that were sent on this LED.
     */
    uint8_t _last_red, _last_green, _last_blue, _last_white;

    /**
     * Number of times we need to repeat LED commands.
     */
    uint8_t _repeat_count;

    /**
     * Number of times we still need to repeat the last LED command.
     */
    uint8_t _repeat_count_left;

public:
    DroneShowLED() :
        _gamma(0.0f), _min_brightness(0.0f), _last_red(0), _last_green(0), _last_blue(0), _last_white(0),
        _repeat_count(0), _repeat_count_left(0)
    {
        set_gamma(1.0f);
        set_repeat_count(1);
    };
    virtual ~DroneShowLED() {};

    /**
     * Initializes the LED.
     */
    virtual bool init() { return true; };

    /**
     * Sets the gamma correction exponent of the LED.
     */
    void set_gamma(float value) {
        if (is_equal(value, _gamma)) {
            return;
        }

        _gamma = value;

        _update_gamma_lookup_table();
        _reset_repeat_count();
    }

    /**
     * Sets the minimum brightness threshold of the LED.
     */
    void set_min_brightness(float value) {
        _min_brightness = value;
        _min_brightness_scaled = value < 0 ? 0 : value < 1 ? static_cast<uint8_t>(value * 255.0f) : 255;
    }

    /**
     * Sets the repeat count of the LED, i.e. the number of times a color setting
     * command should be repeated.
     */
    void set_repeat_count(uint8_t value) {
        if (value < 1) {
            value = 1;
        }

        if (value != _repeat_count) {
            _repeat_count = value;
            _reset_repeat_count();
        }
    }

    /**
     * Sets the color of the LED in RGB space. The value of the white channel
     * is assumed to be zero.
     * 
     * Red, green and blue channel values used in this function are _before_
     * gamma correction. The function will apply gamma correction on its own.
     */
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
        set_rgbw(red, green, blue, 0);
    }

    /**
     * Sets the color of the LED in RGBW space.
     * 
     * Red, green, blue and white channel values used in this function are
     * _before_ gamma correction. The function will apply gamma correction on
     * its own.
     */
    void set_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
        if (red != _last_red || green != _last_green || blue != _last_blue || white != _last_white) {
            _last_red = red;
            _last_green = green;
            _last_blue = blue;
            _last_white = white;
            _reset_repeat_count();
        }

        repeat_last_command_if_needed();
    }

    /**
     * Returns whether the LED supports the W channel. LEDs that do not support
     * the W channel will ignore whatever value is sent to them on the white
     * channel. It is the responsibility of the caller to ensure that the W
     * channel is not used if this function returns false.
     */
    virtual bool supports_white_channel() { return false; }

    /**
     * Repeats the last command to set the color of the RGB LED if needed.
     */
    void repeat_last_command_if_needed() {
        uint8_t red, green, blue, white;

        if (_repeat_count_left == 0) {
            return;
        }

        // Calculate overall brightness as maximum of all channels
        uint8_t max_brightness = MAX(
            MAX(_last_red, _last_green),
            MAX(_last_blue, _last_white)
        );

        // If overall brightness is below threshold, turn off all channels
        if (max_brightness < _min_brightness_scaled) {
            red = green = blue = white = 0;
        } else {
            red = _last_red;
            green = _last_green;
            blue = _last_blue;
            white = _last_white;
        }

        if (set_raw_rgbw(red, green, blue, white)) {
            _repeat_count_left--;
        }
    }

protected:
    /**
     * Sets the raw color of the LED in RGBW space.
     * 
     * Red, green, blue and white channel values used in this function are
     * _after_ gamma correction. The function will simply forward them to the
     * appropriate output device.
     * 
     * Returns whether the new color was set successfully.
     */
    virtual bool set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) = 0;

private:
    /**
     * Resets the repeat count to its maximum value.
     */
    void _reset_repeat_count() {
        _repeat_count_left = _repeat_count;
    }

    /**
     * Updates the gamma correction lookup table when the gamma exponent changes.
     */
    void _update_gamma_lookup_table();
};
