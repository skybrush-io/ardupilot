#pragma once

#include "RGBLed.h"

class RCOutputRGBWLed: public RGBLed {
public:
    RCOutputRGBWLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel, uint8_t white_channel, uint8_t led_off, uint8_t led_full,
                   uint8_t led_medium, uint8_t led_dim);
    RCOutputRGBWLed(uint8_t red_channel, uint8_t green_channel,
                   uint8_t blue_channel, uint8_t white_channel);
    bool init() override;

protected:
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override;
    virtual uint16_t get_duty_cycle_for_color(const uint8_t color, const uint16_t usec_period) const;

private:
    uint8_t _red_channel;
    uint8_t _green_channel;
    uint8_t _blue_channel;
    uint8_t _white_channel;
};

class RCOutputRGBWLedInverted : public RCOutputRGBWLed {
public:
    RCOutputRGBWLedInverted(uint8_t red_channel, uint8_t green_channel, uint8_t blue_channel, uint8_t white_channel)
        : RCOutputRGBWLed(red_channel, green_channel, blue_channel, white_channel)
    { }
protected:
    virtual uint16_t get_duty_cycle_for_color(const uint8_t color, const uint16_t usec_period) const override;
};

class RCOutputRGBWLedOff : public RCOutputRGBWLed {
public:
    RCOutputRGBWLedOff(uint8_t red_channel, uint8_t green_channel,
                      uint8_t blue_channel, uint8_t white_channel, uint8_t led_off)
        : RCOutputRGBWLed(red_channel, green_channel, blue_channel, white_channel,
                         led_off, led_off, led_off, led_off)
    { }

    /* Override the hw_set_rgb method to turn leds off regardless of the
     * values passed */
    bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override
    {
        return RCOutputRGBWLed::hw_set_rgb(_led_off, _led_off, _led_off);
    }
};
