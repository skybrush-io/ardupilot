#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowLED_Servo.h"

extern const AP_HAL::HAL& hal;

static uint16_t get_duty_cycle_for_color(
    const uint8_t color, const uint16_t usec_period, uint8_t channel,
    bool use_servo_limits, bool off_is_zero
);
static uint16_t get_duty_cycle_for_color_with_limits(
    const uint8_t color, const uint16_t usec_period, uint8_t channel,
    bool off_is_zero
);
static uint16_t get_duty_cycle_for_color_without_limits(
    const uint8_t color, const uint16_t usec_period, uint8_t channel
);

DroneShowLED_Servo::DroneShowLED_Servo(
    uint8_t red_channel, uint8_t green_channel, uint8_t blue_channel,
    uint8_t white_channel, bool inverted, bool use_servo_limits,
    bool off_is_zero
) :
    _red_channel(red_channel), _green_channel(green_channel),
    _blue_channel(blue_channel), _white_channel(white_channel),
    _inverted(inverted), _use_servo_limits(use_servo_limits),
    _off_is_zero(off_is_zero)
{
}

bool DroneShowLED_Servo::init()
{
    hal.rcout->enable_ch(_red_channel);
    hal.rcout->enable_ch(_green_channel);
    hal.rcout->enable_ch(_blue_channel);
    
    if (supports_white_channel()) {
        hal.rcout->enable_ch(_white_channel);
    }

    return true;
}

bool DroneShowLED_Servo::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    if (_inverted) {
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
        white = 255 - white;
    }

    /* This section of the code was taken from RCOutputRGBLed.cpp */
    const uint16_t freq_motor = hal.rcout->get_freq(0);
    const uint16_t freq = hal.rcout->get_freq(_red_channel);
    const uint16_t usec_period = hz_to_usec(freq);

    if (freq_motor != freq) {
        /*
         * keep at same frequency as the first RCOutput channel, some RCOutput
         * drivers can not operate in different frequency between channels
         */
        uint32_t mask = (1 << _red_channel) | (1 << _green_channel) | (1 << _blue_channel);
        if (supports_white_channel()) {
            mask |= (1 << _white_channel);
        }

        hal.rcout->set_freq(mask, freq_motor);
    }

    uint16_t red_duty = get_duty_cycle_for_color(red, usec_period, _red_channel, _use_servo_limits, _off_is_zero);
    uint16_t green_duty = get_duty_cycle_for_color(green, usec_period, _green_channel, _use_servo_limits, _off_is_zero);
    uint16_t blue_duty = get_duty_cycle_for_color(blue, usec_period, _blue_channel, _use_servo_limits, _off_is_zero);
    uint16_t white_duty = 0;
    if (supports_white_channel()) {
        white_duty = get_duty_cycle_for_color(white, usec_period, _white_channel, _use_servo_limits, _off_is_zero);
    }

    SRV_Channels::set_output_pwm_chan(_red_channel, red_duty);
    SRV_Channels::set_output_pwm_chan(_green_channel, green_duty);
    SRV_Channels::set_output_pwm_chan(_blue_channel, blue_duty);
    if (supports_white_channel()) {
        SRV_Channels::set_output_pwm_chan(_white_channel, white_duty);
    }

    return true;
}

static uint16_t get_duty_cycle_for_color(const uint8_t color, const uint16_t usec_period, uint8_t channel, bool use_limits, bool off_is_zero)
{
    return use_limits
        ? get_duty_cycle_for_color_with_limits(color, usec_period, channel, off_is_zero)
        : get_duty_cycle_for_color_without_limits(color, usec_period, channel);
}

static uint16_t get_duty_cycle_for_color_without_limits(const uint8_t color, const uint16_t usec_period, uint8_t channel)
{
    uint16_t result = usec_period * color / 255;
    /* We cannot use zero duty cycle because some LED drivers treat this as
     * "no PWM signal". Since LEDs do not tend to turn on at low duty cycles
     * we can safely use a pulse length of 1 usec instead of 0 usec if we want
     * the LED to be off */
    return result < 1 ? 1 : result > 2450 ? 2450 : result;
}

static uint16_t get_duty_cycle_for_color_with_limits(const uint8_t color, const uint16_t usec_period, uint8_t channel, bool off_is_zero)
{
    const SRV_Channel* srv_chan = SRV_Channels::srv_channel(channel);
    if (srv_chan == nullptr) {
        // fallback to original
        return get_duty_cycle_for_color_without_limits(color, usec_period, channel);
    }

    const uint16_t min_pwm = srv_chan->get_output_min();
    const uint16_t max_pwm = srv_chan->get_output_max();

    if (off_is_zero) {
        if (color == 0) {
            return 0;
        } else {
            return min_pwm + ((max_pwm - min_pwm) * (color - 1)) / 255;
        }
    } else {
        return min_pwm + ((max_pwm - min_pwm) * color) / 255;
    }
}
