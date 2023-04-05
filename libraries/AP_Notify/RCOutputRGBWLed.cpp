/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RCOutputRGBWLed.h"

#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define LED_OFF 0
#define LED_FULL_BRIGHT 255
#define LED_MEDIUM ((LED_FULL_BRIGHT / 5) * 4)
#define LED_DIM ((LED_FULL_BRIGHT / 5) * 2)

RCOutputRGBWLed::RCOutputRGBWLed(uint8_t red_channel, uint8_t green_channel, uint8_t blue_channel, uint8_t white_channel)
    : RCOutputRGBWLed(red_channel, green_channel, blue_channel, white_channel, LED_OFF,
                     LED_FULL_BRIGHT, LED_MEDIUM, LED_DIM)
{
}

RCOutputRGBWLed::RCOutputRGBWLed(uint8_t red_channel, uint8_t green_channel,
                               uint8_t blue_channel, uint8_t white_channel, uint8_t led_off,
                               uint8_t led_full, uint8_t led_medium,
                               uint8_t led_dim)
    : RGBLed(led_off, led_full, led_medium, led_dim)
    , _red_channel(red_channel)
    , _green_channel(green_channel)
    , _blue_channel(blue_channel)
    , _white_channel(white_channel)
{
}

bool RCOutputRGBWLed::init()
{
    hal.rcout->enable_ch(_red_channel);
    hal.rcout->enable_ch(_green_channel);
    hal.rcout->enable_ch(_blue_channel);
    hal.rcout->enable_ch(_white_channel);

    return true;
}

uint16_t RCOutputRGBWLed::get_duty_cycle_for_color(const uint8_t color, const uint16_t usec_period) const
{
    return  usec_period * color / _led_bright;
}

uint16_t RCOutputRGBWLedInverted::get_duty_cycle_for_color(const uint8_t color, const uint16_t usec_period) const
{
    return  usec_period * (255 - color) / _led_bright;
}

struct hsv {
    float h, s, v; 
};

struct hsi {
    float h, s, i; 
};

struct rgbw {
    int r, g, b, w;
};

static float constrain(float val, float min, float max) {
    float ret = val;
    if(val <= min) {
        ret = min;
    }
    if(val >= max) {
        ret = max;
    }
    return ret;
}

static hsi rgb_to_hsi(int r, int g, int b) {
    float _r = constrain((float)r / 255.0, 0.0, 1.0);
    float _g = constrain((float)g / 255.0, 0.0, 1.0);
    float _b = constrain((float)b / 255.0, 0.0, 1.0);
    float intensity = 0.33333 * (_r + _g + _b);
    float M = std::max(_r, std::max(_g, _b));
    float m = std::min(_r, std::min(_g, _b));

    float saturation = 0.0;
    if(intensity == 0.0) {
        saturation = 0.0;
    }else {
        saturation = 1.0 - (m / intensity);
    }

    float hue = 0.0;
    if(M == m) {
        hue = 0.0;
    }
    if(M == _r) {
        if(M == m) {
            hue = 0.0;
        }else {
            hue = 60.0 * (0.0 + ((_g - _b) / (M - m)));
        }
    }
    if(M == _g) {
        if(M == m) {
            hue = 0.0;
        }else {
            hue = 60.0 * (2.0 + ((_b - _r) / (M - m)));
        }
    }
    if(M == _b) {
        if(M == m) {
            hue = 0.0;
        }else {
            hue = 60.0 * (4.0 + ((_r - _g) / (M - m)));
        }
    }
    if(hue < 0.0) {
        hue = hue + 360.0;
    }

    hsi ret;
    ret.h = hue;
    ret.s = std::abs(saturation);
    ret.i = intensity;

    return ret;
}

static rgbw hsi_to_rgbw(hsi value) {
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
    float w = 0.0;
    float cos_h = 0.0;
    float cos_1047_h = 0.0;
    float H = std::fmod(value.h, 360.0);
    H = 3.14159 * H / 180.0;
    float S = constrain(value.s, 0.0, 1.0);
    float I = constrain(value.i, 0.0, 1.0);

    if(H < 2.09439) {
        cos_h = std::cos(H);
        cos_1047_h = std::cos(1.047196667 - H);
        r = S * 255.0 * I / 3.0 * (1.0 + cos_h / cos_1047_h);
        g = S * 255.0 * I / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        b = 0.0;
        w = 255.0 * (1.0 - S) * I;
    } else if(H < 4.188787) {
        H = H - 2.09439;
        cos_h = std::cos(H);
        cos_1047_h = std::cos(1.047196667 - H);
        g = S * 255.0 * I / 3.0 * (1.0 + cos_h / cos_1047_h);
        b = S * 255.0 * I / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        r = 0.0;
        w = 255.0 * (1.0 - S) * I;
    } else {
        H = H - 4.188787;
        cos_h = std::cos(H);
        cos_1047_h = std::cos(1.047196667 - H);
        b = S * 255.0 * I / 3.0 * (1.0 + cos_h / cos_1047_h);
        r = S * 255.0 * I / 3.0 * (1.0 + (1.0 - cos_h / cos_1047_h));
        g = 0.0;
        w = 255.0 * (1.0 - S) * I;
    }
    rgbw ret;
    ret.r = (int) constrain(r * 3, 0, 255);
    ret.g = (int) constrain(g * 3, 0, 255);
    ret.b = (int) constrain(b * 3, 0, 255);
    ret.w = (int) constrain(w, 0, 255);
    return ret;
}

static rgbw rgb_to_rgbw(int Ri, int Gi, int Bi) {
    // hal.console->printf("RGB(%d, %d, %d)\n", Ri, Gi, Bi);

    hsi b = rgb_to_hsi(Ri, Gi, Bi);
    rgbw c = hsi_to_rgbw(b);

    // hal.console->printf("RGBW(%d, %d, %d, %d)\n", c.r, c.g, c.b, c.w);
    return c;
}

bool RCOutputRGBWLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t freq_motor = hal.rcout->get_freq(0);
    const uint16_t freq = hal.rcout->get_freq(_red_channel);
    const uint16_t usec_period = hz_to_usec(freq);

    if (freq_motor != freq) {
        /*
         * keep at same frequency as the first RCOutput channel, some RCOutput
         * drivers can not operate in different frequency between channels
         */
        const uint32_t mask = 1 << _red_channel | 1 << _green_channel
                              | 1 << _blue_channel;
        hal.rcout->set_freq(mask, freq_motor);
    }

    rgbw color = rgb_to_rgbw(red, green, blue);

    uint16_t usec_duty = get_duty_cycle_for_color(color.r, usec_period);
    SRV_Channels::set_output_pwm_chan(_red_channel, usec_duty);

    usec_duty = get_duty_cycle_for_color(color.g, usec_period);
    SRV_Channels::set_output_pwm_chan(_green_channel, usec_duty);

    usec_duty = get_duty_cycle_for_color(color.b, usec_period);
    SRV_Channels::set_output_pwm_chan(_blue_channel, usec_duty);

    usec_duty = get_duty_cycle_for_color(color.w, usec_period);
    SRV_Channels::set_output_pwm_chan(_white_channel, usec_duty);

    return true;
}
