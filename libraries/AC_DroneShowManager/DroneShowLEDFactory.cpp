#include "DroneShowLEDFactory.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/RCOutputRGBLed.h>
#include <AP_Notify/RCOutputRGBWLed.h>
#include <AP_Notify/SITL_SFML_LED.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowLED_Debug.h"
#include "DroneShowLED_I2C.h"
#include "DroneShowLED_MAVLink.h"
#include "DroneShowLED_RGB.h"
#include "DroneShowLED_SerialLED.h"
#include "DroneShowLED_UART_WGDrones.h"

/// Default constructor.
DroneShowLEDFactory::DroneShowLEDFactory()
{
}

DroneShowLED* DroneShowLEDFactory::new_rgb_led_by_type(
    DroneShowLEDType type, uint8_t channel, uint8_t num_leds, float gamma
) {
    uint8_t chan_white, chan_red, chan_green, chan_blue;
    RGBLed* rgb_led = NULL;

    DroneShowLED* result = NULL;

    switch (type) {
        case DroneShowLEDType_MAVLink:
            if (channel < 4) {
                result = new DroneShowLED_MAVLink(channel + MAVLINK_COMM_0);
            }
            break;
            
        case DroneShowLEDType_SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(WITH_SITL_RGBLED)
            rgb_led = new SITL_SFML_LED();
#endif
            break;

        case DroneShowLEDType_Servo:
        case DroneShowLEDType_InvertedServo:
            if (
                SRV_Channels::find_channel(SRV_Channel::k_scripting13, chan_white) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting14, chan_red) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting15, chan_green) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting16, chan_blue)
            ) {
                if (type == DroneShowLEDType_InvertedServo) {
                    rgb_led = new RCOutputRGBWLedInverted(chan_red, chan_green, chan_blue, chan_white);
                } else {
                    rgb_led = new RCOutputRGBWLed(chan_red, chan_green, chan_blue, chan_white);
                }
            }else if (
                SRV_Channels::find_channel(SRV_Channel::k_scripting14, chan_red) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting15, chan_green) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting16, chan_blue)
            ) {
                if (type == DroneShowLEDType_InvertedServo) {
                    rgb_led = new RCOutputRGBLedInverted(chan_red, chan_green, chan_blue);
                } else {
                    rgb_led = new RCOutputRGBLed(chan_red, chan_green, chan_blue);
                }
            }
            break;

        case DroneShowLEDType_NeoPixel:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_NeoPixel, channel, num_leds);
            break;

        case DroneShowLEDType_ProfiLED:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_ProfiLED, channel, num_leds);
            break;

        case DroneShowLEDType_I2C:
            /* channel is the 7-bit I2C address, count is (ab)used as the I2C bus */
            result = new DroneShowLED_I2C(num_leds, channel);
            break;

        case DroneShowLEDType_Debug:
            result = new DroneShowLED_Debug();
            break;

        case DroneShowLEDType_WGDrones:
            result = new DroneShowLED_UART_WGDrones(channel);
            break;

        default:
            break;
    }

    if (rgb_led) {
        // Connect the RGB led instance to the notification manager to prevent
        // crashes when RGBLed->set_rgb() tries to access it
        rgb_led->pNotify = &AP::notify();
    }

    if (rgb_led) {
        // Ownership of rgb_led now taken by the DroneShowLED_RGB instance
        result = new DroneShowLED_RGB(rgb_led);
    }

    if (result && !result->init()) {
        // Initialization failed
        delete result;
        result = NULL;
    }

    if (result) {
        result->set_gamma(gamma);
    }

    return result;
}
