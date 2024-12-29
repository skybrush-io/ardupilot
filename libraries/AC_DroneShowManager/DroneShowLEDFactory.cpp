#include "DroneShowLEDFactory.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/RCOutputRGBLed.h>
#include <AP_Notify/SITL_SFML_LED.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowLED_Debug.h"
#include "DroneShowLED_I2C.h"
#include "DroneShowLED_MAVLink.h"
#include "DroneShowLED_NeoPixel_RGBW.h"
#include "DroneShowLED_Notify.h"
#include "DroneShowLED_SerialLED.h"
#include "DroneShowLED_Servo.h"
#include "DroneShowLED_UART_WGDrones.h"

/// Default constructor.
DroneShowLEDFactory::DroneShowLEDFactory()
{
}

DroneShowLED* DroneShowLEDFactory::new_rgb_led_by_type(
    DroneShowLEDType type, uint8_t channel, uint8_t num_leds
) {
    uint8_t chan_red, chan_green, chan_blue, chan_white;
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
        case DroneShowLEDType_ServoWithLimits:
            if (
                SRV_Channels::find_channel(SRV_Channel::k_scripting14, chan_red) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting15, chan_green) &&
                SRV_Channels::find_channel(SRV_Channel::k_scripting16, chan_blue)
            ) {
                if (!SRV_Channels::find_channel(SRV_Channel::k_scripting13, chan_white)) {
                    chan_white = 255;
                }
                result = new DroneShowLED_Servo(
                    chan_red, chan_green, chan_blue, chan_white,
                    type == DroneShowLEDType_InvertedServo,
                    type == DroneShowLEDType_ServoWithLimits
                );
            }
            break;

        case DroneShowLEDType_NeoPixel:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_NeoPixel, channel, num_leds);
            break;

        case DroneShowLEDType_ProfiLED:
            result = new DroneShowLED_SerialLED(DroneShowLED_SerialLEDType_ProfiLED, channel, num_leds);
            break;

        case DroneShowLEDType_I2C:
        case DroneShowLEDType_I2C_RGBW:
            /* channel is the 7-bit I2C address, count is (ab)used as the I2C bus */
            result = new DroneShowLED_I2C(num_leds, channel, type == DroneShowLEDType_I2C_RGBW);
            break;

        case DroneShowLEDType_Debug:
            result = new DroneShowLED_Debug();
            break;

        case DroneShowLEDType_WGDrones:
            result = new DroneShowLED_UART_WGDrones(channel);
            break;

        case DroneShowLEDType_NeoPixel_RGBW:
            result = new DroneShowLED_NeoPixel_RGBW(channel, num_leds);
            break;

        case DroneShowLEDType_Notify:
            result = new DroneShowLED_Notify();
            break;

        default:
            break;
    }

    if (result && !result->init()) {
        // Initialization failed
        delete result;
        result = NULL;
    }

    return result;
}
