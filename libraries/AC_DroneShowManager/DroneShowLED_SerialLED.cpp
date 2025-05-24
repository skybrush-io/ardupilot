#include <AP_SerialLED/AP_SerialLED.h>
#include "DroneShowLED_SerialLED.h"

DroneShowLED_SerialLED::DroneShowLED_SerialLED(
    DroneShowLED_SerialLEDType type, uint8_t chan, uint8_t num_leds
) : _chan(chan), _type(type), _num_leds(num_leds)
{
}

bool DroneShowLED_SerialLED::init(void)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();
    if (!serialLed) {
        return false;
    }

    if (_type == DroneShowLED_SerialLEDType_NeoPixel) {
        return serialLed->set_num_neopixel(_chan, _num_leds);
    } else if (_type == DroneShowLED_SerialLEDType_ProfiLED) {
        return serialLed->set_num_profiled(_chan, _num_leds);
    }

    return false;
}

bool DroneShowLED_SerialLED::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();

    if (serialLed) {
        // white channel ignored
        serialLed->set_RGB(_chan, -1, red, green, blue);
        serialLed->send(_chan);
        return true;
    } else {
        return false;
    }
}
