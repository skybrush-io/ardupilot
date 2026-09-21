#include <AP_SerialLED/AP_SerialLED.h>
#include "DroneShowLED_NeoPixel_RGBW.h"

/*
 * The implementation in this file is a horrible hack, but it seems to work.
 * We handle NeoPixel RGBW LED strips by pretending that it's a standard
 * WS2812B NeoPixel LED strip, where the color of each LED is transmitted in
 * three bytes in GRB order at 800 kHz. RGBW LED strips seem to use four bytes
 * per LED in RGBW order, so we remap N "real" LEDs into ceil(N*4/3) "virtual"
 * LEDs and shuffle the components around to get the right ordering on the
 * wire.
 */

DroneShowLED_NeoPixel_RGBW::DroneShowLED_NeoPixel_RGBW(uint8_t chan, uint8_t num_leds)
    : _chan(chan), _num_leds(num_leds)
{
    _virtual_num_leds = _num_leds > 191 ? 255 : static_cast<uint8_t>(ceilf(_num_leds * 4.0f / 3));
}

bool DroneShowLED_NeoPixel_RGBW::init(void)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();
    if (!serialLed) {
        return false;
    }

    return serialLed->set_num_neopixel(_chan, _virtual_num_leds);
}

bool DroneShowLED_NeoPixel_RGBW::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    AP_SerialLED* serialLed = AP_SerialLED::get_singleton();
    uint8_t i;

    if (serialLed) {
        // The pattern to be sent repeats itself after every 4th LED.
        i = 0;
        while (i < _virtual_num_leds) {
            serialLed->set_RGB(_chan, i, green, red, blue);
            i++;

            if (i < _virtual_num_leds - 1) {
                serialLed->set_RGB(_chan, i, red, white, green);
            } else if (i == _virtual_num_leds - 1) {
                // to make sure that we don't accidentally light up the next
                // LED on the strip if the user has more than what was specified
                serialLed->set_RGB(_chan, i, 0, white, 0);
            } else {
                break;
            }
            i++;

            if (i < _virtual_num_leds - 1) {
                serialLed->set_RGB(_chan, i, white, blue, red);
            } else if (i == _virtual_num_leds - 1) {
                // to make sure that we don't accidentally light up the next
                // LED on the strip if the user has more than what was specified
                serialLed->set_RGB(_chan, i, white, blue, 0);
            } else {
                break;
            }
            i++;

            if (i < _virtual_num_leds) {
                serialLed->set_RGB(_chan, i, blue, green, white);
            } else {
                break;
            }
            i++;
        }

        serialLed->send(_chan);

        return true;
    } else {
        return false;
    }
}
