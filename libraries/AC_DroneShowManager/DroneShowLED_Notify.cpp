#include <AP_Notify/AP_Notify.h>

#include "DroneShowLED_Notify.h"

bool DroneShowLED_Notify::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    AP_Notify::handle_rgb(red, green, blue, 0);
    return true;
}
