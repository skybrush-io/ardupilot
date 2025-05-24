#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>     // because of CHECK_PAYLOAD_SIZE

#include "DroneShowLED_Debug.h"

bool DroneShowLED_Debug::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    static uint32_t last_sent_at = 0;
    uint32_t now = AP_HAL::millis();

    if (now - last_sent_at >= 100) {
        gcs().send_text(MAV_SEVERITY_INFO, "%02X%02X%02X%02X", red, green, blue, white);
        return true;
    } else {
        return false;
    }
}
