#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>     // because of CHECK_PAYLOAD_SIZE2

#include "DroneShowLED_MAVLink.h"

static const char message_name[10] = "rgb";

DroneShowLED_MAVLink::DroneShowLED_MAVLink(uint8_t instance) : DroneShowLED(),
    _instance(instance),
    _chan(MAVLINK_COMM_0)
{
}

bool DroneShowLED_MAVLink::init()
{
    GCS_MAVLINK* gcs_channel = gcs().chan(_instance);

    if (gcs_channel) {
        _chan = gcs_channel->get_chan();
        return true;
    } else {
        return false;
    }
}

bool DroneShowLED_MAVLink::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    mavlink_channel_t chan = _chan;

    // white channel ignored

    if (valid_channel(chan) && mavlink_comm_port[chan] != 0) {
        CHECK_PAYLOAD_SIZE2(DEBUG_VECT);
        mavlink_msg_debug_vect_send(
            _chan, message_name, AP_HAL::millis(),
            red / 255.0f, green / 255.0f, blue / 255.0f
        );
    }

    return true;
}
