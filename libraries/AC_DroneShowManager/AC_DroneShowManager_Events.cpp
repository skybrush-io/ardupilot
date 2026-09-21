#include <AP_Logger/AP_Logger.h>

#include "AC_DroneShowManager.h"
#include "AC_DroneShowManager/DroneShow_Enums.h"
#include "DroneShowPyroDevice.h"

const float EVENT_TIMING_TOLERANCE_SEC = 3.0f;

void AC_DroneShowManager::_trigger_show_events()
{
    const sb_event_t* event;
    DroneShowEventResult result;
    uint8_t events_left = 10;

    if (_stage_in_drone_show_mode != DroneShow_Performing) {
        // We are not performing the show, no need to trigger events
        return;
    }
    
    sb_control_output_time_t time_info = sb_show_controller_get_current_output_time(&_show_controller);
    float show_clock_sec = time_info.warped_time_in_scene_sec;
    
    // No need to update the show controller with the timestamp; it has been done
    // earlier before this function was called.

    // We are guarding this loop with a limit of 10 events to prevent
    // infinite loops in case the event list is corrupted or contains
    // events that are not properly ordered. This is a safety measure
    // to ensure that we do not lock the main loop of the flight controller.

    while (events_left > 0) {
        event = sb_show_controller_get_next_event(&_show_controller);
        if (!event) {
            // No more pending events
            break;
        }

        switch (event->type) {
            case SB_EVENT_TYPE_PYRO:
                // Handle pyro events
                if (!_pyro_device) {
                    result = DroneShowEventResult_NotSupported;
                } else if (event->payload.as_uint32 == UINT32_MAX) {
                    result = _pyro_device->off(event->subtype);
                } else if (!_is_pyro_safe_to_fire()) {
                    result = DroneShowEventResult_Unsafe;
                } else if (event->time_msec < (show_clock_sec - EVENT_TIMING_TOLERANCE_SEC) * 1000.0f) {
                    result = DroneShowEventResult_TimeMissed;
                } else {
                    result = _pyro_device->fire(event->subtype);
                }
                break;

            default:
                // Unknown event type, ignore
                result = DroneShowEventResult_NotSupported;
                break;
        }
        
        write_show_event_log_message(event, result);

        events_left--;
    }
}
