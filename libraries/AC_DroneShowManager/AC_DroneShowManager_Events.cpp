#include <AP_Logger/AP_Logger.h>

#include "AC_DroneShowManager.h"
#include "DroneShowPyroDevice.h"
#include "LogStructure.h"

#include <skybrush/events.h>

void AC_DroneShowManager::_trigger_events()
{
    const sb_event_t* event;
    DroneShowEventResult result;
    uint8_t events_left = 10;

    if (!_event_list_valid) {
        return;
    }

    float elapsed_time_sec = get_elapsed_time_since_start_sec();
    if (elapsed_time_sec < 0) {
        return;
    }

    // We are guarding this loop with a limit of 10 events to prevent
    // infinite loops in case the event list is corrupted or contains
    // events that are not properly ordered. This is a safety measure
    // to ensure that we do not lock the main loop of the flight controller.

    while (events_left > 0) {
        event = sb_event_list_player_get_next_event_not_later_than(
            _event_list_player, elapsed_time_sec
        );
        if (!event) {
            // No more pending events
            break;
        }

        switch (event->type) {
            case SB_EVENT_TYPE_PYRO:
            
                if (!_pyro_device) {
                    result = DroneShowEventResult_NotSupported;
                    break;
                } else if (event->payload.as_uint32 == UINT32_MAX) {
                    result = _pyro_device->off(event->subtype);
                } else if (!_is_pyro_safe_to_fire()) {
                    result = DroneShowEventResult_Unsafe;
                } else if (event->time_msec < (elapsed_time_sec - 3) * 1000.0f) {
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

        if (result != DroneShowEventResult_SkipLogging) {
            const struct log_DroneShowEvent pkt {
                LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_EVENT_MSG),
                time_us         : AP_HAL::micros64(),
                show_clock_ms   : static_cast<int32_t>(elapsed_time_sec * 1000.0f),
                type            : static_cast<uint8_t>(event->type),
                subtype         : event->subtype,
                payload         : event->payload.as_uint32,
                result          : static_cast<uint8_t>(result),
            };

            AP::logger().WriteBlock(&pkt, sizeof(pkt));
        }

        events_left--;
    }
}
