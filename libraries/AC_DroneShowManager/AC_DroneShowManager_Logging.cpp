#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>
#include <skybrush/events.h>

// Write a drone show status log entry
void AC_DroneShowManager::write_show_status_log_message() const
{
    sb_rgb_color_t color;
    Vector3f dist;

    get_last_rgb_led_color(color);
    get_distance_from_desired_position(dist);
    
    const struct log_DroneShowStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_MSG),
        time_us         : AP_HAL::micros64(),
        show_clock_ms   : get_elapsed_time_since_start_msec(),
        stage           : static_cast<uint8_t>(get_stage_in_drone_show_mode()),
        red             : color.red,
        green           : color.green,
        blue            : color.blue,
        h_dist          : hypotf(dist.x, dist.y),
        v_dist          : dist.z,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a geofence status log entry
void AC_DroneShowManager::write_fence_status_log_message() const
{
    AC_Fence* fence = AC_Fence::get_singleton();

    const struct log_FenceStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_FENCE_STATUS_MSG),
        time_us          : AP_HAL::micros64(),
        geo_enabled      : static_cast<uint8_t>(fence ? fence->get_enabled_fences() : 0),
        geo_breaches     : static_cast<uint8_t>(fence ? fence->get_breaches() : 0),
        geo_breach_count : static_cast<uint16_t>(fence ? fence->get_breach_count() : 0),
        hard_breach_state: static_cast<uint8_t>(hard_fence.get_breach_state()),
        bubble_breach_state: static_cast<uint8_t>(bubble_fence.get_breach_state()),
        bubble_breach_count: bubble_fence.get_action_counter(),
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a drone show event log entry
void AC_DroneShowManager::write_show_event_log_message(
    const sb_event_t *event, DroneShowEventResult result) const
{
    if (!event) {
        return;
    }

    const struct log_DroneShowEvent pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_EVENT_MSG),
        time_us         : AP_HAL::micros64(),
        show_clock_ms   : get_elapsed_time_since_start_msec(),
        type            : static_cast<uint8_t>(event->type),
        subtype         : event->subtype,
        payload         : event->payload.as_uint32,
        result          : static_cast<uint8_t>(result),
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
