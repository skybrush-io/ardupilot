#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>

// Write a drone show status log entry
void AC_DroneShowManager::write_log_message() const
{
    sb_rgb_color_t color;
    get_last_rgb_led_color(color);

    const struct log_DroneShowStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_MSG),
        time_us         : AP_HAL::micros64(),
        show_clock_ms   : get_elapsed_time_since_start_msec(),
        stage           : static_cast<uint8_t>(get_stage_in_drone_show_mode()),
        red             : color.red,
        green           : color.green,
        blue            : color.blue,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
