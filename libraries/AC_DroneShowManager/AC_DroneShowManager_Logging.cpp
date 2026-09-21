#include "AC_DroneShowManager.h"

#include <AP_Logger/AP_Logger.h>
#include <skybrush/skybrush.h>

// Write a drone show status log entry
void AC_DroneShowManager::write_show_status_log_message() const
{
    sb_rgb_color_t color;
    Vector3f dist;
    ssize_t scene;
    float show_clock_sec;
    const sb_control_output_t* output = sb_show_controller_get_current_output(&_show_controller);
    sb_vector3_t position;
    
    if (!sb_control_output_get_position_if_set(output, &position)) {
        memset(&position, 0, sizeof(position));
    }
    
    get_last_rgb_led_color(color);
    get_distance_from_desired_position(dist);
    get_scene_index_and_show_clock_within_scene(&scene, &show_clock_sec);
    
    const struct log_DroneShowStatus pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_MSG),
        time_us         : AP_HAL::micros64(),
        wall_clock_ms   : get_elapsed_time_since_start_msec(),
        stage           : static_cast<uint8_t>(get_stage_in_drone_show_mode()),
        scene           : static_cast<uint8_t>(scene),
        show_clock_ms   : static_cast<int32_t>(show_clock_sec * 1000.0f),
        x               : position.x / 1000.0f,   // convert from millimeters to meters
        y               : position.y / 1000.0f,   // convert from millimeters to meters
        z               : position.z / 1000.0f,   // convert from millimeters to meters
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
    ssize_t scene;
    float show_clock_sec;

    if (!event) {
        return;
    }

    get_scene_index_and_show_clock_within_scene(&scene, &show_clock_sec);

    const struct log_DroneShowEvent pkt {
        LOG_PACKET_HEADER_INIT(LOG_DRONE_SHOW_EVENT_MSG),
        time_us         : AP_HAL::micros64(),
        wall_clock_ms   : get_elapsed_time_since_start_msec(),
        scene           : static_cast<uint8_t>(scene),
        show_clock_ms   : static_cast<int32_t>(show_clock_sec * 1000.0f),
        type            : static_cast<uint8_t>(event->type),
        subtype         : event->subtype,
        payload         : event->payload.as_uint32,
        result          : static_cast<uint8_t>(result),
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a message containing details about a collective RTH trigger
void AC_DroneShowManager::write_crth_trigger_log_message(float rth_start_time_sec, sb_vector3_t start) const
{
    const struct log_CollectiveRTHTrigger pkt {
        LOG_PACKET_HEADER_INIT(LOG_CRTH_TRIGGER_MSG),
        time_us           : AP_HAL::micros64(),
        wall_clock_ms     : get_elapsed_time_since_start_msec(),
        rth_start_time_ms : static_cast<uint32_t>(rth_start_time_sec * 1000.0f),
        start_x           : start.x / 1000.0f,   // convert from millimeters to meters
        start_y           : start.y / 1000.0f,   // convert from millimeters to meters
        start_z           : start.z / 1000.0f,   // convert from millimeters to meters
    };
    
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

// Write a sequence of log messages representing the state of the screenplay
void AC_DroneShowManager::write_screenplay_log_messages()
{
    size_t i, j, num_scenes, num_segments;
    sb_screenplay_scene_t* scene;
    sb_time_axis_t* time_axis;
    const sb_time_segment_t* segment;
    
    struct log_ScreenplayEntry pkt {
        LOG_PACKET_HEADER_INIT(LOG_SCREENPLAY_ENTRY_MSG),
        time_us         : AP_HAL::micros64(),
    };
    
    pkt.seq_no = _last_time_axis_config_seq_no <= 0xFF ? static_cast<uint8_t>(_last_time_axis_config_seq_no) : 0xFF;

    num_scenes = sb_screenplay_size(&_screenplay);
    for (i = 0; i < num_scenes; i++) {
        if (i > 255) {
            break;   // too many scenes
        }

        pkt.scene = static_cast<uint8_t>(i);
        
        scene = sb_screenplay_get_scene_ptr(&_screenplay, i);
        time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
        if (time_axis == nullptr) {
            break;
        }
        
        pkt.origin_ms = time_axis->origin_msec;
        
        num_segments = sb_time_axis_num_segments(time_axis);
        for (j = 0; j < num_segments; j++) {
            if (j > 255) {
                break;   // too many segments
            }

            pkt.index = static_cast<uint8_t>(j);
            segment = sb_time_axis_get_segment(time_axis, j);
            if (segment == nullptr) {
                break;
            }
            
            pkt.duration_ms = segment->duration_msec;
            pkt.initial_rate = segment->initial_rate;
            pkt.final_rate = segment->final_rate;

            AP::logger().WriteBlock(&pkt, sizeof(pkt));
            
            // we do not repeat the origin from the second entry onwards
            pkt.origin_ms = 0;
        }
    }
}
