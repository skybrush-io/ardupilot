#include <GCS_MAVLink/GCS.h>
#include <skybrush/skybrush.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"
#include "DroneShow_CustomPackets.h"
#include "DroneShow_Enums.h"
#include "DroneShowPyroDevice.h"

MAV_RESULT AC_DroneShowManager::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_USER_1: {
        // param1: command code
        // remaining params depend on param1
        //
        // 0 = reload current show
        // 1 = clear current show
        // 2 = trigger pyro test
        // 3 = execute another COMMAND_INT when the group index of the drone is
        //     set to a specific value. param6 (y) contains the _real_ command
        //     code to execute in its lower 16 bits, while bits 30:16 (inclusive)
        //     contain the group index plus 1, zero meaning "all groups". MSB
        //     (sign bit) must be zero. We reserve the right to repurpose high
        //     bits in future versions and sacrifice higher group indices.
        //     param7 of the command is remapped to param1 of the injected command,
        //     while param7 in the injected command is always zero.
        if (is_zero(packet.param1)) {
            // Reload current show
            if (reload_or_clear_show(/* do_clear = */ 0)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_equal(packet.param1, 1.0f)) {
            // Clear current show
            if (reload_or_clear_show(/* do_clear = */ 1)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_equal(packet.param1, 2.0f)) {
            // Trigger pyro test
            uint8_t start = packet.param2 >= 0 && packet.param2 < 255 ? packet.param2 : 255;
            uint8_t num_channels = packet.param3 >= 0 && packet.param3 < 256 ?
                static_cast<uint8_t>(packet.param3) : 255;
            uint32_t delay_msec = isfinite(packet.param4) && packet.param4 >= 0 ? (packet.param4 * 1000.0f) : 0;
            
            if (num_channels == 0) {
                // num_channels == 0 means all channels starting from 'start'
                // up to whatever the pyro device supports. We just use 255 and
                // then clamp it later
                num_channels = 255;
            }

            if (start < 255) {
                if (_pyro_device == nullptr) {
                    // No pyro device is configured, cannot start the test
                    return MAV_RESULT_FAILED;
                } else {
                    if (static_cast<uint16_t>(start) + num_channels > _pyro_device->num_channels()) {
                        num_channels = _pyro_device->num_channels() - start;
                    }
                    _pyro_test_state.start(start, num_channels, delay_msec);
                    return MAV_RESULT_ACCEPTED;
                }
            }
        } else if (is_equal(packet.param1, 3.0f)) {
            // Execute group-specific command.
            int group_index_plus_one = packet.y >> 16;
            if (packet.y < 0) {
                // MSB is 1, ignore.
                return MAV_RESULT_UNSUPPORTED;
            } else if (group_index_plus_one <= 0 || is_in_group(group_index_plus_one - 1)) {
                // Broadcast to all groups (group_index_plus_one == 0) or
                // targeted to our group
                mavlink_command_int_t injected_packet = packet;

                injected_packet.command = packet.y & UINT16_MAX;
                injected_packet.param1 = packet.z;
                injected_packet.y = 0;
                injected_packet.z = 0;

                return gcs().inject_command_int_packet(injected_packet);
            } else {
                return MAV_RESULT_ACCEPTED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    case MAV_CMD_USER_2: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Set show origin, orientation and AMSL with a single command.
            // This is supported with COMMAND_INT MAVLink packets only as we
            // do not want to lose precision in the lat/lng direction due to
            // float representation
            //
            // param4: orientation
            // param5 (x): latitude (degE7)
            // param6 (y): longitude (degE7)
            // param7 (z): AMSL (mm)
            if (configure_show_coordinate_system(
                packet.x, packet.y, static_cast<int32_t>(packet.z),
                packet.param4
            )) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_equal(packet.param1, 1.0f)) {
            // Reserved for debugging purposes
            return _run_debug_request_handler(packet) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    default:
        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }
}

bool AC_DroneShowManager::handle_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        // DATA16, DATA32, DATA64, DATA96 packets are used for custom commands.
        // We do not distinguish between them because MAVLink2 truncates the
        // trailing zeros anyway.
        case MAVLINK_MSG_ID_DATA16:
            return _handle_data16_message(chan, msg);

        case MAVLINK_MSG_ID_DATA32:
            return _handle_data32_message(chan, msg);

        case MAVLINK_MSG_ID_DATA64:
            return _handle_data64_message(chan, msg);

        case MAVLINK_MSG_ID_DATA96:
            return _handle_data96_message(chan, msg);

        case MAVLINK_MSG_ID_LED_CONTROL:
            // The drone show LED listens on the "secret" LED ID 42 with a
            // pattern of 42 as well. Any message that does not match this
            // specification is handled transparently by the "core" MAVLink
            // GCS module.
            return _handle_led_control_message(msg);

        default:
            return false;
    }
}

bool AC_DroneShowManager::_handle_custom_data_message(mavlink_channel_t chan, uint8_t type, void* data, uint8_t length)
{
    uint8_t reply[16];
    bool handled = false;
    bool needs_reply = false;
    
    if (data == nullptr) {
        return false;
    }

    needs_reply = (type == CustomPackets::SIMPLE_GEOFENCE_SETUP);
    if (needs_reply) {
        memset(reply, 0, sizeof(reply));
    }

    // We allocate type 0x5C for the GCS-to-drone packets (0X5B is the drone-to-GCS
    // status packet), and sacrifice the first byte of the payload to identify
    // the _real_ message type. This reduces the chance of clashes with other
    // DATA* messages from third parties. The type that we receive in this
    // function is the _real_ message type.
    switch (type) {
        // Broadcast start time and authorization state of the show
        case CustomPackets::START_CONFIG:
            handled = _handle_start_time_configuration_packet(data, length);
            break;

        // Schedule collective RTL; obsolete, does nothing, kept for backward compatibility
        case CustomPackets::DEPRECATED_CRTL_TRIGGER:
            handled = true;
            break;

        // Configure geofences with a single call
        case CustomPackets::SIMPLE_GEOFENCE_SETUP:
            handled = _handle_geofence_setup_packet(data, length, reply);
            break;

        // Acknowledgment packets; these can be ignored (but we still return
        // true as we do not want any other handlers to handle them)
        case CustomPackets::ACKNOWLEDGMENT:
            handled = _handle_acknowledgment_packet(data, length);
            break;

        // Time axis configuration packet, used to implement suspension and resume
        case CustomPackets::TIME_AXIS_CONFIG:
            handled = _handle_time_axis_configuration_packet(data, length);
            break;
    }

    if (handled && needs_reply) {
        // Send the reply packet
        mavlink_msg_data16_send(
            chan,
            CustomPackets::DRONE_TO_GCS,   // Skybrush status packet type marker
            sizeof(CustomPackets::acknowledgment_t),     // effective packet length
            reply
        );
    }

    return handled;
}

bool AC_DroneShowManager::_handle_data16_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data16_t packet;
    mavlink_msg_data16_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data32_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data32_t packet;
    mavlink_msg_data32_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data64_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data64_t packet;
    mavlink_msg_data64_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data96_message(mavlink_channel_t chan, const mavlink_message_t& msg)
{
    mavlink_data96_t packet;
    mavlink_msg_data96_decode(&msg, &packet);
    if (packet.type != CustomPackets::GCS_TO_DRONE || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(chan, packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_acknowledgment_packet(void* data, uint8_t length)
{
    // Only a length check; we don't need to do anything with this packet
    return length >= sizeof(CustomPackets::acknowledgment_t);
}

bool AC_DroneShowManager::_handle_geofence_setup_packet(void* data, uint8_t length, uint8_t* reply)
{
    if (length < sizeof(CustomPackets::simple_geofence_setup_header_t)) {
        // Packet too short
        return false;
    }

    CustomPackets::simple_geofence_setup_header_t* geofence_setup = static_cast<CustomPackets::simple_geofence_setup_header_t*>(data);
    DroneShow_FenceConfig fence_config;
    CustomPackets::acknowledgment_t* ack_packet = reinterpret_cast<CustomPackets::acknowledgment_t*>(reply + 1);
    size_t points_payload_length_in_bytes = length - sizeof(CustomPackets::simple_geofence_setup_header_t);
    size_t num_points = points_payload_length_in_bytes / sizeof(DroneShow_FencePoint);
    MAV_RESULT result = MAV_RESULT_FAILED;

    // Convert from geofence_setup to fence_config
    fence_config.max_altitude_dm = geofence_setup->max_altitude_dm;
    fence_config.radius_dm = geofence_setup->radius_dm;
    fence_config.action = geofence_setup->flags & 0x0f;
    fence_config.num_points = geofence_setup->num_points;
    fence_config.points = reinterpret_cast<DroneShow_FencePoint*>(
        reinterpret_cast<uint8_t*>(geofence_setup) +
        sizeof(CustomPackets::simple_geofence_setup_header_t)
    );
    if (fence_config.num_points <= num_points) {
        result = configure_fences(fence_config) ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
    }

    // Prepare the reply packet
    reply[0] = CustomPackets::ACKNOWLEDGMENT;
    ack_packet->ack_token = geofence_setup->ack_token;
    ack_packet->result = result;

    return true;
}

bool AC_DroneShowManager::_handle_start_time_configuration_packet(void* data, uint8_t length)
{
    if (length < offsetof(CustomPackets::start_config_t, v2_extended_part)) {
        // Packet too short
        return false;
    }

    CustomPackets::start_config_t* start_config = static_cast<CustomPackets::start_config_t*>(data);

    // The start time is encoded as a little-endian 32-bit signed integer, meaning the
    // number of seconds elapsed since the start of the current GPS week in the range
    // [0; 604799].
    //
    // Additional special values in start_config->start_time are:
    //
    // 0x7FFFFFFF (decimal 2147483647) = "do not change start time"
    // -0x80000000 (decimal -2147483648) = "clear start time / no start time"
    //
    // We are liberal in what we accept so any negative value is treated as
    // "clear start time", while any value greater than or equal to GPS_WEEK_LENGTH_SEC
    // (604800) is treated as "do not change start time".
    //
    // Since May 2026 we added support for setting the start time in millisecond
    // precision. The millisecond part is transmitted separately in the optional part
    // of the start time configuration packet, for backward compatibility reasons.

    // Update start time expressed in GPS time of week
    if (start_config->start_time < 0) {
        // Clear start time
        set_scheduled_start_time_in_gps_time_of_week(-1, 0);
    } else if (start_config->start_time >= GPS_WEEK_LENGTH_SEC) {
        // Do not change start time
    } else {
        // Try to set the start time since the range is valid
        uint16_t millisecond_offset;

        // Do we have the optional v3 extensino part?
        if (length >= sizeof(CustomPackets::start_config_t)) {
            // Optional millisecond part is present in the packet; use it if it's valid
            // Only the 10 least significant bits are used. Remaining bits are ignored;
            // GCS should send them as zeros so we can use them later for other purposes
            // if needed.
            millisecond_offset = start_config->v3_extended_part.start_time_msec_offset & 0x3FF;
        } else {
            // Optional millisecond part is not present; assume zero
            millisecond_offset = 0;
        }

        set_scheduled_start_time_in_gps_time_of_week(start_config->start_time, millisecond_offset);
    }

    // Update authorization scope
    if (start_config->authorization == 255) {
        // Do not change authorization
    } else if (start_config->authorization <= DroneShowAuthorization_Last) {
        // Update authorization scope
        _params.authorization.set(start_config->authorization);
    } else {
        // Invalid value, revoke authorization for safety reasons
        _params.authorization.set(DroneShowAuthorization_Revoked);
    }

    // Do we have the optional v2 extension part?
    if (length >= offsetof(CustomPackets::start_config_t, v3_extended_part)) {
        // Optional v2 part is used by the GCS to convey how many
        // milliseconds there are until the start of the show. If this
        // part exists and is positive, _and_ we are using the internal
        // clock to synchronize the start, then we update the start
        // time based on this
        if (!uses_gps_time_for_show_start()) {
            int32_t countdown_msec = start_config->v2_extended_part.countdown_msec;

            if (countdown_msec == 0x7FFFFFFF) {
                // Do not change start time
            } else if (countdown_msec < -GPS_WEEK_LENGTH_MSEC) {
                // Outside normal range; clear start time
                clear_scheduled_start_time();
            } else if (countdown_msec >= 0 && countdown_msec < GPS_WEEK_LENGTH_MSEC) {
                // Start time is in the future; schedule the takeoff
                schedule_delayed_start_after(countdown_msec);
            } else {
                // Start time is in the past but within the current GPS week; ignore
                // it as the show might be running normally
            }
        }
    }

    return true;
}

bool AC_DroneShowManager::_handle_time_axis_configuration_packet(void* data, uint8_t length)
{
    CustomPackets::time_axis_config_header_t* header;
    CustomPackets::time_axis_config_scene_header_t* scene_header;
    CustomPackets::time_axis_config_scene_entry_t* entry;
    uint64_t epoch_msec;
    uint8_t num_scenes, num_entries, scene_index, entry_index;
    uint8_t *ptr, *end;
    sb_screenplay_t new_screenplay;
    sb_screenplay_scene_t *scene;
    sb_time_axis_t* time_axis;
    sb_time_segment_t segment;
    bool success;

    _time_axis_configuration_packet_count++;

    if (length < sizeof(CustomPackets::time_axis_config_header_t)) {
        // Packet too short - even with no scenes the packet must be at least this long
        _time_axis_configuration_last_error = 1;
        return false;
    }

    header = static_cast<CustomPackets::time_axis_config_header_t*>(data);
    num_scenes = header->num_scenes;
    
    if (length < (
        sizeof(CustomPackets::time_axis_config_header_t) +
        num_scenes * sizeof(CustomPackets::time_axis_config_scene_header_t)
    )) {
        // Packet too short - even with no segments in each of the scenes the packet
        // must be at least this long
        _time_axis_configuration_last_error = 2;
        return false;
    }
    
    if (header->seq_no == _last_time_axis_config_seq_no) {
        // Duplicate packet
        return true;
    }
    
    if (_last_time_axis_config_seq_no <= 0xFF) {
        // Check for packets received out-of-order. Note that we need a separate 'diff'
        // variable, otherwise the integer promotion rules in C would make the
        // subtraction signed.
        uint8_t diff = header->seq_no - static_cast<uint8_t>(_last_time_axis_config_seq_no);
        if (diff >= 0xF0) {
            // Probably the packets are being sent on two or more redundant channels and
            // we are receiving them out-of-order
            _time_axis_configuration_out_of_order_count++;
            return true;
        }
    }

    // Remember the sequence number
    _last_time_axis_config_seq_no = header->seq_no;

    // Check whether we can accept this time axis configuration packet given its header
    // and the current state of the drone show manager. If we cannot, we ignore the
    // packet and return false.
    if (!_is_safe_to_accept_time_axis_configuration_packet(*header)) {
        _time_axis_configuration_last_error = 17;
        return false;
    }

    // Figure out the epoch relative to which all origin fields in the packet will be
    // interpreted
    if (uses_gps_time_for_show_start()) {
        // When using GPS time for show start, the origin is assumed to be an absolute
        // time in milliseconds since the UNIX epoch, written in the header, and we
        // use this to update the SHOW_START_TIME parameter.
        epoch_msec = header->start_time_msec;
        if (epoch_msec > 0) {
            // Start time set, but we need to convert from milliseconds to seconds
            // since the start of the GPS week
            uint64_t epoch_msec_since_gps_time_origin = epoch_msec - UNIX_OFFSET_MSEC;
            set_scheduled_start_time_in_gps_time_of_week(
                (epoch_msec_since_gps_time_origin / 1000) % GPS_WEEK_LENGTH_SEC,
                epoch_msec_since_gps_time_origin % 1000
            );
        } else {
            // Start time not set
            set_scheduled_start_time_in_gps_time_of_week(-1, 0);
        }
    } else {
        // When using the internal clock for show start, the origin is assumed to be
        // relative to the show start time. This is not really recommended but we need
        // to handle it nevertheless.
        epoch_msec = 0;
    }
    
    // We need to be extra careful here; if an error happens while we are setting up the
    // new scenes, we want to leave the existing screenplay intact. Therefore, we first
    // create a new screenplay, and then swap it with the existing one only if everything
    // went well.
    if (sb_screenplay_init(&new_screenplay) != SB_SUCCESS) {
        _time_axis_configuration_last_error = 3;
        return false;
    }
    
    // From this point onwards we need to clean up the new screenplay if anything
    // goes wrong, so we can't return directly -- we need to jump to the exit label
    // instead. We use the 'success' variable to decide whether everything went well
    // (in which case we need to swap the new screenplay with the old one and destroy
    // the old one) or something went wrong (in which case we just destroy the new
    // screenplay and leave the old one intact).
    success = false;
    
    // Make sure that the new screenplay refers to the same RTH plan as the existing one
    sb_screenplay_set_rth_plan(&new_screenplay, sb_screenplay_get_rth_plan(&_screenplay));
    
    // Header processed; now process each of the scenes
    ptr = reinterpret_cast<uint8_t*>(data);
    end = ptr + length;
    ptr += sizeof(CustomPackets::time_axis_config_header_t);

    for (scene_index = 0; scene_index < num_scenes; scene_index++) {
        if (ptr + sizeof(CustomPackets::time_axis_config_scene_header_t) > end) {
            // Packet too short
            _time_axis_configuration_last_error = 4;
            goto exit;
        }
        
        scene_header = reinterpret_cast<CustomPackets::time_axis_config_scene_header_t*>(ptr);
        ptr += sizeof(CustomPackets::time_axis_config_scene_header_t);
        
        // Add a new scene to the screenplay
        if (sb_screenplay_append_new_scene(&new_screenplay, &scene) != SB_SUCCESS) {
            // Could not add new scene
            _time_axis_configuration_last_error = 5;
            goto exit;
        }
        
        // Check the scene ID and figure out whether this scene is for the main show
        // or for a coordinated RTH plan
        if (scene_header->scene_id == 0) {
            // Main show
            sb_screenplay_scene_set_tag(scene, SceneTag_MainShow);
            sb_screenplay_scene_update_contents_from(scene, &_main_show_scene);
        } else if ((scene_header->scene_id & 0xC000) == 0xC000) {
            // Coordinated RTH plan, starting at the number of seconds described by the 
            // lower 14 bits
            sb_rth_plan_t* rth_plan = sb_screenplay_get_rth_plan(&new_screenplay);
            sb_rth_plan_entry_t rth_plan_entry;
            float rth_start_time = static_cast<float>(scene_header->scene_id & 0x3FFF);
            if (rth_plan == NULL || sb_rth_plan_evaluate_at(rth_plan, rth_start_time, &rth_plan_entry) != SB_SUCCESS) {
                // Could not evaluate RTH plan at the given time
                _time_axis_configuration_last_error = 6;
                goto exit;
            }
            
            // Update scene tag to mark it as a CRTH scene
            sb_screenplay_scene_set_tag(scene, SceneTag_CRTH);

            // Create a trajectory based on rth_plan_entry and set it to the scene
            {
                sb_trajectory_t* rth_trajectory = sb_trajectory_new();
                sb_trajectory_player_t player;
                sb_vector3_with_yaw_t start_with_yaw;
                sb_vector3_t start;
                
                if (rth_trajectory == nullptr) {
                    // Out of memory
                    _time_axis_configuration_last_error = 7;
                    goto exit;
                }

                if (_show_controller.trajectory_player != nullptr) {
                    // We have started the show and the show controller has a trajectory
                    // player so we can clone it to find out the position of the drone
                    // at the start of the RTH plan, then construct a trajectory from
                    // there according to the plan.
                    if (sb_trajectory_player_clone(&player, _show_controller.trajectory_player) != SB_SUCCESS) {
                        // should not happen
                        _time_axis_configuration_last_error = 9;
                        goto exit;
                    }
                    
                    if (sb_trajectory_player_get_position_at(&player, rth_start_time, &start_with_yaw) != SB_SUCCESS) {
                        // should not happen
                        _time_axis_configuration_last_error = 10;
                        sb_trajectory_player_destroy(&player);
                        goto exit;
                    }
                    
                    start.x = start_with_yaw.x;
                    start.y = start_with_yaw.y;
                    start.z = start_with_yaw.z;
                } else {
                    // No trajectory player yet. This may happen if the CRTH command
                    // arrives during the "Waiting for start time" or "Takeoff" phases.
                    // In that case we just need to create a landing trajectory.
                    if (_stage_in_drone_show_mode == DroneShow_WaitForStartTime ||
                        _stage_in_drone_show_mode == DroneShow_Takeoff) {
                        start.x = _takeoff_position_mm.x;
                        start.y = _takeoff_position_mm.y;
                        start.z = _takeoff_position_mm.z;

                        sb_rth_plan_entry_clear(&rth_plan_entry, rth_plan, 0.0f);
                    } else {
                        // We are in an unexpected stage; we cannot create a valid RTH
                        // trajectory
                        _time_axis_configuration_last_error = 8;
                        goto exit;
                    }
                }

                // rth_plan_entry contains the start time of the RTH plan, but we don't
                // need that -- we want to create a trajectory that starts at T=0 in
                // show clock because the clock of the new RTH scene starts from 0
                rth_plan_entry.time_sec = 0.0f;
    
                if (sb_trajectory_update_from_rth_plan_entry(rth_trajectory, &rth_plan_entry, start) != SB_SUCCESS) {
                    // Could not create RTH plan trajectory
                    _time_axis_configuration_last_error = 11;
                    sb_trajectory_player_destroy(&player);
                    SB_DECREF(rth_trajectory);
                    goto exit;
                }

                sb_screenplay_scene_set_trajectory(scene, rth_trajectory);
                SB_DECREF(rth_trajectory);

                sb_trajectory_player_destroy(&player);
                
                // Log the details of the CRTH trigger for debugging purposes
                write_crth_trigger_log_message(rth_start_time, start);
            }
            
            // Use a fixed light program with RTH color
            {
                sb_light_program_t* rth_light_program = sb_light_program_new();
                if (rth_light_program) {
                    if (sb_light_program_set_constant_color(rth_light_program, get_rth_transition_color()) != SB_SUCCESS) {
                        // Probably out of memory; clean up and proceed without setting
                        // a light program
                        SB_DECREF(rth_light_program);
                        rth_light_program = nullptr;
                    }
                } else {
                    // Out of memory; proceed without setting a light program
                }
                sb_screenplay_scene_set_light_program(scene, rth_light_program);
                SB_XDECREF(rth_light_program);
            }
        } else {
            // Unknown scene ID; may be used in the future but it has no meaning now
            _time_axis_configuration_last_error = 12;
            goto exit;
        }

        num_entries = scene_header->num_entries;
    
        // Find the time axis to manipulate
        time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
        if (time_axis == nullptr) {
            // Could not get time axis; this should not happen but let's be defensive
            _time_axis_configuration_last_error = 13;
            goto exit;
        }
        
        // First, we set the origin of the new time axis
        sb_time_axis_set_origin_msec(time_axis, scene_header->origin_msec);
        
        // Add segments
        for (entry_index = 0; entry_index < num_entries; entry_index++) {
            if (ptr + sizeof(CustomPackets::time_axis_config_scene_entry_t) > end) {
                // Packet too short
                _time_axis_configuration_last_error = 14;
                goto exit;
            }
            
            entry = reinterpret_cast<CustomPackets::time_axis_config_scene_entry_t*>(ptr);
            ptr += sizeof(CustomPackets::time_axis_config_scene_entry_t);
    
            if (entry->duration_msec == 0) {
                if (!_ensure_scene_covers_relevant_part_of_trajectory(
                    scene, entry->initial_rate_scaled / 65535.0f,
                    entry->final_rate_scaled / 65535.0f
                )) {
                    _time_axis_configuration_last_error = 15;
                    goto exit;
                }
            } else {
                segment = sb_time_segment_make(
                    entry->duration_msec,
                    entry->initial_rate_scaled / 65535.0f,
                    entry->final_rate_scaled / 65535.0f
                );

                if (sb_time_axis_append_segment(time_axis, segment) != SB_SUCCESS) {
                    _time_axis_configuration_last_error = 16;
                    goto exit;
                }
            }
        }
        
        // Finally, set total duration of scene to be the duration of its segments on
        // the time axis
        sb_screenplay_scene_set_duration_msec(scene, sb_time_axis_get_total_duration_msec(time_axis));
    }

    success = true;

exit:
    if (success) {
        // Clean up the old screenplay and replace it with the new one
        sb_screenplay_destroy(&_screenplay);
        _screenplay = new_screenplay;
        
        // Notify the show controller that the screenplay has been updated
        sb_show_controller_notify_screenplay_changed(&_show_controller);

        // Invalidate the projected takeoff time because it depends on the screenplay.
        // It will be realculated later if needed.
        _invalidate_projected_wall_clock_time_at_takeoff();

        // Add log entries containing the current screenplay
        write_screenplay_log_messages();
    } else {
        // Clean up the new screenplay that we tried to prepare
        sb_screenplay_destroy(&new_screenplay);
    }
    
    return success;
}

bool AC_DroneShowManager::_is_safe_to_accept_time_axis_configuration_packet(
    const CustomPackets::time_axis_config_header_t& header) const
{
    if (!uses_gps_time_for_show_start()) {
        // When using the internal clock for show start, we can accept any time axis
        // configuration packet at any time
        return true;
    }

    // Slight adjustments to the start time is okay; this can happen if the time code
    // is slipping a bit and the GCS is trying to correct it.
    bool would_change_start_time_significantly = (
        (header.start_time_msec - _start_time_unix_usec / 1000) >= 100
    );
    if (
        would_change_start_time_significantly &&
        !_is_safe_to_change_start_time_in_current_stage()
    ) {
        // We are not allowed to change the start time in the current stage of the
        // drone show mode, so we cannot accept a time axis configuration packet
        // that would change the start time
        return false;
    }

    return true;
}
