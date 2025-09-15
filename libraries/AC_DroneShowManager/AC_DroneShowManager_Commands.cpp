#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"
#include "DroneShow_CustomPackets.h"
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
                mavlink_command_int_t injected_packet = packet;

                injected_packet.command = packet.y & UINT16_MAX;
                injected_packet.y = 0;

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
    
    if (data == nullptr) {
        return false;
    }

    // We allocate type 0x5C for the GCS-to-drone packets (0X5B is the drone-to-GCS
    // status packet), and sacrifice the first byte of the payload to identify
    // the _real_ message type. This reduces the chance of clashes with other
    // DATA* messages from third parties. The type that we receive in this
    // function is the _real_ message type.
    switch (type) {
        // Broadcast start time and authorization state of the show
        case CustomPackets::START_CONFIG:
            {
                if (length < offsetof(CustomPackets::start_config_t, optional_part)) {
                    // Packet too short
                    return false;
                }

                CustomPackets::start_config_t* start_config = static_cast<CustomPackets::start_config_t*>(data);

                // Update start time expressed in GPS time of week
                if (start_config->start_time < 0) {
                    _params.start_time_gps_sec.set(-1);
                } else if (start_config->start_time < GPS_WEEK_LENGTH_SEC) {
                    _params.start_time_gps_sec.set(start_config->start_time);
                }

                // Update authorization flag
                _params.authorization.set(start_config->authorization);

                // Do we have the optional second part?
                if (length >= sizeof(CustomPackets::start_config_t)) {
                    // Optional second part is used by the GCS to convey how many
                    // milliseconds there are until the start of the show. If this
                    // part exists and is positive, _and_ we are using the internal
                    // clock to synchronize the start, then we update the start
                    // time based on this
                    if (!uses_gps_time_for_show_start()) {
                        int32_t countdown_msec = start_config->optional_part.countdown_msec;

                        if (countdown_msec < -GPS_WEEK_LENGTH_MSEC) {
                            clear_scheduled_start_time();
                        } else if (countdown_msec >= 0 && countdown_msec < GPS_WEEK_LENGTH_MSEC) {
                            schedule_delayed_start_after(countdown_msec);
                        }
                    }
                }

                return true;
            }
            break;

        // Schedule collective RTL
        case CustomPackets::CRTL_TRIGGER:
            if (length >= sizeof(CustomPackets::crtl_trigger_t)) {
                CustomPackets::crtl_trigger_t* crtl_trigger = static_cast<CustomPackets::crtl_trigger_t*>(data);
                if (crtl_trigger->start_time == 0) {
                    clear_scheduled_collective_rtl();
                } else if (crtl_trigger->start_time > 0) {
                    schedule_collective_rtl_at_show_timestamp_msec(
                        crtl_trigger->start_time * 1000 /* [s] --> [msec] */
                    );
                }

                return true;
            }
            break;

        // Configure geofences with a single call
        case CustomPackets::SIMPLE_GEOFENCE_SETUP:
            if (length >= sizeof(CustomPackets::simple_geofence_setup_header_t)) {
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
                memset(reply, 0, sizeof(reply));
                reply[0] = CustomPackets::ACKNOWLEDGMENT;
                ack_packet->ack_token = geofence_setup->ack_token;
                ack_packet->result = result;

                // Send the reply packet
                mavlink_msg_data16_send(
                    chan,
                    CustomPackets::DRONE_TO_GCS,   // Skybrush status packet type marker
                    sizeof(CustomPackets::acknowledgment_t),     // effective packet length
                    reply
                );

                return true;
            }
            break;

        // Acknowledgment packets; these can be ignored (but we still return
        // true as we do not want any other handlers to handle them)
        case CustomPackets::ACKNOWLEDGMENT:
            if (length >= sizeof(CustomPackets::acknowledgment_t)) {
                return true;
            }
    }

    return false;
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
