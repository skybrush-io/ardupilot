#include <AP_Common/AP_Common.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"
#include "AC_DroneShowManager/DroneShow_Enums.h"
#include "DroneShow_CustomPackets.h"

#define NO_MORE_MESSAGES { MSG_LAST, 0 }

static const AC_DroneShowManager::TelemetryRequest default_telemetry_stream[] = {
    { MSG_DRONE_SHOW_STATUS, 500 },  // drone show status as DATA16, 2 Hz
    { MSG_LOCATION, 500 },           // GLOBAL_POSITION_INT, 2 Hz
    { MSG_SYS_STATUS, 1000 },        // SYS_STATUS, 1 Hz
    { MSG_GPS_RAW, 1000 },           // GPS_RAW_INT, 1 Hz
    NO_MORE_MESSAGES
};

static const AC_DroneShowManager::TelemetryRequest compact_telemetry_stream[] = {
    { MSG_EXTENDED_DRONE_SHOW_STATUS, 1000 },  // extended drone show status, 1 Hz
    { MSG_SYS_STATUS, 1000 },                  // SYS_STATUS, 1 Hz
    NO_MORE_MESSAGES
};

static int32_t encode_gps_start_time(int32_t gps_sec, int32_t gps_msec_offset);

const AC_DroneShowManager::TelemetryRequest* AC_DroneShowManager::get_preferred_telemetry_messages() const
{
    switch (_params.telemetry_profile) {
    case TelemetryProfile_Standard:
        return default_telemetry_stream;
    case TelemetryProfile_Compact:
        return compact_telemetry_stream;
    default:
        // Just in case
        return default_telemetry_stream;
    }
}

void AC_DroneShowManager::send_drone_show_status(const mavlink_channel_t chan) const
{
    uint8_t packet[16];
    uint8_t* end;

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(packet));

    /* fill the packet and send it */
    end = _fill_drone_show_status_packet_buffer(packet);
    mavlink_msg_data16_send(
        chan,
        CustomPackets::DRONE_TO_GCS_STATUS,   // Skybrush status packet type marker
        end - packet,     // effective packet length
        packet
    );
}

void AC_DroneShowManager::send_extended_drone_show_status(const mavlink_channel_t chan) const
{
    uint8_t packet[96];
    CustomPackets::drone_show_extended_status_t* status =
        reinterpret_cast<CustomPackets::drone_show_extended_status_t*>(packet);

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(packet));

    /* fill the packet with basic status info */
    _fill_drone_show_status_packet_buffer(packet);

    /* this bit is adapted from GCS_MAVLINK::send_global_position_int() */
    AP_AHRS &ahrs = AP::ahrs();
    Location pos;
    Vector3f vel;
    float relative_alt;

    UNUSED_RESULT(ahrs.get_location(pos)); // return value ignored; we send stale data
    if (!ahrs.get_velocity_NED(vel)) {
        vel.zero();
    }
    ahrs.get_relative_position_D_home(relative_alt);
    relative_alt *= -1000.0f; // change from down to up and metres to millimeters

    status->lat = pos.lat;
    status->lng = pos.lng;
    status->alt = pos.alt * 10UL;
    status->relative_alt = relative_alt;
    status->vx = vel.x * 100;
    status->vy = vel.y * 100;
    status->vz = vel.z * 100;
    status->hdg = ahrs.yaw_sensor;

    /* this bit is adapted from GCS_MAVLINK::send_mavlink_gps_raw() */
    AP_GPS &gps = AP::gps();
    status->gps_hdop = gps.get_hdop(0);
    status->gps_vdop = gps.get_vdop(0);

    /* show metadata: show ID and index of trajectory within show (if known) */
    /* TODO(ntamas) */
    status->show_id = 0;
    status->trajectory_index = 0xFFFF; // not known

    /* send the packet */
    mavlink_msg_data96_send(
        chan,
        CustomPackets::DRONE_TO_GCS_STATUS,   // Skybrush status packet type marker
        sizeof(CustomPackets::drone_show_extended_status_t),  // effective packet length
        packet
    );
}

uint8_t* AC_DroneShowManager::_fill_drone_show_status_packet_buffer(uint8_t* buf) const
{
    CustomPackets::drone_show_status_t* status =
        reinterpret_cast<CustomPackets::drone_show_status_t*>(buf);
    const AP_GPS& gps = AP::gps();

    uint8_t flags, flags2, flags3, gps_health;
    sb_control_output_time_t time_info;
    float elapsed_time_sec;
    int16_t encoded_elapsed_time;
    DroneShowModeStage stage = get_stage_in_drone_show_mode();

    /* calculate status flags */
    flags = 0;
    if (loaded_show_data_successfully() && is_trajectory_plausible()) {
        flags |= (1 << 7);
    }
    if (has_scheduled_start_time()) {
        flags |= (1 << 6);
    }
    if (has_explicit_show_origin_set_by_user()) {
        flags |= (1 << 5);
    }
    if (has_explicit_show_orientation_set_by_user()) {
        flags |= (1 << 4);
    }
    if (AP::fence()->enabled()) {
        flags |= (1 << 3);
    }
    if (has_authorization()) {
        // This is superseded by the full authorization scope but we need to
        // keep on sending this for backward compatibility with older GCS
        // versions
        flags |= (1 << 2);
    }
    if (uses_gps_time_for_show_start() && !_is_gps_time_ok()) {
        flags |= (1 << 1);
    }
    if (AP::fence()->get_breaches()) {
        /* this bit is sent because ArduCopter's SYS_STATUS message does not
         * mark the fence as "enabled and not healthy" when FENCE_ACTION is
         * set to zero, so the GCS would not be notified about fence breaches
         * if we only looked at SYS_STATUS */
        flags |= (1 << 0);
    }

    /* calculate second byte of status flags */
    flags2 = _preflight_check_failures & 0xf0;
    flags2 |= static_cast<uint8_t>(stage) & 0x0f;

    /* calculate GPS health */
    gps_health = gps.status();
    if (gps_health > 7) {
        gps_health = 7;
    }
    gps_health |= (gps.num_sats() > 31 ? 31 : gps.num_sats()) << 3;

    /* calculate third byte of status flags.
     *
     * Bits 0 and 1: boot count modulo 4
     * Bits 2 and 3: authorization scope
     * Bits 4-5: reserved, set to zero
     * Bit 6: indicates that at least one ESC is reporting a high error rate.
     * Bit 7: indicates that the drone has deviated from its expected position.
     */
    flags3 = _boot_count & 0x03;
    flags3 |= (static_cast<uint8_t>(get_authorization_scope()) & 0x03) << 2;
    if (
        _params.max_esc_error_rate_pcnt >= 0 &&
        AP::esc_telem().has_high_error_rate(_params.max_esc_error_rate_pcnt)
    ) {
        flags3 |= (1 << 6);
    }
    if (!_is_at_expected_position()) {
        flags3 |= (1 << 7);
    }

    /* calculate time on show clock since start of the current scene */
    if (_stage_in_drone_show_mode == DroneShow_Performing &&
        sb_show_controller_is_output_valid(&_show_controller)) {
        /* Show is currently being performed, show the clock from the output info of the
         * show controller */
        /* TODO(ntamas): we should probably also send the scene index somewhere */
        time_info = sb_show_controller_get_current_output_time(&_show_controller);
        elapsed_time_sec = time_info.warped_time_in_scene_sec;
    } else {
        /* Not in the "performing" phase; probably takeoff, landing or something else.
         * Show the number of seconds elapsed since the show start, in wall clock time. */
         elapsed_time_sec = get_elapsed_time_since_start_sec();
    }
    
    if (elapsed_time_sec > 32767) {
        encoded_elapsed_time = 32767;
    } else if (elapsed_time_sec <= -32768) {
        encoded_elapsed_time = -32768;
    } else {
        encoded_elapsed_time = static_cast<int16_t>(floorf(elapsed_time_sec));
    }

    /* fill the packet. Note that in the first four bytes we _always_ put the
     * start time according to GPS timestamps, even if we are using the
     * internal clock to synchronize the start. This is to make sure that the
     * UI on Skybrush Live shows the GPS timestamp set by the user */
    status->start_time = encode_gps_start_time(_params.start_time_gps_sec, _params.start_time_gps_msec_offset);
    status->led_color = sb_rgb_color_encode_rgb565(_last_rgb_led_color);
    status->flags = flags;
    status->flags2 = flags2;
    status->flags3 = flags3;
    status->gps_health = gps_health;
    status->elapsed_time = encoded_elapsed_time;

    // MAVLink channel RTCM stats. MAVLink channel 0 is the USB port and we
    // do not really care about that, so we start from 1 (which is TELEM1) and
    // also send the status of channel 2 (which is TELEM2).
    for (uint8_t i = 1; i <= 2; i++) {
        GCS_MAVLINK* gcs_chan;
        int16_t count;

        gcs_chan = gcs().chan(MAVLINK_COMM_0 + i);
        count = gcs_chan ? gcs_chan->rtcm_message_counter().get_count() : -1;

        // count == -1 means that we have never seen an RTCM message on
        // this channel. However, for backward compatibility reasons we
        // need to ensure that the packet can always be safely padded with
        // zero bytes, therefore we need to post the count that we receive
        // plus one -- hence the convoluted expression below.
        status->rtcm_counters[i - 1] = count < 0 ? 0 : ((count > 254 ? 254 : count) + 1);
    }

    return buf + sizeof(CustomPackets::drone_show_status_t);
}

static int32_t encode_gps_start_time(int32_t gps_sec, int32_t gps_msec_offset)
{
    if (gps_sec < 0) {
        return -1;
    }

    // gps_sec is between 0 and GPS_WEEK_LENGTH_SEC so 20 bits are enough. We store
    // the millisecond offset shifted right by 20 bits, and we mask the remaining
    // two bits.
    return (gps_sec & 0xFFFFF) | ((gps_msec_offset & 0x3FF) << 20);
}
