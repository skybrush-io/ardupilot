#include <AP_GPS/AP_GPS.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_CustomPackets.h"

static const AC_DroneShowManager::TelemetryRequest default_telemetry_streams[] = {
    { MAVLINK_MSG_ID_DATA16, 500000 }, // drone show status, 2 Hz
    { MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000 }, // global position, 2 Hz
    { MAVLINK_MSG_ID_SYS_STATUS, 1000000 },  // system status, 1 Hz
    { MAVLINK_MSG_ID_GPS_RAW_INT, 1000000 }, // raw GPS data, 1 Hz
    { 0, 0 }                                  // End marker
};

const AC_DroneShowManager::TelemetryRequest* AC_DroneShowManager::get_preferred_telemetry_messages() const
{
    return default_telemetry_streams;
}

void AC_DroneShowManager::send_drone_show_status(const mavlink_channel_t chan) const
{
    const AP_GPS& gps = AP::gps();

    uint8_t packet[16] = { 0x62, };
    uint8_t flags, flags2, flags3, gps_health;
    float elapsed_time;
    int16_t encoded_elapsed_time;
    int32_t encoded_start_time;
    uint16_t encoded_led_color;
    DroneShowModeStage stage = get_stage_in_drone_show_mode();

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(uint8_t));

    /* calculate status flags */
    flags = 0;
    if (loaded_show_data_successfully() && has_valid_takeoff_time()) {
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
     * Bits 4-6: reserved, set to zero
     * Bit 7: indicate that the drone has deviated from its expected position.
     */
    flags3 = _boot_count & 0x03;
    flags3 |= (static_cast<uint8_t>(get_authorization_scope()) & 0x03) << 2;
    if (!_is_at_expected_position()) {
        flags3 |= (1 << 7);
    }

    /* calculate elapsed time */
    elapsed_time = get_elapsed_time_since_start_sec();
    if (elapsed_time > 32767) {
        encoded_elapsed_time = 32767;
    } else if (elapsed_time <= -32768) {
        encoded_elapsed_time = -32768;
    } else {
        encoded_elapsed_time = static_cast<int16_t>(elapsed_time);
    }

    /* fill the packet. Note that in the first four bytes we _always_ put the
     * start time according to GPS timestamps, even if we are using the
     * internal clock to synchronize the start. This is to make sure that the
     * UI on Skybrush Live shows the GPS timestamp set by the user */
    encoded_start_time = _params.start_time_gps_sec;
    encoded_led_color = sb_rgb_color_encode_rgb565(_last_rgb_led_color);
    memcpy(packet, &encoded_start_time, sizeof(encoded_start_time));
    memcpy(packet + 4, &encoded_led_color, sizeof(encoded_led_color));
    packet[6] = flags;
    packet[7] = flags2;
    packet[8] = gps_health;
    packet[9] = flags3;
    memcpy(packet + 10, &encoded_elapsed_time, sizeof(encoded_elapsed_time));

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
        packet[11 + i] = count < 0 ? 0 : ((count > 254 ? 254 : count) + 1);
    }

    mavlink_msg_data16_send(
        chan,
        CustomPackets::DRONE_TO_GCS_STATUS,   // Skybrush status packet type marker
        14,     // effective packet length
        packet
    );
}

