#include <AP_GPS/AP_GPS.h>

#include "AC_DroneShowManager.h"
#include "AC_DroneShowManager/DroneShow_Enums.h"

static uint64_t get_gps_timestamp_usec();
static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);

int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    uint64_t now, reference, diff;
    
    if (uses_gps_time_for_show_start()) {
        now = get_gps_timestamp_usec();
        reference = _start_time_unix_usec;
    } else {
        now = AP_HAL::micros64();
        reference = _start_time_on_internal_clock_usec;
    }

    if (reference > 0) {
        if (reference > now) {
            diff = reference - now;
            if (diff < INT64_MAX) {
                return -diff;
            } else {
                return INT64_MIN;
            }
        } else if (reference < now) {
            diff = now - reference;
            if (diff < INT64_MAX) {
                return diff;
            } else {
                return INT64_MAX;
            }
        } else {
            return 0;
        }
    } else {
        return INT64_MIN;
    }
}

int32_t AC_DroneShowManager::get_elapsed_time_since_start_msec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    if (elapsed_usec <= -86400000000) {
        return -86400000;
    } else if (elapsed_usec >= 86400000000) {
        return 86400000;
    } else {
        return static_cast<int32_t>(elapsed_usec / 1000);
    }
}

float AC_DroneShowManager::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    return elapsed_usec == INT64_MIN ? -86400 : static_cast<float>(elapsed_usec / 1000) / 1000.0f;
}

void AC_DroneShowManager::get_scene_index_and_show_clock_within_scene(
    ssize_t* scene_out, float* show_clock_sec_out
) const {
    sb_control_output_time_t time_info;
    ssize_t scene;
    float show_clock_sec;
    
    if (_stage_in_drone_show_mode != DroneShow_Performing) {
        scene = 0;
        show_clock_sec = 0;
    } else {
        time_info = sb_show_controller_get_current_output_time(&_show_controller);
        scene = time_info.scene;
        show_clock_sec = time_info.warped_time_in_scene_sec;
        
        if (scene < 0) {
            scene = sb_screenplay_size(&_screenplay);  // out of range value
        }
        if (scene >= 255) {
            scene = 255;  // out of range value or too many chapters
        }
    
        if (!isfinite(show_clock_sec)) {
            show_clock_sec = 0;
        }
    }
    
    if (scene_out) {
        *scene_out = scene;
    }
    
    if (show_clock_sec_out) {
        *show_clock_sec_out = show_clock_sec;
    }
}

int64_t AC_DroneShowManager::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

float AC_DroneShowManager::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

uint32_t AC_DroneShowManager::_get_gps_synced_timestamp_in_millis_for_lights() const
{
    // No need to worry about loss of GPS fix; AP::gps().time_epoch_usec() is
    // smart enough to extrapolate from the timestamp of the latest fix.
    //
    // Also no need to worry about overflow; AP::gps().time_epoch_usec() / 1000
    // is too large for an uint32_t but it doesn't matter as we will truncate
    // the high bits.
    if (_is_gps_time_ok()) {
        return get_gps_timestamp_usec() / 1000;
    } else {
        return AP_HAL::millis();
    }
}

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}

bool AC_DroneShowManager::_is_safe_to_change_start_time_in_current_stage() const {
    return is_safe_to_change_start_time_in_stage(_stage_in_drone_show_mode);
}

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}

// Returns the current time according to the GPS, in microseconds.
//
// This function takes care of eliminating glitches in the GPS timestamp that
// may happen when a GPS message updates the stored GPS time-of-week but not
// the correspnding GPS week number. This is a glitch that is known to have
// happened with U-blox GPS modules in ArduPilot 4.6, but other versions or
// GPS drivers may also be affected so we try to protect against it.
//
// The fix we use here is simply to assume that GPS time cannot move backward.
// If we receive a reported GPS time that is earlier than the previous value
// (which we store here), we return the previous value instead.
//
// We assume that glitches only occur "backwards" in time, not forward. That
// would require a GPS message handler that updates the GPS week number _without_
// updating the GPS time-of-week, which is unlikely to happen in practice
// (all GPS messages that carry the week number are also likely to carry the
// time-of-week).
static uint64_t get_gps_timestamp_usec()
{
    // AP::gps().time_epoch_usec() is smart enough to handle the case when
    // the GPS fix was lost so no need to worry about loss of GPS fix here.
    static uint64_t last_gps_time_usec = 0;
    uint64_t current_gps_time_usec = AP::gps().time_epoch_usec();

    if (current_gps_time_usec < last_gps_time_usec) {
        return last_gps_time_usec;
    } else {
        last_gps_time_usec = current_gps_time_usec;
        return current_gps_time_usec;
    }
}
