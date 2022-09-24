#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"

bool AC_DroneShowManager::clear_scheduled_collective_rtl(bool force)
{
    if (!force && get_stage_in_drone_show_mode() != DroneShow_Performing)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    _crtl_start_time_sec = 0;

    return true;
}

void AC_DroneShowManager::handle_rc_collective_rtl_switch()
{
    if (_are_rc_switches_blocked())
    {
        return;
    }

    schedule_collective_rtl_at_show_timestamp_msec(get_elapsed_time_since_start_msec());
}

bool AC_DroneShowManager::schedule_collective_rtl_at_show_timestamp_msec(uint32_t timestamp_ms)
{
    if (get_stage_in_drone_show_mode() != DroneShow_Performing)
    {
        return false;
    }

    _crtl_start_time_sec = timestamp_ms / 1000.0f;

    return true;
}
