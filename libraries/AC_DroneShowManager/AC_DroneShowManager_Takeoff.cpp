#include "AC_DroneShowManager.h"

bool AC_DroneShowManager::get_global_takeoff_position(Location& loc) const
{
    // This function may be called any time, not only during the show, so we
    // need to take the parameters provided by the user, convert them into a
    // ShowCoordinateSystem object, and then use that to get the GPS coordinates
    sb_vector3_with_yaw_t vec;

    if (!_tentative_show_coordinate_system.is_valid())
    {
        return false;
    }

    vec.x = _takeoff_position_mm.x;
    vec.y = _takeoff_position_mm.y;
    vec.z = _takeoff_position_mm.z;

    _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

    return true;
}

float AC_DroneShowManager::get_motor_spool_up_time_sec() const {
    float value = 0.0f;

    if (AP_Param::get("MOT_SPOOL_TIME", value)) {
        if (value >= 0.0f) {
            return value;
        }
    }

    return DEFAULT_MOTOR_SPOOL_UP_TIME_SEC;
}

float AC_DroneShowManager::get_time_until_takeoff_sec() const
{
    return get_time_until_start_sec() + get_relative_takeoff_time_sec();
}

bool AC_DroneShowManager::is_prepared_to_take_off() const
{
    return (!_preflight_check_failures && _is_gps_time_ok());
}

bool AC_DroneShowManager::notify_takeoff_attempt()
{
    if (!is_prepared_to_take_off())
    {
        return false;
    }
    
    return _copy_show_coordinate_system_from_parameters_to(_show_coordinate_system);
}

bool AC_DroneShowManager::_is_at_takeoff_position_xy(float xy_threshold) const
{
    Location takeoff_loc;
    
    if (!_tentative_show_coordinate_system.is_valid())
    {
        // User did not set up the takeoff position yet
        return false;
    }

    if (!get_global_takeoff_position(takeoff_loc))
    {
        // Show coordinate system not set up yet
        return false;
    }

    return _is_close_to_position(
        takeoff_loc,
        xy_threshold > 0 ? xy_threshold : _params.max_xy_placement_error_m,
        0
    );
}
