#include "AC_DroneShowManager.h"

#include <GCS_MAVLink/GCS.h>

bool AC_DroneShowManager::get_global_takeoff_position(Location& loc) const
{
    // This function may be called any time, not only during the show, so we
    // need to take the parameters provided by the user, convert them into a
    // ShowCoordinateSystem object, and then use that to get the GPS coordinates
    sb_vector3_t vec;

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

float AC_DroneShowManager::get_takeoff_speed_m_sec() const {
    float result = _wp_nav ? _wp_nav->get_default_speed_up() / 100.0f : 0;
    if (result <= 0 || !isfinite(result)) {
        /* safety check */
        result = DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC;
    }
    return result;
}

float AC_DroneShowManager::get_time_until_takeoff_sec()
{
    if (isnan(_projected_wall_clock_time_at_takeoff_sec)) {
        _projected_wall_clock_time_at_takeoff_sec = sb_screenplay_get_time_sec_for_scene_tag_and_warped_time_in_scene(
            &_screenplay, SceneTag_MainShow, _trajectory_stats.takeoff_time_sec
        );
        
        if (isnan(_projected_wall_clock_time_at_takeoff_sec)) {
            // This should not happen, but if it does, we just use infinity
            _projected_wall_clock_time_at_takeoff_sec = INFINITY;
        }
    }

    return get_time_until_start_sec() + _projected_wall_clock_time_at_takeoff_sec;
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
    
    if (!_copy_show_coordinate_system_from_parameters_to(_show_coordinate_system))
    {
        return false;
    }

    return true;
}

void AC_DroneShowManager::notify_takeoff_started()
{
    Location takeoff_location;
    sb_trajectory_t* trajectory;
    sb_vector3_t end;
    float land_speed_mm_s;

    // If the trajectory is circular (i.e. drone is supposed to land where it
    // took off from), tweak the end of the trajectory to account for placement
    // inaccuracies (we want to land where we took off from, not where we
    // _should_ have taken off from in a perfect world).
    //
    // This correction is nice to have but is not crucial. If an error happens
    // in the process below, we just bail out and proceed without the correction.
    if (!_has_option(DroneShowOption_CorrectLandingPositionForCircularTrajectories) ||
        !_trajectory_is_circular || _trajectory_modified_for_landing
    ) {
        return;
    }
    
    if (!get_current_location(takeoff_location))
    {
        return;
    }

    _show_coordinate_system.convert_global_to_show_coordinate(takeoff_location, end);

    // Correction is applied in a way that the horizontal correction is done when we
    // are halfway through the landing (at 50% of the takeoff altitude)
    end.z += get_takeoff_altitude_cm() * 5; /* [cm] --> [mm] */

    // Get a handle to the current trajectory from the show controller so we can
    // modify its end point
    trajectory = sb_screenplay_scene_get_trajectory(&_main_show_scene);
    if (trajectory == nullptr)
    {
        return;
    }
    
    land_speed_mm_s = get_landing_speed_m_sec() * 1000.0f;   /* [mm/s] */
    if (sb_trajectory_replace_end_to_land_at(trajectory, &_trajectory_stats, end, land_speed_mm_s)) {
        return;
    }

    _trajectory_modified_for_landing = true;
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
