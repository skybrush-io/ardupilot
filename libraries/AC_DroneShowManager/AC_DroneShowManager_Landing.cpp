#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

PostAction AC_DroneShowManager::get_action_at_end_of_show() const
{
    // After a collective RTH, the RTH plan has already taken the drone to its
    // landing position (typically all the way to the ground), so we land in
    // place. RTL would take the drone back to its home position instead.
    if (is_collective_rth_triggered()) {
        return PostAction_Land;
    }

    // If the trajectory took the drone close to the ground (or all the way to
    // the ground), we land in place. RTL or loiter would make the drone take
    // off again; this would happen even if the drone has touched down already
    // because the land detector of ArduCopter does not trigger while the drone
    // is following a trajectory that ends at ground level.
    if (_is_close_to_ground_at_end_of_show()) {
        return PostAction_Land;
    }

    switch (_params.post_action) {
        case PostAction_Land:
            return PostAction_Land;

        case PostAction_Loiter:
            return PostAction_Loiter;

        case PostAction_RTL:
            return PostAction_RTL;

        case PostAction_RTLOrLand:
            return (
                _is_at_takeoff_position_xy(2 * DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS) &&
                _trajectory_is_circular
            ) ? PostAction_RTL : PostAction_Land;

        default:
            // Legacy behaviour when we did not have a parameter for the
            // post-show action
            return PostAction_Land;
    }
}

bool AC_DroneShowManager::_is_close_to_ground_at_end_of_show() const
{
    Location loc;
    int32_t altitude_above_home_cm;
    float threshold_cm = MAX_ALTITUDE_FOR_LANDING_IN_PLACE_AT_END_OF_SHOW_METERS * 100.0f;
    int32_t takeoff_altitude_cm = get_takeoff_altitude_cm();

    if (takeoff_altitude_cm > 0) {
        threshold_cm = MIN(threshold_cm, takeoff_altitude_cm * 0.5f);
    }

    if (
        !get_current_location(loc) ||
        !loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_above_home_cm)
    ) {
        // Altitude not known; let the post-show action decide
        return false;
    }

    return altitude_above_home_cm < threshold_cm;
}

float AC_DroneShowManager::get_landing_speed_m_sec() const {
    float value = 0.0f;

    if (AP_Param::get("LAND_SPEED", value)) {
        if (value >= 0.0f && isfinite(value)) {
            return value / 100.0f; // Convert from cm/s to m/s
        }
    }

    return DEFAULT_LANDING_SPEED_METERS_PER_SEC;
}
