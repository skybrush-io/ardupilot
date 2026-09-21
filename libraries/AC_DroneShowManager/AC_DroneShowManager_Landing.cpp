#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

PostAction AC_DroneShowManager::get_action_at_end_of_show() const
{
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

float AC_DroneShowManager::get_landing_speed_m_sec() const {
    float value = 0.0f;

    if (AP_Param::get("LAND_SPEED", value)) {
        if (value >= 0.0f && isfinite(value)) {
            return value / 100.0f; // Convert from cm/s to m/s
        }
    }

    return DEFAULT_LANDING_SPEED_METERS_PER_SEC;
}
