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

void AC_DroneShowManager::_clear_start_time_after_landing()
{
    _params.start_time_gps_sec.set(-1);
    _start_time_on_internal_clock_usec = 0;
    _check_changes_in_parameters();
}

void AC_DroneShowManager::_handle_switch_to_landed_state()
{
    _cancel_requested = false;

    // Let's not clear the start time; there's not really much point but at
    // least we don't confuse the GCS (not Skybrush but Mission Planner) with
    // a parameter suddenly changing behind its back. This is just a theoretical
    // possibility but let us be on the safe side.
    // _clear_start_time_after_landing();
}
