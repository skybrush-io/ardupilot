#include <AC_Fence/AC_Fence.h>
#include <AP_Notify/AP_Notify.h>
#include "AC_DroneShowManager.h"

#include "DroneShowPyroDeviceFactory.h"

bool AC_DroneShowManager::_is_pyro_safe_to_fire() const
{
    // The pyro is safe to fire if and only if:
    //
    // - The show is currently performing (implies that we are in show mode)
    // - Motors are running (implies that the crash check has not triggered)
    // - We are currently flying, have absolute position, and GPS is not glitching
    // - Geofence is not breached
    // - The drone is not in EKF or battery failsafe
    // - We are not drifting from the expected position during the show
    // - The altitude is higher than the minimum altitude for firing pyrotechnics
    if (
        get_stage_in_drone_show_mode() != DroneShow_Performing ||
        !AP_Notify::flags.armed ||
        !AP_Notify::flags.flying ||
        !AP_Notify::flags.have_pos_abs ||
        AP_Notify::flags.gps_glitching ||
        AP_Notify::flags.ekf_bad ||
        AP_Notify::flags.failsafe_battery ||
        AP::fence()->get_breaches() != 0 ||
        !_is_at_expected_position()
    ) {
        return false;
    }

    Location current_loc;
    float alt_dist;

    if (!get_current_location(current_loc)) {
        // No position known, it is not safe to fire
        return false;
    }

    if (!current_loc.get_alt_m(Location::AltFrame::ABOVE_HOME, alt_dist)) {
        // Altitude frame is not usable; this should not happen
        return false;
    }

    if (alt_dist < _params.pyro_min_altitude_m) {
        // Altitude is too low
        return false;
    }

    return true;
}

void AC_DroneShowManager::_update_pyro_device()
{
    if (_pyro_device != NULL) {
        _pyro_device->turn_off_channels_if_needed(_params.pyro_spec.ignition_duration_msec);
    }
}

void AC_DroneShowManager::_update_pyro_device_instance()
{
    static int previous_pyro_type = -1;

    // We try to avoid the re-creation of _pyro_device if the type of the pyro
    // device did not change

    if (_pyro_device_factory) {
        int pyro_type = _params.pyro_spec.type;

        if (pyro_type != previous_pyro_type) {
            // Deinitializes the old pyrotechnic device if it exists
            if (_pyro_device)
            {
                _pyro_device->deinit();

                delete _pyro_device;
                _pyro_device = NULL;
            }

            // Construct the new device
            _pyro_device = _pyro_device_factory->new_pyro_device_by_type(
                static_cast<DroneShowPyroDeviceType>(pyro_type)
            );

            // Store the settings
            previous_pyro_type = pyro_type;
        }
    }

    if (_pyro_device) {
        // Update settings if needed.
        // Nothing to do here at the moment.
    }
}
