#pragma once

/// @file   AC_HardFence.h
/// @brief  Hard geofence with motor shutdown after a prolonged breach

#include <AP_Common/Location.h>

/// @class  AC_HardFence
/// @brief  Class managing a hard geofence that shuts down the motors after a prolonged breach
class AC_HardFence {

private:
    enum BreachState {
        // No breach reported at the moment
        NONE = 0,

        // Fence breach was reported, but we are not far enough from the breach
        // point to classify it as a hard breach
        SOFT = 1,

        // Fence breach was reported and we are far enough from the breach point
        HARD = 2
    };

public:
    AC_HardFence();
    ~AC_HardFence();

    /* Do not allow copies */
    AC_HardFence(const AC_HardFence &other) = delete;
    AC_HardFence &operator=(const AC_HardFence&) = delete;

    // Initializes the hard geofence module at boot time
    void init();

    // Returns the current breach state of the hard fence
    BreachState get_breach_state() const { return _state; }

    // Notifies the hard geofence about the current set of breaches from the
    // fence module. Should be updated whenever the fence module of the vehicle
    // checked for breaches. The parameter is a bit field where each bit belongs
    // to a different type of fence; passing zero means that no fences are
    // breached at the moment.
    //
    // Returns whether the module requests the motors to stop.
    //
    // It is assumed that this functions is called regularly, at least once
    // per second.
    bool notify_active_breaches(uint8_t breaches) WARN_IF_UNUSED;

    // Notifies the hard geofence that there are no active breaches at the moment.
    //
    // Returns whether the module requests the motors to stop. Should always be
    // false in a sensible implementation.
    bool notify_no_breaches() WARN_IF_UNUSED {
        return notify_active_breaches(0);
    }

    // Returns the number of seconds elapsed since the last breach if there is
    // at least one active breach, or a large negative number if there are no breaches
    float get_seconds_since_last_soft_breach() const {
        return (
            _state >= BreachState::SOFT
            ? (AP_HAL::millis() - _current_soft_breach_started_at) / 1000.0f
            : -1000000.0f
        );
    }

    // Returns the number of seconds elapsed since the time the current breach
    // was classified as a hard breach, or a large negative number if the current
    // breach is not a hard breach yet
    float get_seconds_since_last_hard_breach() const {
        return (
            _state >= BreachState::HARD
            ? (AP_HAL::millis() - _current_hard_breach_started_at) / 1000.0f
            : -1000000.0f
        );
    }

private:
    // Structure holding all the parameters settable by the user
    struct {
        // Stores whether the fence is enabled
        AP_Int8 enabled;

        // Minimum distance between the normal geofence and its hard counterpart
        AP_Float distance;

        // Number of seconds that the drone needs to spend outside the hard
        // geofence to trigger a shutdown
        AP_Float timeout;
    } _params;

    // Current breach state
    BreachState _state;

    // Timestamp holding the time when the current breach started, in milliseconds.
    uint32_t _current_soft_breach_started_at;

    // Timestamp holding the time when the current _hard_ breach started, in
    // milliseconds. A hard breach starts if there is a soft breach and the
    // distance from the last breach point is larger than the distance threshold.
    uint32_t _current_hard_breach_started_at;

    // Position where the current breach started.
    struct Location _current_breach_point;

    // Stores whether _current_breach_point has valid location data.
    bool _current_breach_point_valid;

    // Current distance from the point where the fence was breached, in meters
    float _current_distance_from_fence;

    // Returns the distance of the vehicle from the current breach point, or
    // returns zero if there was no breach. Returns a large number if it is not
    // possible to retrieve the current position from the AHRS subsystem (e.g.,
    // because the position lock was lost).
    float _get_distance_from_breach_point() const;

    // Records the current location of the vehicle as the breach point. Failures
    // are handled gracefully.
    void _record_current_location_as_breach_point();

    // Resets the state of the hard fence module
    void _reset();

    // Sets the state of the hard fence module. This is the function that all
    // modifications of _state should pass through
    void _set_state(BreachState new_state);

    // Returns whether the hard fence module suggests that the vehicle should disarm
    bool _should_disarm() const;

    // Shows a message to the GCS when the state of the hard fence changes
    void _show_gcs_message_on_state_change(BreachState old_state = BreachState::NONE) const;

    friend class AC_DroneShowManager;
};
