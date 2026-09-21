#pragma once

/// @file   AC_BubbleFence.h
/// @brief  Drift-based geofence for drone shows with a configurable action after a prolonged breach

#include <AP_Common/Location.h>

/// @class  AC_BubbleFence
/// @brief  Drift-based geofence for drone shows with a configurable action after a prolonged breach
class AC_BubbleFence {

private:
    enum BreachState {
        // No breach reported at the moment
        NONE = 0,

        // Fence was triggered, but we are waiting for the timeout to
        // pass before declaring it a hard breach
        SOFT = 1,

        // Fence was triggered for long enough to perform the associated action
        HARD = 2
    };

public:
    enum class FenceAction : uint8_t {
        // Do nothing at all
        NONE = 0,

        // Count how many times the fence would have triggered but do not take
        // any action
        REPORT_ONLY = 1,

        // Flash the lights of the drone when the fence is triggered
        FLASH_LIGHTS = 2,

        // Switch to RTL mode when the fence is triggered
        RTL = 3,

        // Switch to land mode when the fence is triggered
        LAND = 4,

        // Disarm the motors when the fence is triggered
        DISARM = 5
    };

public:
    AC_BubbleFence();
    ~AC_BubbleFence();

    /* Do not allow copies */
    AC_BubbleFence(const AC_BubbleFence &other) = delete;
    AC_BubbleFence &operator=(const AC_BubbleFence&) = delete;

    // Initializes the bubble fence module at boot time
    void init();

    // Returns the breach state of the bubble fence
    BreachState get_breach_state() const { return _state; }

    // Returns how many times the fence action was triggered
    uint16_t get_action_counter() const { return _action_counter; }

    // Returns whether the bubble fence is breacned
    bool is_breached() const { return _state != BreachState::NONE; }

    // Notifies the bubble fence about the distance to the desired position
    // of the drone
    FenceAction notify_distance_from_desired_position(Vector3f& distance);

    // Resets the action counter to zero
    void reset_action_counter() { _action_counter = 0; }

    // Returns whether the fence action is configured to flash the LEDs of the drone
    bool should_flash_leds() const { return _params.action == static_cast<uint8_t>(FenceAction::FLASH_LIGHTS); }

private:
    // Structure holding all the parameters settable by the user
    struct {
        // Stores whether the fence is enabled
        AP_Int8 enabled;

        // Maximum allowed distance between the show path and the current
        // position in the XY plane before the fence is triggered
        AP_Float distance_xy;

        // Maximum allowed distance between the show path and the current
        // position in the Z axis before the fence is triggered
        AP_Float distance_z;

        // Action to perform when the fence is breached
        AP_Int8 action;

        // Number of seconds that the drone needs to spend in a triggered state
        // to perform the associated action
        AP_Float timeout;
    } _params;

    // Current breach state
    BreachState _state;

    // Timestamp holding the time when the current breach started, in milliseconds.
    uint32_t _current_breach_started_at;

    // Timestamp of the last action performed, in milliseconds.
    uint32_t _last_action_performed_at;

    // Number of times the bubble fence action was executed
    uint16_t _action_counter;

    // Resets the state of the hard fence module
    void _reset();

    friend class AC_DroneShowManager;
};
