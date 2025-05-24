#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#include "AC_HardFence.h"

AC_HardFence::AC_HardFence() :
    _state(BreachState::NONE),
    _current_soft_breach_started_at(0),
    _current_hard_breach_started_at(0),
    _current_breach_point_valid(false),
    _current_distance_from_fence(0)
{
}

AC_HardFence::~AC_HardFence()
{
}

void AC_HardFence::init()
{
    _reset();
}

bool AC_HardFence::notify_active_breaches(uint8_t breaches)
{
    static bool _was_enabled = false;
    bool is_enabled = _params.enabled;

    if (is_enabled && !_was_enabled) {
        BreachState current_state = _state;

        // We have just been activated. If the current state of the hard fence
        // is not 'none" (i.e. there is a soft or hard breach), let's make sure
        // that the timeout is cleared so the drone doesn't fall off the sky
        // immediately even if it has been out of the hard fence for a long
        // time now
        if (current_state >= BreachState::HARD) {
            _current_hard_breach_started_at = AP_HAL::millis();
        }
        if (current_state >= BreachState::SOFT) {
            _current_soft_breach_started_at = AP_HAL::millis();
        }
        if (current_state >= BreachState::SOFT) {
            _show_gcs_message_on_state_change();
        }
    }

    _was_enabled = is_enabled;

    switch (_state) {
        case BreachState::NONE:
            // If there were no breaches so far, and we have some now, that's a
            // soft breach. We cannot make it a hard breach immediately because
            // first we need to record our current position as the basis for
            // distance comparisons.
            if (breaches) {
                _set_state(BreachState::SOFT);
            }
            break;

        case BreachState::SOFT:
            // If we have no breaches now, return to base state
            if (!breaches) {
                _set_state(BreachState::NONE);
            } else if (
                _params.distance >= 0 &&
                _get_distance_from_breach_point() > _params.distance
            ) {
                // Soft breaches become hard breaches if the distance from the
                // last breach point is large enough
                _set_state(BreachState::HARD);
            }
            break;

        case BreachState::HARD:
            // If we have no breaches now, return to base state
            if (!breaches) {
                _set_state(BreachState::NONE);
            } else if (
                _params.distance < 0 || _get_distance_from_breach_point() <= _params.distance
            ) {
                // Hard breaches become soft breaches if the distance from the
                // last breach point is small enough
                _set_state(BreachState::SOFT);
            }
            break;

        default:
            // Nothing to do, no breach
            break;
    }

    return _should_disarm();
}

float AC_HardFence::_get_distance_from_breach_point() const
{
    struct Location loc;

    if (_state < BreachState::SOFT) {
        // No breach, return zero distance.
        return 0.0f;
    }

    if (AP::ahrs().get_location(loc)) {
        if (_current_breach_point_valid) {
            return loc.get_distance(_current_breach_point);
        } else {
            // This _may_ happen if the GPS lock is completely lost (not sure if
            // it does, because in that case there could not be a fence breach
            // at all). Anyhow, in case of a GPS loss the failsafe mechanisms
            // kick in and try to land the drone immediately so we should not
            // interfere.
            return 0.0f;
        }
    } else {
        // Cannot retrieve current position. If we do have a breach, treat it
        // as having infinite distance from the fence as it seems serious
        // enough. Note that this condition still has to prevail for at least
        // SHOW_HFENCE_TO seconds to cause a motor disarm.
        return FLT_MAX;
    }
}

void AC_HardFence::_record_current_location_as_breach_point()
{
    _current_breach_point_valid = AP::ahrs().get_location(_current_breach_point);
}

void AC_HardFence::_reset()
{
    _state = BreachState::NONE;
    _current_soft_breach_started_at = 0;
    _current_hard_breach_started_at = 0;
    _current_breach_point_valid = false;
    _current_breach_point.zero();
    _current_distance_from_fence = 0;
}

void AC_HardFence::_set_state(BreachState new_state)
{
    BreachState old_state;

    if (new_state == _state) {
        return;
    }

    old_state = _state;
    _state = new_state;

    // Show messages if we enter or exit the hard breach state, but only if we
    // are enabled -- otherwise we just track the breach state but don't
    // report it
    if (_params.enabled) {
        _show_gcs_message_on_state_change(old_state);
    }

    switch (_state) {
        case BreachState::NONE:
            _reset();
            break;

        case BreachState::SOFT:
        case BreachState::HARD:
            if (old_state == BreachState::NONE) {
                // Breach has started now so remember the current position
                _current_soft_breach_started_at = AP_HAL::millis();
                _record_current_location_as_breach_point();
            }
            _current_hard_breach_started_at = (
                _state == BreachState::HARD ? AP_HAL::millis() : 0
            );
            break;

        default:
            // We should not get here so reset
            _reset();
            break;
    }
}

bool AC_HardFence::_should_disarm() const
{
    // Do not disarm if there are no breaches, the hard fence is disabled or
    // we have a negative distance threshold (which does not make sense). Note the
    // condition -- it also protects us from the case when _params.distance
    // somehow ends up being NaN
    if (_state == BreachState::NONE || !_params.enabled || !(_params.distance >= 0)) {
        return false;
    } else {
        return get_seconds_since_last_hard_breach() >= _params.timeout;
    }
}

void AC_HardFence::_show_gcs_message_on_state_change(BreachState old_state) const
{
    if (_state == BreachState::HARD) {
        if (_params.timeout > 0) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Hard fence breached, countdown started");
        } else {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Hard fence breached");
        }
    } else if (old_state == BreachState::HARD) {
        gcs().send_text(MAV_SEVERITY_INFO, "Hard fence breach resolved");
    }
}
