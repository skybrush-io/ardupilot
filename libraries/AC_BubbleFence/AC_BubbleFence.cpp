#include "AC_BubbleFence.h"

AC_BubbleFence::AC_BubbleFence() :
    _state(BreachState::NONE),
    _current_breach_started_at(0),
    _last_action_performed_at(0)
{
}

AC_BubbleFence::~AC_BubbleFence()
{
}

void AC_BubbleFence::init()
{
    _reset();
}

AC_BubbleFence::FenceAction AC_BubbleFence::notify_distance_from_desired_position(Vector3f& distance)
{
    bool dist_too_large = (
        (_params.distance_xy > 0 && hypotf(distance.x, distance.y) > _params.distance_xy) ||
        (_params.distance_z > 0 && fabsf(distance.z) > _params.distance_z)
    );

    if (dist_too_large)
    {
        if (_state == BreachState::NONE)
        {
            // Fence triggered
            _state = BreachState::SOFT;
            _current_breach_started_at = AP_HAL::millis();
            _last_action_performed_at = 0;
        }
    }
    else
    {
        if (_state != BreachState::NONE)
        {
            // Fence not triggered any more
            _state = BreachState::NONE;
            _current_breach_started_at = 0;
            _last_action_performed_at = 0;
        }
    }

    if (_state == BreachState::SOFT)
    {
        if (AP_HAL::millis() - _current_breach_started_at > _params.timeout * 1000)
        {
            _state = BreachState::HARD;
            _last_action_performed_at = 0;
        }
    }

    if (_state == BreachState::HARD)
    {
        // Perform the action once per second
        if (_last_action_performed_at == 0 || AP_HAL::millis() - _last_action_performed_at > 1000)
        {
            _last_action_performed_at = AP_HAL::millis();
            _action_counter++;
            return static_cast<FenceAction>(static_cast<uint8_t>(_params.action));
        }
    }

    // Nothing to do at the moment
    return FenceAction::NONE;
}

void AC_BubbleFence::_reset()
{
    _state = BreachState::NONE;
    _current_breach_started_at = 0;
    _last_action_performed_at = 0;
    _action_counter = 0;
}
