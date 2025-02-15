#include "Copter.h"

// Code to integrate AC_HardFence library with main ArduCopter code

#if AP_FENCE_ENABLED && MODE_DRONE_SHOW_ENABLED == ENABLED

// hard_fence_check - inform the hard fence library about the active breaches
// and take action if needed
void Copter::hard_fence_check()
{
    static uint32_t last_breach_notification_sent = 0;

    // Do nothing if the vehicle is disarmed
    if (!AP::arming().is_armed()) {
        return;
    }

    // Check the current breaches, forward it to the hard fence module and let
    // it make a decision. If the fence is breached, disarm.
    uint8_t breaches = fence.get_breaches();

    // fence.get_breach_distance() is not reliable as it does not include
    // polygon fences, so we don't use that. See AC_HardFence.cpp for the
    // logic, which does not require fence.get_breach_distance()
    if (g2.drone_show_manager.hard_fence.notify_active_breaches(breaches)) {
        // Module requested the motors to be stopped forcibly. Send a notification
        // first if we haven't done so recently. There is no need to check whether
        // the hard fence is enabled as it is not supposed to return true if it
        // is disabled.
        if (last_breach_notification_sent < AP_HAL::millis() - 5000) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Hard fence breached, disarming");
            last_breach_notification_sent = AP_HAL::millis();
        }

        // Try to disarm the motors forcibly via the AP_Arming module
        if (!AP::arming().disarm(AP_Arming::Method::FENCEBREACH, /* do_disarm_checks = */ false)) {
            // AP_Arming module refused to disarm. There must be a reason for this,
            // but the hard fence overrides everything, so we talk directly to the
            // motors instead
            motors->armed(false);
        }
    }
}

#endif // AP_FENCE_ENABLED && MODE_DRONE_SHOW_ENABLED == ENABLED
