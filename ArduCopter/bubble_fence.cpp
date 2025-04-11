#include "Copter.h"

// Code to integrate AC_BubbleFence library with main ArduCopter code

#if MODE_DRONE_SHOW_ENABLED == ENABLED

static void send_breach_notification_if_needed(const char* message);

// bubble_fence_check - inform the bubble fence library about the current path
// deviation and take action if needed
void Copter::bubble_fence_check()
{
    AC_BubbleFence::FenceAction action;

    // Do nothing if the vehicle is disarmed
    if (!AP::arming().is_armed()) {
        return;
    }

    action = g2.drone_show_manager.get_bubble_fence_action();
    switch (action) {
        case AC_BubbleFence::FenceAction::REPORT_ONLY:
            send_breach_notification_if_needed(nullptr);
            break;

        case AC_BubbleFence::FenceAction::FLASH_LIGHTS:
            // Nothing to do, handled inside the drone show manager
            break;

        case AC_BubbleFence::FenceAction::RTL:
            if (set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                send_breach_notification_if_needed("return to launch");
            } else {
                // RTL mode was refused, so we try to land instead
                if (set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED)) {
                    send_breach_notification_if_needed("landing");
                } else {
                    // Land was also refused, so we send a notification
                    send_breach_notification_if_needed("RTL and land refused");
                }
            }
            break;

        case AC_BubbleFence::FenceAction::LAND:
            if (set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED)) {
                send_breach_notification_if_needed("landing");
            } else {
                // Land was also refused, so we send a notification
                send_breach_notification_if_needed("land refused");
            }
            break;

        case AC_BubbleFence::FenceAction::DISARM:
            send_breach_notification_if_needed("disarming");
            force_disarm_without_questions(AP_Arming::Method::FENCEBREACH);
            break;

        default:
            // Nothing to do
            break;
    }
}

static void send_breach_notification_if_needed(const char* message) {
    static uint32_t last_breach_notification_sent = 0;

    if (last_breach_notification_sent < AP_HAL::millis() - 5000) {
        if (message) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Bubble fence breached, %s", message);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Bubble fence breached");
        }
        last_breach_notification_sent = AP_HAL::millis();
    }
}

#endif // MODE_DRONE_SHOW_ENABLED == ENABLED
