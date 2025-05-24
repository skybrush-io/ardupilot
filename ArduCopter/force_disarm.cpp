#include "Copter.h"

/// Disarms the vehicle forcibly and unconditionally. Used for the hard fence,
/// the bubble fence and the EKF failsafe when it is set to "disarm".
void Copter::force_disarm_without_questions(const AP_Arming::Method method)
{
    // Try to disarm the motors forcibly via the AP_Arming module
    if (!AP::arming().disarm(method, /* do_disarm_checks = */ false)) {
        // AP_Arming module refused to disarm. There must be a reason for this,
        // but this function is meant for cases when we do not want to be
        // overridden at all, so we talk directly to the motors instead
        motors->armed(false);
        motors->output();
    }
}
