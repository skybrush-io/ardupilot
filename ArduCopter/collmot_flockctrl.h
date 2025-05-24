#pragma once

#include <AP_Param/AP_Param.h>

#if COLLMOT_EXTENSIONS_ENABLED

class CollMotFlockCtrl {

public:

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // constructor
    CollMotFlockCtrl();

    /* Do not allow copies */
    CollMotFlockCtrl(const CollMotFlockCtrl &other) = delete;
    CollMotFlockCtrl &operator=(const CollMotFlockCtrl&) = delete;

    // Returns whether we allow continuing a mission in guided mode if there
    // is no RC and no GCS connection
    bool allowContinueInGuidedModeWithoutGCSAndRC() {
        return isOptionSet(Options::AllowGuidedContinueWithoutGCSAndRC);
    }

    // Returns whether we prevent resetting the yaw mode to the default
    // WP_YAW_BEHAVIOUR in guided mode when a guided mode command is set
    // without specifying yaw or yaw rate
    //
    // Note the double negation in the body -- this is intentional and needed
    // for backward compatibility
    bool shouldResetToDefaultYawModeInGuidedMode() {
        return !isOptionSet(Options::DoNotResetGuidedYawModeToDefault);
    }

private:

    // Enum specifying the meaning of individual bits in the CM_FLOCK_OPTS
    // parameter
    enum class Options : int32_t {
        All = (1 << 0U),
        AllowGuidedContinueWithoutGCSAndRC = (1 << 1U),
        DoNotResetGuidedYawModeToDefault = (1 << 2U)
    };

    // Placeholder for the value of the CM_FLOCK_OPTS parameter
    AP_Int32 _options;

    bool isOptionSet(Options option) {
        return _options & (
            static_cast<int32_t>(Options::All) |
            static_cast<int32_t>(option)
        );
    }
};

#endif
