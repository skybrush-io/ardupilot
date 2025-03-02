// Configuration
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Terrain/AP_Terrain.h>

#include "defines.h"
#include "config.h"

#include "collmot_flockctrl.h"

#if COLLMOT_EXTENSIONS_ENABLED == ENABLED

// table of user settable parameters
const AP_Param::GroupInfo CollMotFlockCtrl::var_info[] = {

    // @Param: FLOCK_OPTIONS
    // @DisplayName: flockctrl options bitmask
    // @Description: Bitmask of what addition behaviour to enable when controlled from flockctrl.
    // @Bitmask: 0:All,1:Allow continuing in guided mode without GCS connection,2:Do not reset yaw mode to default in guided mode
    // @User: Advanced
    AP_GROUPINFO("FLOCK_OPTIONS",  0, CollMotFlockCtrl, _options, 0),

    AP_GROUPEND
};

CollMotFlockCtrl::CollMotFlockCtrl() {

}

#endif
