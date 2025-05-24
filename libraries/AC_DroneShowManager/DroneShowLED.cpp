#include <AP_Math/AP_Math.h>

#include "DroneShowLED.h"

void DroneShowLED::_update_gamma_lookup_table()
{
    int i;
    float ratio;
    float corrected;

    /* Prevent out-of-range values */
    if (!is_equal(_gamma, 1.0f) && _gamma > 0 && _gamma < 10) {
        for (i = 0; i < 256; i++) {
            ratio = static_cast<float>(i) / 255;
            corrected = constrain_float(roundf(powf(ratio, _gamma) * 255), 0, 255);
            _gamma_lookup_table[i] = static_cast<uint8_t>(corrected);
        }
    } else if (_gamma >= 10) {
        /* Extremely large gamma, just use 255 for full brightness and 0 everywhere else */
        for (i = 0; i < 256; i++) {
            _gamma_lookup_table[i] = i == 255 ? 255 : 0;
        }
    } else {
        /* NaN and other pathological values are covered by this branch */
        for (i = 0; i < 256; i++) {
            _gamma_lookup_table[i]  = i;
        }
    }
}
