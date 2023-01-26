#pragma once

#include "AP_Beacon_Backend.h"
#include <AP_HAL/AP_HAL.h>

#define ALLOW_DOUBLE_MATH_FUNCTIONS
#define ALLOW_DOUBLE_TRIG_FUNCTIONS  //enable double math

extern const AP_HAL::HAL& hal;

class AP_Beacon_LuminousBees : public AP_Beacon_Backend
{
public:
    // constructor
    AP_Beacon_LuminousBees(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // update
    void update() override;

    bool healthy() override;

private:

    void send_externalNav(Vector3f position_ned, float position_error, uint64_t pkt0_time, Vector3f estimated_position);

    uint32_t _last_estimation_update;

    AP_HAL::UARTDriver *_tag;
    uint32_t last_update_ms = 0;
};
