#pragma once

/// @file   DroneShowPyroDevice.h
/// @brief  Abstract pyrotechnic device that is used by pyro events in a drone show

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "DroneShow_Enums.h"

class DroneShowPyroDevice
{
private:

public:
    DroneShowPyroDevice() {}
    virtual ~DroneShowPyroDevice() {};

    /**
     * Deinitializes the pyrotechnic device.
     */
    virtual void deinit() {};

    /**
     * Initializes the pyrotechnic device.
     * 
     * @return whether the initialization was successful.
     */
    virtual bool init() { return true; };

    /**
     * Fires the given channel of the pyrotechnic device.
     */
    virtual DroneShowEventResult fire(uint8_t channel) = 0;

    /**
     * Turns off the igition of the given channel of the pyrotechnic device.
     */
    virtual DroneShowEventResult off(uint8_t channel) {
        return DroneShowEventResult_SkipLogging;
    };

protected:

private:
};
