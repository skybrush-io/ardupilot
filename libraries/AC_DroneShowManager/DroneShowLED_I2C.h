#pragma once

/// @file   AC_DroneShowLED_I2C.h
/// @brief  Drone show LED that sends its output to an I2C bus.

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#include "DroneShowLED.h"


class DroneShowLED_I2C : public DroneShowLED {

public:
    /// Constructor
    DroneShowLED_I2C(uint8_t bus, uint8_t addr, bool use_white_channel = false);

    /* Do not allow copies */
    DroneShowLED_I2C(const DroneShowLED_I2C &other) = delete;
    DroneShowLED_I2C &operator=(const DroneShowLED_I2C&) = delete;

    bool init(void) override;
    bool supports_white_channel() override { return _use_white_channel; }

protected:
    bool set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) override;

private:
    /** The index of the I2C bus */
    uint8_t _bus;

    /** The 7-bit device address on the I2C bus */
    uint8_t _addr;

    /** I2C device that can be used to write to the I2C bus */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    /** Semaphore that synchronizes access to certain parts of the I2C LED
     * data structure between the main thread and the I2C timer thread */
    HAL_Semaphore _sem;

    /** Storage for the message that we are about to send on the I2C bus.
     * Protected by the semaphore. Uses 4 bytes because we anticipate RGBW
     * LEDs, although we may transfer only 3 bytes if we are configured not
     * to use the W channel */
    uint8_t _msg[4];

    /** Stores whether a new command has to be sent on the I2C bus to change the
     * LED state */
    bool _send_required;

    /** Stores whether we should send the W channel */
    bool _use_white_channel;

    void _timer(void);
};
