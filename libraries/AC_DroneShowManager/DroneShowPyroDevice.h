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
    DroneShowPyroDevice() : _ignited_at(0) {}
    virtual ~DroneShowPyroDevice() {
        deinit();
    };

    /**
     * Deinitializes the pyrotechnic device.
     * 
     * Device-specific parts should be added in the `deinit_impl()`
     * function, which is to be overridden by derived classes.
     */
    void deinit() {
        deinit_impl();

        if (_ignited_at) {
            free(_ignited_at);
            _ignited_at = NULL;
        }
    };

    /**
     * Initializes the pyrotechnic device.
     * 
     * Device-specific parts should be added in the `init_impl()`
     * function, which is to be overridden by derived classes.
     * 
     * @return whether the initialization was successful.
     */
    bool init() {
        uint8_t num_channels = this->num_channels();

        if (_ignited_at) {
            deinit();
        }

        if (num_channels > 0) {
            _ignited_at = static_cast<uint32_t*>(calloc(num_channels, sizeof(uint32_t)));
            if (!_ignited_at) {
                return false; // Memory allocation failed
            }
        }

        if (!init_impl()) {
            deinit();
            return false; // Device-specific initialization failed
        }

        return true; // Initialization successful
    };

    /**
     * Fires the given channel of the pyrotechnic device.
     * 
     * No safety checks are to be performed in this function, it is assumed that
     * whenever this function is called, the safety checks have already been
     * performed _or_ the function is called from a test event that is not
     * subject to safety checks.
     * 
     * This function also performs extra bookkeeping that takes care of recording
     * the time when a pyro channel was fired so we can turn it off automatically
     * after the ignition period. To this end, this function is not to be
     * overridden. Derived classes should override `fire_impl()` instead.
     */
    DroneShowEventResult fire(uint8_t channel) {
        DroneShowEventResult result = DroneShowEventResult_NotSupported;

        if (channel < num_channels()) {
            result = fire_impl(channel);
            if (result == DroneShowEventResult_Success) {
                if (_ignited_at) {
                    _ignited_at[channel] = AP_HAL::millis();
                }
            }
        }

        return result;
    }

    /**
     * @brief Returns the number of channels supported by this pyrotechnic device.
     */
    virtual uint8_t num_channels() const {
        return 0;
    }

    /**
     * Turns off the ignition of the given channel of the pyrotechnic device.
     * 
     * This function also performs extra bookkeeping that takes care of resetting
     * the recorded time when a pyro channel was fired. To this end, this
     * function is not to be overridden. Derived classes should override
     * `off_inner()` instead.
     */
    DroneShowEventResult off(uint8_t channel) {
        DroneShowEventResult result = DroneShowEventResult_NotSupported;

        if (channel < num_channels()) {
            result = off_impl(channel);

            // Reset the ignition time for the channel unconditionally, even if
            // the off_impl() function failed. This is to ensure that we do not
            // keep on calling off_impl() for the same channel repeatedly.
            if (_ignited_at) {
                _ignited_at[channel] = 0;
            }
        }

        return result;
    };

    /**
     * @brief Turns off all channels that have been ignited for longer than the given duration.
     * 
     * @param duration_msec  the max duration for which a channel can remain ignited;
     *        zero or negative values mean that no channels should be turned off
     *        automatically.
     */
    void turn_off_channels_if_needed(int32_t duration_msec) {
        if (duration_msec <= 0) {
            return; // No channels to turn off
        }

        if (!_ignited_at) {
            // Ignition time array not initialized, nothing to do
            return;
        }

        uint8_t num_channels = this->num_channels();
        uint32_t now = AP_HAL::millis();
        for (uint8_t i = 0; i < num_channels; i++) {
            if (_ignited_at[i] != 0 && (now - _ignited_at[i]) >= static_cast<uint32_t>(duration_msec)) {
                off(i);
            }
        }
    }

protected:
    virtual bool init_impl() { return true; }
    virtual void deinit_impl() {}
    virtual DroneShowEventResult fire_impl(uint8_t channel) = 0;
    virtual DroneShowEventResult off_impl(uint8_t channel) = 0;

private:
    /**
     * Array storing the timestamps when the channels were ignited; zero if
     * they are currently off.
     */
    uint32_t *_ignited_at;
};
