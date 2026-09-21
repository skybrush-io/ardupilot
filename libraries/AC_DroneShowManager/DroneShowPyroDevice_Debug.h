#pragma once

/// @file   DroneShowPyroDevice_Debug.h
/// @brief  Pyrotechnic "device" that sends its output in MAVLink STATUSTEXT messages for debugging purposes.

#include "DroneShowPyroDevice.h"

/**
 * Pyro device implementation that sends STATUSTEXT messages on all MAVLink
 * channels when a pyro event is encountered in the show sequence.
 */
class DroneShowPyroDevice_Debug : public DroneShowPyroDevice {
public:
    DroneShowPyroDevice_Debug() : DroneShowPyroDevice() {};
    uint8_t num_channels() const override;

protected:
    DroneShowEventResult fire_impl(uint8_t channel) override;
    DroneShowEventResult off_impl(uint8_t channel) override;

private:
};
