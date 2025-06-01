#pragma once

/// @file   DroneShowPyroDevice_SingleServo.h
/// @brief  Pyrotechnic device that uses a single servo output to trigger pyro effects

#include "DroneShowPyroDevice.h"

/**
 * Pyro device implementation that sends STATUSTEXT messages on all MAVLink
 * channels when a pyro event is encountered in the show sequence.
 */
class DroneShowPyroDevice_SingleServo : public DroneShowPyroDevice {
public:
    DroneShowPyroDevice_SingleServo(uint8_t servo_channel);

    bool init() override;
    void deinit() override;
    DroneShowEventResult fire(uint8_t channel) override;
    DroneShowEventResult off(uint8_t channel) override;

protected:
    bool set_duty_cycle_percentage(uint8_t pct) const;

private:
    uint8_t _servo_channel;
};
