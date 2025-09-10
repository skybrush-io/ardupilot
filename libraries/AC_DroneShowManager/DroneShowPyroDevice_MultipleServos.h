#pragma once

/// @file   DroneShowPyroDevice_MultipleServos.h
/// @brief  Pyrotechnic device that uses a separate servo output for all pyro channels to trigger pyro effects

#include "DroneShowPyroDevice.h"
#include "DroneShow_Constants.h"

/**
 * Pyro device implementation that sets the predefined servo output on the
 * proper servo channel when a pyro event is encountered in the show sequence.
 * This implementation uses one servo channel per pyro channel.
 */
class DroneShowPyroDevice_MultipleServos : public DroneShowPyroDevice {
public:
    DroneShowPyroDevice_MultipleServos(uint32_t servo_channel_mask)
        : DroneShowPyroDevice(), _servo_channel_mask(servo_channel_mask) {}

    uint8_t num_channels() const override;

protected:
    bool init_impl() override;
    void deinit_impl() override;
    DroneShowEventResult fire_impl(uint8_t channel) override;
    DroneShowEventResult off_impl(uint8_t channel) override;
    bool set_duty_cycle_percentage(uint8_t servo_channel, uint8_t pct) const;

private:
    uint8_t _num_servo_channels;
    uint8_t _servo_channels[PYRO_MULTIPLE_SERVOS_MAX_CHANNELS];
    uint32_t _servo_channel_mask;
};
