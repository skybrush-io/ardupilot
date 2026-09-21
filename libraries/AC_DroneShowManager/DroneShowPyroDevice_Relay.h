#pragma once

/// @file   DroneShowPyroDevice_Relay.h
/// @brief  Pyrotechnic device that uses GPIO pins to trigger pyro effects

#include "DroneShowPyroDevice.h"

/**
 * Pyro device implementation that sets the appropriate GPIO pins high when a
 * pyro event is encountered in the show sequence. This implementation uses one
 * GPIO pin per pyro channel.
 */
class DroneShowPyroDevice_Relay : public DroneShowPyroDevice {
public:
    DroneShowPyroDevice_Relay(uint32_t relay_index_mask)
        : DroneShowPyroDevice(), _relay_index_mask(relay_index_mask) {}

    uint8_t num_channels() const override;

protected:
    bool init_impl() override;
    void deinit_impl() override;
    DroneShowEventResult fire_impl(uint8_t channel) override;
    DroneShowEventResult off_impl(uint8_t channel) override;

private:
    uint8_t _num_channels;
    uint8_t _relay_indices[8];
    uint32_t _relay_index_mask;
};
