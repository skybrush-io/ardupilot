#pragma once

/// @file   DroneShowPyroDevice_Cobra.h
/// @brief  Cobra firing module support

#include "DroneShowPyroDevice.h"

/**
 * Pyro device implementation that talks to a Cobra 6M firing module using a
 * specific UART.
 */
class DroneShowPyroDevice_Cobra : public DroneShowPyroDevice {
public:
    DroneShowPyroDevice_Cobra() : DroneShowPyroDevice(), _serial(nullptr) {};
    uint8_t num_channels() const override;

protected:
    bool init_impl() override;
    DroneShowEventResult fire_impl(uint8_t channel) override;
    DroneShowEventResult off_impl(uint8_t channel) override;

private:
    AP_HAL::UARTDriver* _serial;

    void send_command(uint8_t command, uint8_t data);
};
