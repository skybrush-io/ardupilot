#pragma once

/// @file   AC_DroneShowLED_UART_WGDrones.h
/// @brief  Drone show LED that sends its output to a serial UART in WGDrones format

#include "DroneShowLED_UART.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED in
 * WGDrones format to a UART.
 */
class DroneShowLED_UART_WGDrones : public DroneShowLED_UART {
public:
    DroneShowLED_UART_WGDrones(uint8_t uart_index = 0) : DroneShowLED_UART(uart_index) {}

    /* Do not allow copies */
    DroneShowLED_UART_WGDrones(const DroneShowLED_UART_WGDrones &other) = delete;
    DroneShowLED_UART_WGDrones &operator=(const DroneShowLED_UART_WGDrones&) = delete;

protected:
    bool send_rgbw_command_to_uart(
        AP_HAL::UARTDriver* uart, uint8_t red, uint8_t green, uint8_t blue, uint8_t white
    ) override {
        // white ignored
        uart->printf("$c1@%d\n$c2@%d\n$c3@%d\n", red, green, blue);
        return true;
    }
};
