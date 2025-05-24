#pragma once

/// @file   AC_DroneShowLED_UART.h
/// @brief  Drone show LED that sends its output to a serial UART in some unspecified format

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "DroneShowLED.h"

/**
 * RGB LED implementation that sends the current state of the RGB LED in
 * some unspecified format to a UART.
 * 
 * This is an abstract class; should be subclassed with the details of the
 * protocol.
 */
class DroneShowLED_UART : public DroneShowLED {
public:
    DroneShowLED_UART(uint8_t uart_index = 0);

    /* Do not allow copies */
    DroneShowLED_UART(const DroneShowLED_UART &other) = delete;
    DroneShowLED_UART &operator=(const DroneShowLED_UART&) = delete;

protected:
    bool init(void) override;
    bool set_raw_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) override;

    /* Sends a command to the given UART that sets the given RGB values on
     * the LED. Must be overridden in subclasses. uart is guaranteed not to
     * be a null pointer.
     * 
     * Returns whether the sending attempt was successful. Returning false
     * means that we will attempt to re-send the same command in the next
     * iteration.
     */
    virtual bool send_rgbw_command_to_uart(AP_HAL::UARTDriver* uart, uint8_t r, uint8_t g, uint8_t b, uint8_t w) = 0;

private:
    uint8_t _uart_index;
};
