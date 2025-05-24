#include "DroneShowLED_I2C.h"

extern const AP_HAL::HAL& hal;

DroneShowLED_I2C::DroneShowLED_I2C(uint8_t bus, uint8_t addr, bool use_white_channel)
    : DroneShowLED(),
    _bus(bus), _addr(addr), _send_required(false), _use_white_channel(use_white_channel)
{
}

bool DroneShowLED_I2C::init()
{
    _dev = hal.i2c_mgr->get_device(_bus, _addr);
    if (!_dev) {
        return false;
    }

    // Register a periodic callback to be called at 50 Hz; this will be used to
    // initiate I2C transfers if needed
    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&DroneShowLED_I2C::_timer, void));
    return true;
}

bool DroneShowLED_I2C::set_raw_rgbw(uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    WITH_SEMAPHORE(_sem);

    _msg[0] = red;
    _msg[1] = green;
    _msg[2] = blue;
    _msg[3] = white;

    _send_required = true;

    return true;
}

void DroneShowLED_I2C::_timer(void)
{
    uint8_t to_send = _use_white_channel ? 4 : 3;

    if (!_send_required) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    _send_required = false;

    _dev->transfer(_msg, to_send, nullptr, 0);
}
