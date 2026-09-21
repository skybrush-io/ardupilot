#include <AP_Relay/AP_Relay.h>

#include "DroneShowPyroDevice_Relay.h"

extern const AP_HAL::HAL& hal;

bool DroneShowPyroDevice_Relay::init_impl()
{
    uint32_t relays = _relay_index_mask;
    uint8_t relay_index;
    const uint8_t max_channels = sizeof(_relay_indices) / sizeof(_relay_indices[0]);

    memset(_relay_indices, 0, sizeof(_relay_indices));
    _num_channels = 0;

    while (relays && _num_channels < max_channels) {
        relay_index = __builtin_ffs(relays);
        relays &= relays - 1;

        if (relay_index--) {
            AP::relay()->off(relay_index);
            _relay_indices[_num_channels++] = relay_index;
        }
    }

    return true;
}

void DroneShowPyroDevice_Relay::deinit_impl()
{
    for (uint8_t i = 0; i < _num_channels; i++) {
        AP::relay()->off(_relay_indices[i]);
    }

    _num_channels = 0;
}

uint8_t DroneShowPyroDevice_Relay::num_channels() const
{
    return _num_channels;
}

DroneShowEventResult DroneShowPyroDevice_Relay::fire_impl(uint8_t channel)
{
    if (channel >= _num_channels)
        return DroneShowEventResult_Failure;

    AP::relay()->on(_relay_indices[channel]);
    return DroneShowEventResult_Success;
}

DroneShowEventResult DroneShowPyroDevice_Relay::off_impl(uint8_t channel)
{
    if (channel >= _num_channels)
        return DroneShowEventResult_Failure;

    AP::relay()->off(_relay_indices[channel]);
    return DroneShowEventResult_Success;
}
