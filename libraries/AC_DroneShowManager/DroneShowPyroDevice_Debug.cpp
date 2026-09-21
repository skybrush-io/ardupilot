#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include "DroneShowPyroDevice_Debug.h"

uint8_t DroneShowPyroDevice_Debug::num_channels() const
{
    // Arbitrary number of channels, only relevant for pyro tests
    return 4;
}

DroneShowEventResult DroneShowPyroDevice_Debug::fire_impl(uint8_t channel)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Firing channel %d", channel);
    return DroneShowEventResult_Success;
}

DroneShowEventResult DroneShowPyroDevice_Debug::off_impl(uint8_t channel)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Turning off channel %d", channel);
    return DroneShowEventResult_Success;
}
