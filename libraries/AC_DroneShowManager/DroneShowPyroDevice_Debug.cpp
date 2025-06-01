#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include "DroneShowPyroDevice_Debug.h"

DroneShowEventResult DroneShowPyroDevice_Debug::fire(uint8_t channel)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Firing channel %d", channel);
    return DroneShowEventResult_Success;
}

DroneShowEventResult DroneShowPyroDevice_Debug::off(uint8_t channel)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Turning off channel %d", channel);
    return DroneShowEventResult_Success;
}
