/*
  Drone show module notification management driver
*/

#include "DroneShowNotificationBackend.h"

#include <AP_HAL/AP_HAL.h>

#include "AP_Notify.h"

DroneShowNotificationBackend *DroneShowNotificationBackend::_singleton;
struct DroneShowNotificationBackend::events_type DroneShowNotificationBackend::events;

// Default constructor
DroneShowNotificationBackend::DroneShowNotificationBackend()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("DroneShowNotificationBackend must be singleton");
    }
    _singleton = this;
}

bool DroneShowNotificationBackend::init()
{
    clear_events();
    return true;
}

void DroneShowNotificationBackend::update()
{
    DroneShowNotificationBackend::events.initiated_compass_cal |= AP_Notify::events.initiated_compass_cal;
    DroneShowNotificationBackend::events.compass_cal_saved |= AP_Notify::events.compass_cal_saved;
    DroneShowNotificationBackend::events.compass_cal_failed |= AP_Notify::events.compass_cal_failed;
    DroneShowNotificationBackend::events.compass_cal_canceled |= AP_Notify::events.compass_cal_canceled;
}

void DroneShowNotificationBackend::clear_events()
{
    memset(&DroneShowNotificationBackend::events, 0, sizeof(DroneShowNotificationBackend::events));
}
