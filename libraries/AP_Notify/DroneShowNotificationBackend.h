/*
  Drone show module notification management driver
*/
#pragma once

#include "NotifyDevice.h"

/**
 * Class that allows the drone show module to get notifications about events in
 * the notification subsystem.
 */
class DroneShowNotificationBackend : public NotifyDevice
{
public:
    /// Constructor
    DroneShowNotificationBackend();

    /* Do not allow copies */
    DroneShowNotificationBackend(const DroneShowNotificationBackend &other) = delete;
    DroneShowNotificationBackend &operator=(const DroneShowNotificationBackend&) = delete;

    // get singleton instance
    static DroneShowNotificationBackend *get_singleton(void) {
        return _singleton;
    }
    
    bool init(void) override;
    void update(void) override;

    /* Clears the events that the notification backend has collected */
    void clear_events();

    struct events_type {
        uint8_t initiated_compass_cal  : 1;    // 1 when user input to begin compass cal was accepted
        uint8_t compass_cal_saved      : 1;    // 1 when compass calibration was just saved
        uint8_t compass_cal_failed     : 1;    // 1 when compass calibration has just failed
        uint8_t compass_cal_canceled   : 1;    // 1 when compass calibration was just canceled
    };

    /* Stores a copy of the events that we have observed from the notification
     * framework until we clear them explicitly. This is a subset of
     * notify_events_type from AP_Notify.h */
    static struct events_type events;

private:

    static DroneShowNotificationBackend *_singleton;

};
