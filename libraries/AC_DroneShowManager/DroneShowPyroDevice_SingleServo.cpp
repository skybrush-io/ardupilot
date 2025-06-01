#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowPyroDevice_SingleServo.h"

extern const AP_HAL::HAL& hal;

DroneShowPyroDevice_SingleServo::DroneShowPyroDevice_SingleServo(uint8_t servo_channel)
    : _servo_channel(servo_channel)
{
}

bool DroneShowPyroDevice_SingleServo::init()
{
    hal.rcout->enable_ch(_servo_channel);
    set_duty_cycle_percentage(0);
    return true;
}

void DroneShowPyroDevice_SingleServo::deinit()
{
    set_duty_cycle_percentage(0);
}

DroneShowEventResult DroneShowPyroDevice_SingleServo::fire(uint8_t channel)
{
    // 85% triggers channel 0, 100% triggers channel 1. We support two channels
    // at most.
    uint8_t pct = channel == 0 ? 85 : 100;
    return set_duty_cycle_percentage(pct)
        ? DroneShowEventResult_Success
        : DroneShowEventResult_Failure;
}

DroneShowEventResult DroneShowPyroDevice_SingleServo::off(uint8_t channel)
{
    // Turn off the ignition by setting the duty cycle to 0%
    return set_duty_cycle_percentage(0)
        ? DroneShowEventResult_Success
        : DroneShowEventResult_Failure;
}

bool DroneShowPyroDevice_SingleServo::set_duty_cycle_percentage(uint8_t pct) const
{
    const SRV_Channel* srv_chan = SRV_Channels::srv_channel(_servo_channel);
    if (srv_chan == nullptr) {
        return false;
    }

    if (pct > 100) {
        pct = 100;
    }

    const uint16_t min_pwm = srv_chan->get_output_min();
    const uint16_t max_pwm = srv_chan->get_output_max();
    const uint16_t duty_cycle = min_pwm + ((max_pwm - min_pwm) * pct) / 100;

    /* This section of the code was taken from RCOutputRGBLed.cpp */
    const uint16_t freq_motor = hal.rcout->get_freq(0);
    const uint16_t freq = hal.rcout->get_freq(_servo_channel);

    if (freq_motor != freq) {
        /*
         * keep at same frequency as the first RCOutput channel, some RCOutput
         * drivers can not operate in different frequency between channels
         */
        uint32_t mask = (1 << _servo_channel);
        hal.rcout->set_freq(mask, freq_motor);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Servo %d to %d%% (%d)", _servo_channel, pct, duty_cycle);
    SRV_Channels::set_output_pwm_chan(_servo_channel, duty_cycle);

    return true;
}
