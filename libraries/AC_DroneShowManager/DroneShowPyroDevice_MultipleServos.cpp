#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowPyroDevice_MultipleServos.h"

extern const AP_HAL::HAL& hal;

bool DroneShowPyroDevice_MultipleServos::init_impl()
{
    uint32_t chans = _servo_channel_mask;
    uint8_t servo_channel;

    _servo_channels.clear();

    while (chans) {
        servo_channel = __builtin_ffs(chans);
        chans &= chans - 1;

        if (servo_channel--) {
            hal.rcout->enable_ch(servo_channel);
            set_duty_cycle_percentage(servo_channel, 0);
            _servo_channels.push_back(servo_channel);
        }
    }

    return true;
}

void DroneShowPyroDevice_MultipleServos::deinit_impl()
{
    for (uint8_t servo_channel : _servo_channels) {
        set_duty_cycle_percentage(servo_channel, 0);
    }

    _servo_channels.clear();
}

uint8_t DroneShowPyroDevice_MultipleServos::num_channels() const
{
    return _servo_channels.size();
}

DroneShowEventResult DroneShowPyroDevice_MultipleServos::fire_impl(uint8_t channel)
{
    // Turn on the ignition by setting the duty cycle to 100%
    return set_duty_cycle_percentage(_servo_channels[channel], 100)
        ? DroneShowEventResult_Success
        : DroneShowEventResult_Failure;
}

DroneShowEventResult DroneShowPyroDevice_MultipleServos::off_impl(uint8_t channel)
{
    // Turn off the ignition by setting the duty cycle to 0%
    return set_duty_cycle_percentage(_servo_channels[channel], 0)
        ? DroneShowEventResult_Success
        : DroneShowEventResult_Failure;
}

bool DroneShowPyroDevice_MultipleServos::set_duty_cycle_percentage(uint8_t servo_channel, uint8_t pct) const
{
    const SRV_Channel* srv_chan = SRV_Channels::srv_channel(servo_channel);
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
    const uint16_t freq = hal.rcout->get_freq(servo_channel);

    if (freq_motor != freq) {
        /*
         * keep at same frequency as the first RCOutput channel, some RCOutput
         * drivers can not operate in different frequency between channels
         */
        uint32_t mask = (1 << servo_channel);
        hal.rcout->set_freq(mask, freq_motor);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Servo %d to %d%% (%d)", (servo_channel + 1), pct, duty_cycle);
    SRV_Channels::set_output_pwm_chan(servo_channel, duty_cycle);

    return true;
}
