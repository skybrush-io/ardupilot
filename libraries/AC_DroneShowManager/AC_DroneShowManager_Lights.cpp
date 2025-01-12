#include "AC_DroneShowManager.h"

#include <AP_GPS/AP_GPS.h>
#include <AP_Notify/AP_Notify.h>
#include <GCS_MAVLink/GCS.h>

#include <skybrush/skybrush.h>

#include "DroneShowLEDFactory.h"

// Group mask indicating all groups
#define ALL_GROUPS 0

// Undefine some macros from RGBLed.h that are in comflict with the code below
#undef BLACK
#undef RED
#undef GREEN
#undef BLUE
#undef YELLOW
#undef WHITE

namespace Colors {
    static const sb_rgb_color_t BLACK = { 0, 0, 0 };
    static const sb_rgb_color_t RED = { 255, 0, 0 };
    static const sb_rgb_color_t YELLOW = { 255, 255, 0 };
    static const sb_rgb_color_t GREEN = { 0, 255, 0 };
    static const sb_rgb_color_t GREEN_DIM = { 0, 128, 0 };
    static const sb_rgb_color_t ORANGE = { 255, 128, 0 };
    static const sb_rgb_color_t WHITE_DIM = { 128, 128, 128 };
    static const sb_rgb_color_t LIGHT_BLUE = { 0, 128, 255 };
    static const sb_rgb_color_t MAGENTA = { 255, 0, 255 };
    static const sb_rgb_color_t WHITE = { 255, 255, 255 };
};

static float get_modulation_factor_for_light_effect(
    uint32_t timestamp, LightEffectType effect, uint16_t period_msec, uint16_t phase_msec
);

void AC_DroneShowManager::get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color)
{
    *color = sb_light_player_get_color_at(_light_player, time < 0 || time > 86400000 ? 0 : time * 1000);
}

uint32_t AC_DroneShowManager::_get_gps_synced_timestamp_in_millis_for_lights() const
{
    // No need to worry about loss of GPS fix; AP::gps().time_epoch_usec() is
    // smart enough to extrapolate from the timestamp of the latest fix.
    //
    // Also no need to worry about overflow; AP::gps().time_epoch_usec() / 1000
    // is too large for an uint32_t but it doesn't matter as we will truncate
    // the high bits.
    if (_is_gps_time_ok()) {
        return AP::gps().time_epoch_usec() / 1000;
    } else {
        return AP_HAL::millis();
    }
}

void AC_DroneShowManager::_flash_leds_after_failure()
{
    _flash_leds_with_color(255, 0, 0, /* count = */ 3, LightEffectPriority_Internal);
}

void AC_DroneShowManager::_flash_leds_after_success()
{
    _flash_leds_with_color(0, 255, 0, /* count = */ 3, LightEffectPriority_Internal);
}

void AC_DroneShowManager::_flash_leds_to_attract_attention(LightEffectPriority priority)
{
    _flash_leds_with_color(
        255, 255, 255, /* count = */ 5, priority,
        /* enhance_brightness = */ true
    );
}

void AC_DroneShowManager::_flash_leds_with_color(
    uint8_t red, uint8_t green, uint8_t blue, uint8_t count,
    LightEffectPriority priority, bool enhance_brightness
) {
    _light_signal.started_at_msec = AP_HAL::millis();
    _light_signal.priority = priority;
    _light_signal.enhance_brightness = enhance_brightness;

    _light_signal.duration_msec = count * 300 - 200;  /* 100ms on, 200ms off */
    _light_signal.color[0] = red;
    _light_signal.color[1] = green;
    _light_signal.color[2] = blue;
    _light_signal.effect = LightEffect_Blinking;
    _light_signal.period_msec = 300;  /* 100ms on, 200ms off */
    _light_signal.phase_msec = 0;
    _light_signal.sync_to_gps = true;
}

bool AC_DroneShowManager::_handle_led_control_message(const mavlink_message_t& msg)
{
    mavlink_led_control_t packet;
    LightEffectPriority priority;
    uint8_t mask = ALL_GROUPS;

    mavlink_msg_led_control_decode(&msg, &packet);

    if (packet.instance != 42 || packet.pattern != 42) {
        // Not handled by us
        return false;
    }

    // LED control packets exist in five variants:
    //
    // custom_len == 0 --> simple blink request
    // custom_len == 1 --> simple blink request, but only if the drone matches
    //                     the group mask given in the last byte
    // custom_len == 3 --> set the LED to a specific color for five seconds
    // custom_len == 5 --> set the LED to a specific color for a given duration (msec)
    // custom_len == 6 --> set the LED to a specific color for a given duration (msec),
    //                     modulated by a given effect
    // custom_len == 7 --> set the LED to a specific color for a given duration (msec),
    //                     modulated by a given effect, but only if the drone matches
    //                     the group mask given in the last byte

    if (packet.custom_len >= 8 || packet.custom_len == 2 || packet.custom_len == 4) {
        // Not handled by us
        return false;
    }
    
    // Individual messages take precedence over broadcast messages so we need to
    // know whether this message is broadcast
    priority = (packet.target_system == 0)
        ? LightEffectPriority_Broadcast
        : LightEffectPriority_Individual;
    if (priority < _light_signal.priority) {
        // Previous light signal has a higher priority, but maybe it ended already?
        if (_light_signal.started_at_msec + _light_signal.duration_msec < AP_HAL::millis()) {
            _light_signal.priority = LightEffectPriority_None;
        } else {
            // Handled but ignored by us because a higher priority effect is still
            // playing.
            return true;
        }
    }

    _light_signal.started_at_msec = AP_HAL::millis();
    _light_signal.priority = priority;
    _light_signal.enhance_brightness = false;

    // Sync the light signal to GPS time when it is broadcast so all drones would
    // flash at the same time. However, when this is an individual command,
    // start the flash as soon as possible for responsiveness.
    _light_signal.sync_to_gps = (packet.target_system == 0);

    if (packet.custom_len < 2) {
        // Start blinking the drone show LED
        if (packet.custom_len == 1) {
            mask = packet.custom_bytes[0];
        }
        if (matches_group_mask(mask)) {
            _flash_leds_to_attract_attention(priority);
        }
    } else if (packet.custom_len == 3) {
        // Set the drone show LED to a specific color for five seconds
        _light_signal.duration_msec = 5000;
        _light_signal.color[0] = packet.custom_bytes[0];
        _light_signal.color[1] = packet.custom_bytes[1];
        _light_signal.color[2] = packet.custom_bytes[2];
        _light_signal.effect = LightEffect_Solid;
        _light_signal.period_msec = 0;    /* doesn't matter */
        _light_signal.phase_msec = 0;     /* doesn't matter */
    } else if (packet.custom_len == 5) {
        // Set the drone show LED to a specific color for a given number of
        // milliseconds
        _light_signal.duration_msec = packet.custom_bytes[3] + (packet.custom_bytes[4] << 8);
        _light_signal.color[0] = packet.custom_bytes[0];
        _light_signal.color[1] = packet.custom_bytes[1];
        _light_signal.color[2] = packet.custom_bytes[2];
        _light_signal.effect = LightEffect_Solid;
        _light_signal.period_msec = 0;    /* doesn't matter */
        _light_signal.phase_msec = 0;     /* doesn't matter */
    } else if (packet.custom_len == 6 || packet.custom_len == 7) {
        if (packet.custom_len == 7) {
            mask = packet.custom_bytes[6];
        }
        if (matches_group_mask(mask)) {
            // Set the drone show LED to a specific color for a given number of
            // milliseconds, modulated by a given effect
            _light_signal.duration_msec = packet.custom_bytes[3] + (packet.custom_bytes[4] << 8);
            _light_signal.period_msec = 5000;
            _light_signal.color[0] = packet.custom_bytes[0];
            _light_signal.color[1] = packet.custom_bytes[1];
            _light_signal.color[2] = packet.custom_bytes[2];

            // Reset the phase of the effect if the previous effect was of a
            // different type, but keep the phase counter at its current value
            if (_light_signal.effect != packet.custom_bytes[5]) {
                bool is_effect_synced_to_gps;

                if (packet.custom_bytes[5] > LightEffect_Last) {
                    _light_signal.effect = LightEffect_Last;
                } else {
                    _light_signal.effect = static_cast<LightEffectType>(packet.custom_bytes[5]);
                }

                is_effect_synced_to_gps = (
                    _light_signal.effect == LightEffect_Blinking ||
                    _light_signal.effect == LightEffect_Off ||
                    _light_signal.effect == LightEffect_Solid
                );

                _light_signal.phase_msec = is_effect_synced_to_gps ? 0 : get_random16() % _light_signal.period_msec;
            }
        }
    }

    // Handle zero duration; it means that we need to turn off whatever
    // effect we have now.
    if (matches_group_mask(mask) && _light_signal.duration_msec == 0) {
        _light_signal.started_at_msec = 0;
        _light_signal.effect = LightEffect_Off;
    }

    return true;
}

void AC_DroneShowManager::_update_lights()
{
    // TODO(ntamas): mode numbers are hardcoded here; we cannot import them
    // from ../ArduCopter/mode.h due to circular imports. We should inject them
    // from Copter.cpp after the construction of AC_DroneShowManager.cpp instead.
    const uint32_t MODE_RTL = 6, MODE_SMART_RTL = 21, MODE_LAND = 9, MODE_DRONE_SHOW = 127;
    sb_rgb_color_t color = Colors::BLACK;
    bool light_signal_affected_by_brightness_setting = true;
    bool enhance_brightness = false;
    int brightness = _params.preflight_light_signal_brightness;
    uint8_t pattern = 0b11111111;
    const uint8_t BLINK = 0b11110000;
    const uint8_t BLINK_TWICE_PER_SECOND = 0b11001100;
    const uint8_t FLASH_ONCE_PER_SECOND = 0b10000000;
    const uint8_t FLASH_TWICE_PER_SECOND = 0b10100000;
    const uint8_t FLASH_FOUR_TIMES_PER_SECOND = 0b10101010;
    float elapsed_time;
    float pulse = 0.0f;
    float factor;

#define IS_LANDING(mode) (  \
    mode == MODE_LAND ||    \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_Landing) \
)

#define IS_LANDED(mode) (  \
    mode == MODE_LAND ||    \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_Landed) \
)

#define IS_RTL(mode) (        \
    mode == MODE_RTL ||       \
    mode == MODE_SMART_RTL || \
    (mode == MODE_DRONE_SHOW && _stage_in_drone_show_mode == DroneShow_RTL) \
)

    // During compass calibration, the light should be purple no matter what.
    // Compass calibration is always requested by the user so he can rightly
    // expect any light signal that was previously set up from the GCS to be
    // overridden.
    if (AP_Notify::flags.compass_cal_running) {
        color = Colors::MAGENTA;
        pulse = 0.5;

        // Make sure that this light signal is visible with a minimum intensity
        // if the user otherwise turned off the light signals
        if (brightness < 1) {
            brightness = 1;
        }
    } else if (_light_signal.started_at_msec) {
        // If the user requested a light signal, it trumps everything except
        // the compass calibration.
        uint32_t now = AP_HAL::millis();
        if (now < _light_signal.started_at_msec) {
            // Something is wrong, let's just clear the light signal
            _light_signal.started_at_msec = 0;
            _light_signal.priority = LightEffectPriority_None;
        } else {
            // Get duration since the start of the light signal and calculate
            // the color based on the time elapsed
            uint32_t diff = (now - _light_signal.started_at_msec);
            if (diff > _light_signal.duration_msec) {
                // Light signal ended
                _light_signal.started_at_msec = 0;
                _light_signal.priority = LightEffectPriority_None;
            } else {
                // Light signal is in progress
                factor = get_modulation_factor_for_light_effect(
                    _light_signal.sync_to_gps
                        ? _get_gps_synced_timestamp_in_millis_for_lights()
                        : diff,
                    _light_signal.effect, _light_signal.period_msec,
                    _light_signal.phase_msec
                );
                color.red = _light_signal.color[0] * factor;
                color.green = _light_signal.color[1] * factor;
                color.blue = _light_signal.color[2] * factor;

                // If the user requested enhanced brightness, and the color
                // is a shade of gray, set the white channel to the same value
                // as the R, G and B channels. This is used to enhance the
                // brightness of the "where are you" signal sent from the GCS.
                enhance_brightness = _light_signal.enhance_brightness;
            }
        }

        // If the user explicitly requested a light signal, do not dim it
        light_signal_affected_by_brightness_setting = false;
    } else if (
        AP_Notify::flags.ekf_bad || AP_Notify::flags.failsafe_battery ||
        AP_Notify::flags.failsafe_gcs || AP_Notify::flags.failsafe_radio
    ) {
        // Ideally, the condition above should be the same as the one that triggers
        // MAV_STATE_CRITICAL in GCS_Mavlink.cpp. The conditions for this flag
        // are encoded in Copter::any_failsafe_triggered(). However, we need
        // some tweaks so we list the conditions above explicitly, with the
        // following considerations:
        // 
        // * In case of EKF failure or battery failsafe conditions, the light
        //   should be red.
        //
        // * We do not trigger the red light for radio or GCS failsafes because
        //   both are quite common during a show when the drone is far from the
        //   GCS and/or the pilot, but these usually do not represent a problem.
        //
        // * We do not trigger the red light for ADSB or terrain failsafes
        //   either; we do not use terrain following during a show and we do
        //   not use ADSB either at the moment.
        //
        // The conditions above and Copter::any_failsafe_triggered() should be
        // reviewed regularly to see if these are still applicable.
        color = Colors::RED;
        pattern = BLINK;
    } else if (AP_Notify::flags.flying) {
        uint32_t mode = gcs().custom_mode();

        // If we are flying, we don't want to dim the LED light
        light_signal_affected_by_brightness_setting = false;

        if (IS_RTL(mode)) {
            // If we are flying and we are in RTL or smart RTL mode, blink with orange color
            color = Colors::ORANGE;
            pattern = BLINK;
        } else if (IS_LANDING(mode)) {
            // If we are flying and we are in landing mode, show a solid orange color
            color = Colors::ORANGE;
        } else if (mode == MODE_DRONE_SHOW) {
            // If we are flying in drone show mode, show the color that we are
            // supposed to show during the drone show if the show has started.
            // If the show has not started, show white, which is useful if the
            // user took off manually from the GCS just to test the takeoff.
            // Also show white if we are loitering, which happens in certain
            // conditions (e.g., after a takeoff test).
            if (_stage_in_drone_show_mode == DroneShow_Error) {
                color = Colors::RED;
                pattern = BLINK;
            } else if (_stage_in_drone_show_mode == DroneShow_Loiter) {
                color = Colors::WHITE;
            } else {
                elapsed_time = get_elapsed_time_since_start_sec();
                if (elapsed_time >= 0) {
                    get_color_of_rgb_light_at_seconds(elapsed_time, &color);
                } else {
                    color = Colors::WHITE_DIM;
                }
            }
        } else {
            // Otherwise, show a bright white color so we can see the drone from the ground
            color = Colors::WHITE;
        }
    } else if (AP::motors()->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN) {
        uint32_t mode = gcs().custom_mode();

        if (IS_LANDING(mode)) {
            // If the landing algorithm is running but we are not flying, _but_
            // the motors have not shut down yet, flash four times per second
            // as an alarm signal so people know not to approach the drone yet
            color = Colors::ORANGE;
            pattern = FLASH_FOUR_TIMES_PER_SECOND;
        } else if (mode == MODE_DRONE_SHOW) {
            // If we are not flying in drone show mode but the motors are
            // running, show the color that we are supposed to show if the
            // show has started already; otherwise blink green twice per second.
            elapsed_time = get_elapsed_time_since_start_sec();
            if (elapsed_time >= 0) {
                get_color_of_rgb_light_at_seconds(elapsed_time, &color);
                light_signal_affected_by_brightness_setting = false;
            } else {
                color = Colors::GREEN;
                pattern = BLINK_TWICE_PER_SECOND;
            }
        } else {
            // In all other cases, blink green twice per second.
            color = Colors::GREEN;
            pattern = BLINK_TWICE_PER_SECOND;
        }
    } else if (
        AP_Notify::flags.initialising || !AP_Notify::flags.pre_arm_check ||
        !AP_Notify::flags.pre_arm_gps_check
    ) {
        // If the prearm checks are running, flash yellow once per second
        color = Colors::YELLOW;
        pattern = FLASH_ONCE_PER_SECOND;
    } else {
        // We are on the ground, motors are not running and the prearm checks
        // have passed. We are essentially in standby.
        uint32_t mode = gcs().custom_mode();

        if (IS_LANDED(mode)) {
            // Landing-related mode, show a green dim light to indicate success.
            color = Colors::GREEN_DIM;
        } else if (mode == MODE_DRONE_SHOW) {
            // Drone show mode is distinguished from the rest by a pulsating
            // light ("breathing" pattern).

            // Preflight check failures --> slow pulsating yellow light
            // No authorization yet --> slow pulsating blue light; dark if no
            // start time was set, not-so-dark if some start time was set
            // Authorized, far from start --> slow pulsating green light
            // Authorized, about to start --> green flashes, twice per second,
            // synced to GPS

            if (_stage_in_drone_show_mode == DroneShow_Error) {
                color = Colors::RED;
                pattern = BLINK;
            } else if (_preflight_check_failures) {
                color = Colors::YELLOW;
                pulse = 0.5;
            } else if (has_authorization()) {
                if (get_time_until_landing_sec() < 0) {
                    // if we have already landed but show mode is reset from
                    // another mode, we just keep calm with solid green
                    color = Colors::GREEN_DIM;
                } else if (get_time_until_takeoff_sec() > 10) {
                    // if there is plenty of time until takeoff, we pulse slowly
                    color = Colors::GREEN_DIM;
                    pulse = 0.5;
                } else {
                    // if we are about to take off soon, flash quickly
                    color = Colors::GREEN;
                    pattern = FLASH_TWICE_PER_SECOND;
                }
            } else {
                color = Colors::LIGHT_BLUE;
                pulse = has_scheduled_start_time() && get_time_until_takeoff_sec() >= 0 ? 0.5 : 0.3;
            }
        } else {
            // Show a green dim light to indicate that we are ready.
            color = Colors::GREEN_DIM;
        }
    }

    if (pulse > 0) {
        // "Pulsating", "breathing" light pattern. Modulate intensity with a
        // sine wave.
        factor = pulse * get_modulation_factor_for_light_effect(
            _get_gps_synced_timestamp_in_millis_for_lights(),
            LightEffect_Breathing, /* period_msec = */ 5000, /* phase_msec = */ 0
        );
        color.red *= factor;
        color.green *= factor;
        color.blue *= factor;
    } else if (pattern < 255) {
        // check the time and set the color to black if needed - this creates a
        // blinking pattern when needed
        uint32_t timestamp = _get_gps_synced_timestamp_in_millis_for_lights() % 1000;
        if (!(pattern & (0x80 >> (timestamp / 125))))
        {
            color = Colors::BLACK;
        }
    }

    // Dim the lights if we are on the ground before the flight
    if (light_signal_affected_by_brightness_setting) {
        uint8_t shift = 0;

        if (brightness <= 0) {
            // <= 0 = completely off, shift by 8 bits
            shift = 8;
        } else if (brightness == 1) {
            // 1 = low brightness, keep the 6 MSB so the maximum is 64
            shift = 2;
        } else if (brightness == 2) {
            // 2 = medium brightness, keep the 7 MSB so the maximum is 128
            shift = 1;
        } else {
            // >= 2 = full brightness
            shift = 0;
        }

        color.red >>= shift;
        color.green >>= shift;
        color.blue >>= shift;
    }

    _last_rgb_led_color = color;

    if (_rgb_led) {
        // No need to test whether the RGB values or the gamma correction
        // changed because the LED classes do this on their own
        _rgb_led->set_gamma(_params.led_specs[0].gamma);

        if (_rgb_led->supports_white_channel()) {
            // Code path for LEDs that support a white channel
            sb_rgbw_conversion_t conv;
            sb_rgbw_color_t rgbw_color;

            if (enhance_brightness && color.red == color.green && color.green == color.blue) {
                sb_rgbw_conversion_use_fixed_value(&conv, color.red);
            } else if (_params.led_specs[0].white_temperature > 0) {
                sb_rgbw_conversion_use_color_temperature(&conv, _params.led_specs[0].white_temperature);
            } else {
                sb_rgbw_conversion_use_min_subtraction(&conv);
            }

            rgbw_color = sb_rgb_color_to_rgbw(color, conv);
            _rgb_led->set_rgbw(rgbw_color.red, rgbw_color.green, rgbw_color.blue, rgbw_color.white);
        } else {
            // Code path for standard RGB LEDs
            _rgb_led->set_rgb(color.red, color.green, color.blue);
        }
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_sock_rgb_open) {
        uint8_t data[4];

        data[0] = mavlink_system.sysid;
        data[1] = color.red;
        data[2] = color.green;
        data[3] = color.blue;

        _sock_rgb.send(data, sizeof(data));
    }
#endif

#undef IS_LANDING
#undef IS_RTL

}

void AC_DroneShowManager::_update_rgb_led_instance()
{
    static int previous_led_type = -1;
    static uint8_t previous_channel = 255;
    static uint8_t previous_num_leds = 0;

    // We need to avoid the re-creation of _rgb_led if the type of the LED did
    // not change because it causes problems with I2C LEDs on a MatekH743 Slim,
    // leading to watchdog timeouts

    if (_rgb_led_factory) {
        int led_type = _params.led_specs[0].type;
        uint8_t channel = _params.led_specs[0].channel;
        uint8_t num_leds = _params.led_specs[0].count;

        if (
            led_type != previous_led_type ||
            channel != previous_channel ||
            num_leds != previous_num_leds
        ) {
            // Turn off the old RGB LED
            if (_rgb_led)
            {
                _rgb_led->set_rgb(0, 0, 0);

                delete _rgb_led;
                _rgb_led = NULL;
            }

            // Construct the new LED
            _rgb_led = _rgb_led_factory->new_rgb_led_by_type(
                static_cast<DroneShowLEDType>(led_type), channel, num_leds
            );

            // Store the settings
            previous_led_type = led_type;
            previous_channel = channel;
            previous_num_leds = num_leds;
        }
    }

    if (_rgb_led) {
        // Update gamma correction parameter of LED
        float gamma = _params.led_specs[0].gamma;
        _rgb_led->set_gamma(gamma);
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

bool AC_DroneShowManager::_open_rgb_led_socket()
{
#if defined(RGB_SOCKET_PORT)
    if (_sock_rgb_open) {
        return true;
    }

    if (!_sock_rgb.connect("127.0.0.1", RGB_SOCKET_PORT)) {
        return false;
    }

    _sock_rgb.set_blocking(false);
    _sock_rgb_open = true;
#endif

    return true;
}

#endif

void AC_DroneShowManager::_repeat_last_rgb_led_command()
{
    if (_rgb_led) {
        _rgb_led->repeat_last_command_if_needed();
    }
}

static float get_modulation_factor_for_light_effect(
    uint32_t timestamp, LightEffectType effect, uint16_t period_msec, uint16_t phase_msec
) {
    if (effect == LightEffect_Off) {
        return 0.0;
    } else if (effect == LightEffect_Solid || period_msec == 0) {
        return 1.0;
    }

    timestamp = (timestamp + phase_msec) % period_msec;

    switch (effect) {
        case LightEffect_Off:
            return 0.0;

        case LightEffect_Solid:
            return 1.0;

        case LightEffect_Blinking:
            return timestamp < period_msec / 3.0f ? 1.0 : 0.0;

        case LightEffect_Breathing:
            return 0.5f * (1 + sinf(timestamp / 5000.0f * 2.0f * M_PI));

        default:
            return 0.0;
    }
}

