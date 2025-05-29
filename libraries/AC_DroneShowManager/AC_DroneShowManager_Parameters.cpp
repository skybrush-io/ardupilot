#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);

const AP_Param::GroupInfo AC_DroneShowManager::var_info[] = {
    // @Param: START_TIME
    // @DisplayName: Start time
    // @Description: Start time of drone show as a GPS time of week timestamp (sec), negative if unset
    // @Range: -1 604799
    // @Increment: 1
    // @Units: sec
    // @Volatile: True
    // @User: Standard
    //
    // Note that we cannot use UNIX timestamps here because ArduPilot stores
    // all parameters as floats, and floats can represent integers accurately
    // only up to 2^23 - 1
    AP_GROUPINFO("START_TIME", 1, AC_DroneShowManager, _params.start_time_gps_sec, -1),

    // @Param: ORIGIN_LAT
    // @DisplayName: Show origin (latitude)
    // @Description: Latitude of the origin of the drone show coordinate system, zero if unset
    // @Range: -900000000 900000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LAT", 2, AC_DroneShowManager, _params.origin_lat, 0),

    // @Param: ORIGIN_LNG
    // @DisplayName: Show origin (longitude)
    // @Description: Longitude of the origin of the drone show coordinate system, zero if unset
    // @Range: -1800000000 1800000000
    // @Increment: 1
    // @Units: 1e-7 degrees
    // @User: Standard
    AP_GROUPINFO("ORIGIN_LNG", 3, AC_DroneShowManager, _params.origin_lng, 0),

    // @Param: ORIGIN_AMSL
    // @DisplayName: Show origin (altitude)
    // @Description: AMSL altitude of the origin of the drone show coordinate system, -10000000 or smaller if unset
    // @Range: -10000000 10000000
    // @Increment: 1
    // @Units: mm
    // @User: Standard
    AP_GROUPINFO("ORIGIN_AMSL", 12, AC_DroneShowManager, _params.origin_amsl_mm, SMALLEST_VALID_AMSL - 1),

    // @Param: ORIENTATION
    // @DisplayName: Show orientation
    // @Description: Orientation of the X axis of the show coordinate system in CW direction relative to North, -1 if unset
    // @Range: -1 360
    // @Increment: 1
    // @Units: degrees
    // @User: Standard
    AP_GROUPINFO("ORIENTATION", 4, AC_DroneShowManager, _params.orientation_deg, -1),

    // @Param: START_AUTH
    // @DisplayName: Authorization to start
    // @Description: Whether the drone is authorized to start the show
    // @Values: 0:Revoked, 1:Granted, 2:Granted in rehearsal mode, 3:Granted with lights only
    // @Volatile: True
    // @User: Standard
    AP_GROUPINFO("START_AUTH", 5, AC_DroneShowManager, _params.authorization, DroneShowAuthorization_Revoked),

    // @Param: LED0_TYPE
    // @DisplayName: Assignment of LED channel 0 to a LED output type
    // @Description: Specifies where the output of the main LED light track of the show should be sent
    // @Values: 0:Off, 1:MAVLink, 2:NeoPixel, 3:ProfiLED, 4:Debug, 5:SITL, 6:Servo, 7:I2C RGB, 8:Inverted servo, 9:UART (WGDrones), 10:NeoPixel RGBW, 11:I2C RGBW, 12:Notification LED, 13:Servo with limits (off=0), 14:Servo with limits
    // @User: Advanced
    AP_GROUPINFO("LED0_TYPE", 6, AC_DroneShowManager, _params.led_specs[0].type, 0),

    // @Param: LED0_CHAN
    // @DisplayName: PWM, MAVLink or UART channel to use for the LED output
    // @Description: PWM channel to use for the LED output (1-based) if the LED type is "NeoPixel", "ProfiLED" or "NeoPixel RGBW"; the MAVLink channel to use if the LED type is "MAVLink"; the I2C address of the LED if the LED type is "I2C"; the UART index if the LED type is "WGDrones". For UART-driven LEDs, you also need to set the baud rate in SERIALx_BAUD and set SERIALx_PROTOCOL to "Scripting" to ensure that the UART is initialized.
    // @User: Advanced
    AP_GROUPINFO("LED0_CHAN", 8, AC_DroneShowManager, _params.led_specs[0].channel, 0),

    // @Param: LED0_COUNT
    // @DisplayName: Number of individual LEDs on a LED channel
    // @Description: For NeoPixel or ProfiLED LED strips: specifies how many LEDs there are on the strip. For I2C LEDs: specifies the index of the bus that the LED is attached to.
    // @User: Advanced
    AP_GROUPINFO("LED0_COUNT", 7, AC_DroneShowManager, _params.led_specs[0].count, 16),

    // @Param: LED0_GAMMA
    // @DisplayName: Gamma correction factor for the LED channel
    // @Description: Specifies the exponent of the gamma correction to apply on the RGB values of this channel. Set this to 1 if you do not want to use gamma correction or if the LEDs perform gamma correction on their own; otherwise typical values are in the range 2.2 to 2.8 for LEDs. Set a value that provides an approximately linear perceived brightness response when the LEDs are faded from full black to full white.
    // @Range: 1 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LED0_GAMMA", 19, AC_DroneShowManager, _params.led_specs[0].gamma, 1.0f),

    // @Param: LED0_WTEMP
    // @DisplayName: Color temperature of the white LED of the channel
    // @Description: Specifies the color temperature of the white LED of the channel if the channel makes use of an additional white LED. Set to zero if you don't know the color temperature of the white LED or if there is no white LED.
    // @Range: 0 15000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("LED0_WTEMP", 23, AC_DroneShowManager, _params.led_specs[0].white_temperature, 0.0f),

    // @Param: LED0_MINBRI
    // @DisplayName: Minimum LED brightness threshold
    // @Description: Minimum brightness threshold (as a ratio 0.0-1.0) below which LED is turned off completely
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("LED0_MINBRI", 32, AC_DroneShowManager, _params.led_specs[0].min_brightness, 0.0f),

    // @Param: MODE_BOOT
    // @DisplayName: Conditions for entering show mode
    // @Description: Bitfield that specifies when the drone should switch to show mode automatically
    // @Values: 3:At boot and when authorized,2:When authorized,1:At boot,0:Never
    // @Bitmask: 0:At boot,1:When authorized
    // @User: Standard
    AP_GROUPINFO("MODE_BOOT", 9, AC_DroneShowManager, _params.show_mode_settings, 2),

    // @Param: PRE_LIGHTS
    // @DisplayName: Brightness of preflight check related lights
    // @Description: Controls the brightness of light signals on the drone that are used to report status information when the drone is on the ground. 0 is off, 1 is low brightness (25%), 2 is medium brightness (50%), 3 is full brightness (100%). Values greater than 3 and less than or equal to 100 are interpreted as percentages. Negative values are treated as zero.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PRE_LIGHTS", 10, AC_DroneShowManager, _params.preflight_light_signal_brightness, 2),

    // @Param: CTRL_MODE
    // @DisplayName: Flags to configure the show position control algorithm
    // @Description: Controls various aspects of the position control algorithm built into the firmware
    // @Values: 3:Position/velocity/acceleration control,1:Position and velocity control,0:Position control only
    // @Bitmask: 0:Velocity control,1:Acceleration control
    // @User: Advanced
    AP_GROUPINFO("CTRL_MODE", 11, AC_DroneShowManager, _params.control_mode_flags, DroneShowControl_VelocityControlEnabled),

    // @Param: GROUP
    // @DisplayName: Show group index
    // @Description: Index of the group that this drone belongs to
    // @Range: 0 7
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GROUP", 13, AC_DroneShowManager, _params.group_index, 0),

    // @Param: CTRL_RATE
    // @DisplayName: Target update rate
    // @Description: Update rate of the target position and velocity during the show
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("CTRL_RATE", 14, AC_DroneShowManager, _params.control_rate_hz, DEFAULT_UPDATE_RATE_HZ),

    // @Param: VEL_FF_GAIN
    // @DisplayName: Velocity feed-forward gain
    // @Description: Multiplier used when mixing the desired velocity of the drone into the velocity target of the position controller. Lower values will result in more relaxed/stable behaviour, at the price of a smoothed trajectory with rounded corners, less accuracy and more lag behind desired position. Higher values will decrease lag, make trajectory following more accurate, sharp and agressive, but might increase overshoot at corners and decrease stability if general attitude control is not tuned well.
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_FF_GAIN", 16, AC_DroneShowManager, _params.velocity_feedforward_gain, 1.0f),

    // @Param: TAKEOFF_ALT
    // @DisplayName: Takeoff altitude
    // @Description: Altitude above current position to take off to when starting the show
    // @Range: 0 5
    // @Increment: 0.1
    // @Units: m
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TAKEOFF_ALT", 17, AC_DroneShowManager, _params.takeoff_altitude_m, DEFAULT_TAKEOFF_ALTITUDE_METERS),

    // @Param: TAKEOFF_ERR
    // @DisplayName: Maximum placement error in XY direction
    // @Description: Maximum placement error that we tolerate before takeoff, in meters. Zero to turn off XY placement accuracy checks.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TAKEOFF_ERR", 15, AC_DroneShowManager, _params.max_xy_placement_error_m, DEFAULT_XY_PLACEMENT_ERROR_METERS),

    // @Param: SYNC_MODE
    // @DisplayName: Time synchronization mode
    // @Description: Time synchronization mode to use when starting the show
    // @Values: 0:Countdown, 1:GPS time
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("SYNC_MODE", 18, AC_DroneShowManager, _params.time_sync_mode, DEFAULT_SYNC_MODE),

    // @Param: HFENCE_EN
    // @DisplayName: Hard fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the hard fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("HFENCE_EN", 20, AC_DroneShowManager, hard_fence._params.enabled, 0),

    // @Param: HFENCE_DIST
    // @DisplayName: Hard fence minimum distance
    // @Description: Minimum distance that the hard fence extends beyond the standard geofence
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("HFENCE_DIST", 21, AC_DroneShowManager, hard_fence._params.distance, 25),

    // @Param: HFENCE_TO
    // @DisplayName: Hard fence timeout
    // @Description: Minimum time that the vehicle needs to spend outside the hard geofence to trigger a motor shutdown
    // @Units: sec
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("HFENCE_TO", 22, AC_DroneShowManager, hard_fence._params.timeout, 5),

    // @Param: MAX_XY_ERR
    // @DisplayName: Maximum allowed drift in XY direction during show
    // @Description: Maximum allowed drift from planned trajectory in XY plane that we tolerate during show, in meters. Zero to turn off XY checks. Drifts exceeding the threshold will trigger a status flag but do not abort the show.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MAX_XY_ERR", 24, AC_DroneShowManager, _params.max_xy_drift_during_show_m, DEFAULT_MAX_XY_DRIFT_METERS),

    // @Param: MAX_Z_ERR
    // @DisplayName: Maximum allowed drift in Z direction during show
    // @Description: Maximum allowed drift from planned trajectory in Z direction that we tolerate during show, in meters. Zero to turn off Z checks. Drifts exceeding the threshold will trigger a status flag but do not abort the show.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MAX_Z_ERR", 25, AC_DroneShowManager, _params.max_z_drift_during_show_m, DEFAULT_MAX_Z_DRIFT_METERS),

    // @Param: POST_ACTION
    // @DisplayName: Action to perform when the show trajectory ends
    // @Description: Specifies what to do at the end of the show trajectory
    // @Values: 3:RTL if above takeoff position and land otherwise,2:RTL unconditionally,1:Land,0:Loiter (position hold)
    // @User: Advanced
    AP_GROUPINFO("POST_ACTION", 26, AC_DroneShowManager, _params.post_action, DEFAULT_POST_ACTION),

    // @Param: BFENCE_EN
    // @DisplayName: Bubble fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the bubble fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("BFENCE_EN", 27, AC_DroneShowManager, bubble_fence._params.enabled, 1),

    // @Param: BFENCE_DXY
    // @DisplayName: Bubble fence XY distance
    // @Description: Maximum allowed deviation from the flight path in the XY plane. Set to zero to disable XY checks.
    // @Units: m
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("BFENCE_DXY", 28, AC_DroneShowManager, bubble_fence._params.distance_xy, DEFAULT_BUBBLE_FENCE_MAX_XY_DRIFT_METERS),

    // @Param: BFENCE_DZ
    // @DisplayName: Bubble fence Z distance
    // @Description: Maximum allowed deviation from the flight path along the Z axis. Set to zero to disable Z checks.
    // @Units: m
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("BFENCE_DZ", 29, AC_DroneShowManager, bubble_fence._params.distance_z, DEFAULT_BUBBLE_FENCE_MAX_Z_DRIFT_METERS),

    // @Param: BFENCE_TO
    // @DisplayName: Bubble fence timeout
    // @Description: Minimum time that the bubble fence needs to be breached to trigger the associated action
    // @Units: sec
    // @Range: 0 120
    // @User: Standard
    AP_GROUPINFO("BFENCE_TO", 30, AC_DroneShowManager, bubble_fence._params.timeout, 5),

    // @Param: BFENCE_ACT
    // @DisplayName: Bubble fence action
    // @Description: Action to take when the bubble fence is breached beyond the timeout
    // @Values: 0:None, 1:Report only, 2:Flash lights, 3:RTL, 4:Land, 5:Disarm
    // @User: Standard
    AP_GROUPINFO("BFENCE_ACT", 31, AC_DroneShowManager, bubble_fence._params.action, 1),

    // Currently used max parameter ID: 32; update this if you add more parameters.
    // Note that the max parameter ID may appear in the middle of the above list.

    AP_GROUPEND
};

bool AC_DroneShowManager::has_explicit_show_altitude_set_by_user() const
{
    return _params.origin_amsl_mm >= SMALLEST_VALID_AMSL;
}

bool AC_DroneShowManager::has_explicit_show_orientation_set_by_user() const
{
    return _params.orientation_deg >= 0;
}

bool AC_DroneShowManager::has_explicit_show_origin_set_by_user() const
{
    return _params.origin_lat != 0 && _params.origin_lng != 0;
}

bool AC_DroneShowManager::should_switch_to_show_mode_at_boot() const
{
    return _params.show_mode_settings & 1;
}

bool AC_DroneShowManager::should_switch_to_show_mode_when_authorized() const
{
    return _params.show_mode_settings & 2;
}

void AC_DroneShowManager::_check_changes_in_parameters()
{
    static int32_t last_seen_start_time_gps_sec = -1;
    static bool last_seen_show_authorization_state = false;
    static int16_t last_seen_control_rate_hz = DEFAULT_UPDATE_RATE_HZ;
    static int32_t last_seen_origin_lat = 200000000;        // intentionally invalid
    static int32_t last_seen_origin_lng = 200000000;        // intentionally invalid
    static int32_t last_seen_origin_amsl_mm = -200000000;  // intentionally invalid
    static float last_seen_orientation_deg = INFINITY;      // intentionally invalid
    uint32_t start_time_gps_msec;

    bool new_control_rate_pending = _params.control_rate_hz != last_seen_control_rate_hz;    
    bool new_coordinate_system_pending = (
        _params.origin_lat != last_seen_origin_lat ||
        _params.origin_lng != last_seen_origin_lng ||
        _params.origin_amsl_mm != last_seen_origin_amsl_mm ||
        !is_zero(_params.orientation_deg - last_seen_orientation_deg)
    );
    bool new_start_time_pending = _params.start_time_gps_sec != last_seen_start_time_gps_sec;
    bool new_show_authorization_pending = _params.authorization != last_seen_show_authorization_state;

    if (new_coordinate_system_pending) {
        // We can safely mess around with the coordinate system as we are only
        // updating the _tentative_ coordinate system here, which will take
        // effect at the next takeoff only.
        last_seen_origin_lat = _params.origin_lat;
        last_seen_origin_lng = _params.origin_lng;
        last_seen_origin_amsl_mm = _params.origin_amsl_mm;
        last_seen_orientation_deg = _params.orientation_deg;

        _copy_show_coordinate_system_from_parameters_to(_tentative_show_coordinate_system);
    }

    if (new_start_time_pending) {
        // We don't allow the user to mess around with the start time if we are
        // already performing the show, or if the show is supposed to be started
        // with a countdown
        if (
            !is_safe_to_change_start_time_in_stage(_stage_in_drone_show_mode) ||
            !uses_gps_time_for_show_start()
        ) {
            new_start_time_pending = false;
        }
    }

    if (new_start_time_pending && (_is_gps_time_ok() || _params.start_time_gps_sec < 0)) {
        last_seen_start_time_gps_sec = _params.start_time_gps_sec;

        if (last_seen_start_time_gps_sec >= 0) {
            start_time_gps_msec = last_seen_start_time_gps_sec * 1000;
            if (AP::gps().time_week_ms() < start_time_gps_msec) {
                // Interpret the given timestamp in the current GPS week as it is in
                // the future even with the same GPS week number
                _start_time_unix_usec = AP::gps().istate_time_to_epoch_ms(
                    AP::gps().time_week(), start_time_gps_msec
                ) * 1000ULL;
            } else {
                // Interpret the given timestamp in the next GPS week as it is in
                // the past with the same GPS week number
                _start_time_unix_usec = AP::gps().istate_time_to_epoch_ms(
                    AP::gps().time_week() + 1, start_time_gps_msec
                ) * 1000ULL;
            }

            if (_start_time_requested_by == StartTimeSource::NONE) {
                _start_time_requested_by = StartTimeSource::PARAMETER;
            }
        } else {
            _start_time_unix_usec = 0;
            _start_time_requested_by = StartTimeSource::NONE;
        }
    }

    if (new_show_authorization_pending) {
        last_seen_show_authorization_state = _params.authorization;

        // Show authorization state changed recently. We might need to switch
        // flight modes, but we cannot change flight modes from here so we just
        // set a flag.
        if (has_authorization() && should_switch_to_show_mode_when_authorized()) {
            _request_switch_to_show_mode();
        }
    }

    if (new_control_rate_pending) {
        last_seen_control_rate_hz = _params.control_rate_hz;

        // Validate the control rate from the parameters and convert it to
        // milliseconds
        if (last_seen_control_rate_hz < 1) {
            _controller_update_delta_msec = 1000;
        } else if (last_seen_control_rate_hz > 50) {
            _controller_update_delta_msec = 20;
        } else {
            _controller_update_delta_msec = 1000 / last_seen_control_rate_hz;
        }
    }
}

bool AC_DroneShowManager::_copy_show_coordinate_system_from_parameters_to(
    ShowCoordinateSystem& _coordinate_system
) const {
    if (!has_explicit_show_origin_set_by_user() || !has_explicit_show_orientation_set_by_user()) {
        _coordinate_system.clear();
        return false;
    }
        
    _coordinate_system.orientation_rad = radians(_params.orientation_deg);
    _coordinate_system.origin_lat = static_cast<int32_t>(_params.origin_lat);
    _coordinate_system.origin_lng = static_cast<int32_t>(_params.origin_lng);

    if (has_explicit_show_altitude_set_by_user()) {
        _coordinate_system.origin_amsl_mm = _params.origin_amsl_mm;
        if (_coordinate_system.origin_amsl_mm >= LARGEST_VALID_AMSL) {
            _coordinate_system.origin_amsl_mm = LARGEST_VALID_AMSL;
        }
        _coordinate_system.origin_amsl_valid = true;
    } else {
        _coordinate_system.origin_amsl_mm = 0;
        _coordinate_system.origin_amsl_valid = false;
    }

    return true;
}

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}
