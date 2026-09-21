#include <sys/types.h>

#include <AC_Fence/AC_Fence.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/DroneShowNotificationBackend.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#include "AC_DroneShowManager.h"

#include <skybrush/skybrush.h>

#include "DroneShow_Constants.h"
#include "DroneShowLEDFactory.h"
#include "DroneShowPyroDeviceFactory.h"

extern const AP_HAL::HAL &hal;

// LED factory that is used to create new RGB LED instances
static DroneShowLEDFactory _rgb_led_factory_singleton;

// Pyro device factory that is used to create new pyro device instances
static DroneShowPyroDeviceFactory _pyro_device_factory_singleton;

AC_DroneShowManager::AC_DroneShowManager() :
    hard_fence(),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _sock_rgb(true),
    _sock_rgb_open(false),
#endif
    _init_ok(false),
    _show_data(0),
    _stage_in_drone_show_mode(DroneShow_Off),
    _start_time_requested_by(StartTimeSource::NONE),
    _start_time_on_internal_clock_usec(0),
    _start_time_unix_usec(0),
    _trajectory_is_circular(false),
    _controller_update_delta_msec(1000 / DEFAULT_UPDATE_RATE_HZ),
    _pyro_device(0),
    _rgb_led(0),
    _rc_switches_blocked_until(0),
    _boot_count(0),
    _projected_wall_clock_time_at_takeoff_sec(NAN),
    _last_time_axis_config_seq_no(0xFFFF)    // 0xFFFF is never a valid sequence number
{
    bool ok = true;

    AP_Param::setup_object_defaults(this, var_info);
    
    ok &= (sb_trajectory_stats_init(&_trajectory_stats) == SB_SUCCESS);
    ok &= (sb_screenplay_init(&_screenplay) == SB_SUCCESS);
    ok &= (sb_screenplay_scene_init(&_main_show_scene) == SB_SUCCESS);
    ok &= (sb_show_controller_init(&_show_controller, &_screenplay) == SB_SUCCESS);
    
    _init_ok = ok;

    // Don't call _update_rgb_led_instance() or _update_pyro_device_instance()
    // here, servo framework is not set up yet
}

AC_DroneShowManager::~AC_DroneShowManager()
{
    sb_show_controller_destroy(&_show_controller);
    SB_DECREF_STATIC(&_main_show_scene);
    sb_screenplay_destroy(&_screenplay);
    sb_trajectory_stats_destroy(&_trajectory_stats);
}

void AC_DroneShowManager::early_init()
{
    _init_ok = _init_ok && _create_show_directory();
}

void AC_DroneShowManager::init(const AC_WPNav* wp_nav)
{
    // Get the boot count from the parameters
    enum ap_var_type ptype;
    AP_Int16* boot_count_param = static_cast<AP_Int16*>(AP_Param::find("STAT_BOOTCNT", &ptype));
    _boot_count = boot_count_param ? (*boot_count_param) : 0;

    // Get references to the RGB LED and pyro device factories
    _pyro_device_factory = &_pyro_device_factory_singleton;
    _rgb_led_factory = &_rgb_led_factory_singleton;
    
    // Store a reference to wp_nav so we can ask what the takeoff speed will be
    _wp_nav = wp_nav;

    // Clear start time and authorization now; at this point the parameter
    // subsystem has already loaded back the previous value from the EEPROM so
    // we are safe to overwrite it
    _params.start_time_gps_sec.set(-1);
    _params.start_time_gps_msec_offset.set(0);
    _params.authorization.set(DroneShowAuthorization_Revoked);

    _load_show_file_from_storage();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _open_rgb_led_socket();
#endif
    _update_pyro_device_instance();
    _update_rgb_led_instance();

    // initialise safety features
    hard_fence.init();
    bubble_fence.init();
}


bool AC_DroneShowManager::clear_scheduled_start_time(bool force)
{
    if (!force && _stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    _params.start_time_gps_sec.set(-1);
    _params.start_time_gps_msec_offset.set(0);
    _start_time_on_internal_clock_usec = 0;
    _start_time_requested_by = StartTimeSource::NONE;
    _start_time_unix_usec = 0;

    return true;
}

bool AC_DroneShowManager::configure_show_coordinate_system(
    int32_t lat, int32_t lng, int32_t amsl_mm, float orientation_deg
) {
    if (!check_latlng(lat, lng)) {
        return false;
    }

    if (amsl_mm >= LARGEST_VALID_AMSL) {
        return false;
    }

    // We need to set the new values _and_ save them to the EEPROM. The save
    // operation is asynchronous; it might not go through immediately, but it
    // should go through in a few milliseconds.
    _params.origin_lat.set_and_save(lat);
    _params.origin_lng.set_and_save(lng);
    _params.origin_amsl_mm.set_and_save(amsl_mm);
    _params.orientation_deg.set_and_save(orientation_deg);

#if HAL_LOGGING_ENABLED
    // Log the new values
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter("SHOW_ORIGIN_LAT", static_cast<float>(lat));
        logger->Write_Parameter("SHOW_ORIGIN_LNG", static_cast<float>(lng));
        logger->Write_Parameter("SHOW_ORIGIN_AMSL", static_cast<float>(amsl_mm));
        logger->Write_Parameter("SHOW_ORIENTATION", static_cast<float>(orientation_deg));
    }
#endif

    return true;
}

AC_BubbleFence::FenceAction AC_DroneShowManager::get_bubble_fence_action()
{
    Vector3f dist;
    AC_BubbleFence::FenceAction action;

    // Check the distance from the desired position during the performance
    // only, not in any of the other stages
    if (get_stage_in_drone_show_mode() == DroneShow_Performing) {
        get_distance_from_desired_position(dist);
        action = bubble_fence.notify_distance_from_desired_position(dist);
    } else {
        action = AC_BubbleFence::FenceAction::NONE;
    }

    return action;
}

bool AC_DroneShowManager::get_current_guided_mode_command_to_send(
    GuidedModeCommand& command,
    int32_t default_yaw_cd,
    bool altitude_locked_above_takeoff_altitude
) {
    Location loc;
    
    const uint8_t POSITION_WARNING = 1;
    const uint8_t VELOCITY_WARNING = 2;
    const uint8_t YAW_WARNING = 4;
    static uint8_t warnings_sent = 0;
    // static uint8_t counter = 0;

    float elapsed = get_elapsed_time_since_start_sec();
    float yaw_cd = default_yaw_cd;
    float yaw_rate_cd_s = 0;
    
    command.clear();
    command.yaw_cd = default_yaw_cd;

    if (!get_desired_global_position_at_seconds(elapsed, loc))
    {
        // Unable to get desired position. This is either because we have now reached
        // the end of the trajectory or because the show controller produced an output
        // without a global position. The former is not a problem so we handle that
        // gracefully.
        if (is_performance_completed()) {
            command.reached_end = true;
            return true;
        } else {
            return false;
        }
    }

    if (get_desired_yaw_cd_and_yaw_rate_cd_s_at_seconds(elapsed, yaw_cd, yaw_rate_cd_s))
    {
        // TODO(vasarhelyi): handle auto yaw mode as well

        // Prevent invalid yaw information from leaking into the guided
        // mode controller
        if (isnan(yaw_cd) || isinf(yaw_cd) || isnan(yaw_rate_cd_s) || isinf(yaw_rate_cd_s))
        {
            if (!(warnings_sent & YAW_WARNING))
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid yaw or yaw rate command; not using yaw control");
                warnings_sent |= YAW_WARNING;
            }
        }
        else
        {
            command.yaw_cd = yaw_cd;
            command.yaw_rate_cds = yaw_rate_cd_s;
        }
    }

    if (loc.get_vector_from_origin_NEU(command.pos))
    {
        /*
        counter++;
        if (counter > 4) {
            gcs().send_text(MAV_SEVERITY_INFO, "%.2f %.2f %.2f -- %.2f %.2f %.2f", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
        }
        */

        if (is_velocity_control_enabled())
        {
            float gain = get_velocity_feedforward_gain();

            if (gain > 0)
            {
                if (!get_desired_velocity_neu_in_cms_per_seconds_at_seconds(elapsed, command.vel))
                {
                    // This should not happen, but let's pretend that we received
                    // a zero velocity command instead of bailing out here
                    command.vel.zero();
                }
                
                command.vel *= gain;
            }

            // Prevent invalid velocity information from leaking into the guided
            // mode controller
            if (command.vel.is_nan() || command.vel.is_inf())
            {
                if (!(warnings_sent & VELOCITY_WARNING))
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid velocity command; using zero");
                    warnings_sent |= VELOCITY_WARNING;
                }
                command.vel.zero();
            }
        }

        // Prevent the drone from temporarily sinking below the takeoff altitude
        // if the "real" trajectory has a slow takeoff
        if (altitude_locked_above_takeoff_altitude) {
            int32_t target_altitude_above_home_cm;
            int32_t takeoff_altitude_cm;

            if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_altitude_above_home_cm)) {
                takeoff_altitude_cm = get_takeoff_altitude_cm();
                if (target_altitude_above_home_cm < takeoff_altitude_cm) {
                    // clamp the position to the target altitude, and zero out
                    // the Z component of the velocity and the acceleration
                    loc.set_alt_cm(takeoff_altitude_cm, Location::AltFrame::ABOVE_HOME);
                    if (loc.get_vector_from_origin_NEU(command.pos)) {
                        command.vel.z = 0;
                        command.acc.z = 0;
                    } else {
                        // this should not happen either, but let's handle this
                        // gracefully
                        command.unlock_altitude = true;
                    }
                } else {
                    // we want to go above the takeoff altitude so we can
                    // release the lock
                    command.unlock_altitude = true;
                }
            } else {
                // let's not blow up if get_alt_cm() fails, it's not mission-critical,
                // just release the lock
                command.unlock_altitude = true;
            }
        }

        // Prevent invalid position information from leaking into the guided
        // mode controller
        if (command.pos.is_nan() || command.pos.is_inf())
        {
            if (!(warnings_sent & POSITION_WARNING))
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid position command");
                warnings_sent |= POSITION_WARNING;
            }
            return false;
        }

        return true;
    }
    else
    {
        // No EKF origin yet, this should not have happened
        return false;
    }
}

const sb_control_output_t* AC_DroneShowManager::_get_raw_show_control_output_at_seconds(float time)
{
    uint32_t time_msec = static_cast<uint32_t>(time * 1000.0f);
    
    if (sb_show_controller_update_time_msec(&_show_controller, time_msec))
    {
        return nullptr;
    }
    
    return sb_show_controller_get_current_output(&_show_controller);
}

bool AC_DroneShowManager::get_desired_global_position_at_seconds(float time, Location& loc)
{
    const sb_control_output_t* output = _get_raw_show_control_output_at_seconds(time);
    sb_vector3_t position;

    if (!output || !sb_control_output_get_position_if_set(output, &position))
    {
        return false;
    }

    _show_coordinate_system.convert_show_to_global_coordinate(position, loc);
    return true;
}

bool AC_DroneShowManager::get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel)
{
    const sb_control_output_t* output = _get_raw_show_control_output_at_seconds(time);
    sb_vector3_t vec;
    float vel_north, vel_east;
    float orientation_rad = _show_coordinate_system.orientation_rad;

    if (!output || !sb_control_output_get_velocity_if_set(output, &vec))
    {
        return false;
    }
    
    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    vel_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    vel_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have mm/s so far, need to convert to cm/s
    vel.x = vel_north / 10.0f;
    vel.y = vel_east / 10.0f;
    vel.z = vec.z / 10.0f;
    
    return true;
}

bool AC_DroneShowManager::get_desired_yaw_cd_and_yaw_rate_cd_s_at_seconds(float time, float& yaw_cd, float& yaw_rate_cd_s)
{
    const sb_control_output_t* output = _get_raw_show_control_output_at_seconds(time);
    float yaw_deg;
    float yaw_rate_deg_s;
    
    if (
        !output ||
        !sb_control_output_get_yaw_if_set(output, &yaw_deg) ||
        !sb_control_output_get_yaw_rate_if_set(output, &yaw_rate_deg_s)
    )
    {
        return false;
    }

    yaw_cd = _show_coordinate_system.convert_show_to_global_yaw_and_scale_to_cd(yaw_deg);
    yaw_rate_cd_s = yaw_rate_deg_s * 100.0f; /* [deg] -> [cdeg] */

    return true;
}

void AC_DroneShowManager::get_distance_from_desired_position(Vector3f& vec) const
{
    if (_stage_in_drone_show_mode == DroneShow_Performing) {
        if (!get_current_relative_position_NED_origin(vec)) {
            // EKF does not know its own position yet
            vec.zero();
        } else {
            // Setpoints are in centimeters, so we need to convert the units.
            // Furthermore, the relative position is given in NED but the
            // setpoint is in NEU so we need to invert the Z axis.
            vec.z *= -1;
            vec -= (_last_setpoint.pos / 100.0f);
        }
    } else {
        vec.zero();
    }
}

bool AC_DroneShowManager::notify_drone_show_mode_initialized()
{
    _update_pyro_device_instance();
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
    
    // If an error happened during initialization (in the constructor, where we cannot
    // sensibly return an error code), or in early_init() or init(), we return false here
    return _init_ok;
}

void AC_DroneShowManager::notify_drone_show_mode_entered_stage(DroneShowModeStage stage)
{
    if (stage == _stage_in_drone_show_mode) {
        return;
    }

    _stage_in_drone_show_mode = stage;

    // Force-update preflight checks so we see the errors immediately if we
    // switched to the "waiting for start time" stage
    _update_preflight_check_result(/* force = */ true);
    
    // If we have just started the takeoff, log the current time axis configuration
    // because the first one is typically not logged (since the motors are not armed
    // when we receive it)
    if (stage == DroneShow_Takeoff) {
        write_screenplay_log_messages();
    }
}

void AC_DroneShowManager::notify_drone_show_mode_exited()
{
    _update_pyro_device_instance();
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
    _last_setpoint.clear();
}

void AC_DroneShowManager::notify_guided_mode_command_sent(const GuidedModeCommand& command)
{
    _last_setpoint = command;   
}

void AC_DroneShowManager::handle_rc_start_switch()
{
    if (_are_rc_switches_blocked())
    {
        return;
    }

    if (schedule_delayed_start_after(10000 /* msec */)) {
        // Rewrite the start time source to be "RC switch", not
        // "start method", even though we implemented it using
        // the schedule_delayed_start_after() method
        if (_start_time_requested_by == StartTimeSource::START_METHOD) {
            _start_time_requested_by = StartTimeSource::RC_SWITCH;
        }
    }
}

bool AC_DroneShowManager::schedule_delayed_start_after(uint32_t delay_ms)
{
    bool success = false;

    if (_stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    if (uses_gps_time_for_show_start()) {
        if (_is_gps_time_ok()) {
            uint32_t desired_start_time = AP::gps().time_week_ms() + delay_ms;
            success = set_scheduled_start_time_in_gps_time_of_week(
                (desired_start_time / 1000) % GPS_WEEK_LENGTH_SEC,
                desired_start_time % 1000
            );
        }
    } else {
        _start_time_on_internal_clock_usec = AP_HAL::micros64() + (delay_ms * 1000);
        success = true;
    }

    if (success) {
        _start_time_requested_by = StartTimeSource::START_METHOD;
    }

    return success;
}

bool AC_DroneShowManager::set_scheduled_start_time_in_gps_time_of_week(
    int32_t gps_sec, int16_t gps_msec_offset
) {
    bool success = true;

    if (gps_sec < 0) {
        _params.start_time_gps_sec.set(-1);
    } else if (gps_sec < GPS_WEEK_LENGTH_SEC) {
        _params.start_time_gps_sec.set(gps_sec);
    } else {
        success = false;
    }

    if (success) {
        _params.start_time_gps_msec_offset.set(gps_msec_offset);
    }

    return success;
}

void AC_DroneShowManager::update()
{
    static bool main_cycle = true;

    if (main_cycle) {
        _check_changes_in_parameters();
        _check_events();
        _check_radio_failsafe();
        _update_preflight_check_result();
        _trigger_show_events();
        _update_lights();
        _update_pyro_device();
    } else {
        _repeat_last_rgb_led_command();
    }

    main_cycle = !main_cycle;
}

bool AC_DroneShowManager::_are_rc_switches_blocked()
{
    return _rc_switches_blocked_until && _rc_switches_blocked_until >= AP_HAL::millis();
}

void AC_DroneShowManager::_check_events()
{
    DroneShowNotificationBackend* backend = DroneShowNotificationBackend::get_singleton();

    if (DroneShowNotificationBackend::events.compass_cal_failed) {
        _flash_leds_after_failure();
    } else if (DroneShowNotificationBackend::events.compass_cal_saved) {
        _flash_leds_after_success();
    }

    if (backend) {
        backend->clear_events();
    }
}

void AC_DroneShowManager::_check_radio_failsafe()
{
    if (AP_Notify::flags.failsafe_radio) {
        // Block the handling of RC switches for the next second so we don't
        // accidentally trigger a function if the user changes the state of the
        // RC switch while we are not connected to the RC.
        //
        // (E.g., switch is low, drone triggers RC failsafe because it is out
        // of range, user changes the switch, then the drone reconnects)
        _rc_switches_blocked_until = AP_HAL::millis() + 1000;
    }
}

void AC_DroneShowManager::_clear_start_time_if_set_by_switch()
{
    if (_start_time_requested_by == StartTimeSource::RC_SWITCH) {
        clear_scheduled_start_time(/* force = */ true);
    }
}

bool AC_DroneShowManager::_ensure_scene_covers_relevant_part_of_trajectory(
    sb_screenplay_scene_t* scene, float initial_rate, float final_rate
)
{
    float duration_sec;
    sb_time_axis_t* time_axis;
    sb_time_segment_t segment;

    time_axis = scene ? sb_screenplay_scene_get_time_axis(scene) : nullptr;
    if (!time_axis) {
        return false;
    }

    if (sb_screenplay_scene_get_uncovered_trajectory_duration_sec(scene, &duration_sec) != SB_SUCCESS) {
        return false;
    }
    
    switch (sb_screenplay_scene_get_tag(scene)) {
        case SceneTag_MainShow:
            if (
                isfinite(_trajectory_stats.duration_sec) &&
                isfinite(_trajectory_stats.landing_time_sec) &&
                _trajectory_stats.duration_sec >= 0 &&
                _trajectory_stats.landing_time_sec >= 0 &&
                _trajectory_stats.landing_time_sec <= _trajectory_stats.duration_sec
            ) {
                duration_sec -= _trajectory_stats.duration_sec - _trajectory_stats.landing_time_sec;
            }
            break;

        default:
            break;
    }
    
    if (duration_sec <= 0) {
        return true;
    }
    
    // We need to add a new segment to the time axis to cover the relevant part of the trajectory
    segment = sb_time_segment_make_warped(duration_sec, initial_rate, final_rate);
    return sb_time_axis_append_segment(time_axis, segment) == SB_SUCCESS;
}

bool AC_DroneShowManager::_is_at_expected_position() const
{
    if (_stage_in_drone_show_mode != DroneShow_Performing)
    {
        // Not performing a show; any position is suitable
        return true;
    }

    Location expected_loc(_last_setpoint.pos.tofloat(), Location::AltFrame::ABOVE_ORIGIN);
    return _is_close_to_position(
        expected_loc, _params.max_xy_drift_during_show_m,
        _params.max_z_drift_during_show_m
    );
}

bool AC_DroneShowManager::_is_close_to_position(
    const Location& target_loc, float xy_threshold, float z_threshold
) const
{
    Location current_loc;
    ftype alt_dist;

    if (!get_current_location(current_loc)) {
        // EKF does not know its own position yet so we report that we are not
        // at the target position
        return false;
    }

    // Location.get_distance() checks XY distance only so this is okay
    if (xy_threshold > 0 && current_loc.get_distance(target_loc) > xy_threshold) {
        return false;
    }

    if (z_threshold > 0) {
        if (!current_loc.get_alt_distance(target_loc, alt_dist)) {
            // Altitude frame is not usable; this should not happen
            return false;
        }

        if (alt_dist > z_threshold) {
            return false;
        }
    }

    return true;
}

void AC_DroneShowManager::_update_preflight_check_result(bool force)
{
    static uint32_t last_updated_at = 0;

    uint32_t now = AP_HAL::millis();
    if (!force && (now - last_updated_at) < 1000) {
        return;
    }

    last_updated_at = now;

    _preflight_check_failures = 0;

    if (_stage_in_drone_show_mode != DroneShow_WaitForStartTime) {
        /* We are not in the "waiting for start time" stage so we don't perform
         * any checks */
        return;
    }

    if (
        !loaded_show_data_successfully() ||
        !has_explicit_show_origin_set_by_user() ||
        !has_explicit_show_orientation_set_by_user()
    ) {
        _preflight_check_failures |= DroneShowPreflightCheck_ShowNotConfiguredYet;
    }

    if (_tentative_show_coordinate_system.is_valid() && !_is_at_takeoff_position_xy()) {
        _preflight_check_failures |= DroneShowPreflightCheck_NotAtTakeoffPosition;
    }
}

void AC_DroneShowManager::ShowCoordinateSystem::clear()
{
    origin_lat = origin_lng = origin_amsl_mm = 0;
    orientation_rad = 0;
    origin_amsl_valid = false;
}

float AC_DroneShowManager::ShowCoordinateSystem::convert_show_to_global_yaw_and_scale_to_cd(
    float yaw
) const {
    // show coordinates are in degrees relative to X axis orientation,
    // we need centidegrees relative to North
    return (degrees(orientation_rad) + yaw) * 100.0f;
}

void AC_DroneShowManager::ShowCoordinateSystem::convert_global_to_show_coordinate(
    const Location& loc, sb_vector3_t& vec
) const {
    Location origin;
    Vector3f diff;

    origin.zero();
    origin.lat = origin_lat;
    origin.lng = origin_lng;

    if (origin_amsl_valid) {
        // Show is controlled in AMSL
        origin.set_alt_cm(
            origin_amsl_mm / 10.0 /* [mm] -> [cm] */,
            Location::AltFrame::ABSOLUTE
        );
    } else {
        // Show is controlled in AGL. We use altitude above home because the
        // EKF origin could be anywhere -- it is typically established early
        // during the initialization process, while the home is set to the
        // point where the drone is armed.
        origin.set_alt_cm(0.0f, Location::AltFrame::ABOVE_HOME);
    }

    diff = origin.get_distance_NED_alt_frame(loc);

    diff.x *= 1000.0f;   // [m] -> [mm]
    diff.y *= -1000.0f;   // [m] -> [mm], East --> West
    diff.z *= -10.0f;     // [cm] -> [mm], Down --> Up

    // We need to rotate the X axis by -orientation_rad radians so it points
    // towards the orientation of the show axis
    vec.x = cosf(-orientation_rad) * diff.x + sinf(-orientation_rad) * diff.y;
    vec.y = -sinf(-orientation_rad) * diff.x + cosf(-orientation_rad) * diff.y;
    vec.z = diff.z;
}

void AC_DroneShowManager::ShowCoordinateSystem::convert_show_to_global_coordinate(
    sb_vector3_t vec, Location& loc
) const {
    float offset_north, offset_east, altitude;
    
    // We need to rotate the X axis by -orientation_rad radians so it points
    // North. At the same time, we also flip the Y axis so it points East and
    // not West.
    offset_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    offset_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have millimeters so far, need to convert the North and East offsets
    // to meters in the XY plane first. In the Z axis, we will need centimeters.
    offset_north = offset_north / 1000.0f; // [mm] -> [m]
    offset_east = offset_east / 1000.0f;   // [mm] -> [m]
    altitude = vec.z / 10.0f;              // [mm] -> [cm]

    // Finally, we need to offset the show origin with the calculated North and
    // East offset to get a global position

    loc.zero();
    loc.lat = origin_lat;
    loc.lng = origin_lng;

    if (origin_amsl_valid) {
        // Show is controlled in AMSL
        loc.set_alt_cm(
            static_cast<int32_t>(altitude) /* [cm] */ +
            origin_amsl_mm / 10.0 /* [mm] -> [cm] */,
            Location::AltFrame::ABSOLUTE
        );
    } else {
        // Show is controlled in AGL. We use altitude above home because the
        // EKF origin could be anywhere -- it is typically established early
        // during the initialization process, while the home is set to the
        // point where the drone is armed.
        loc.set_alt_cm(
            static_cast<int32_t>(altitude) /* [cm] */,
            Location::AltFrame::ABOVE_HOME
        );
    }

    loc.offset(offset_north, offset_east);
}
