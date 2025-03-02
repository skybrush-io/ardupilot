#include "Copter.h"

#include <skybrush/colors.h>

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Implementation of drone show flight mode
 */

bool AC_DroneShowManager_Copter::get_current_location(Location& loc) const
{
    return copter.ahrs.get_location(loc);
}

bool AC_DroneShowManager_Copter::get_current_relative_position_NED_origin(Vector3f& vec) const
{
    return copter.ahrs.get_relative_position_NED_origin(vec);
}

void AC_DroneShowManager_Copter::_request_switch_to_show_mode()
{
    // Drone show manager requested the copter to switch to show mode. We do this
    // only if the motors are not armed.
    if (!copter.motors->armed()) {
        copter.set_mode(Mode::Number::DRONE_SHOW, ModeReason::SCRIPTING);
    }
};

// Constructor.
ModeDroneShow::ModeDroneShow(void) : Mode(),
    _stage(DroneShow_Off),
    _last_home_position_reset_attempt_at(0),
    _last_stage_change_at(0)
{
}

bool ModeDroneShow::init(bool ignore_checks)
{
    initialization_start();
    return true;
}

void ModeDroneShow::exit()
{
    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Set the stage to "off"
    _set_stage(DroneShow_Off);

    // Notify the drone show manager that the drone show mode exited
    copter.g2.drone_show_manager.notify_drone_show_mode_exited();
}

bool ModeDroneShow::allows_arming(AP_Arming::Method method) const
{
    return (
        // Always allow arming from GCS in case the operator wants to test
        // the motors before takeoff. When the command does not come from the
        // GCS, arm only if we have loaded the show, the takeoff time is
        // valid and we have a show origin and orientation explicitly set up
        // by the user
        method == AP_Arming::Method::MAVLINK || (
            copter.g2.drone_show_manager.loaded_show_data_successfully() &&
            copter.g2.drone_show_manager.has_valid_takeoff_time() &&
            copter.g2.drone_show_manager.has_explicit_show_origin_set_by_user() &&
            copter.g2.drone_show_manager.has_explicit_show_orientation_set_by_user()
        )
    );
}

bool ModeDroneShow::cancel_requested() const
{
    return copter.g2.drone_show_manager.cancel_requested();
}

// Handles the takeoff command when sent from the GCS. This can be used for
// testing the takeoff before the show.
//
// We need to override the default implementation of Mode::do_user_takeoff_start()
// because that one would require the pilot to use the throttle to take off.
// Here we simply send the drone to the "takeoff" state.
bool ModeDroneShow::do_user_takeoff_start(float takeoff_alt_cm)
{
    // takeoff_alt_cm is ignored. This is deliberate; I do not want to complicate
    // the logic in takeoff_start(). The takeoff altitude can be configured in
    // a parameter (SHOW_TAKEOFF_ALT)
    if (try_to_start_motors_if_prepared_to_take_off()) {
        takeoff_start();
    }

    // takeoff_start() may not succeed if the user has not configured a show
    // origin so we check whether we have entered the takeoff stage and return
    // false if we have not.
    if (_stage == DroneShow_Takeoff) {
        // Remember to start loitering after takeoff, and indicate successs
        _next_stage_after_takeoff = DroneShow_Loiter;
        return true;
    } else {
        // Return failure to the GCS
        return false;
    }
}

int32_t ModeDroneShow::get_default_yaw_cd() const
{
    return copter.initial_armed_bearing;
}

int32_t ModeDroneShow::get_elapsed_time_since_last_home_position_reset_attempt_msec() const
{
    return AP_HAL::millis() - _last_home_position_reset_attempt_at;
}

int32_t ModeDroneShow::get_elapsed_time_since_last_stage_change_msec() const
{
    return AP_HAL::millis() - _last_stage_change_at;
}

// ModeDroneShow::run - runs the main drone show controller
// should be called at 25hz or more. This function is actually running at
// 400 Hz
void ModeDroneShow::run()
{
    check_changes_in_parameters();

    // call the correct auto controller
    switch (_stage) {

    case DroneShow_Init:
        // mode has just been initialized
        initialization_run();
        break;

    case DroneShow_WaitForStartTime:
        // waiting for start time
        wait_for_start_time_run();
        break;

    case DroneShow_Takeoff:
        // taking off
        takeoff_run();
        break;

    case DroneShow_Performing:
        // performing show
        performing_run();
        break;

    case DroneShow_Landing:
        // landing at the end of the show (normal termination)
        landing_run();
        break;

    case DroneShow_RTL:
        // returning to home position
        // - either due to abnormal termination
        // - or because the show is over and the post-show action is RTL or "RTL or land"
        rtl_run();
        break;

    case DroneShow_Loiter:
        // holding position
        // - either because joined the show while airborne
        // - or because the show is over and the post-show action is "loiter"
        loiter_run();
        break;

    case DroneShow_Landed:
        // landed successfully after a show
        landed_run();
        break;

    case DroneShow_Error:
        // failed to start a show
        error_run();
        break;

    case DroneShow_TestingLights:
        // testing light program without starting motors
        light_testing_run();
        break;

    default:
        break;
    }
}

// Checks changes in relevant parameter values and reports them to the console
void ModeDroneShow::check_changes_in_parameters()
{
    static DroneShowAuthorization last_seen_authorization;
    static uint64_t last_seen_start_time;
    DroneShowAuthorization current_authorization;
    uint64_t current_start_time;

    current_start_time = copter.g2.drone_show_manager.get_start_time_epoch_undefined();
    current_authorization = copter.g2.drone_show_manager.get_authorization_scope();

    if (current_start_time != last_seen_start_time) {
        last_seen_start_time = current_start_time;
        notify_start_time_changed();
    }

    if (last_seen_authorization != current_authorization) {
        last_seen_authorization = current_authorization;
        notify_authorization_changed();
    }
}

bool ModeDroneShow::get_wp(Location& destination) const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.get_wp(destination);
    case DroneShow_Loiter:
        return copter.mode_loiter.get_wp(destination);
    case DroneShow_Landing:
        return copter.mode_land.get_wp(destination);
    case DroneShow_RTL:
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
}

int32_t ModeDroneShow::wp_bearing() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.wp_bearing();
    case DroneShow_Landing:
        return copter.mode_land.wp_bearing();
    case DroneShow_RTL:
        return copter.mode_rtl.wp_bearing();
    default:
        return false;
    }
}

uint32_t ModeDroneShow::wp_distance() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.wp_distance();
    case DroneShow_Landing:
        return copter.mode_land.wp_distance();
    case DroneShow_RTL:
        return copter.mode_rtl.wp_distance();
    default:
        return false;
    }
}

float ModeDroneShow::crosstrack_error() const
{
    switch (_stage) {
    case DroneShow_Performing:
        return copter.mode_guided.crosstrack_error();
    case DroneShow_Loiter:
        return copter.mode_loiter.crosstrack_error();
    case DroneShow_Landing:
        return copter.mode_land.crosstrack_error();
    case DroneShow_RTL:
        return copter.mode_rtl.crosstrack_error();
    default:
        return false;
    }
}

bool ModeDroneShow::is_landing() const
{
    switch (_stage) {
        case DroneShow_Landing:
            return true;
        case DroneShow_RTL:
            return copter.mode_rtl.is_landing();
        default:
            return false;
    }
    return false;
}

bool ModeDroneShow::is_taking_off() const
{
    return ((_stage == DroneShow_Takeoff) && !wp_nav->reached_wp_destination());
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeDroneShow::use_pilot_yaw(void) const
{
    // We allow using the pilot's yaw input if guided mode is configured in this
    // way. This is because we are essentially calling copter.mode_guided later
    // in the "performing" stage periodically, and in that function it's the
    // return value of copter.mode_guided.use_pilot_yaw() that decides whether
    // yaw input is accepted anyway. This function just makes it consistent that
    // when we are doing our own takeoff, then we also do the same (because
    // takeoff is implemented with auto_takeoff_run(), which asks _us_ how the
    // pilot input should be handled).
    return copter.mode_guided.use_pilot_yaw();
}

// starts the initialization phase of the drone
void ModeDroneShow::initialization_start()
{
    // Set the appropriate stage
    _set_stage(DroneShow_Init);

    // Assume normal operation: we will start performing after the takeoff
    _next_stage_after_takeoff = DroneShow_Performing;

    // Clear the timestamp when we last attempted to arm the drone
    _prevent_arming_until_msec = 0;

    // This is copied from ModeAuto::init()

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // Clear the limits of the guided mode; we will use guided mode internally
    // to control the show
    copter.mode_guided.limit_clear();

    // Set auto-yaw mode to HOLD -- we don't want the drone to start turning
    // towards waypoints, but we don't have a fixed heading at this point where
    // we could force the drone to.
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // Part from ModeAuto::init() ends here

    // Clear all the status information that depends on the start time
    notify_start_time_changed();

    // Notify the drone show manager that the drone show mode was initialized
    copter.g2.drone_show_manager.notify_drone_show_mode_initialized();
}

// initializes the drone show mode after it has been activated the first time
void ModeDroneShow::initialization_run()
{
    // First we need to decide whether we are in the air or not
    if (is_disarmed_or_landed()) {
        // Great, let's move to the state where we wait for the start time
        wait_for_start_time_start();
    } else {
        // We are already in the air. We enter position hold mode as we don't
        // know where the show clock is
        loiter_start();
    }
}

// starts the phase where we are waiting for the start time of the show
void ModeDroneShow::wait_for_start_time_start()
{
    _set_stage(DroneShow_WaitForStartTime);

    // Remember that we have not attempted to start the motors yet
    _motors_started = false;

    // Reset home position to current location
    try_to_update_home_position();
}

// waits for the start time of the show
void ModeDroneShow::wait_for_start_time_run()
{
    AC_DroneShowManager_Copter& show_manager = copter.g2.drone_show_manager;
    float time_until_takeoff_sec = show_manager.get_time_until_takeoff_sec();
    float time_since_takeoff_sec = -time_until_takeoff_sec;
    const float latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds = 5.0f;

    // Drone is in standby so keep all I terms in controllers at zero
    attitude_control->reset_yaw_target_and_rate();
    attitude_control->reset_rate_controller_I_terms();
    pos_control->standby_xyz_reset();

    // Force attitude controller to zero target angles and yaw rate in case it
    // received something else from somewhere before we switched to this mode
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

    // This is copied from ModeStabilize::run() -- it is needed to allow the 
    // user to turn on the motors and spin them up while idling on the ground.
    // The part that allows unlimited throttle is removed; we allow unlimited
    // throttle only if we somehow ended up in the air for some strange reason
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (!copter.ap.land_complete) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    if (time_since_takeoff_sec > latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds + 1) {
        // We are late to the party, just move to the landed or poshold state.
        // The +1 second is needed to ensure that we show a "giving up"
        // failure message below
        if (is_disarmed_or_landed()) {
            landed_start();
        } else {
            // In theory, this branch should not happen because we don't move to
            // the "wait for start time" phase if we are flying
            loiter_start();
        }
    } else {
        if (time_until_takeoff_sec <= 10) {
            if (!_preflight_calibration_done) {
                // We calibrate the barometer 10 seconds before our takeoff time.
                //
                // Preflight calibration does not hurt anyone so we don't need the
                // takeoff authorization for this

                // This is copied from GCS_MAVLINK::_handle_command_preflight_calibration_baro()
                AP::baro().update_calibration();

                _preflight_calibration_done = true;
            }

            if (!_home_position_set) {
                // Update our home to the current location so we have zero AGL
                if (!try_to_update_home_position()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Could not set home position, giving up");
                    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
                    error_start();
                } else {
                    _home_position_set = true;
                }
            }
        } else {
            // We still have plenty of time until takeoff so note that we haven't
            // done the preflight calibration and haven't set the home position.
            _preflight_calibration_done = false;
            _home_position_set = false;
        }

        // Handle starting or stopping the motors depending on the authorization
        if (show_manager.has_authorization_to_start_motors()) {
            if (time_until_takeoff_sec <= 8 && !_motors_started) {
                // We attempt to start the motors 8 seconds before our takeoff time,
                // and we keep on doing so until 5 seconds after the takeoff time, when
                // we give up.
                //
                // No need to set the home position once again; arming the motors
                // will reset AGL to zero.
                if (time_since_takeoff_sec < latest_takeoff_attempt_after_scheduled_takeoff_time_in_seconds) {
                    if (try_to_start_motors_if_prepared_to_take_off()) {
                        // Great, motors started
                        _motors_started = true;
                    }
                } else {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start motors, giving up");
                    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
                    error_start();
                }
            }

            if (time_until_takeoff_sec <= 0 && _motors_started && _home_position_set) {
                // Time to take off!
                takeoff_start();
            }
        } else {
            // No authorization; stop the motors if we have started them. Note
            // that we don't do anything if _motors_started is false. The motors
            // may still be running in this case, but in this case they were
            // started by the operator with a MAVLink command for testing
            // purposes.
            if (_motors_started) {
                if (AP::arming().is_armed()) {
                    AP::arming().disarm(AP_Arming::Method::SCRIPTING);
                }

                _motors_started = false;
            }

            // If we are authorized to test the lights, check whether it is
            // time to start the test
            if (show_manager.get_authorization_scope() == DroneShowAuthorization_Granted_Lights_Only) {
                float to_wait = show_manager.get_time_until_start_sec();
                if (to_wait <= 0) {
                    light_testing_start();
                }
            }
        }
    }
}

// starts the phase where we are taking off at the start of the show
void ModeDroneShow::takeoff_start()
{
    Location current_loc(copter.current_loc);
    int32_t current_alt, target_alt;

    // check whether the drone knows its own position
    if (!copter.current_loc.initialised())
    {
        // This should not happen, but nevertheless let's move to the
        // error state if we don't know where we are
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to take off, no known location");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
        error_start();
        return;
    }

    // notify the drone show manager that we are about to take off. The drone
    // show manager _may_ cancel the takeoff if it deems that the drone is not
    // prepared for takeoff (e.g., the show origin or orientation was not
    // configured)
    if (!copter.g2.drone_show_manager.notify_takeoff_attempt())
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Takeoff cancelled by show manager");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_INITIALISE);
        error_start();
        return;
    }

    // get current altitude above EKF origin because auto_takeoff_start() works
    // with altitude above EKF origin
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, current_alt)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to get current altitude above home");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        error_start();
        return;
    }

    // now that we are past the basic checks, we can commit ourselves to entering
    // takeoff mode
    _set_stage(DroneShow_Takeoff);

    // set the target altitude of the takeoff
    target_alt = current_alt + copter.g2.drone_show_manager.get_takeoff_altitude_cm();

    // the body of this function from here on is mostly adapted from
    // ModeAuto::takeoff_start()

    // clear I term when we're taking off
    pos_control->init_z_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(target_alt, /* terrain_alt = */ false);

    // part adapted from ModeAuto::takeoff_start() ends here

    // make sure that the yaw target is our current heading and there are no I terms
    // in the attitude control rate controller
    attitude_control->reset_yaw_target_and_rate();
    attitude_control->reset_rate_controller_I_terms();

    // set yaw target to initial bearing where we were armed. Note that the yaw
    // input of the pilot will override this in auto_takeoff.run() if an RC is
    // connected and pilot yaw input in guided mode is allowed.
    //
    // Note the order of function calls here due to how AutoYaw::set_fixed_yaw()
    // is structured as of ArduCopter 4.5. If we use absolute angles, the routine
    // will start from the current yaw angle in the AutoYaw instance (which could
    // be zero if it was never used) and slowly slew to the desired angle. We
    // force the current yaw angle to be initialized by also calling
    // set_yaw_angle_rate(), which seems to set the internal yaw angle immediately
    auto_yaw.set_yaw_angle_rate(get_default_yaw_cd() * 0.01f, 0);
    auto_yaw.set_fixed_yaw(
        get_default_yaw_cd() * 0.01f,  /* [cd] -> [deg] */
        /* turn_rate_dps = */ 0, /* direction = */ 0, /* relative_angle = */ 0
    );

    // pretend that we were armed by the user by raising the throttle; the auto
    // takeoff routine won't work without this.
    copter.set_auto_armed(true);

	// also reset the landing detector state
	copter.set_land_complete(false);
}

// performs the takeoff stage
void ModeDroneShow::takeoff_run()
{
    bool completed = false;

    auto_takeoff.run();

    if (cancel_requested()) {
        // if a cancellation was requested, land immediately
        landing_start();
    } else if (!motors->armed()) {
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during takeoff.
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during takeoff");
        error_start();
    } else if (takeoff_completed()) {
        // if the takeoff has finished, move to the next stage
        completed = true;
    }

    if (completed) {
        // Choose what the next stage should be. The default stage is "performing",
        // except if we are specifically instructed to start loitering or
        // landing instead (for testing purposes)
        switch (_next_stage_after_takeoff) {
            case DroneShow_Loiter:
                loiter_start();
                break;

            case DroneShow_Landing:
            case DroneShow_Landed:
                landing_start();
                break;

            case DroneShow_TestingLights:
                // light testing is done when landed so this is an erroneous
                // configuration -- just land
                landing_start();
                break;

            default:
                performing_start();
        }

        // Reset the "next stage after takeoff" marker to its default
        _next_stage_after_takeoff = DroneShow_Performing;
    }
}

// returns whether the takeoff operation has finished successfully. Must be called
// from the takeoff stage only.
bool ModeDroneShow::takeoff_completed() const
{
    if (_stage == DroneShow_Takeoff) {
        if (_next_stage_after_takeoff == DroneShow_Performing) {
            /* Next step will start following the show trajectory. We can safely
             * enter that stage if we have reached 70% of our takeoff altitude
             * _and_ the desired altitude of the show trajectory at that time
             * is above the takeoff altitude to prevent temporarily stopping the
             * drone at the takeoff altitude */
            Location loc;
            int32_t altitude_above_home_cm;
            int32_t desired_altitude_above_home_cm;
            AC_DroneShowManager* show_manager = &copter.g2.drone_show_manager;

            if (
                show_manager->get_current_location(loc) &&
                loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_above_home_cm)
            )
            {
                if (altitude_above_home_cm >= 0.7 * show_manager->get_takeoff_altitude_cm())
                {
                    // Altitude above home seems high enough, but is the trajectory
                    // already ahead of us?
                    float elapsed = show_manager->get_elapsed_time_since_start_sec();
                    show_manager->get_desired_global_position_at_seconds(elapsed, loc);
                    if (loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, desired_altitude_above_home_cm))
                    {
                        // If the desired target is above our current altitude,
                        // we can go to the next stage, otherwise we wait until
                        // it catches up.
                        return desired_altitude_above_home_cm >= altitude_above_home_cm;
                    }
                    else
                    {
                        // This should not happen, especially because we've already been
                        // through a successful call to loc.get_alt_cm() if we managed to get
                        // here.
                        return false;
                    }
                }
                else
                {
                    // We are not above 70% of the takeoff altitude, so we continue
                    // the takeoff procedure to rise higher.
                    return false;
                }
            }
            else
            {
                // This should not happen; it usually means that we do not have an
                // EKF origin yet. The safest is to return false so we do not
                // proceed to the "performing" phase with this error.
                return false;
            }
        } else {
            /* This branch belongs to the case when we will either start
             * loitering after takeoff, or we will land immediately. In both
             * cases, ensure that we spend at least ten seconds with taking off.
             * This is needed because wp_nav->reached_wp_destination() will
             * trigger as soon as we are within WPNAV_RADIUS of the target
             * altitude, and switching to loitering immediately will mean that
             * we start loitering at an altitude below the desired one.
             * Yes, this is an ugly hack, but there is no way to start the
             * loiter mode while also specifying a target altitude to loiter at
             */
            return wp_nav->reached_wp_destination() && takeoff_timed_out();
        }
    } else if (
        (_stage >= DroneShow_Performing && _stage <= DroneShow_Landed) ||
        _stage == DroneShow_TestingLights
    ) {
        return true;
    } else {
        return false;
    }
}

// returns whether the current takeoff attempt is taking too long time
bool ModeDroneShow::takeoff_timed_out() const
{
    if (_stage == DroneShow_Takeoff) {
        return get_elapsed_time_since_last_stage_change_msec() > 10000;
    } else {
        return false;
    }
}

// starts the phase where we are actually performing the show
void ModeDroneShow::performing_start()
{
    _set_stage(DroneShow_Performing);

#if AP_LANDINGGEAR_ENABLED
    // optionally retract landing gear
    copter.landinggear.retract_after_takeoff();
#endif

    // call regular guided flight mode initialisation
    copter.mode_guided.init(true);

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();
}

// executes the show performance
void ModeDroneShow::performing_run()
{
    static uint32_t last_guided_command = 0;
    bool exited_mode = 0;
    uint32_t now = AP_HAL::millis();
    uint32_t target_dt = copter.g2.drone_show_manager.get_controller_update_delta_msec();

    if (now - last_guided_command >= target_dt) {
        if (!send_guided_mode_command_during_performance()) {
            // Failed to send guided mode command; try to switch to position
            // hold instead. This should not happen anyway.
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to send guided mode command");
            loiter_start();
            exited_mode = 1;
        }
        last_guided_command = now;
    }

    // call regular guided flight mode run function
    if (!exited_mode) {
        copter.mode_guided.run();
    }

    if (cancel_requested()) {
        // if a cancellation was requested, return to home and then land
        rtl_start();
    } else if (!motors->armed()) {
        // if the motors are not armed any more, something is wrong so move to the
        // error stage. This typically happens if we crash during a show.
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during show");
        error_start();
    } else if (performing_completed()) {
        // if we have finished the show, check the configured post-show action
        // and switch to RTL, position hold or land
        switch (copter.g2.drone_show_manager.get_action_at_end_of_show()) {
            case PostAction_RTL:
                rtl_start();
                break;
            case PostAction_Loiter:
                loiter_start();
                break;
            case PostAction_Land:
                landing_start();
                break;
            default:
                // This should not happen but let's be defensive. Safest is to
                // land in place, and it is consistent with legacy behaviour
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid post-show action, landing in place");
                landing_start();
                break;
        }
    }
}

bool ModeDroneShow::performing_completed() const
{
    // TODO(ntamas): what if we are late and we are not at the designated landing
    // position yet?
    return copter.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}

// starts the phase where we are landing at the place where we are, used at
// the end of a show
void ModeDroneShow::landing_start()
{
    _set_stage(DroneShow_Landing);

    // TODO(ntamas): set stopping point of loiter nav properly so we land as
    // close to our destination as possible

    // call regular land flight mode initialisation and ask it to ignore checks
    copter.mode_land.init(/* ignore_checks = */ true);
}

// performs the landing stage
void ModeDroneShow::landing_run()
{
    // call regular land flight mode run function
    copter.mode_land.run();

    // if we have finished landing, move to the "landed" state
    if (landing_completed()) {
        landed_start();
    }
}

// returns whether the landing operation has finished successfully. Must be called
// from the landing stage only.
bool ModeDroneShow::landing_completed() const
{
    if (_stage == DroneShow_Landing) {
        return (
            copter.ap.land_complete && (
                motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE ||
                motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN
            )
        );
    } else {
        return false;
    }
}

// starts the phase where we are returning to our home position, used during
// aborted shows or when the show trajectory has ended and the post-show action
// is set up to "RTL" or "RTL or land"
void ModeDroneShow::rtl_start()
{
    _set_stage(DroneShow_RTL);

    // call regular RTL flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(/* ignore_checks = */ true);
}

// performs the return to landing position stage
void ModeDroneShow::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(/* disarm_on_land = */ false);

    // if we have finished landing, move to the "landed" state
    if (rtl_completed()) {
        landed_start();
    }
}

// returns whether the RTL operation has finished successfully. Must be called
// from the RTL stage only.
bool ModeDroneShow::rtl_completed() const
{
    if (_stage == DroneShow_RTL) {
        return (
            copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE)
        );
    } else {
        return false;
    }
}

// starts the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_start()
{
    _set_stage(DroneShow_Loiter);

    // call regular position hold flight mode initialisation
    copter.mode_loiter.init(true);
}

// performs the phase where we are holding our position indefinitely; this happens
// when we exited show mode and then entered it again while in the air
void ModeDroneShow::loiter_run()
{
    // call regular position hold flight mode run function
    copter.mode_loiter.run();
}

// starts the phase where we have landed after a show and we do nothing any more
void ModeDroneShow::landed_start()
{
    _set_stage(DroneShow_Landed);

    copter.g2.drone_show_manager.notify_landed();
}

// performs the landed stage where we do nothing any more
void ModeDroneShow::landed_run()
{
    bool has_start_time = copter.g2.drone_show_manager.has_scheduled_start_time();

    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    // If the start time of the show is moved to the future, start the
    // initialization process again
    if (has_start_time) {
        float time_until_start_sec = copter.g2.drone_show_manager.get_time_until_start_sec();

        // The upper limit (43200 sec = 12 hours) is needed to cater for the
        // case when the GCS sends us the _original_ start time of the show
        // again (after landing) as it will then be interpreted in the next
        // GPS week (since it is in the past in the current GPS week). We don't
        // want to go back to the "waiting for start time" state in this case.
        if (time_until_start_sec > 10.0f && time_until_start_sec <= 43200.f) {
            initialization_start();
        }
    }
}

// starts the error phase where we have failed to start a show and we do nothing any more
void ModeDroneShow::error_start()
{
    _set_stage(DroneShow_Error);
}

// performs the error stage where we do nothing any more
void ModeDroneShow::error_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

// starts the light testing phase on the ground
void ModeDroneShow::light_testing_start()
{
    _set_stage(DroneShow_TestingLights);
}

// performs the light testing stage where we do nothing any more except waiting
// for the light program to finish
void ModeDroneShow::light_testing_run()
{
    // Ensure that we stay disarmed even if someone tries to arm us remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }

    if (!copter.g2.drone_show_manager.has_authorization()) {
        initialization_start();
    } else if (light_testing_completed()) {
        landed_start();
    }
}

// returns whether we should exit the light testing mode
bool ModeDroneShow::light_testing_completed() const
{
    return copter.g2.drone_show_manager.get_time_until_landing_sec() <= 0;
}

// Handler function that is called when the authorization state of the show has
// changed in the drone show manager
void ModeDroneShow::notify_authorization_changed()
{
    if (
        _stage == DroneShow_WaitForStartTime &&
        copter.g2.drone_show_manager.has_authorization()
    ) {
        // Update home position and reset AGL to zero when the show is
        // authorized and we are in the "waiting for start time" phase
        try_to_update_home_position();
    }
}

// Handler function that is called when the start time of the show has changed in the
// drone show manager
void ModeDroneShow::notify_start_time_changed()
{
    // Clear whether the preflight calibration was performed
    _preflight_calibration_done = false;

    // Clear whether the home position was set before takeoff
    _home_position_set = false;
}

// Sends a guided mode command during the show performance, calculated from the
// trajectory that the drone should follow
bool ModeDroneShow::send_guided_mode_command_during_performance()
{
    AC_DroneShowManager::GuidedModeCommand command;

    if (copter.g2.drone_show_manager.get_current_guided_mode_command_to_send(
        command, get_default_yaw_cd(),
        _altitude_locked_above_takeoff_altitude
    )) {
        copter.mode_guided.set_destination_posvelaccel(
            command.pos, command.vel, command.acc,
            /* use_yaw = */ true,
            command.yaw_cd, /* [cd] */
            /* use_yaw_rate = */ true,
            command.yaw_rate_cds  /* [cd/s] */
        );

        if (command.unlock_altitude) {
            _altitude_locked_above_takeoff_altitude = false;
        }

        copter.g2.drone_show_manager.notify_guided_mode_command_sent(command);

        return true;
    } else {
        return false;
    }
}

// Starts the motors before the show if they are not running already, irrespectively
// of whether the drone is ready to perform the show or not.
bool ModeDroneShow::start_motors_if_not_running()
{
    bool success = false;

    if (AP::arming().is_armed()) {
        // Already armed
        success = true;
    } else if (_prevent_arming_until_msec > AP_HAL::millis()) {
        // Arming prevented because we have tried it recently
    } else if (AP::arming().arm(AP_Arming::Method::SCRIPTING, /* do_arming_checks = */ true)) {
        // Started motors successfully
        success = true;
    } else {
        // Prearm checks failed; prevent another attempt for the next second
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
    }

    return success;
}

// Starts the motors before the show if they are not running already, after
// checking whether the drone is prepared to take off (according to the
// show manager)
bool ModeDroneShow::try_to_start_motors_if_prepared_to_take_off()
{
    return copter.g2.drone_show_manager.is_prepared_to_take_off() && start_motors_if_not_running();
}

// Tries to update the home position of the drone to its current location
bool ModeDroneShow::try_to_update_home_position()
{
    _last_home_position_reset_attempt_at = AP_HAL::millis();

    if (!is_disarmed_or_landed()) {
        // Don't update home position if we might be flying
        return false;
    }

    return copter.set_home_to_current_location(/* lock = */ false);
}

// Sets the stage of the drone show module and synchronizes it with the DroneShowManager
void ModeDroneShow::_set_stage(DroneShowModeStage value)
{
    _stage = value;
    _last_stage_change_at = AP_HAL::millis();

    _altitude_locked_above_takeoff_altitude = (_stage == DroneShowModeStage::DroneShow_Performing);

    copter.g2.drone_show_manager.notify_drone_show_mode_entered_stage(_stage);
}

#else

#error "You need to enable guided mode support to use the drone show mode."

#endif
