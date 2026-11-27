#include "Rover.h"

#if MODE_DRONE_SHOW_ENABLED

// --- AC_DroneShowManager_Rover ---

bool AC_DroneShowManager_Rover::get_current_location(Location& loc) const
{
    return AP::ahrs().get_location(loc);
}

bool AC_DroneShowManager_Rover::get_current_relative_position_NED_origin(Vector3f& vec) const
{
    return AP::ahrs().get_relative_position_NED_origin_float(vec);
}

void AC_DroneShowManager_Rover::_request_switch_to_show_mode()
{
    // same as ArduCopter: never switch to show mode while armed
    if (!AP::arming().is_armed()) {
        rover.set_mode(Mode::Number::DRONE_SHOW, ModeReason::MISSION_CMD);
    }
}

// --- ModeDroneShow ---

const AP_Param::GroupInfo ModeDroneShow::var_info[] = {
    // @Param: KPOS
    // @DisplayName: Show tracking catch-up gain
    // @Description: Gain applied to the along-track error: commanded speed = show speed + KPOS x along-track error. Also used as the approach gain when the show point is stationary and the vehicle is further than RSTOP
    // @Units: 1/s
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("KPOS", 1, ModeDroneShow, _trk_kpos, 0.5f),

    // @Param: TLOOK
    // @DisplayName: Show tracking look-ahead time
    // @Description: The heading command points towards the show position TLOOK seconds ahead along the show velocity
    // @Units: s
    // @Range: 0.2 5
    // @User: Advanced
    AP_GROUPINFO("TLOOK", 2, ModeDroneShow, _trk_tlook, 1.0f),

    // @Param: RSTOP
    // @DisplayName: Show tracking stop radius
    // @Description: When the show point is stationary and the vehicle is closer than this distance, the speed command is zero. Also the drift tolerance of the position hold
    // @Units: m
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("RSTOP", 3, ModeDroneShow, _trk_rstop, 0.3f),

    // @Param: TRDIST
    // @DisplayName: Pre-show transit maximum distance
    // @Description: Maximum distance of the autonomous transit to the start point of the show after the vehicle has been armed by the operator. Beyond this distance the vehicle holds its position and sends a warning
    // @Units: m
    // @Range: 0 300
    // @User: Advanced
    AP_GROUPINFO("TRDIST", 4, ModeDroneShow, _trk_trdist, 150.0f),

    // @Param: TRVEL
    // @DisplayName: Pre-show transit speed
    // @Description: Maximum speed of the autonomous transit to the start point of the show
    // @Units: m/s
    // @Range: 0.1 1.5
    // @User: Advanced
    AP_GROUPINFO("TRVEL", 5, ModeDroneShow, _trk_trvel, 0.8f),

    AP_GROUPEND
};

ModeDroneShow::ModeDroneShow() : ModeGuided()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeDroneShow::_enter()
{
    if (ModeGuided::_enter() == false) {
        return false;
    }
    // Counterpart of notify_drone_show_mode_exited() in _exit(). Without it the
    // pyro device and the RGB LED were only created at boot, so changing
    // SHOW_PYRO_TYPE required a reboot instead of leaving and re-entering the
    // show mode.
    if (!rover.g2.drone_show_manager.notify_drone_show_mode_initialized()) {
        return false;
    }
    initialization_start();
    return true;
}

void ModeDroneShow::_exit()
{
    _set_stage(DroneShowModeStage::DroneShow_Off);
    rover.g2.drone_show_manager.notify_drone_show_mode_exited();
}

void ModeDroneShow::update()
{
    switch (_stage) {
    case DroneShowModeStage::DroneShow_Init:
        initialization_run();
        break;

    case DroneShowModeStage::DroneShow_WaitForStartTime:
        wait_for_start_time_run();
        break;

    case DroneShowModeStage::DroneShow_Takeoff:
        takeoff_run();
        break;

    case DroneShowModeStage::DroneShow_Performing:
        performing_run();
        break;

    case DroneShowModeStage::DroneShow_Loiter:
        loiter_run();
        break;

    case DroneShowModeStage::DroneShow_Landing:
        landing_run();
        break;

    case DroneShowModeStage::DroneShow_Landed:
        // end of the show: keep holding position while armed
        if (_hold_active) {
            hold_run();
        }
        break;

    case DroneShowModeStage::DroneShow_Error:
        error_run();
        break;

    default:
        break;
    }

    ModeGuided::update();
}

void ModeDroneShow::_set_stage(DroneShowModeStage value)
{
    _stage = value;
    _last_stage_change_at = AP_HAL::millis();
    rover.g2.drone_show_manager.notify_drone_show_mode_entered_stage(_stage);
}

void ModeDroneShow::initialization_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Init);
    _home_ok = false;

    // Rover sets its home at boot, possibly before the altitude is known
    // (home at ~0 m AMSL). An altitude fence would then be breached and block
    // arming, so home is set to the current location while disarmed.
    if (!AP::arming().is_armed()) {
        if (!rover.set_home_to_current_location(false)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Show: home not updated");
        }
    }
}

void ModeDroneShow::initialization_run()
{
    // surface vehicle: always "on the ground", wait for the start time
    wait_for_start_time_start();
}

void ModeDroneShow::wait_for_start_time_start()
{
    _home_refreshed_before_arming = false;

    _set_stage(DroneShowModeStage::DroneShow_WaitForStartTime);
    _motors_started = false;
    _hold_active = false;
    _armed_by_operator = false;
    _warned_not_armed = false;
}

void ModeDroneShow::wait_for_start_time_run()
{
    AC_DroneShowManager_Rover& mgr = rover.g2.drone_show_manager;
    float time_until_takeoff_sec = mgr.get_time_until_takeoff_sec();
    float time_since_takeoff_sec = -time_until_takeoff_sec;
    const float latest_attempt_after_takeoff_sec = 5.0f;

    // When booting directly into show mode, the mode is entered before the GPS
    // has a fix and setting home fails. Retry once per second while disarmed.
    if (!_home_ok && !AP::arming().is_armed() && AP_HAL::millis() - _last_home_attempt_ms > 1000) {
        _last_home_attempt_ms = AP_HAL::millis();
        if (rover.set_home_to_current_location(false)) {
            _home_ok = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Show: home set");
        }
    }

    // A boat is launched and armed by the operator before the start: it goes
    // to its start point (first point of its trajectory) and holds it until T0.
    if (AP::arming().is_armed() && !_motors_started && !_hold_active) {
        _armed_by_operator = true;
        hold_start_at_takeoff_position();
    } else if (!AP::arming().is_armed()) {
        _hold_active = false;
        _armed_by_operator = false;
    }
    if (_hold_active) {
        hold_run();
    }

    if (mgr.has_authorization_to_start_motors()) {
        // warn the operator at T-30 if the vehicle has not been armed yet
        if (time_until_takeoff_sec <= 30 && time_until_takeoff_sec > 8 && !AP::arming().is_armed() && !_warned_not_armed) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Show: not armed by operator (T-30)");
            _warned_not_armed = true;
        }
        // set home once more at T-10, in case the EKF was not using the GPS
        // yet when the mode was entered
        if (time_until_takeoff_sec <= 10 && !_home_refreshed_before_arming && !AP::arming().is_armed()) {
            if (!rover.set_home_to_current_location(false)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Show: home not updated at T-10");
            }
            _home_refreshed_before_arming = true;
        }

        if (time_until_takeoff_sec <= 8 && _motors_started == false) {
            // try to arm from T-8, give up at T+5
            if (time_since_takeoff_sec < latest_attempt_after_takeoff_sec) {
                if (try_to_start_motors_if_prepared_to_take_off()) {
                    _motors_started = true;
                    if (!_armed_by_operator) {
                        gcs().send_text(MAV_SEVERITY_WARNING, "Show: armed automatically at T-8 (fallback)");
                    }
                }
            } else if (AP::arming().is_armed()) {
                // Mode re-entered after the start window with the vehicle
                // armed, i.e. on the water: hold position (loiter) instead of
                // going to the error stage, which would disarm and let it drift.
                gcs().send_text(MAV_SEVERITY_WARNING, "Show: late re-entry, holding position");
                loiter_start();
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start motors, giving up");
                error_start();
            }
        }

        if (time_until_takeoff_sec <= 0 && _motors_started) {
            takeoff_start();
        }
    } else {
        // authorization revoked: stop the motors if we started them
        if (_motors_started) {
            // a vehicle armed by the operator is on the water: keep holding
            if (AP::arming().is_armed() && !_armed_by_operator) {
                AP::arming().disarm(AP_Arming::Method::SCRIPTING);
            }
            _motors_started = false;
        }
    }
}

// surface vehicle: no takeoff, go straight to the performing stage
void ModeDroneShow::takeoff_start()
{
    AC_DroneShowManager_Rover& mgr = rover.g2.drone_show_manager;

    if (rover.current_loc.initialised() == false) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start, no known location");
        error_start();
        return;
    }

    // the show manager may cancel the start, e.g. if the origin or the
    // orientation of the show is missing
    if (mgr.notify_takeoff_attempt() == false) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Start cancelled by show manager");
        error_start();
        return;
    }

    mgr.notify_takeoff_started();

    _set_stage(DroneShowModeStage::DroneShow_Takeoff);
}

// surface vehicle: nothing to wait for, continue with the performance
void ModeDroneShow::takeoff_run()
{
    AC_DroneShowManager_Rover& mgr = rover.g2.drone_show_manager;

    if (mgr.is_collective_rth_triggered()) {
        landing_start();
        return;
    }

    if (mgr.is_motor_output_disabled() == false && AP::arming().is_armed() == false) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during takeoff");
        error_start();
        return;
    }

    performing_start();
}

void ModeDroneShow::performing_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Performing);
}

bool ModeDroneShow::send_guided_mode_command_during_performance(bool& reached_end)
{
    AC_DroneShowManager::GuidedModeCommand command;
    AC_DroneShowManager* show_manager = &rover.g2.drone_show_manager;

    reached_end = false;

    // The altitude lock argument is omitted on purpose: altitude is ignored
    // on a surface vehicle.
    if (!show_manager->get_current_guided_mode_command_to_send(command, AP::ahrs().get_yaw_rad())) {
        // no command available: stop the vehicle
        set_desired_turn_rate_and_speed(0, 0);
        return false;
    }

    if (command.reached_end) {
        reached_end = true;
        return true;
    }

    if (!show_manager->is_motor_output_disabled()) {
        // Heading and speed tracking: heading towards the show position (with
        // look-ahead) and show speed corrected by the along-track error.
        // Aiming at a point ahead of the show position instead left a
        // constant delay of about 1 s.
        // Requires SHOW_VEL_FF_GAIN = 1 so that command.vel is the actual
        // show velocity.
        Vector3f actual_cm;
        if (!rover.current_loc.get_vector_from_origin_NEU(actual_cm)) {
            set_desired_turn_rate_and_speed(0, 0);
            return false;
        }
        const Vector2f pos_m(command.pos.x * 0.01f, command.pos.y * 0.01f);
        const Vector2f vel_ms(command.vel.x * 0.01f, command.vel.y * 0.01f);
        const Vector2f actual_m(actual_cm.x * 0.01f, actual_cm.y * 0.01f);
        const Vector2f err = pos_m - actual_m;   // from the vehicle to the show position (m)
        const float v_show = vel_ms.length();
        const float k_pos = _trk_kpos.get();
        const float t_look = _trk_tlook.get();
        const float r_stop = _trk_rstop.get();
        float speed;
        Vector2f aim;
        if (v_show > 0.05f) {
            const float e_along = err * (vel_ms / v_show);   // > 0: vehicle is late
            speed = v_show + k_pos * e_along;
            aim = err + vel_ms * t_look;
        } else {
            const float d = err.length();
            speed = (d > r_stop) ? k_pos * d : 0.0f;
            aim = err;
        }
        speed = constrain_float(speed, 0.0f, g2.wp_nav.get_default_speed());
        if (speed > 0.0f && aim.length() > 0.05f) {
            const float heading_cd = wrap_360_cd(degrees(atan2f(aim.y, aim.x)) * 100.0f);
            set_desired_heading_and_speed(heading_cd, speed);
        } else {
            set_desired_turn_rate_and_speed(0, 0);
        }
    }

    show_manager->notify_guided_mode_command_sent(command);
    return true;
}

void ModeDroneShow::performing_run()
{
    static uint32_t last_guided_command_attempted_at = 0;
    const uint32_t now = AP_HAL::millis();
    AC_DroneShowManager* show_manager = &rover.g2.drone_show_manager;
    bool should_exit_mode = false;
    bool reached_end = false;

    if (now - last_guided_command_attempted_at >= show_manager->get_controller_update_delta_msec()) {
        if (!send_guided_mode_command_during_performance(reached_end)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Failed to send guided mode command");
            should_exit_mode = true;
        }
        last_guided_command_attempted_at = now;
    }

    // ModeGuided::update() is called by update() after the switch

    if (!show_manager->is_motor_output_disabled() && !AP::arming().is_armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during show");
        error_start();
    } else if (should_exit_mode) {
        // anomaly: stay armed and hold position
        loiter_start();
    } else if (reached_end) {
        // normal end of the show
        landing_start();
    }
}

bool ModeDroneShow::allows_arming() const
{
    // no arming in show mode without a valid show, with origin and
    // orientation set by the ground station
    const AC_DroneShowManager_Rover& mgr = rover.g2.drone_show_manager;
    return mgr.loaded_show_data_successfully() &&
        mgr.is_trajectory_plausible() &&
        mgr.has_explicit_show_origin_set_by_user() &&
        mgr.has_explicit_show_orientation_set_by_user();
}

void ModeDroneShow::loiter_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Loiter);

    // Fallback on anomaly: the vehicle stays armed and holds its position
    // (loiter sub-mode of ModeGuided) instead of disarming and drifting.
    if (!start_loiter()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Show: loiter failed, stopping");
        set_desired_turn_rate_and_speed(0, 0);
    }
}

void ModeDroneShow::loiter_run()
{
    // the loiter sub-mode is run by ModeGuided::update(), called after the switch
    if (!AP::arming().is_armed()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors disarmed during loiter");
        error_start();
    }
}

void ModeDroneShow::landing_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Landing);
}

void ModeDroneShow::landing_run()
{
    // End of the show on the water: the vehicle stays armed and holds its
    // position. Disarming is left to the operator when the boat is recovered.
    _hold_active = false;
    if (AP::arming().is_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Show: end, holding position");
        hold_start(rover.current_loc, 0.3f);
    }
    landed_start();
}

// Position hold and transit use set_desired_location(), like GUIDED (wp_nav,
// with AP_OADijkstra if OA_TYPE != 0), so that exclusion zones are avoided.
// ModeGuided::update(), called after the switch in update(), runs the
// navigation at each loop.
void ModeDroneShow::hold_start(const Location& target, float vmax)
{
    _hold_target = target;
    _hold_active = true;
    _hold_announced = false;
    _hold_last_check_ms = AP_HAL::millis();
    if (!set_desired_location(target)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Show: hold target rejected");
    }
    set_desired_speed(vmax);
}

void ModeDroneShow::hold_start_at_takeoff_position()
{
    const float max_distance = _trk_trdist.get();
    Location target;
    if (!rover.g2.drone_show_manager.get_global_takeoff_position(target)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Show: start point unknown, holding here");
        hold_start(rover.current_loc, 0.3f);
        return;
    }
    const float d = rover.current_loc.get_distance(target);
    if (d > max_distance) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Show: start point %.0fm away (> TRDIST), holding here", (double)d);
        hold_start(rover.current_loc, 0.3f);
        return;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "Show: going to start point (%.0fm)", (double)d);
    hold_start(target, _trk_trvel.get());
}

void ModeDroneShow::hold_run()
{
    if (!AP::arming().is_armed()) {
        _hold_active = false;
        return;
    }

    if (!reached_destination()) {
        // navigation is done by ModeGuided::update() (WP sub-mode, avoidance included)
        return;
    }

    if (!_hold_announced) {
        _hold_announced = true;
        _hold_heading_cd = AP::ahrs().yaw_sensor;
        gcs().send_text(MAV_SEVERITY_WARNING, "Show: in position");
    }

    // Once the destination is reached, wp_nav never detects a drift again (its
    // "reached" flag is only reset by a new target). During long waits between
    // two scenes the position is re-checked every 2 s and the navigation is
    // restarted if the vehicle has drifted.
    if (AP_HAL::millis() - _hold_last_check_ms > 2000) {
        _hold_last_check_ms = AP_HAL::millis();
        if (rover.current_loc.get_distance(_hold_target) > _trk_rstop.get()) {
            _hold_announced = false;
            if (!set_desired_location(_hold_target)) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Show: hold target rejected");
            }
            set_desired_speed(0.3f);
            return;
        }
    }

    // While not drifting, lock the heading (zero speed) instead of letting the
    // boat turn freely with wind and current: an LED cable can wind around the
    // mast during long waits.
    set_desired_heading_and_speed(_hold_heading_cd, 0.0f);
}

void ModeDroneShow::landed_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Landed);
    _motors_started = false;
}

void ModeDroneShow::error_start()
{
    _set_stage(DroneShowModeStage::DroneShow_Error);
}

void ModeDroneShow::error_run()
{
    // stay disarmed even if someone tries to arm remotely
    if (AP::arming().is_armed()) {
        AP::arming().disarm(AP_Arming::Method::SCRIPTING);
    }
}

bool ModeDroneShow::try_to_start_motors_if_prepared_to_take_off()
{
    return rover.g2.drone_show_manager.is_prepared_to_take_off() && start_motors_if_not_running();
}

bool ModeDroneShow::start_motors_if_not_running()
{
    bool success = false;

    if (rover.g2.drone_show_manager.is_motor_output_disabled()) {
        success = true;
    } else if (AP::arming().is_armed()) {
        success = true;
    } else if (_prevent_arming_until_msec > AP_HAL::millis()) {
        // arming blocked after a recent failure
    } else if (AP::arming().arm(AP_Arming::Method::SCRIPTING, true)) {
        success = true;
    } else {
        _prevent_arming_until_msec = AP_HAL::millis() + 1000;
    }

    return success;
}

#endif  // MODE_DRONE_SHOW_ENABLED
