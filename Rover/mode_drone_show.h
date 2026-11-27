#pragma once

#include "mode.h"
#include <AC_DroneShowManager/AC_DroneShowManager.h>

#if MODE_DRONE_SHOW_ENABLED

// Rover implementation of the drone show manager. The show logic stays in
// AC_DroneShowManager; this class only provides vehicle-specific accessors.
class AC_DroneShowManager_Rover : public AC_DroneShowManager {
public:
    using AC_DroneShowManager::AC_DroneShowManager;

    AC_DroneShowManager_Rover(const AC_DroneShowManager_Rover &other) = delete;
    AC_DroneShowManager_Rover &operator=(const AC_DroneShowManager_Rover&) = delete;

    bool get_current_location(Location& loc) const override;
    bool get_current_relative_position_NED_origin(Vector3f& vec) const override;
    void _request_switch_to_show_mode() override;
};

// Mode 127: executes a pre-programmed Skybrush show.
// Derived from ModeGuided to reuse its position control and its path
// planning (object avoidance) for the pre-show transit and the position hold.
class ModeDroneShow : public ModeGuided
{
public:
    ModeDroneShow();
    bool allows_arming() const override;
    Number mode_number() const override { return Number::DRONE_SHOW; }
    const char *name4() const override { return "SHOW"; }

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool _enter() override;
    void _exit() override;

private:
    // trajectory tracking parameters (SHTRK_*)
    AP_Float _trk_kpos;     // SHTRK_KPOS
    AP_Float _trk_tlook;    // SHTRK_TLOOK
    AP_Float _trk_rstop;    // SHTRK_RSTOP
    AP_Float _trk_trdist;   // SHTRK_TRDIST
    AP_Float _trk_trvel;    // SHTRK_TRVEL
    DroneShowModeStage _stage = DroneShowModeStage::DroneShow_Off;
    uint32_t _last_stage_change_at = 0;
    bool _motors_started = false;
    bool _home_refreshed_before_arming = false;
    uint32_t _prevent_arming_until_msec = 0;

    // position hold before the start (vehicle armed by the operator) and
    // after the end of the show
    bool _hold_active = false;
    bool _armed_by_operator = false;
    bool _home_ok = false;              // home has been set to the actual position
    uint32_t _last_home_attempt_ms = 0;
    bool _warned_not_armed = false;     // T-30 "not armed" warning already sent
    bool _hold_announced = false;       // "in position" already announced
    Location _hold_target;              // kept to re-check drift (wp_nav latches "reached")
    int32_t _hold_heading_cd = 0;       // heading locked on arrival
    uint32_t _hold_last_check_ms = 0;
    void hold_start(const Location& target, float vmax);
    void hold_start_at_takeoff_position();
    void hold_run();

    void _set_stage(DroneShowModeStage value);

    void initialization_start();
    void initialization_run();
    void wait_for_start_time_start();
    void wait_for_start_time_run();
    void takeoff_start();
    void takeoff_run();
    void performing_start();
    void performing_run();
    void loiter_start();
    void loiter_run();
    bool send_guided_mode_command_during_performance(bool& reached_end) WARN_IF_UNUSED;
    void landing_start();
    void landing_run();
    void landed_start();
    void error_start();
    void error_run();
    bool try_to_start_motors_if_prepared_to_take_off();
    bool start_motors_if_not_running();
};

#endif  // MODE_DRONE_SHOW_ENABLED
