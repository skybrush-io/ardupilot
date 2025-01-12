#pragma once

/// @file   AC_DroneShowManager.h
/// @brief  Drone show state management library

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/RGBLed.h>
#include <AP_Param/AP_Param.h>

#include <AC_HardFence/AC_HardFence.h>
#include <AC_WPNav/AC_WPNav.h>

#include <skybrush/colors.h>

struct sb_trajectory_s;
struct sb_trajectory_player_s;

struct sb_light_program_s;
struct sb_light_player_s;

struct sb_yaw_control_s;
struct sb_yaw_player_s;

class DroneShowLEDFactory;
class DroneShowLED;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#  include <AP_HAL/utility/Socket.h>
#endif

// Drone show mode stages
enum DroneShowModeStage {
    DroneShow_Off,
    DroneShow_Init,
    DroneShow_WaitForStartTime,
    DroneShow_Takeoff,
    DroneShow_Performing,
    DroneShow_RTL,
    DroneShow_Loiter,
    DroneShow_Landing,
    DroneShow_Landed,
    DroneShow_Error,
};

// Enum representing the flags in the control mode bitmasp
enum DroneShowControlModeFlag {
    DroneShowControl_VelocityControlEnabled = 1,
    DroneShowControl_AccelerationControlEnabled = 2,
};

// Enum representing the authorization types for the start of the show
enum DroneShowAuthorization : uint8_t {
    // Show not authorized to start
    DroneShowAuthorization_Revoked = 0,

    // SHow authorized to start in live mode (all safety features enabled)
    DroneShowAuthorization_Granted_Live = 1,

    // Show authorized to start in rehearsal mode (bubble fence action forcibly
    // set to reporting only)
    DroneShowAuthorization_Granted_Rehearsal = 2,

    // Show authorized to start with lights only, no takeoff is allowed
    DroneShowAuthorization_Granted_Lights_Only = 3,
};

// Flags representing various failures in drone show specific preflight checks.
// These are not part of the standard ArduPilot prearm check framework; we
// check these periodically on our own when we are in the "waiting for start
// time" stage.
//
// The numeric values are important. In the status packet we have four bits to
// send information about preflight checks, but some things that are checked in
// the preflight code are also sent in the status packet in other places (due to
// historical reasons). The flags that must be sent in those four bits that we
// have available for preflight check information must have values >= 16. Sorry,
// this is a bit of a mess but we need to keep backwards compatibility.
enum DroneShowPreflightCheckFlag {
    DroneShowPreflightCheck_ShowNotConfiguredYet = (1 << 0),
    // Flags from this point onwards are sent in the dedicated four bits of the
    // status packet
    DroneShowPreflightCheck_NotAtTakeoffPosition = (1 << 7),
};

// Light effect type when the lights are driven from the GCS
enum LightEffectType {
    LightEffect_Off,
    LightEffect_Solid,
    LightEffect_Blinking,
    LightEffect_Breathing,
    LightEffect_Last = LightEffect_Breathing
};

// Priority of light effects to allow internal requests to take precedence over
// individual user requests, and to allow individual user requests to take
// precedence over requests broadcast from the GCS
enum LightEffectPriority {
    LightEffectPriority_None = 0,
    LightEffectPriority_Broadcast = 1, // preferred swarm-level color sent from GCS
    LightEffectPriority_Individual = 2, // preferred color requested individually
    LightEffectPriority_Internal = 3 // internal light signals, e.g. compass calibration light signal
};

// Time synchronization mode used when starting the show
enum TimeSyncMode {
    TimeSyncMode_Countdown = 0, // Ignore SHOW_START_TIME and expect countdown messages from GCS
    TimeSyncMode_GPS = 1 // Use SHOW_START_TIME and synchronize based on GPS time
};

// Possible actions to take at the end of the show
enum PostAction {
    PostAction_Loiter = 0, // Switch to loiter flight mode
    PostAction_Land = 1,   // Switch to landing flight mode
    PostAction_RTL = 2,    // Switch to RTL flight mode unconditionally
    PostAction_RTLOrLand = 3 // Switch to RTL flight mode if above home, otherwise switch to landing flight mode
};

/// @class  AC_DroneShowManager
/// @brief  Class managing the trajectory and light program of a drone show
class AC_DroneShowManager {

private:
    class ShowCoordinateSystem {
    public:
        // Latitude of the origin of the show coordinate system, in 1e-7 degrees
        int32_t origin_lat;

        // Longitude of the origin of the show coordinate system, in 1e-7 degrees
        int32_t origin_lng;

        // Altitude of the origin of the show coordinate system above mean sea level, in millimeters.
        // Valid if and only if _origin_amsL_is_valid is set to true.
        int32_t origin_amsl_mm;

        // Orientation of the X axis of the show coordinate system, in radians
        float orientation_rad;

        // Stores whether the altitude above mean sea level is valid. When it is true,
        // the show is controlled in AMSL. When it is false, the show is controlled
        // in AGL.
        bool origin_amsl_valid;

        // Clears the show coordinate system, resetting the origin back to Null Island
        void clear();

        // Converts a coordinate given in the show coordinate system, in millimeters, to
        // the global GPS coordinate system
        void convert_show_to_global_coordinate(sb_vector3_with_yaw_t vec, Location& loc) const;

        // Converts a yaw angle given in the show coordinate system, in degrees, to
        // centidegrees relative to north
        float convert_show_to_global_yaw_and_scale_to_cd(float value) const;

        // Returns whether the coordinate system is valid.
        bool is_valid() const { return origin_lat != 0 && origin_lng != 0; };
    };

public:
    AC_DroneShowManager();
    ~AC_DroneShowManager();

    /* Do not allow copies */
    AC_DroneShowManager(const AC_DroneShowManager &other) = delete;
    AC_DroneShowManager &operator=(const AC_DroneShowManager&) = delete;

    // Enum describing the possible sources of the start time currently set in the manager
    enum StartTimeSource {
        NONE = 0,         // No start time was set
        PARAMETER = 1,    // start time was set by the user via the START_TIME parameter
        START_METHOD = 2, // start time was set by calling the schedule_delayed_start_after() method
        RC_SWITCH = 3     // start time was set via the RC switch action
    };

    // Simple struct to contain a guided mode command that should be sent during
    // performance
    struct GuidedModeCommand {
        Vector3f pos;
        Vector3f vel;
        Vector3f acc;
        bool unlock_altitude;
        float yaw_cd;
        float yaw_rate_cds;

        void clear() {
            pos.zero();
            vel.zero();
            acc.zero();
            unlock_altitude = false;
            yaw_cd = 0.0f;
            yaw_rate_cds = 0.0f;
        }
    };

    // Early initialization steps that have to be called early in the boot process
    // to ensure we have enough memory to do them even on low-memory boards like
    // a Pixhawk1
    void early_init();

    // Initializes the drone show subsystem at boot time
    void init(const AC_WPNav* wp_nav);

    // Returns whether the user has asked the drone show manager to cancel the
    // show as soon as possible. This flag is checked regularly from
    // mode_drone_show.cpp
    bool cancel_requested() const { return _cancel_requested; }

    // Clears the scheduled time for a collective RTL maneuver. Returns whether
    // the request was processed.
    //
    // This function is a no-op if the drone show is not in the "performing"
    // phase and 'force' is set to false.
    bool clear_scheduled_collective_rtl(bool force = false);

    // Clears the scheduled start time of the show (but does not cancel the
    // show if it is already running). Returns whether the request was
    // processed.
    //
    // This function is a no-op if the drone show is not in the "waiting for
    // start time" phase and 'force' is set to false.
    bool clear_scheduled_start_time(bool force = false);

    // Configures the show origin, orientation and AMSL reference in a single
    // call. This function sets the corresponding SHOW_ parameters as if they
    // were set with multiple PARAM_SET MAVLink commands.
    //
    // The show will be controlled in AGL if the amsl argument is less than
    // SMALLEST_VALID_AMSL, i.e. less than or equal to
    bool configure_show_coordinate_system(
        int32_t lat, int32_t lon, int32_t amsl_mm, float orientation_deg
    ) WARN_IF_UNUSED;

    // Returns the color of the LED light on the drone according to its light
    // program the given number of seconds after the start time.
    void get_color_of_rgb_light_at_seconds(float time, sb_rgb_color_t* color);

    // Returns the preferred duration between consecutive guided mode commands
    // during the execution of the show.
    uint32_t get_controller_update_delta_msec() const { return _controller_update_delta_msec; }

    // Returns the guided mode command that should be sent during the performance
    // when the function is invoked
    bool get_current_guided_mode_command_to_send(
        GuidedModeCommand& command,
        int32_t default_yaw_cd,
        bool altitude_locked_above_takeoff_altitude = false
    ) WARN_IF_UNUSED;

    // Retrieves the current absolute location of the vehicle from the EKF
    virtual bool get_current_location(Location& loc) const { return false; }

    // Retrieves the current location of the vehicle from the EKF, relative to
    // the EKF origin, in meters
    virtual bool get_current_relative_position_NED_origin(Vector3f& vec) const { return false; }

    // Returns the desired position of the drone during the drone show the
    // given number of seconds after the start time, in the global coordinate
    // system, using centimeters as units.
    void get_desired_global_position_at_seconds(float time, Location& loc);

    // Returns the desired velocity of the drone during the drone show the
    // given number of seconds after the start time, in the global NEU
    // cooordinate system, using centimeters per seconds as units.
    void get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel);

    // Returns the desired acceleration of the drone during the drone show the
    // given number of seconds after the start time, in the global NEU
    // cooordinate system, using centimeters per seconds squared as units.
    void get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(float time, Vector3f& acc);

    // Returns the desired yaw of the drone during the drone show the
    // given number of seconds after the start time, in centidegrees 
    // relative to North.
    float get_desired_yaw_cd_at_seconds(float time);

    // Returns the desired yaw rate of the drone during the drone show the
    // given number of seconds after the start time, in centidegrees/seconds.
    float get_desired_yaw_rate_cds_at_seconds(float time);

    // Returns the distance of the drone from its desired position during the
    // "Performing" stage of the show. Returns zero distance when not doing a show.
    void get_distance_from_desired_position(Vector3f& vec) const;

    // Returns the action to perform at the end of the show. This function
    // evaluates the SHOW_POST_ACTION parameter and the distance between the
    // takeoff point and the current position of the drone to decide whether to
    // RTL or land if the post-show action is set to "RTL or land", otherwise
    // it returns the action set in the SHOW_POST_ACTION parameter. Note that
    // the function is guaranteed never to return PostAction_RTLOrLand
    PostAction get_action_at_end_of_show() const;

    // Retrieves the position where the drone is supposed to be at the start of the show.
    // Returns true if successful or false if the show coordinate system was not set up
    // by the user yet.
    bool get_global_takeoff_position(Location& loc) const;
    
    // Returns the last color that was emitted to the RGB light
    void get_last_rgb_led_color(sb_rgb_color_t& color) const { color = _last_rgb_led_color; }

    // Returns the landing time relative to the start of the show
    float get_relative_landing_time_sec() const { return _landing_time_sec; }

    // Returns the takeoff time relative to the start of the show
    float get_relative_takeoff_time_sec() const { return _takeoff_time_sec; }

    // Returns the start time in microseconds. Depending on the value of the
    // SHOW_SYNC_MODE parameter, this might be an internal timestamp or a
    // UNIX timestamp. Do _not_ use this method for anything else than
    // determining whether the start time was changed.
    uint64_t get_start_time_epoch_undefined() const {
        return (
            _params.time_sync_mode == TimeSyncMode_Countdown
            ? _start_time_on_internal_clock_usec
            : _start_time_unix_usec
        );
    }

    // Returns the total duration of the loaded trajectory, in seconds
    float get_total_duration_sec() const { return _total_duration_sec; }

    // Returns the number of seconds elapsed since show start, in microseconds
    int64_t get_elapsed_time_since_start_usec() const;

    // Returns the number of seconds elapsed since show start, in milliseconds
    int32_t get_elapsed_time_since_start_msec() const;

    // Returns the number of seconds elapsed since show start, in seconds
    float get_elapsed_time_since_start_sec() const;

    // Returns the current stage that the drone show mode is in
    DroneShowModeStage get_stage_in_drone_show_mode() const { return _stage_in_drone_show_mode; }

    // Returns the takeoff acceleration in meters per second squared
    float get_motor_spool_up_time_sec() const;
    
    // Returns the takeoff acceleration in meters per second squared
    float get_takeoff_acceleration_m_ss() const {
        float result = _wp_nav ? _wp_nav->get_accel_z() / 100.0f : 0;
        if (result <= 0) {
            /* safety check */
            result = DEFAULT_TAKEOFF_ACCELERATION_METERS_PER_SEC_SEC;
        }
        return result;
    }

    // Returns the altitude to take off to above the current position of the drone, in centimeters
    int32_t get_takeoff_altitude_cm() const { return _params.takeoff_altitude_m * 100.0f; }

    // Returns the takeoff speed in meters per second
    float get_takeoff_speed_m_s() const {
        float result = _wp_nav ? _wp_nav->get_default_speed_up() / 100.0f : 0;
        if (result <= 0) {
            /* safety check */
            result = DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC;
        }
        return result;
    }

    // Returns the number of seconds left until show start, in microseconds
    int64_t get_time_until_start_usec() const;

    // Returns the number of seconds left until show start, in seconds
    float get_time_until_start_sec() const;

    // Returns the number of seconds left until the time when we should take off
    float get_time_until_takeoff_sec() const;

    // Returns the number of seconds left until the time when we should land
    float get_time_until_landing_sec() const;

    // Returns the velocity feed-forward gain factor to use during velocity control
    float get_velocity_feedforward_gain() const { return _params.velocity_feedforward_gain; }

    // Handles a MAVLink user command forwarded to the drone show manager by the central MAVLink handler
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    // Handles a MAVLink message forwarded to the drone show manager by the central MAVLink handler
    bool handle_message(const mavlink_message_t& msg) WARN_IF_UNUSED;

    // Asks the drone show manager to schedule a start as soon as possible if
    // the show is not running yet, assuming that the signal was sent from the
    // remote controller.
    void handle_rc_start_switch();

    // Asks the drone show manager to schedule a collective RTH operation if
    // the show is running, assuming that the signal was sent from the remote
    // controller.
    void handle_rc_collective_rtl_switch();

    // Returns whether the drone has been authorized to start automatically by the user
    bool has_authorization_to_start() const;

    // Returns whether the show altitude was set explicitly by the user
    bool has_explicit_show_altitude_set_by_user() const;

    // Returns whether the show origin was set explicitly by the user
    bool has_explicit_show_origin_set_by_user() const;

    // Returns whether the show orientation was set explicitly by the user
    bool has_explicit_show_orientation_set_by_user() const;

    // Returns whether a scheduled start time was determined or set by the user
    // for the show. When using GPS for time synchronization, this flag is true
    // if the user has set a start time in the SHOW_START_TIME parameter. When
    // using the internal clock and a countdown packet for time synchronization,
    // this flag is true if at least one countdown packet was received and no
    // countdown cancellation packet was received.
    bool has_scheduled_start_time() const {
        return (
            uses_gps_time_for_show_start()
            ? _start_time_unix_usec > 0
            : _start_time_on_internal_clock_usec > 0
        );
    }

    // Returns whether a valid takeoff time was determined for the show
    bool has_valid_takeoff_time() const {
        return _takeoff_time_sec >= 0 && _landing_time_sec > _takeoff_time_sec;
    }

    // Returns whether the drone is in the group with the given index
    bool is_in_group(uint8_t index) const {
        return _params.group_index == index;
    }

    // Returns whether the drone is prepared to take off. This function provides
    // valid results only if the drone is in the "waiting for start time" stage;
    // otherwise it returns false unconditionally.
    bool is_prepared_to_take_off() const;

    // Returns whether we are feeding desired acceleration information into the
    // lower-level position controller of ArduPilot
    bool is_acceleration_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_AccelerationControlEnabled;
    }

    // Returns whether we are feeding desired velocity information into the
    // lower-level position controller of ArduPilot
    bool is_velocity_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_VelocityControlEnabled;
    }

    // Returns whether a show file was identified and loaded at boot time
    bool loaded_show_data_successfully() const;

    // Returns whether yaw control was loaded from the show file at boot time
    bool loaded_yaw_control_data_successfully() const;

    // Returns whether the drone matches the given group mask
    bool matches_group_mask(uint8_t mask) const {
        return mask == 0 || mask & (1 << _params.group_index);
    }
    
    // Notifies the drone show manager that the drone show mode was initialized
    void notify_drone_show_mode_initialized();

    // Notifies the drone show manager that the drone show mode exited
    void notify_drone_show_mode_exited();

    // Notifies the drone show manager that the drone show mode has entered the given execution stage
    void notify_drone_show_mode_entered_stage(DroneShowModeStage stage);

    // Notifies the drone show manager that a guided mode command was sent to the drone
    void notify_guided_mode_command_sent(const GuidedModeCommand& command);

    // Notifies the drone show manager that the drone has landed after the show
    void notify_landed();

    // Notifies the drone show manager that the takeoff is about to take place.
    // The drone show manager may decide to cancel the takeoff by returning false.
    bool notify_takeoff_attempt() WARN_IF_UNUSED;

    // Handler for the MAVLink CMD_USER1 message that allows the user to reload _or_ clear the show
    bool reload_or_clear_show(bool do_clear) WARN_IF_UNUSED;

    // Asks the drone show manager to reload the show file from the storage. Returns true
    // if the show file was reloaded successfully, _or_ if there is no file on the storage
    // at all. Returns false if the motors are running, the show file is corrupted or
    // there was an IO error.
    bool reload_show_from_storage() WARN_IF_UNUSED;

    // Asks the drone show manager to schedule the start of a collective RTL
    // maneuver at the given timestamp. Returns whether the CRTL maneuver was
    // scheduled successfully.
    //
    // This function is a no-op if the drone show is not in the "performing"
    // phase.
    bool schedule_collective_rtl_at_show_timestamp_msec(uint32_t timestamp_ms);

    // Asks the drone show manager to schedule a start as soon as possible if
    // the show is not running yet. The delay parameter specifies the number of
    // milliseconds until the show start. It is the responsibility of the caller
    // to ensure that the drone has enough time to prepare after receiving the
    // request. Returns whether the start time was scheduled successfully.
    //
    // This function is a no-op if the drone show is not in the "waiting for
    // start time" phase.
    bool schedule_delayed_start_after(uint32_t delay_ms);

    // Sends a drone show status message (wrapped in a DATA16 packet) on the given MAVLink channel
    void send_drone_show_status(const mavlink_channel_t chan) const;

    // Returns whether the drone should switch to show mode automatically
    // after boot if there is no RC input
    bool should_switch_to_show_mode_at_boot() const;

    // Returns whether the drone should switch to show mode when authorized to start
    bool should_switch_to_show_mode_when_authorized() const;

    // Asks the drone show manager to cancel the show as soon as possible if
    // the show is running yet
    void stop_if_running();

    // Updates the state of the LED light on the drone and performs any additional
    // tasks that have to be performed regularly (such as checking for changes
    // in parameter values). This has to be called at 50 Hz, but most of its
    // subroutines run at 25 Hz, except _repeat_last_rgb_led_command(), which
    // may be called more frequently.
    void update();

    // Returns whether the manager uses GPS time to start the show
    bool uses_gps_time_for_show_start() const { return _params.time_sync_mode == TimeSyncMode_GPS; }

    // Writes the log message specific to the drone show manager subsystem into the logs
    void write_log_message() const;

    static const struct AP_Param::GroupInfo var_info[];

    // Hard fence subsystem. This should really have to be in Copter or some
    // other top-level class; we put it here because we are trying to restrict
    // ourselves to the SHOW_ parameter group only, and that one is managed by
    // AC_DroneShowManager.
    AC_HardFence hard_fence;

    // Takeoff acceleration; we assume that the drone attempts to take off with
    // this vertical acceleration if WPNAV_ACCEL_Z seems invalid
    static constexpr float DEFAULT_TAKEOFF_ACCELERATION_METERS_PER_SEC_SEC = 1.0f;

    // Takeoff speed; we assume that the drone attempts to take off with this
    // vertical speed if WPNAV_SPEED_UP seems invalid
    static constexpr float DEFAULT_TAKEOFF_SPEED_METERS_PER_SEC = 1.0f;

    // Motor spool up time; the drone starts its takeoff procedure with this
    // amount of time needed to spin up the motors up to minimum throttle,
    // before actually rising into the air. This parameter is used only if the
    // value of MOT_SPOOL_TIME seems invalid
    static constexpr float DEFAULT_MOTOR_SPOOL_UP_TIME_SEC = 0.5f;

private:
    // Structure holding all the parameters settable by the user
    struct {
        // Start time in GPS time of week (seconds), as set by the user in a parameter.
        // Used only when time_sync_mode == TimeSyncMode_GPS
        AP_Int32 start_time_gps_sec;

        // Latitude of drone show coordinate system, in 1e-7 degrees, as set in the parameters by the user
        AP_Int32 origin_lat;

        // Longitude of drone show coordinate system, in 1e-7 degrees, as set in the parameters by the user
        AP_Int32 origin_lng;

        // Altitude of drone show coordinate system above mean sea level, in millimeters, as set in the parameters by the user
        AP_Int32 origin_amsl_mm;

        // Orientation of drone show coordinate system, in degrees, as set in the parameters by the user
        AP_Float orientation_deg;

        // Authorization type of the show; see the DroneShowAuthorization enum
        AP_Int8 authorization;

        // Whether the drone should boot in show mode, and whether we should enter show mode automatically when authorized
        AP_Int8 show_mode_settings;

        // Brightness of status light signals when the drone is on the ground
        AP_Int8 preflight_light_signal_brightness;

        // Bitmask to set up various aspects of the control algorithm
        AP_Int16 control_mode_flags;

        // Specifies how many times we send a new guided mode command during the show, per second.
        AP_Int8 control_rate_hz;
        
        // Index of the group that this drone belongs to. Currently we support at most 8 groups, indexed from 0 to 7.
        AP_Int8 group_index;

        // Maximum allowed placement error before takeoff in the XY plane, in meters.
        AP_Float max_xy_placement_error_m;

        // Maximum allowed distance between expected position and setpoint in the XY plane during show, in meters.
        AP_Float max_xy_drift_during_show_m;

        // Maximum allowed altitude difference between expected position and setpoint during show, in meters.
        AP_Float max_z_drift_during_show_m;

        // Velocity feed-forward gain when velocity control is being used.
        AP_Float velocity_feedforward_gain;

        // Takeoff altitude
        AP_Float takeoff_altitude_m;

        // Time synchronization mode
        AP_Int8 time_sync_mode;

        struct {
            // Specifies where the a given LED light channel of the show should be sent
            AP_Int8 type;

            AP_Int8 channel;

            // Specifies the number of LEDs on a LED strip at the given channel; used only for NeoPixel or ProfiLED types
            AP_Int8 count;

            // The exponent of the gamma correcion on this LED light channel
            AP_Float gamma;

            // Color temperature of the white LED when the LED light channel uses an extra white LED
            AP_Float white_temperature;
        } led_specs[1];

        // Action to take at the end of the show
        AP_Int8 post_action;
    } _params;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Socket that we use to transmit the current status of the RGB led to an
    // external process for visualization purposes
    SocketAPM _sock_rgb;

    // Stores whether the RGB LED socket is open
    bool _sock_rgb_open;
#endif

    // Memory area holding the entire show file loaded from the storage
    uint8_t* _show_data;

    struct sb_trajectory_s* _trajectory;
    struct sb_trajectory_player_s* _trajectory_player;
    bool _trajectory_valid;

    struct sb_light_program_s* _light_program;
    struct sb_light_player_s* _light_player;
    bool _light_program_valid;

    struct sb_yaw_control_s* _yaw_control;
    struct sb_yaw_player_s* _yaw_player;
    bool _yaw_control_valid;

    // Result of the drone show specific preflight checks. Updated periodically
    // from _update_preflight_check_result(). See the values from the
    // DroneShowPreflightCheckFlag enum for more details.
    uint8_t _preflight_check_failures;

    // Properties of the drone show coordinate system, used in-flight. Updated
    // from the parameters set by the user when the drone takes off.
    ShowCoordinateSystem _show_coordinate_system;

    // Properties of the tentative drone show coordinate system, set up by the
    // user and periodically updated from the parameters. Note that this is
    // _not_ the same as _show_coordinate_system, which is used in-flight.
    ShowCoordinateSystem _tentative_show_coordinate_system;

    // Reason why the start time was set to the current value; used to decide whether
    // it should be cleared when the drone show mode is restarted
    StartTimeSource _start_time_requested_by;

    // Start time of the show, in microseconds, according to the internal clock
    // of the drone, zero if unset. This variable is used _only_ if the time
    // synchronisation mode is set to use a countdown-based method.
    uint64_t _start_time_on_internal_clock_usec;

    // Start time of the show, in microseconds, as a UNIX timestamp, zero if unset.
    // This variable is used _only_ if the start time is synchronized to GPS time or
    // some other absolute (external) time source that is guaranteed to be
    // synchronized across drones.
    uint64_t _start_time_unix_usec;

    // Takeoff position, in local coordinates, relative to the show coordinate system.
    // Zero if no show data is loaded. Units are in millimeters.
    Vector3f _takeoff_position_mm;

    // Time when we need to take off, relative to the start of the show, in seconds
    float _takeoff_time_sec;

    // Time when we need to land relative to the start of the show, in seconds
    float _landing_time_sec;

    // Time when we need to start a coordinated RTL trajectory, relative to the
    // start of the show, in seconds. Zero if unscheduled.
    float _crtl_start_time_sec;

    // Structure storing the details of a light signal requested by the user,
    // including its start time, duration, color, priority etc.
    struct {
        uint32_t started_at_msec;       //< Start time of the light signal
        uint16_t duration_msec;         //< Duration of the light signal
        uint8_t color[3];               //< Color of the light signal
        LightEffectType effect;         //< Type of the light signal (flash, pulsating etc)
        LightEffectPriority priority;   //< Priority of the signal
        uint16_t period_msec;           //< Period of the light signal when applicable
        uint16_t phase_msec;            //< Phase of the light signal when applicable
        bool enhance_brightness;        //< Whether to enhance the brightness using the W LED if possible
        bool sync_to_gps;               //< Whether to sync the light signal to GPS time
    } _light_signal;

    // Current execution stage of the drone show mode. This is pushed here from
    // the drone show mode when the stage changes in the mode.
    DroneShowModeStage _stage_in_drone_show_mode;

    // Total duration of the show, in seconds
    float _total_duration_sec;

    // Flag to indicate whether the trajectory ends at the same location where
    // it started. This is used to determine whether the drone should land or
    // return to the takeoff position at the end of the show.
    bool _trajectory_is_circular;

    // Flag that is set to true if the user has instructed the drone show manager
    // to cancel the show as soon as possible. This is checked regularly by
    // mode_drone_show.cpp
    bool _cancel_requested;

    // The preferred duration between consecutive guided mode commands
    // during the execution of the show. Updated soon after the corresponding
    // parameter changes.
    uint32_t _controller_update_delta_msec;

    // Factory object that can create RGBLed instances that the drone show manager will control
    DroneShowLEDFactory* _rgb_led_factory;

    // RGB led that the drone show manager controls
    DroneShowLED* _rgb_led;

    // Last RGB color that was sent to the RGB led
    sb_rgb_color_t _last_rgb_led_color;

    // Last guided mode command that was sent
    GuidedModeCommand _last_setpoint;

    // Timestamp that defines whether the RC start switch is blocked (and if so, until when)
    uint32_t _rc_switches_blocked_until;

    // Copy of the STAT_BOOTCNT parameter value at boot; we will send the lower
    // two bits of this value regularly in status packets to allow the GCS to
    // detect when the drone was rebooted
    uint16_t _boot_count;

    // Reference to the waypoint navigation module so we can query the takeoff parameters
    const AC_WPNav* _wp_nav;

    // Returns whether the RC switches are currently blocked
    bool _are_rc_switches_blocked();
    
    // Checks whether there were any changes in the parameters relevant to the
    // execution of the drone show. This has to be called regularly from update()
    void _check_changes_in_parameters();

    // Checks whether an event tracked by DroneShowNoficationBackend was triggered
    // recently and handle it if needed
    void _check_events();

    // Checks whether the radio is in failsafe state and blocks the RC start
    // switch until the radio returns from failsafe and at least one second
    // has passed
    void _check_radio_failsafe();

    // Clears the start time of the drone show after a successful landing
    void _clear_start_time_after_landing();

    // Clears the start time of the drone show if it was set by the user with the RC switch
    void _clear_start_time_if_set_by_switch();

    // Copies the settings of the show coordinate system from the parameter section
    // to the given variable. Returns false if the show coordinate system was not
    // specified by the user yet.
    bool _copy_show_coordinate_system_from_parameters_to(
        ShowCoordinateSystem& coordinate_system
    ) const;

    // Produces an internally triggered light signal that indicates a failed
    // operation (like a successful compass calibration)
    void _flash_leds_after_failure();
    
    // Produces an internally triggered light signal that indicates a successful
    // operation (like a successful compass calibration)
    void _flash_leds_after_success();

    // Produces a light signal that tries to attract attention to the drone;
    // typically triggered by the operator from the GCS to find a particular
    // drone in a swarm.
    void _flash_leds_to_attract_attention(LightEffectPriority priority);

    // Flashes the LEDs of the drone with the given color
    void _flash_leds_with_color(
        uint8_t red, uint8_t green, uint8_t blue, uint8_t count,
        LightEffectPriority priority, bool enhance_brightness = false
    );

    // Returns a timestamp meant to be used solely for the purposes of implementing
    // light signals. The timestamp is synced to GPS seconds when the drone has
    // a good GPS fix.
    uint32_t _get_gps_synced_timestamp_in_millis_for_lights() const;

    // Handles a generic MAVLink DATA* message from the ground station.
    bool _handle_custom_data_message(uint8_t type, void* data, uint8_t length);

    // Handles a MAVLink DATA16 message from the ground station.
    bool _handle_data16_message(const mavlink_message_t& msg);

    // Handles a MAVLink DATA32 message from the ground station.
    bool _handle_data32_message(const mavlink_message_t& msg);

    // Handles a MAVLink DATA64 message from the ground station.
    bool _handle_data64_message(const mavlink_message_t& msg);

    // Handles a MAVLink DATA96 message from the ground station.
    bool _handle_data96_message(const mavlink_message_t& msg);

    // Handles a MAVLink LED_CONTROL message from the ground station.
    bool _handle_led_control_message(const mavlink_message_t& msg);

    // Returns whether the drone is close enough to its expected position during a show.
    // Returns true unconditionally if the drone is not performing a show.
    bool _is_at_expected_position() const;
    
    // Returns whether the drone is close enough to its start position in the
    // horizontal plane. Note that the altitude is not checked here.
    //
    // The xy_threshold_m parameter specifies the maximum allowed distance
    // between the current position and the takeoff position in the XY plane.
    // Zero or negative values mean that the default threshold is used from the
    // parameters.
    bool _is_at_takeoff_position_xy(float xy_threshold = 0.0f) const;
    
    // Returns whether the drone is close enough to the given location. Distances
    // are checked separately in the XY plane and in the Z direction. Negative
    // or zero threshold means that the corresponding check is turned off.
    bool _is_close_to_position(const Location& target_loc, float xy_threshold, float z_threshold) const;

    // Returns whether the GPS fix of the drone is good enough so we can trust
    // that it has accurate tiem information.
    bool _is_gps_time_ok() const;

    // Recalculates the values of some internal variables that are derived from
    // the current trajectory when it is loaded.
    bool _recalculate_trajectory_properties() WARN_IF_UNUSED;

    // Requests the vehicle to switch to drone show mode.
    virtual void _request_switch_to_show_mode() {};

    bool _load_show_file_from_storage();
    void _set_light_program_and_take_ownership(struct sb_light_program_s *value);
    void _set_trajectory_and_take_ownership(struct sb_trajectory_s *value);
    void _set_show_data_and_take_ownership(uint8_t *value);
    void _set_yaw_control_and_take_ownership(struct sb_yaw_control_s *value);

    // Updates the state of the LED light on the drone. This has to be called
    // regularly at 25 Hz
    void _update_lights();

    // Checks for error conditions that we can detect on the drone such as not
    // being in the designated start position before takeoff. This has to be
    // called regularly, but it is rate-limited to 1 Hz
    void _update_preflight_check_result(bool force = 0);

    // Updates the RGB LED instance that is used as an output. Note that this
    // function updates the identity of the RGB LED object based on the current
    // servo channel settings, _not_ the state of the LED itself
    void _update_rgb_led_instance();

    // Repeats the last LED command. Used when the channel that transmits the
    // LED color change commands to the LED module is unreliable. Called
    // regularly from update(), typically faster than the regular LED update
    // routine.
    void _repeat_last_rgb_led_command();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    bool _open_rgb_led_socket();
#endif
};
