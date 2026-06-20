#pragma once

/// @file   AC_DroneShowManager.h
/// @brief  Drone show state management library

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/RGBLed.h>
#include <AP_Param/AP_Param.h>

#include <AC_BubbleFence/AC_BubbleFence.h>
#include <AC_HardFence/AC_HardFence.h>
#include <AC_Fence/AC_Fence.h>
#include <AC_WPNav/AC_WPNav.h>

#include <GCS_MAVLink/ap_message.h>

#include <cstdint>
#include <skybrush/skybrush.h>

#include "DroneShow_CustomPackets.h"
#include "DroneShow_Enums.h"
#include "DroneShow_FenceConfig.h"

class DroneShowLEDFactory;
class DroneShowLED;
class DroneShowPyroDeviceFactory;
class DroneShowPyroDevice;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#  include <AP_HAL/utility/Socket.h>
#endif

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

        // Converts a coordinate given in the global GPS coordinate system to
        // the show coordinate system, in millimeters
        void convert_global_to_show_coordinate(const Location& loc, sb_vector3_t& vec) const;

        // Converts a coordinate given in the show coordinate system, in millimeters, to
        // the global GPS coordinate system
        void convert_show_to_global_coordinate(sb_vector3_t vec, Location& loc) const;

        // Converts a yaw angle given in the show coordinate system, in degrees, to
        // centidegrees relative to north
        float convert_show_to_global_yaw_and_scale_to_cd(float value) const;

        // Returns whether the coordinate system is valid.
        bool is_valid() const { return origin_lat != 0 && origin_lng != 0; };
    };

    class PyroTestState {
    public:
        PyroTestState() {
            clear();
        }

        void clear();
        void start(uint8_t channel, uint8_t num_channels, uint32_t delta_msec);
        bool update(uint8_t& channel);

    private:
        // Timestamp when the last channel was fired, in milliseconds. 0 = not started
        uint32_t last_fired_at_msec;

        // Next channel index to teset
        uint8_t next_channel;

        // Number of channels to test
        uint8_t channels_remaining;

        // Time between consecutive channel tests, in milliseconds
        uint32_t delta_msec;
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
        float yaw_cd;
        float yaw_rate_cds;
        bool unlock_altitude;
        bool reached_end;

        void clear() {
            pos.zero();
            vel.zero();
            acc.zero();
            yaw_cd = 0.0f;
            yaw_rate_cds = 0.0f;
            unlock_altitude = false;
            reached_end = false;
        }
    };

    /// Specification of a telemetry request containing a message ID and a message
    /// interval
    struct TelemetryRequest {
        ap_message ap_msg_id;
        uint16_t interval_msec;
    };

    // Early initialization steps that have to be called early in the boot process
    // to ensure we have enough memory to do them even on low-memory boards like
    // a Pixhawk1
    void early_init();

    // Initializes the drone show subsystem at boot time
    void init(const AC_WPNav* wp_nav);

    // Clears the scheduled start time of the show (but does not cancel the
    // show if it is already running). Returns whether the request was
    // processed.
    //
    // This function is a no-op if the drone show is not in the "waiting for
    // start time" phase and 'force' is set to false.
    bool clear_scheduled_start_time(bool force = false);

    // Configures all relevant geofences of the show with a single call.
    bool configure_fences(DroneShow_FenceConfig& config) WARN_IF_UNUSED;

    // Configures the show origin, orientation and AMSL reference in a single
    // call. This function sets the corresponding SHOW_ parameters as if they
    // were set with multiple PARAM_SET MAVLink commands.
    //
    // The show will be controlled in AGL if the amsl argument is less than
    // SMALLEST_VALID_AMSL, i.e. less than or equal to
    bool configure_show_coordinate_system(
        int32_t lat, int32_t lon, int32_t amsl_mm, float orientation_deg
    ) WARN_IF_UNUSED;

    // Returns the current authorization scope of the show
    DroneShowAuthorization get_authorization_scope() const;

    // Returns the color of the LED light on the drone according to its light
    // program.
    sb_rgb_color_t get_desired_color_of_rgb_light();

    // Returns the color of the LED light on the drone according to its light
    // program the given number of seconds after the start time.
    sb_rgb_color_t get_desired_color_of_rgb_light_at_seconds(float time);

    // Returns the preferred duration between consecutive guided mode commands
    // during the execution of the show.
    uint32_t get_controller_update_delta_msec() const { return _controller_update_delta_msec; }

    // Retrieves the guided mode command that should be sent during the performance
    // when the function is invoked. Returns true if the command was updated successfully,
    // false when failed to update the command.
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
    bool get_desired_global_position_at_seconds(float time, Location& loc) WARN_IF_UNUSED;

    // Returns the desired velocity of the drone during the drone show the
    // given number of seconds after the start time, in the global NEU
    // cooordinate system, using centimeters per seconds as units.
    bool get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel) WARN_IF_UNUSED;

    // Returns the desired yaw and yaw rate of the drone during the drone show the
    // given number of seconds after the start time. Yaw is returned in centidegrees
    // relative to North; yaw rate is returned in centidegrees/seconds.
    bool get_desired_yaw_cd_and_yaw_rate_cd_s_at_seconds(float time, float& yaw_cd, float& yaw_rate_cd_s) WARN_IF_UNUSED;

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

    // Returns the action to be performed by the bubble fence module
    AC_BubbleFence::FenceAction get_bubble_fence_action();

    // Retrieves the position where the drone is supposed to be at the start of the show.
    // Returns true if successful or false if the show coordinate system was not set up
    // by the user yet.
    bool get_global_takeoff_position(Location& loc) const;
    
    // Returns the last color that was emitted to the RGB light
    void get_last_rgb_led_color(sb_rgb_color_t& color) const { color = _last_rgb_led_color; }

    // Returns the preferred set of telemetry messages at boot time.
    // The result will be a pointer to a statically allocated array of pairs of
    // message IDs and intervals, ending in an entry with the message ID set to
    // zero, which marks the end of the list.
    const TelemetryRequest* get_preferred_telemetry_messages() const;

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

    // Returns the number of seconds elapsed since show start, in microseconds (wall clock time)
    int64_t get_elapsed_time_since_start_usec() const;

    // Returns the number of seconds elapsed since show start, in milliseconds (wall clock time)
    int32_t get_elapsed_time_since_start_msec() const;

    // Returns the number of seconds elapsed since show start, in seconds (wall clock time)
    float get_elapsed_time_since_start_sec() const;

    // Returns the current stage that the drone show mode is in
    DroneShowModeStage get_stage_in_drone_show_mode() const { return _stage_in_drone_show_mode; }

    // Returns the landing speed in meters per second
    float get_landing_speed_m_sec() const;

    // Returns the takeoff acceleration in meters per second squared
    float get_motor_spool_up_time_sec() const;
    
    // Returns the color to use in return-to-home transitions (both individual and collective)
    sb_rgb_color_t get_rth_transition_color() const;

    // Returns the current scene index and the time elapsed within the scene, according
    // to the show controller. Both the scene index and the show clock time within the scene
    // will be zero if the show is not in the "performing" stage.
    void get_scene_index_and_show_clock_within_scene(ssize_t* scene, float* show_clock_sec) const;

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
    float get_takeoff_speed_m_sec() const;

    // Returns the number of seconds left until show start, in microseconds
    int64_t get_time_until_start_usec() const;

    // Returns the number of seconds left until show start, in seconds
    float get_time_until_start_sec() const;

    // Returns the number of seconds left until the time when we should take off,
    // assuming that there will be no changes to the time axis configuration of the show
    float get_time_until_takeoff_sec();

    // Returns the velocity feed-forward gain factor to use during velocity control
    float get_velocity_feedforward_gain() const { return _params.velocity_feedforward_gain; }

    // Handles a MAVLink user command forwarded to the drone show manager by the central MAVLink handler
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    // Handles a MAVLink message forwarded to the drone show manager by the central MAVLink handler
    bool handle_message(mavlink_channel_t chan, const mavlink_message_t& msg) WARN_IF_UNUSED;

    // Asks the drone show manager to schedule a start as soon as possible if
    // the show is not running yet, assuming that the signal was sent from the
    // remote controller.
    void handle_rc_start_switch();

    // Asks the drone show manager to schedule a collective RTH operation if
    // the show is running, assuming that the signal was sent from the remote
    // controller.
    void handle_rc_collective_rtl_switch();

    // Returns whether the drone has been authorized to start automatically by the user
    bool has_authorization() const;

    // Returns whether the drone is allowed to start its motors in the current
    // authorization state
    bool has_authorization_to_start_motors() const;
    
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
    
    // Returns whether a collective RTH operation was triggered
    bool is_collective_rth_triggered() const {
        // Currently we take a shortcut here and simply return true if we have more than
        // one scene in the screenplay. This is correct for the current implementation
        // of the show controller, but if we add more complexity to the show
        // controller in the future, we might want to track this with a separate flag.
        return sb_screenplay_size(&_screenplay) > 1;
    }

    // Returns whether any motor output from show mode is disabled due to testing
    // purposes.
    bool is_motor_output_disabled() const {
        return _has_option(DroneShowOption_PreventMotorOutput);
    }
    
    // Returns whether the performance of the show has finished, based on the current
    // state and output of the show controller. The return value of this function can
    // be used to trigger the post-show action.
    // 
    // The performance is considered completed if we are in the "performing" stage and
    // the show controller has reported that the current time is beyond the time axis
    // limits, _or_ if we are in the "landing", "landed", "loiter", "RTL" or "error" stages.
    // In all these states, the _normal_ performance of the show has ended, even though
    // the drone may still be flying.
    bool is_performance_completed() const {
        switch (_stage_in_drone_show_mode) {
            case DroneShow_Performing:
                return sb_show_controller_is_output_valid(&_show_controller) &&
                       sb_show_controller_has_reached_end(&_show_controller);

            case DroneShow_Landing:
            case DroneShow_Landed:
            case DroneShow_Loiter:
            case DroneShow_RTL:
            case DroneShow_Error:
                return true;
                
            default:
                return false;
        }
    }

    // Returns whether the trajectory of the show seems plausible:
    // - has a takeoff time (i.e. a moment when the drone ascends above the takeoff altitude)
    // - has a landing time (i.e. a moment when the drone sinks below the takeoff altitude)
    // - landing time comes later than the takeoff time
    bool is_trajectory_plausible() const {
        return (
            _trajectory_stats.takeoff_time_sec >= 0 && 
            _trajectory_stats.landing_time_sec > _trajectory_stats.takeoff_time_sec
        );
    }

    // Returns whether the drone is in the group with the given index
    bool is_in_group(uint8_t index) const {
        return _params.group_index == index;
    }

    // Returns whether the drone is prepared to take off. This function provides
    // valid results only if the drone is in the "waiting for start time" stage;
    // otherwise it returns false unconditionally.
    bool is_prepared_to_take_off() const;

    // Returns whether we are feeding desired velocity information into the
    // lower-level position controller of ArduPilot
    bool is_velocity_control_enabled() const {
        return _params.control_mode_flags & DroneShowControl_VelocityControlEnabled;
    }

    // Returns whether a show file was identified and loaded at boot time
    bool loaded_show_data_successfully() const;

    // Returns whether the drone matches the given group mask
    bool matches_group_mask(uint8_t mask) const {
        return mask == 0 || mask & (1 << _params.group_index);
    }
    
    // Notifies the drone show manager that the drone show mode was initialized
    // 
    // Returns true if the initialization was successful and false if there was
    // an error that prevents the drone show mode from entering drone show mode.
    bool notify_drone_show_mode_initialized();

    // Notifies the drone show manager that the drone show mode exited
    void notify_drone_show_mode_exited();

    // Notifies the drone show manager that the drone show mode has entered the given execution stage
    void notify_drone_show_mode_entered_stage(DroneShowModeStage stage);

    // Notifies the drone show manager that a guided mode command was sent to the drone
    void notify_guided_mode_command_sent(const GuidedModeCommand& command);

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

    // Sets the scheduled start time of the show according to GPS time, in seconds and
    // milliseconds. This function does _not_ set the synchronization mode to GPS time
    // if it is set to something else; it is only responsible for updating the two
    // parameters storing the start time in tandem.
    //
    // Negative values in gps_sec will clear the scheduled GPS-based start time.
    // gps_sec values larger than the number of seconds in a GPS week are considered
    // invalid and will be rejected. Millisecond offsets are accepted and updated if
    // and only if the gps_sec value is valid.
    //
    // Note that the function will update the parameters even if the show is not in the
    // "waiting for start time" phase. However, the changes will not update the _actual_
    // internal variables that we use to start the show.
    //
    // Returns whether the parameters were updated successfully.
    bool set_scheduled_start_time_in_gps_time_of_week(int32_t gps_sec, int16_t gps_msec_offset);

    // Sends a drone show status message (wrapped in a DATA16 packet) on the given MAVLink channel.
    // This is used by the legacy telemetry configuration where information from this packet must
    // be augmented from GLOBAL_POSITION_INT and GPS_RAW_INT.
    void send_drone_show_status(const mavlink_channel_t chan) const;

    // Sends an extended drone show status message (wrapped in a DATA64 packet) on the given MAVLink channel
    // This is used by the compact telemetry configuration where this packet incorporates some parts of
    // GLOBAL_POSITION_INT and GPS_RAW_INT so they do not need to be sent.
    void send_extended_drone_show_status(const mavlink_channel_t chan) const;

    // Returns whether the drone should switch to show mode automatically
    // after boot if there is no RC input
    bool should_switch_to_show_mode_at_boot() const;

    // Returns whether the drone should switch to show mode when authorized to start
    bool should_switch_to_show_mode_when_authorized() const;

    // Updates the state of the LED light on the drone and performs any additional
    // tasks that have to be performed regularly (such as checking for changes
    // in parameter values). This has to be called at 50 Hz, but most of its
    // subroutines run at 25 Hz, except _repeat_last_rgb_led_command(), which
    // may be called more frequently.
    void update();

    // Returns whether the manager uses GPS time to start the show
    bool uses_gps_time_for_show_start() const { return _params.time_sync_mode == TimeSyncMode_GPS; }
    
    // Writes a message containing the trigger of a collective RTL maneuver into the log
    void write_crth_trigger_log_message(float rth_start_time_sec, sb_vector3_t start) const;
    
    // Writes a message containing a summary of the gefence status into the log
    void write_fence_status_log_message() const;

    // Writes a sequence of log messages containing a representation of the current time
    // axis
    void write_screenplay_log_messages();

    // Writes a message holding the status of the drone show subsystem into the log
    void write_show_status_log_message() const;

    // Writes a message logging the execution of an event from the show file into the log
    void write_show_event_log_message(const struct sb_event_s *event, DroneShowEventResult result) const;

    static const struct AP_Param::GroupInfo var_info[];

    // Hard fence subsystem. This should really have to be in Copter or some
    // other top-level class; we put it here because we are trying to restrict
    // ourselves to the SHOW_ parameter group only, and that one is managed by
    // AC_DroneShowManager.
    AC_HardFence hard_fence;

    // Bubble fence subsystem. See also the comment for the hard fence.
    AC_BubbleFence bubble_fence;

    // Landing speed; we assume that the drone should attempt to land with this
    // vertical speed if LAND_SPEED seems invalid
    static constexpr float DEFAULT_LANDING_SPEED_METERS_PER_SEC = 1.0f;

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
        // Used only when time_sync_mode == TimeSyncMode_GPS. Must be considered
        // together with start_time_gps_msec_offset to allow for millisecond-level
        // precision.
        AP_Int32 start_time_gps_sec;

        // Extra millisecond offset to add to the start time in GPS time, as set by the
        // user in a parameter. This allows for millisecond-level precision.
        AP_Int16 start_time_gps_msec_offset;

        // Latitude of drone show coordinate system, in 1e-7 degrees, as set in the parameters by the user
        AP_Int32 origin_lat;

        // Longitude of drone show coordinate system, in 1e-7 degrees, as set in the parameters by the user
        AP_Int32 origin_lng;

        // Altitude of drone show coordinate system above mean sea level, in millimeters, as set in the parameters by the user
        AP_Int32 origin_amsl_mm;

        // Orientation of drone show coordinate system, in degrees, as set in the parameters by the user
        AP_Float orientation_deg;

        // Authorization scope of the show; see the DroneShowAuthorization enum
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

            // Minimum brightness threshold (as a ratio 0-1) below which LED is turned off
            AP_Float min_brightness;
        } led_specs[1];

        struct {
            // Specifies the type of the pyrotechnic device
            AP_Int8 type;

            // Time compensation for the pyro device, in milliseconds
            AP_Int32 time_compensation_msec;

            // Ignition time for the pyro device, in milliseconds
            AP_Int32 ignition_duration_msec;
        } pyro_spec;

        // Minimum altitude above which pyrotechnic effects can be triggered
        AP_Float pyro_min_altitude_m;

        // Action to take at the end of the show
        AP_Int8 post_action;

        // General options related to the show flight mode. See the values from
        // the DroneShowOptionFlag enum for more details.
        AP_Int8 show_options;

        // Telemetry profile to use when determining the initial set of messages
        // that a drone will send
        AP_Int8 telemetry_profile;

        // Maximum allowed ESC error rate percentage, -1 when disabled
        AP_Int8 max_esc_error_rate_pcnt;
    } _params;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Socket that we use to transmit the current status of the RGB led to an
    // external process for visualization purposes
    SocketAPM _sock_rgb;

    // Stores whether the RGB LED socket is open
    bool _sock_rgb_open;
#endif

    // Whether the drone show manager was initialized successfully
    bool _init_ok;

    // Memory area holding the entire show file loaded from the storage
    uint8_t* _show_data;
    
    // Screenplay of the show.
    // 
    // In normal conditions, the screenplay has a single chapter, which holds
    // references to the trajectory, light program, yaw program and event list.
    // However, the screenplay may be extended with an additional return-to-launch
    // chapter if a collective RTL operation is scheduled during the show.
    sb_screenplay_t _screenplay;
    
    // Reference scene containing the trajectory, light program and yaw program of
    // the show. This is needed to ensure that we always have references to the
    // original data, even if the screenplay is modified in a way that it does not
    // refer to the loaded show any more.
    // 
    // Also ensure that we can get a handle to the trajectory of the show any time
    // to modify its ending at takeoff even if the screenplay does not contain it.
    sb_screenplay_scene_t _main_show_scene;

    // Controller that manages the trajectory player, the light program player,
    // the yaw player, the event list and the time axis.
    sb_show_controller_t _show_controller;
    
    // Structure holding information about the current trajectory, including the
    // takeoff time, the landing time, the total duration and the takeoff location.
    // Takeoff and landing times are according to the _show clock_.
    sb_trajectory_stats_t _trajectory_stats;
    
    // Projected duration of the time interval from the start of the show to the
    // takeoff, in wall clock time, according to the current screenplay of the show,
    // under the assumption that the screenplay will not change. This is used to
    // trigger the motor start command and the takeoff command at the right time.
    // 
    // The variable must be updated whenever the screenplay is changed or a new
    // trajectory is loaded.
    // 
    // NaN means that the takeoff time was not calculated yet and needs to be
    // recalculated.
    float _projected_wall_clock_time_at_takeoff_sec;

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

    // Flag to indicate whether the trajectory ends at the same location where
    // it started. This is used to determine whether the drone should land or
    // return to the takeoff position at the end of the show.
    bool _trajectory_is_circular;
    
    // Flag to indicate whether we have already modified the trajectory to
    // ensure precision landing back to the exact takeoff position (even if the
    // drone is slightly misplaced). This is used to avoid modifying the trajectory
    // multiple times when the show is restarted.
    bool _trajectory_modified_for_landing;

    // The preferred duration between consecutive guided mode commands
    // during the execution of the show. Updated soon after the corresponding
    // parameter changes.
    uint32_t _controller_update_delta_msec;

    // Factory object that can create pyro device instances that the drone show manager will control
    DroneShowPyroDeviceFactory* _pyro_device_factory;

    // Pyro device that the drone show manager controls
    DroneShowPyroDevice* _pyro_device;

    // State of the pyro test
    PyroTestState _pyro_test_state;

    // Factory object that can create RGBLed instances that the drone show manager will control
    DroneShowLEDFactory* _rgb_led_factory;

    // RGB led that the drone show manager controls
    DroneShowLED* _rgb_led;

    // Last RGB color that was sent to the RGB led
    sb_rgb_color_t _last_rgb_led_color;

    // Last guided mode command that was sent
    GuidedModeCommand _last_setpoint;
    
    // Sequence number of the last time axis configuration packet that was processed
    uint16_t _last_time_axis_config_seq_no;

    // Timestamp that defines whether the RC start switch is blocked (and if so, until when)
    uint32_t _rc_switches_blocked_until;

    // Copy of the STAT_BOOTCNT parameter value at boot; we will send the lower
    // two bits of this value regularly in status packets to allow the GCS to
    // detect when the drone was rebooted
    uint16_t _boot_count;

    // Number of time axis configuration packets received from the GCS since boot, used
    // for debugging purposes. Rolls over at 65535.
    uint16_t _time_axis_configuration_packet_count;

    // Number of time axis configuration packets received from the GCS since boot that
    // seem to have been received out of order. Rolls over at 256.
    uint8_t _time_axis_configuration_out_of_order_count;

    // Last error condition during the reception of a time axis configuration packet,
    // used for debugging purposes. Zero means no error.
    uint8_t _time_axis_configuration_last_error;

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

    // Clears the start time of the drone show if it was set by the user with the RC switch
    void _clear_start_time_if_set_by_switch();

    // Copies the settings of the show coordinate system from the parameter section
    // to the given variable. Returns false if the show coordinate system was not
    // specified by the user yet.
    bool _copy_show_coordinate_system_from_parameters_to(
        ShowCoordinateSystem& coordinate_system
    ) const;

    // Creates the directory in which the drone show specific files are stored
    // on the filesystem
    bool _create_show_directory();
    
    // Ensures that the time axis of the given scene is set up in a way that the
    // relevant part of the trajectory will be played in full.
    // 
    // The relevant part is defined as follows. If the trajectory is the main show
    // trajectory, its relevant part is all the way up to the point where it crosses
    // the takeoff altitude for the last time from above. Otherwise, it is the full
    // duration of the trajectory. This is to ensure that we trigger the landing at
    // the end of the normal show trajectory at the takeoff altitude so ArduPilot can
    // take over with its own landing algorithm (which has landing detection enabled).
    //
    // If the current time axis configuration has a total duration that is shorter than
    // the duration of the relevant part of the trajectory, the time axis will be
    // extended with a new segment with the given initial and final rates and the
    // necessary duration.
    bool _ensure_scene_covers_relevant_part_of_trajectory(
        sb_screenplay_scene_t* scene, float initial_rate, float final_rate
    ) WARN_IF_UNUSED;
    
    // Fills the given buffer with basic drone show telemetry data and returns
    // a pointer to after the last byte written into the buffer.
    //
    // This is a helper function to share the common bits between the standard
    // and the extended drone show status packet. The two packets need to have
    // an identical format for the initial segment because the GCS essentially
    // treats them the same way (with the last bits being optional).
    //
    // The buffer must be large enough to hold the basic telemetry. A buffer
    // with 16 bytes is enough.
    uint8_t* _fill_drone_show_status_packet_buffer(uint8_t* buf) const;

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
    
    // Returns the raw control output from the show controller at the given number of
    // seconds after show start. Units and vectors returned in the control output are
    // not transformed to the show coordinate system.
    // 
    // May return null if the show controller fails to switch to the given timestamp.
    // This is unlikely and probably indicates bigger problems, but we need to handle
    // it anyway.
    const sb_control_output_t* _get_raw_show_control_output_at_seconds(float time);
    
    // Handles a generic MAVLink DATA* message from the ground station.
    bool _handle_custom_data_message(mavlink_channel_t chan, uint8_t type, void* data, uint8_t length);

    // Handles a MAVLink DATA16 message from the ground station.
    bool _handle_data16_message(mavlink_channel_t chan, const mavlink_message_t& msg);

    // Handles a MAVLink DATA32 message from the ground station.
    bool _handle_data32_message(mavlink_channel_t chan, const mavlink_message_t& msg);

    // Handles a MAVLink DATA64 message from the ground station.
    bool _handle_data64_message(mavlink_channel_t chan, const mavlink_message_t& msg);

    // Handles a MAVLink DATA96 message from the ground station.
    bool _handle_data96_message(mavlink_channel_t chan, const mavlink_message_t& msg);

    // Handles an acknowledgment packet
    bool _handle_acknowledgment_packet(void* data, uint8_t length);

    // Handles a simple geofence setup packet
    bool _handle_geofence_setup_packet(void* data, uint8_t length, uint8_t* reply);

    // Handles a MAVLink LED_CONTROL message from the ground station.
    bool _handle_led_control_message(const mavlink_message_t& msg);

    // Handles a start time configuration packet
    bool _handle_start_time_configuration_packet(void* data, uint8_t length);

    // Handles a time axis configuration packet
    bool _handle_time_axis_configuration_packet(void* data, uint8_t length);

    // Returns whether the given option flag is set in the SHOW_OPTIONS parameter
    bool _has_option(DroneShowOptionFlag option) const {
        return (_params.show_options & option) != 0;
    }
    
    // Invalidates the projected time until takeoff, so it will be recalculated later
    // if needed.
    void _invalidate_projected_wall_clock_time_at_takeoff() {
        _projected_wall_clock_time_at_takeoff_sec = NAN;
    }

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

    // Returns whether pyro events can be handled safely in the current state
    bool _is_pyro_safe_to_fire() const;

    // Returns whether a new time axis configuration packet with the given header can
    // be accepted now, given the current state of the drone show manager. This is used
    // to avoid accepting a new time axis configuration packet with a different start
    // time when the show is already running, which would most likely cause a jump in
    // the expected position of the drone.
    bool _is_safe_to_accept_time_axis_configuration_packet(
        const CustomPackets::time_axis_config_header_t& header) const;

    // Returns whether it is safe to change the start time of the show in the current
    // stage of the drone show mode. This is used to determine whether we can accept a
    // new value of the start time parameter or a new time axis configuration packet
    // whose start time is different from the current one
    bool _is_safe_to_change_start_time_in_current_stage() const;

    // Recalculates the values of some internal variables that are derived from
    // the current trajectory when it is loaded.
    bool _recalculate_trajectory_properties() WARN_IF_UNUSED;

    // Requests the vehicle to switch to drone show mode.
    virtual void _request_switch_to_show_mode() {};
    
    // Generic debug request handler
    bool _run_debug_request_handler(const mavlink_command_int_t &packet) WARN_IF_UNUSED;

    bool _load_show_file_from_storage();
    void _set_show_data_and_take_ownership(uint8_t *value);

    // Triggers pending events from the event list of the show
    void _trigger_show_events();

    // Updates the state of the LED light on the drone. This has to be called
    // regularly at 25 Hz
    void _update_lights();

    // Checks for error conditions that we can detect on the drone such as not
    // being in the designated start position before takeoff. This has to be
    // called regularly, but it is rate-limited to 1 Hz
    void _update_preflight_check_result(bool force = 0);

    // Updates the state of the pyro device on the drone. This has to be called
    // regularly at 25 Hz. Takes care of turning off pyro channels after the
    // ignition time has passed, and also takes care of pending channel test
    // requests.
    void _update_pyro_device();

    // Updates the pyro device _instance_ that is used for pyro effects. Note that
    // this function updates the identity of the pyro device object based on the
    // pyro settings, but does _not_ trigger any pyro events. For triggering
    // pyro events, see `_trigger_show_events()`. For turning off pyro channels based
    // on the ignition time, see `_update_pyro_device()`.
    void _update_pyro_device_instance();

    // Updates the RGB LED instance that is used as an output. Note that this
    // function updates the identity of the RGB LED object based on the current
    // RGB LED settings, _not_ the state of the LED itself
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
