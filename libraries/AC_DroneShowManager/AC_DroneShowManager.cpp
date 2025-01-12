#include <GCS_MAVLink/GCS.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/DroneShowNotificationBackend.h>
#include <AP_Param/AP_Param.h>

#include "AC_DroneShowManager.h"
#include <AC_Fence/AC_Fence.h>

#include <skybrush/skybrush.h>

#include "DroneShowLEDFactory.h"

#ifndef HAL_BOARD_COLLMOT_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_COLLMOT_DIRECTORY "./collmot"
#  else
#    define HAL_BOARD_COLLMOT_DIRECTORY "/COLLMOT"
#  endif
#endif

#define SHOW_FILE (HAL_BOARD_COLLMOT_DIRECTORY "/show.skyb")

// Default update rate for position and velocity targets
#define DEFAULT_UPDATE_RATE_HZ 10

// Length of a GPS week in seconds
#define GPS_WEEK_LENGTH_SEC 604800

// Length of a GPS week in milliseconds
#define GPS_WEEK_LENGTH_MSEC 604800000

// Smallest valid value of show AMSL. Values smaller than this are considered unset.
#define SMALLEST_VALID_AMSL -9999999

// Largest valid value of show AMSL. Values larger than this are considered invalid.
#define LARGEST_VALID_AMSL 10000000

// Default altitude to take off to when starting the show, in meters. The drone
// will take off to this altitude above its current position.
#define DEFAULT_TAKEOFF_ALTITUDE_METERS 2.5f

// Default time synchronization mode
#define DEFAULT_SYNC_MODE TimeSyncMode_GPS

// Default takeoff placement error tolerance level, in meters. The drone will not
// take off if it is placed farther than this distance from its takeoff position.
#define DEFAULT_XY_PLACEMENT_ERROR_METERS 3.0f

// Default horizontal trajectory drift tolerance level, in meters.
#define DEFAULT_MAX_XY_DRIFT_METERS 3.0f

// Default vertical trajectory drift tolerance level, in meters.
#define DEFAULT_MAX_Z_DRIFT_METERS 3.0f

// Default action to take when the show trajectory ends
#define DEFAULT_POST_ACTION PostAction_RTLOrLand

// Distance threshold for the trajectory to be considered circular, in meters.
#define DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS 0.5f

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// UDP port that the drone show manager uses to broadcast the status of the RGB light
// when compiled with the SITL simulator. Uncomment if you need it.
// #  define RGB_SOCKET_PORT 4245
#endif

extern const AP_HAL::HAL &hal;

namespace CustomPackets {
    static const uint8_t START_CONFIG = 1;
    static const uint8_t CRTL_TRIGGER = 2;

    typedef struct PACKED {
        // Start time to set on the drone, in GPS time of week (sec). Anything
        // larger than 604799 means not to touch the start time that is
        // currently set. Negative number means that the start time must be
        // cleared.
        int32_t start_time;
        DroneShowAuthorization authorization;

        struct PACKED {
            // Countdown, i.e. number of milliseconds until the start of the show.
            // Positive number means that there is still some time left.
            int32_t countdown_msec;
        } optional_part;
    } start_config_t;

    typedef struct PACKED {
        // Timestamp to trigger collective RTL at, relative to the show start,
        // in seconds. Zero is a special value, it clears any scheduled
        // collective RTL for the future if the drone has not started the
        // CRTL trajectory yet.
        uint16_t start_time;
    } crtl_trigger_t;
};

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
    AP_GROUPINFO("START_AUTH", 5, AC_DroneShowManager, _params.authorization, 0),

    // @Param: LED0_TYPE
    // @DisplayName: Assignment of LED channel 0 to a LED output type
    // @Description: Specifies where the output of the main LED light track of the show should be sent
    // @Values: 0:Off, 1:MAVLink, 2:NeoPixel, 3:ProfiLED, 4:Debug, 5:SITL, 6:Servo, 7:I2C RGB, 8:Inverted servo, 9:UART (WGDrones), 10:NeoPixel RGBW, 11:I2C RGBW, 12:Notification LED
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

    // @Param: MODE_BOOT
    // @DisplayName: Conditions for entering show mode
    // @Description: Bitfield that specifies when the drone should switch to show mode automatically
    // @Values: 3:At boot and when authorized,2:When authorized,1:At boot,0:Never
    // @Bitmask: 0:At boot,1:When authorized
    // @User: Standard
    AP_GROUPINFO("MODE_BOOT", 9, AC_DroneShowManager, _params.show_mode_settings, 2),

    // @Param: PRE_LIGHTS
    // @DisplayName: Brightness of preflight check related lights
    // @Description: Controls the brightness of light signals on the drone that are used to report status information when the drone is on the ground
    // @Range: 0 3
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

    // Currently used max parameter ID: 26; update this if you add more parameters.
    // Note that the max parameter ID may appear in the middle of the above list.

    AP_GROUPEND
};

// LED factory that is used to create new RGB LED instances
static DroneShowLEDFactory _rgb_led_factory_singleton;

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage);

AC_DroneShowManager::AC_DroneShowManager() :
    hard_fence(),
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _sock_rgb(true),
    _sock_rgb_open(false),
#endif
    _show_data(0),
    _trajectory_valid(false),
    _light_program_valid(false),
    _yaw_control_valid(false),
    _stage_in_drone_show_mode(DroneShow_Off),
    _start_time_requested_by(StartTimeSource::NONE),
    _start_time_on_internal_clock_usec(0),
    _start_time_unix_usec(0),
    _takeoff_time_sec(0),
    _landing_time_sec(0),
    _crtl_start_time_sec(0),
    _total_duration_sec(0),
    _trajectory_is_circular(false),
    _cancel_requested(false),
    _controller_update_delta_msec(1000 / DEFAULT_UPDATE_RATE_HZ),
    _rgb_led(0),
    _rc_switches_blocked_until(0),
    _boot_count(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    _trajectory = new sb_trajectory_t;
    sb_trajectory_init_empty(_trajectory);

    _trajectory_player = new sb_trajectory_player_t;
    sb_trajectory_player_init(_trajectory_player, _trajectory);

    _light_program = new sb_light_program_t;
    sb_light_program_init_empty(_light_program);

    _light_player = new sb_light_player_t;
    sb_light_player_init(_light_player, _light_program);

    _yaw_control = new sb_yaw_control_t;
    sb_yaw_control_init_empty(_yaw_control);

    _yaw_player = new sb_yaw_player_t;
    sb_yaw_player_init(_yaw_player, _yaw_control);

    // Don't call _update_rgb_led_instance() here, servo framework is not set
    // up yet
}

AC_DroneShowManager::~AC_DroneShowManager()
{
    sb_yaw_player_destroy(_yaw_player);
    delete _yaw_player;

    sb_yaw_control_destroy(_yaw_control);
    delete _yaw_control;

    sb_light_player_destroy(_light_player);
    delete _light_player;

    sb_light_program_destroy(_light_program);
    delete _light_program;

    sb_trajectory_player_destroy(_trajectory_player);
    delete _trajectory_player;

    sb_trajectory_destroy(_trajectory);
    delete _trajectory;
}

void AC_DroneShowManager::early_init()
{
    // AP::FS().mkdir() apparently needs lots of free memory, see:
    // https://github.com/ArduPilot/ardupilot/issues/16103
    EXPECT_DELAY_MS(3000);

    if (AP::FS().mkdir(HAL_BOARD_COLLMOT_DIRECTORY) < 0) {
        if (errno == EEXIST) {
            // Directory already exists, this is okay
        } else {
            hal.console->printf(
                "Failed to create directory %s: %s (code %d)\n",
                 HAL_BOARD_COLLMOT_DIRECTORY, strerror(errno), errno
            );
        }
    }
}

void AC_DroneShowManager::init(const AC_WPNav* wp_nav)
{
    // Get the boot count from the parameters
    enum ap_var_type ptype;
    AP_Int16* boot_count_param = static_cast<AP_Int16*>(AP_Param::find("STAT_BOOTCNT", &ptype));
    _boot_count = boot_count_param ? (*boot_count_param) : 0;

    // Get a reference to the RGB LED factory
    _rgb_led_factory = &_rgb_led_factory_singleton;

    // Store a reference to wp_nav so we can ask what the takeoff speed will be
    _wp_nav = wp_nav;

    // Clear start time and authorization now; at this point the parameter
    // subsystem has already loaded back the previous value from the EEPROM so
    // we are safe to overwrite it
    _params.start_time_gps_sec.set(-1);
    _params.authorization.set(DroneShowAuthorization_Revoked);

    _load_show_file_from_storage();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _open_rgb_led_socket();
#endif
    _update_rgb_led_instance();
}


bool AC_DroneShowManager::clear_scheduled_start_time(bool force)
{
    if (!force && _stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    _params.start_time_gps_sec.set(-1);
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

    // Log the new values
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter("SHOW_ORIGIN_LAT", static_cast<float>(lat));
        logger->Write_Parameter("SHOW_ORIGIN_LNG", static_cast<float>(lng));
        logger->Write_Parameter("SHOW_ORIGIN_AMSL", static_cast<float>(amsl_mm));
        logger->Write_Parameter("SHOW_ORIENTATION", static_cast<float>(orientation_deg));
    }

    return true;
}

bool AC_DroneShowManager::get_current_guided_mode_command_to_send(
    GuidedModeCommand& command,
    int32_t default_yaw_cd,
    bool altitude_locked_above_takeoff_altitude
) {
    Location loc;

    static uint8_t invalid_velocity_warning_sent = 0;
    static uint8_t invalid_acceleration_warning_sent = 0;
    static uint8_t invalid_yaw_warning_sent = 0;
    static uint8_t invalid_yaw_rate_warning_sent = 0;
    // static uint8_t counter = 0;

    float elapsed = get_elapsed_time_since_start_sec();
    float yaw_cd = default_yaw_cd;
    float yaw_rate_cds = 0;
    
    get_desired_global_position_at_seconds(elapsed, loc);

    command.clear();
    command.yaw_cd = default_yaw_cd;

    if (loaded_yaw_control_data_successfully())
    {
        // TODO(vasarhelyi): handle auto yaw mode as well

        yaw_cd = get_desired_yaw_cd_at_seconds(elapsed);

        // Prevent invalid yaw information from leaking into the guided
        // mode controller
        if (isnan(yaw_cd) || isinf(yaw_cd))
        {
            if (!invalid_yaw_warning_sent)
            {
                gcs().send_text(MAV_SEVERITY_WARNING, "Invalid yaw command; not using yaw control");
                invalid_yaw_warning_sent = true;
            }
        }
        else
        {
            yaw_rate_cds = get_desired_yaw_rate_cds_at_seconds(elapsed);

            // Prevent invalid yaw rate information from leaking into the guided
            // mode controller
            if (isnan(yaw_rate_cds) || isinf(yaw_rate_cds))
            {
                if (!invalid_yaw_rate_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid yaw rate command; not using yaw control");
                    invalid_yaw_rate_warning_sent = true;
                }
            }
            else
            {
                command.yaw_cd = yaw_cd;
                command.yaw_rate_cds = yaw_rate_cds;
            }
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
                get_desired_velocity_neu_in_cms_per_seconds_at_seconds(elapsed, command.vel);
                command.vel *= gain;
            }

            // Prevent invalid velocity information from leaking into the guided
            // mode controller
            if (command.vel.is_nan() || command.vel.is_inf())
            {
                if (!invalid_velocity_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid velocity command; using zero");
                    invalid_velocity_warning_sent = true;
                }
                command.vel.zero();
            }
        }

        if (is_acceleration_control_enabled())
        {
            get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(elapsed, command.acc);

            // Prevent invalid acceleration information from leaking into the guided
            // mode controller
            if (command.acc.is_nan() || command.acc.is_inf())
            {
                if (!invalid_acceleration_warning_sent)
                {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Invalid acceleration command; using zero");
                    invalid_acceleration_warning_sent = true;
                }
                command.acc.zero();
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

void AC_DroneShowManager::get_desired_global_position_at_seconds(float time, Location& loc)
{
    sb_vector3_with_yaw_t vec;
    sb_trajectory_player_get_position_at(_trajectory_player, time, &vec);
    _show_coordinate_system.convert_show_to_global_coordinate(vec, loc);
}

void AC_DroneShowManager::get_desired_velocity_neu_in_cms_per_seconds_at_seconds(float time, Vector3f& vel)
{
    sb_vector3_with_yaw_t vec;
    float vel_north, vel_east;
    float orientation_rad = _show_coordinate_system.orientation_rad;

    sb_trajectory_player_get_velocity_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    vel_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    vel_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have mm/s so far, need to convert to cm/s
    vel.x = vel_north / 10.0f;
    vel.y = vel_east / 10.0f;
    vel.z = vec.z / 10.0f;
}

void AC_DroneShowManager::get_desired_acceleration_neu_in_cms_per_seconds_squared_at_seconds(float time, Vector3f& acc)
{
    sb_vector3_with_yaw_t vec;
    float acc_north, acc_east;
    float orientation_rad = _show_coordinate_system.orientation_rad;

    sb_trajectory_player_get_acceleration_at(_trajectory_player, time, &vec);

    // We need to rotate the X axis by -_orientation_rad degrees so it
    // points North. At the same time, we also flip the Y axis so it points
    // East and not West.
    acc_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    acc_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have mm/s/s so far, need to convert to cm/s/s
    acc.x = acc_north / 10.0f;
    acc.y = acc_east / 10.0f;
    acc.z = vec.z / 10.0f;
}

float AC_DroneShowManager::get_desired_yaw_cd_at_seconds(float time)
{
    float value;
    sb_yaw_player_get_yaw_at(_yaw_player, time, &value);

    return _show_coordinate_system.convert_show_to_global_yaw_and_scale_to_cd(value);
}

float AC_DroneShowManager::get_desired_yaw_rate_cds_at_seconds(float time)
{
    float value;
    sb_yaw_player_get_yaw_rate_at(_yaw_player, time, &value);

    return value * 100.0f; /* [deg] -> [cdeg] */
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

// returns the elapsed time since the start of the show, in microseconds
int64_t AC_DroneShowManager::get_elapsed_time_since_start_usec() const
{
    uint64_t now, reference, diff;
    
    // AP::gps().time_epoch_usec() is smart enough to handle the case when
    // the GPS fix was lost so no need to worry about loss of GPS fix here.
    if (uses_gps_time_for_show_start()) {
        now = AP::gps().time_epoch_usec();
        reference = _start_time_unix_usec;
    } else {
        now = AP_HAL::micros64();
        reference = _start_time_on_internal_clock_usec;
    }

    if (reference > 0) {
        if (reference > now) {
            diff = reference - now;
            if (diff < INT64_MAX) {
                return -diff;
            } else {
                return INT64_MIN;
            }
        } else if (reference < now) {
            diff = now - reference;
            if (diff < INT64_MAX) {
                return diff;
            } else {
                return INT64_MAX;
            }
        } else {
            return 0;
        }
    } else {
        return INT64_MIN;
    }
}

int32_t AC_DroneShowManager::get_elapsed_time_since_start_msec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    if (elapsed_usec <= -86400000000) {
        return -86400000;
    } else if (elapsed_usec >= 86400000000) {
        return 86400000;
    } else {
        return static_cast<int32_t>(elapsed_usec / 1000);
    }
}

float AC_DroneShowManager::get_elapsed_time_since_start_sec() const
{
    int64_t elapsed_usec = get_elapsed_time_since_start_usec();

    // Using -INFINITY here can lead to FPEs on macOS in the SITL simulator
    // when compiling in release mode, hence we use a large negative number
    // representing one day
    return elapsed_usec == INT64_MIN ? -86400 : static_cast<float>(elapsed_usec / 1000) / 1000.0f;
}

PostAction AC_DroneShowManager::get_action_at_end_of_show() const
{
    switch (_params.post_action) {
        case PostAction_Land:
            return PostAction_Land;

        case PostAction_Loiter:
            return PostAction_Loiter;

        case PostAction_RTL:
            return PostAction_RTL;

        case PostAction_RTLOrLand:
            return (
                _is_at_takeoff_position_xy(2 * DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS) &&
                _trajectory_is_circular
            ) ? PostAction_RTL : PostAction_Land;

        default:
            // Legacy behaviour when we did not have a parameter for the
            // post-show action
            return PostAction_Land;
    }
}

bool AC_DroneShowManager::get_global_takeoff_position(Location& loc) const
{
    // This function may be called any time, not only during the show, so we
    // need to take the parameters provided by the user, convert them into a
    // ShowCoordinateSystem object, and then use that to get the GPS coordinates
    sb_vector3_with_yaw_t vec;

    if (!_tentative_show_coordinate_system.is_valid())
    {
        return false;
    }

    vec.x = _takeoff_position_mm.x;
    vec.y = _takeoff_position_mm.y;
    vec.z = _takeoff_position_mm.z;

    _tentative_show_coordinate_system.convert_show_to_global_coordinate(vec, loc);

    return true;
}

float AC_DroneShowManager::get_motor_spool_up_time_sec() const {
    float value = 0.0f;

    if (AP_Param::get("MOT_SPOOL_TIME", value)) {
        if (value >= 0.0f) {
            return value;
        }
    }

    return DEFAULT_MOTOR_SPOOL_UP_TIME_SEC;
}

int64_t AC_DroneShowManager::get_time_until_start_usec() const
{
    return -get_elapsed_time_since_start_usec();
}

float AC_DroneShowManager::get_time_until_start_sec() const
{
    return -get_elapsed_time_since_start_sec();
}

float AC_DroneShowManager::get_time_until_takeoff_sec() const
{
    return get_time_until_start_sec() + get_relative_takeoff_time_sec();
}

float AC_DroneShowManager::get_time_until_landing_sec() const
{
    return get_time_until_start_sec() + get_relative_landing_time_sec();
}

MAV_RESULT AC_DroneShowManager::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {

    case MAV_CMD_USER_1: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Reload current show
            if (reload_or_clear_show(/* do_clear = */ 0)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        } else if (is_zero(packet.param1 - 1)) {
            // Clear current show
            if (reload_or_clear_show(/* do_clear = */ 1)) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    case MAV_CMD_USER_2: {
        // param1: command code
        // remaining params depend on param1
        if (is_zero(packet.param1)) {
            // Set show origin, orientation and AMSL with a single command.
            // This is supported with COMMAND_INT MAVLink packets only as we
            // do not want to lose precision in the lat/lng direction due to
            // float representation
            //
            // param4: orientation
            // param5 (x): latitude (degE7)
            // param6 (y): longitude (degE7)
            // param7 (z): AMSL (mm)
            if (configure_show_coordinate_system(
                packet.x, packet.y, static_cast<int32_t>(packet.z),
                packet.param4
            )) {
                return MAV_RESULT_ACCEPTED;
            } else {
                return MAV_RESULT_FAILED;
            }
        }

        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }

    default:
        // Unsupported command code
        return MAV_RESULT_UNSUPPORTED;
    }
}

bool AC_DroneShowManager::handle_message(const mavlink_message_t& msg)
{
    switch (msg.msgid)
    {
        // DATA16, DATA32, DATA64, DATA96 packets are used for custom commands.
        // We do not distinguish between them because MAVLink2 truncates the
        // trailing zeros anyway.
        case MAVLINK_MSG_ID_DATA16:
            return _handle_data16_message(msg);

        case MAVLINK_MSG_ID_DATA32:
            return _handle_data32_message(msg);

        case MAVLINK_MSG_ID_DATA64:
            return _handle_data64_message(msg);

        case MAVLINK_MSG_ID_DATA96:
            return _handle_data96_message(msg);

        case MAVLINK_MSG_ID_LED_CONTROL:
            // The drone show LED listens on the "secret" LED ID 42 with a
            // pattern of 42 as well. Any message that does not match this
            // specification is handled transparently by the "core" MAVLink
            // GCS module.
            return _handle_led_control_message(msg);

        default:
            return false;
    }
}

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

bool AC_DroneShowManager::loaded_show_data_successfully() const
{
    return _trajectory_valid;
}

bool AC_DroneShowManager::loaded_yaw_control_data_successfully() const
{
    return _yaw_control_valid;
}

void AC_DroneShowManager::notify_drone_show_mode_initialized()
{
    _cancel_requested = false;
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
}

void AC_DroneShowManager::notify_drone_show_mode_entered_stage(DroneShowModeStage stage)
{
    if (stage == _stage_in_drone_show_mode) {
        return;
    }

    _stage_in_drone_show_mode = stage;

    // Whenever we change the state, we clear the scheduled start time of a
    // collective RTL trajectory
    clear_scheduled_collective_rtl(/* force = */ true);

    // Force-update preflight checks so we see the errors immediately if we
    // switched to the "waiting for start time" stage
    _update_preflight_check_result(/* force = */ true);
}

void AC_DroneShowManager::notify_drone_show_mode_exited()
{
    _cancel_requested = false;
    _update_rgb_led_instance();
    _clear_start_time_if_set_by_switch();
    _last_setpoint.clear();
}

void AC_DroneShowManager::notify_guided_mode_command_sent(const GuidedModeCommand& command)
{
    _last_setpoint = command;   
}

void AC_DroneShowManager::notify_landed()
{
    _cancel_requested = false;

    // Let's not clear the start time; there's not really much point but at
    // least we don't confuse the GCS (not Skybrush but Mission Planner) with
    // a parameter suddenly changing behind its back. This is just a theoretical
    // possibility but let us be on the safe side.
    // _clear_start_time_after_landing();
}

bool AC_DroneShowManager::notify_takeoff_attempt()
{
    if (!is_prepared_to_take_off())
    {
        return false;
    }
    
    return _copy_show_coordinate_system_from_parameters_to(_show_coordinate_system);
}

bool AC_DroneShowManager::reload_or_clear_show(bool do_clear)
{
    // Don't reload or clear the show if the motors are armed
    if (AP::motors()->armed()) {
        return false;
    }

    if (do_clear) {
        if (AP::FS().unlink(SHOW_FILE)) {
            // Error while removing the file; did it exist?
            if (errno == ENOENT) {
                // File was missing already, this is OK.
            } else {
                // This is a genuine failure
                return false;
            }
        }
    }

    return _load_show_file_from_storage();
}

bool AC_DroneShowManager::reload_show_from_storage()
{
    return reload_or_clear_show(/* do_clear = */ false);
}

void AC_DroneShowManager::send_drone_show_status(const mavlink_channel_t chan) const
{
    const AP_GPS& gps = AP::gps();

    uint8_t packet[16] = { 0x62, };
    uint8_t flags, flags2, flags3, gps_health;
    float elapsed_time;
    int16_t encoded_elapsed_time;
    int32_t encoded_start_time;
    uint16_t encoded_led_color;
    DroneShowModeStage stage = get_stage_in_drone_show_mode();

    /* make sure that we can make use of MAVLink packet truncation */
    memset(packet, 0, sizeof(uint8_t));

    /* calculate status flags */
    flags = 0;
    if (loaded_show_data_successfully() && has_valid_takeoff_time()) {
        flags |= (1 << 7);
    }
    if (has_scheduled_start_time()) {
        flags |= (1 << 6);
    }
    if (has_explicit_show_origin_set_by_user()) {
        flags |= (1 << 5);
    }
    if (has_explicit_show_orientation_set_by_user()) {
        flags |= (1 << 4);
    }
    if (AP::fence()->enabled()) {
        flags |= (1 << 3);
    }
    if (has_authorization_to_start()) {
        flags |= (1 << 2);
    }
    if (uses_gps_time_for_show_start() && !_is_gps_time_ok()) {
        flags |= (1 << 1);
    }
    if (AP::fence()->get_breaches()) {
        /* this bit is sent because ArduCopter's SYS_STATUS message does not
         * mark the fence as "enabled and not healthy" when FENCE_ACTION is
         * set to zero, so the GCS would not be notified about fence breaches
         * if we only looked at SYS_STATUS */
        flags |= (1 << 0);
    }

    /* calculate second byte of status flags */
    flags2 = _preflight_check_failures & 0xf0;
    flags2 |= static_cast<uint8_t>(stage) & 0x0f;

    /* calculate GPS health */
    gps_health = gps.status();
    if (gps_health > 7) {
        gps_health = 7;
    }
    gps_health |= (gps.num_sats() > 31 ? 31 : gps.num_sats()) << 3;

    /* calculate third byte of status flags.
     * Currently we use bits 0 and 1 for encoding the boot count modulo 4,
     * and bit 7 to indicate that the drone has deviated from its expected
     * position. */
    flags3 = _boot_count & 0x03;
    if (!_is_at_expected_position()) {
        flags3 |= (1 << 7);
    }

    /* calculate elapsed time */
    elapsed_time = get_elapsed_time_since_start_sec();
    if (elapsed_time > 32767) {
        encoded_elapsed_time = 32767;
    } else if (elapsed_time <= -32768) {
        encoded_elapsed_time = -32768;
    } else {
        encoded_elapsed_time = static_cast<int16_t>(elapsed_time);
    }

    /* fill the packet. Note that in the first four bytes we _always_ put the
     * start time according to GPS timestamps, even if we are using the
     * internal clock to synchronize the start. This is to make sure that the
     * UI on Skybrush Live shows the GPS timestamp set by the user */
    encoded_start_time = _params.start_time_gps_sec;
    encoded_led_color = sb_rgb_color_encode_rgb565(_last_rgb_led_color);
    memcpy(packet, &encoded_start_time, sizeof(encoded_start_time));
    memcpy(packet + 4, &encoded_led_color, sizeof(encoded_led_color));
    packet[6] = flags;
    packet[7] = flags2;
    packet[8] = gps_health;
    packet[9] = flags3;
    memcpy(packet + 10, &encoded_elapsed_time, sizeof(encoded_elapsed_time));

    mavlink_msg_data16_send(
        chan,
        0x5b,   // Skybrush status packet type marker
        12,     // effective packet length
        packet
    );
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

bool AC_DroneShowManager::should_switch_to_show_mode_at_boot() const
{
    return _params.show_mode_settings & 1;
}

bool AC_DroneShowManager::should_switch_to_show_mode_when_authorized() const
{
    return _params.show_mode_settings & 2;
}

bool AC_DroneShowManager::schedule_delayed_start_after(uint32_t delay_ms)
{
    bool success = false;

    _cancel_requested = false;

    if (_stage_in_drone_show_mode != DroneShow_WaitForStartTime)
    {
        // We are not in the "wait for start time" phase so we ignore the request
        return false;
    }

    if (uses_gps_time_for_show_start()) {
        if (_is_gps_time_ok()) {
            // We are modifying a parameter directly here without notifying the
            // param subsystem, but this is okay -- we do not want to save the
            // start time into the EEPROM, and it is reset at the next boot anyway.
            // Delay is rounded down to integer seconds.
            _params.start_time_gps_sec.set(((AP::gps().time_week_ms() + delay_ms) / 1000) % GPS_WEEK_LENGTH_SEC);
            success = true;
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

void AC_DroneShowManager::stop_if_running()
{
    _cancel_requested = true;
}

void AC_DroneShowManager::update()
{
    static bool main_cycle = true;

    if (main_cycle) {
        _check_changes_in_parameters();
        _check_events();
        _check_radio_failsafe();
        _update_preflight_check_result();
        _update_lights();
    } else {
        _repeat_last_rgb_led_command();
    }

    main_cycle = !main_cycle;
}

bool AC_DroneShowManager::_are_rc_switches_blocked()
{
    return _rc_switches_blocked_until && _rc_switches_blocked_until >= AP_HAL::millis();
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
        if (has_authorization_to_start() && should_switch_to_show_mode_when_authorized()) {
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

void AC_DroneShowManager::_clear_start_time_after_landing()
{
    _params.start_time_gps_sec.set(-1);
    _start_time_on_internal_clock_usec = 0;
    _check_changes_in_parameters();
}

void AC_DroneShowManager::_clear_start_time_if_set_by_switch()
{
    if (_start_time_requested_by == StartTimeSource::RC_SWITCH) {
        clear_scheduled_start_time(/* force = */ true);
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

bool AC_DroneShowManager::_handle_custom_data_message(uint8_t type, void* data, uint8_t length)
{
    if (data == nullptr) {
        return false;
    }

    // We allocate type 0x5C for the GCS-to-drone packets (0X5B is the drone-to-GCS
    // status packet), and sacrifice the first byte of the payload to identify
    // the _real_ message type. This reduces the chance of clashes with other
    // DATA* messages from third parties. The type that we receive in this
    // function is the _real_ message type.
    switch (type) {
        // Broadcast start time and authorization state of the show
        case CustomPackets::START_CONFIG:
            {
                if (length < offsetof(CustomPackets::start_config_t, optional_part)) {
                    // Packet too short
                    return false;
                }

                CustomPackets::start_config_t* start_config = static_cast<CustomPackets::start_config_t*>(data);

                // Update start time expressed in GPS time of week
                if (start_config->start_time < 0) {
                    _params.start_time_gps_sec.set(-1);
                } else if (start_config->start_time < GPS_WEEK_LENGTH_SEC) {
                    _params.start_time_gps_sec.set(start_config->start_time);
                }

                // Update authorization flag
                _params.authorization.set(start_config->authorization);

                // Do we have the optional second part?
                if (length >= sizeof(CustomPackets::start_config_t)) {
                    // Optional second part is used by the GCS to convey how many
                    // milliseconds there are until the start of the show. If this
                    // part exists and is positive, _and_ we are using the internal
                    // clock to synchronize the start, then we update the start
                    // time based on this
                    if (!uses_gps_time_for_show_start()) {
                        int32_t countdown_msec = start_config->optional_part.countdown_msec;

                        if (countdown_msec < -GPS_WEEK_LENGTH_MSEC) {
                            clear_scheduled_start_time();
                        } else if (countdown_msec >= 0 && countdown_msec < GPS_WEEK_LENGTH_MSEC) {
                            schedule_delayed_start_after(countdown_msec);
                        }
                    }
                }

                return true;
            }
            break;

        // Schedule collective RTL
        case CustomPackets::CRTL_TRIGGER:
            if (length >= sizeof(CustomPackets::crtl_trigger_t)) {
                CustomPackets::crtl_trigger_t* crtl_trigger = static_cast<CustomPackets::crtl_trigger_t*>(data);
                if (crtl_trigger->start_time == 0) {
                    clear_scheduled_collective_rtl();
                } else if (crtl_trigger->start_time > 0) {
                    schedule_collective_rtl_at_show_timestamp_msec(
                        crtl_trigger->start_time * 1000 /* [s] --> [msec] */
                    );
                }
            }
    }

    return false;
}

bool AC_DroneShowManager::_handle_data16_message(const mavlink_message_t& msg)
{
    mavlink_data16_t packet;
    mavlink_msg_data16_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data32_message(const mavlink_message_t& msg)
{
    mavlink_data32_t packet;
    mavlink_msg_data32_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data64_message(const mavlink_message_t& msg)
{
    mavlink_data64_t packet;
    mavlink_msg_data64_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
}

bool AC_DroneShowManager::_handle_data96_message(const mavlink_message_t& msg)
{
    mavlink_data96_t packet;
    mavlink_msg_data96_decode(&msg, &packet);
    if (packet.type != 0x5C || packet.len < 1) {
        return false;
    }
    return _handle_custom_data_message(packet.data[0], packet.data + 1, packet.len - 1);
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

bool AC_DroneShowManager::_is_at_takeoff_position_xy(float xy_threshold) const
{
    Location takeoff_loc;
    
    if (!_tentative_show_coordinate_system.is_valid())
    {
        // User did not set up the takeoff position yet
        return false;
    }

    if (!get_global_takeoff_position(takeoff_loc))
    {
        // Show coordinate system not set up yet
        return false;
    }

    return _is_close_to_position(
        takeoff_loc,
        xy_threshold > 0 ? xy_threshold : _params.max_xy_placement_error_m,
        0
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

bool AC_DroneShowManager::_is_gps_time_ok() const
{
    // AP::gos().time_week() starts from zero and gets set to a non-zero value
    // when we start receiving full time information from the GPS. It may happen
    // that the GPS subsystem receives iTOW information from the GPS module but
    // no week number; we deem this unreliable so we return false in this case.
    return AP::gps().time_week() > 0;
}

bool AC_DroneShowManager::is_prepared_to_take_off() const
{
    return (!_preflight_check_failures && _is_gps_time_ok());
}

bool AC_DroneShowManager::_load_show_file_from_storage()
{
    int fd;
    int retval;
    struct stat stat_data;
    uint8_t *show_data, *write_ptr, *end_ptr;
    ssize_t to_read, actually_read;
    bool success = false;

    // Clear any previously loaded show
    _set_light_program_and_take_ownership(0);
    _set_trajectory_and_take_ownership(0);
    _set_yaw_control_and_take_ownership(0);
    _set_show_data_and_take_ownership(0);

    // Check whether the show file exists
    retval = AP::FS().stat(SHOW_FILE, &stat_data);
    if (retval)
    {
        // Show file does not exist. This basically means that the operation
        // was successful.
        return true;
    }

    // Ensure that we have a sensible block size that we will use when reading
    // the show file
    if (stat_data.st_blksize < 1)
    {
        stat_data.st_blksize = 4096;
    }

    // Allocate memory for the whole content of the file
    show_data = static_cast<uint8_t *>(calloc(stat_data.st_size, sizeof(uint8_t)));
    if (show_data == 0)
    {
        hal.console->printf(
            "Show file too large: %ld bytes\n",
            static_cast<long int>(stat_data.st_size));
        return false;
    }

    // Read the entire show file into memory
    fd = AP::FS().open(SHOW_FILE, O_RDONLY);
    if (fd < 0)
    {
        free(show_data);
        show_data = write_ptr = end_ptr = 0;
    }
    else
    {
        write_ptr = show_data;
        end_ptr = show_data + stat_data.st_size;
    }

    while (write_ptr < end_ptr)
    {
        to_read = end_ptr - write_ptr;
        if (to_read > stat_data.st_blksize)
        {
            to_read = stat_data.st_blksize;
        }

        if (to_read == 0)
        {
            break;
        }

        actually_read = AP::FS().read(fd, write_ptr, to_read);
        if (actually_read < 0)
        {
            /* Error while reading */
            hal.console->printf(
                "IO error while reading show file near byte %ld, errno = %d\n",
                static_cast<long int>(write_ptr - show_data),
                static_cast<int>(errno)
            );
            free(show_data);
            show_data = 0;
            break;
        }
        else if (actually_read == 0)
        {
            /* EOF */
            break;
        }
        else
        {
            write_ptr += actually_read;
        }
    }

    if (fd > 0)
    {
        AP::FS().close(fd);
    }

    // Parse the show file and find the trajectory, light program and yaw control data in it
    if (show_data)
    {
        sb_trajectory_t loaded_trajectory;
        sb_light_program_t loaded_light_program;
        sb_yaw_control_t loaded_yaw_control;

        _set_show_data_and_take_ownership(show_data);

        retval = sb_trajectory_init_from_binary_file_in_memory(&loaded_trajectory, show_data, stat_data.st_size);
        if (retval)
        {
            hal.console->printf("Error while parsing show file: %d\n", (int) retval);
        }
        else
        {
            _set_trajectory_and_take_ownership(&loaded_trajectory);

            if (has_valid_takeoff_time())
            {
                hal.console->printf(
                    "Loaded show: %.1fs, takeoff at %.1fs, landing at %.1fs\n",
                    _total_duration_sec, _takeoff_time_sec, _landing_time_sec
                );
                success = true;
            }
            else
            {
                hal.console->printf("Takeoff or landing time is invalid!\n");
            }
        }

        retval = sb_light_program_init_from_binary_file_in_memory(&loaded_light_program, show_data, stat_data.st_size);
        if (retval == SB_ENOENT)
        {
            // No light program in show file, this is okay, we just create an
            // empty one
            retval = sb_light_program_init_empty(&loaded_light_program);
        }

        if (retval)
        {
            hal.console->printf("Error while loading light program: %d\n", (int) retval);
        }
        else
        {
            _set_light_program_and_take_ownership(&loaded_light_program);
        }

        retval = sb_yaw_control_init_from_binary_file_in_memory(&loaded_yaw_control, show_data, stat_data.st_size);
        if (retval == SB_ENOENT)
        {
            // No yaw control in show file, this is okay, we just create an
            // empty one
            _set_yaw_control_and_take_ownership(0);
        }
        else if (retval)
        {
            hal.console->printf("Error while parsing show file: %d\n", (int) retval);
        }
        else
        {
            _set_yaw_control_and_take_ownership(&loaded_yaw_control);
        }

    }

    return success;
}

bool AC_DroneShowManager::_recalculate_trajectory_properties()
{
    sb_trajectory_stats_calculator_t stats_calculator;
    sb_trajectory_stats_t stats;
    sb_vector3_with_yaw_t vec;
    bool success = false;

    if (sb_trajectory_stats_calculator_init(&stats_calculator, 1000.0f /* [mm] */) != SB_SUCCESS)
    {
        return false;
    }

    if (sb_trajectory_player_get_position_at(_trajectory_player, 0, &vec) != SB_SUCCESS)
    {
        // Error while retrieving the first position
        vec.x = vec.y = vec.z = 0;
    }

    _takeoff_position_mm.x = vec.x;
    _takeoff_position_mm.y = vec.y;
    _takeoff_position_mm.z = vec.z;

    _total_duration_sec = 0;
    _takeoff_time_sec = _landing_time_sec = -1;
    _trajectory_is_circular = false;

    stats_calculator.min_ascent = get_takeoff_altitude_cm() * 10.0f; /* [mm] */
    stats_calculator.preferred_descent = stats_calculator.min_ascent;
    stats_calculator.takeoff_speed = get_takeoff_speed_m_s() * 1000.0f; /* [mm/s] */
    stats_calculator.acceleration = get_takeoff_acceleration_m_ss() * 1000.0f; /* [mm/s/s] */

    if (sb_trajectory_stats_calculator_run(&stats_calculator, _trajectory, &stats) == SB_SUCCESS)
    {
        _total_duration_sec = stats.duration_sec;
        _takeoff_time_sec = stats.takeoff_time_sec;
        _landing_time_sec = stats.landing_time_sec;
        success = true;
    }

    sb_trajectory_stats_calculator_destroy(&stats_calculator);

    if (success)
    {
        // The trajectory is circular if the takeoff and landing positions are
        // sufficiently close in the XY plane
        _trajectory_is_circular = (
            stats.start_to_end_distance_xy <=
            DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS * 1000.0f /* [mm] */
        );

        // We need to takeoff earlier due to expected motor spool up time
        _takeoff_time_sec -= get_motor_spool_up_time_sec();

        // Make sure that we never take off before the scheduled start of the
        // show, even if we are going to be a bit late with the takeoff
        if (_takeoff_time_sec < 0)
        {
            _takeoff_time_sec = 0;
        }

        // Check whether the landing time is later than the takeoff time. If it is
        // earlier, it shows that there's something wrong with the trajectory so
        // let's not take off at all.
        success = _landing_time_sec >= _takeoff_time_sec;
    }

    if (!success)
    {
        // This should ensure that has_valid_takeoff_time() returns false
        _landing_time_sec = _takeoff_time_sec = -1;
    }

    return true;
}

void AC_DroneShowManager::_set_light_program_and_take_ownership(sb_light_program_t *value)
{
    sb_light_player_destroy(_light_player);
    sb_light_program_destroy(_light_program);

    if (value)
    {
        *_light_program = *value;
        _light_program_valid = true;
    }
    else
    {
        sb_light_program_init_empty(_light_program);
        _light_program_valid = false;
    }

    sb_light_player_init(_light_player, _light_program);
}

void AC_DroneShowManager::_set_show_data_and_take_ownership(uint8_t *value)
{
    if (_show_data == value)
    {
        return;
    }

    if (_show_data)
    {
        free(_show_data);
    }

    _show_data = value;
}

void AC_DroneShowManager::_set_trajectory_and_take_ownership(sb_trajectory_t *value)
{
    sb_trajectory_player_destroy(_trajectory_player);
    sb_trajectory_destroy(_trajectory);

    if (value)
    {
        *_trajectory = *value;
        _trajectory_valid = true;
    }
    else
    {
        sb_trajectory_init_empty(_trajectory);
        _trajectory_valid = false;
    }

    sb_trajectory_player_init(_trajectory_player, _trajectory);

    if (!_recalculate_trajectory_properties()) {
        _trajectory_valid = false;
    }
}

void AC_DroneShowManager::_set_yaw_control_and_take_ownership(sb_yaw_control_t *value)
{
    sb_yaw_player_destroy(_yaw_player);
    sb_yaw_control_destroy(_yaw_control);

    if (value)
    {
        *_yaw_control = *value;
        _yaw_control_valid = true;
    }
    else
    {
        sb_yaw_control_init_empty(_yaw_control);
        _yaw_control_valid = false;
    }

    sb_yaw_player_init(_yaw_player, _yaw_control);
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

void AC_DroneShowManager::ShowCoordinateSystem::convert_show_to_global_coordinate(
    sb_vector3_with_yaw_t vec, Location& loc
) const {
    float offset_north, offset_east, altitude;
    
    // We need to rotate the X axis by -orientation_rad radians so it points
    // North. At the same time, we also flip the Y axis so it points East and
    // not West.
    offset_north = cosf(orientation_rad) * vec.x + sinf(orientation_rad) * vec.y;
    offset_east = sinf(orientation_rad) * vec.x - cosf(orientation_rad) * vec.y;

    // We have millimeters so far, need to convert the North and East offsets
    // to meters in the XY plane first. In the Z axis, we will need centimeters.
    offset_north = offset_north / 1000.0f;
    offset_east = offset_east / 1000.0f;
    altitude = vec.z / 10.0f;

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

static bool is_safe_to_change_start_time_in_stage(DroneShowModeStage stage) {
    return (
        stage == DroneShow_Off ||
        stage == DroneShow_Init ||
        stage == DroneShow_WaitForStartTime ||
        stage == DroneShow_Landed
    );
}
