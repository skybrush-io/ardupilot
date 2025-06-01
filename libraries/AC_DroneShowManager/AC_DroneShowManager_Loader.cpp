#include <sys/stat.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <skybrush/skybrush.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

extern const AP_HAL::HAL &hal;

bool AC_DroneShowManager::loaded_show_data_successfully() const
{
    return _trajectory_valid;
}

bool AC_DroneShowManager::loaded_yaw_control_data_successfully() const
{
    return _yaw_control_valid;
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

bool AC_DroneShowManager::_create_show_directory()
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
            return false;
        }
    }

    return true;
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
    _set_event_list_and_take_ownership(0);
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

    // Allocate memory for the whole content of the file and a few extra bytes
    // at the end to make sure that we have enough space to extend the trajectory
    // with a smooth landing segment if needed.
    //
    // The landing segment will be a single cubic Bezier spline. In the worst
    // case, we need X, Y and Z coordinates for two control points and the
    // final point where the landing is triggered, plus the duration of the
    // segment and a header byte. 2 bytes per coordinate, 3 coordinates per
    // point, 3 points, plus 2 bytes for the duration and 1 byte for the header
    // yields a total of 21 extra bytes. We allocate 32 to be on the safe side.
    show_data = static_cast<uint8_t *>(calloc(stat_data.st_size + 32, sizeof(uint8_t)));
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
        sb_event_list_t loaded_event_list;

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
            hal.console->printf("Error while lading yaw data: %d\n", (int) retval);
        }
        else
        {
            _set_yaw_control_and_take_ownership(&loaded_yaw_control);
        }

        retval = sb_event_list_init(&loaded_event_list, 4);
        if (retval == ENOMEM)
        {
            hal.console->printf("Error while loading events: %d\n", (int) retval);
        }
        else
        {
            retval = sb_event_list_update_from_binary_file_in_memory(&loaded_event_list, show_data, stat_data.st_size);
            if (retval == SB_ENOENT)
            {
                // No event list in show file, this is okay
                retval = SB_SUCCESS;
            }
            else if (retval)
            {
                hal.console->printf("Error while loading events: %d\n", (int) retval);
            }
            else
            {
                // Adjust the timestamps of pyro events if needed
                if (_params.pyro_spec.time_compensation_msec != 0) {
                    sb_event_list_adjust_timestamps_by_type(
                        &loaded_event_list, SB_EVENT_TYPE_PYRO,
                        -_params.pyro_spec.time_compensation_msec
                    );
                }

                // Also add pyro off events. This can potentially fail as we
                // are inserting new events.
                if (_params.pyro_spec.ignition_duration_msec > 0) {
                    retval = sb_event_list_add_pyro_off_events(
                        &loaded_event_list, _params.pyro_spec.ignition_duration_msec
                    );
                    if (retval != SB_SUCCESS) {
                        hal.console->printf(
                            "Error while adding pyro off events: %d\n", (int) retval
                        );
                    }
                }
            }

            if (retval == SB_SUCCESS) {
                _set_event_list_and_take_ownership(&loaded_event_list);
            }
            else {
                sb_event_list_destroy(&loaded_event_list);
            }
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

        // Remember that we can modify the trajectory at takeoff to ensure
        // precise landing back at the takeoff position. This will be done only
        // if the trajectory is circular (i.e. we are meant to land at the
        // same position).
        _trajectory_modified_for_landing = false;

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

void AC_DroneShowManager::_set_event_list_and_take_ownership(sb_event_list_t *value)
{
    sb_event_list_player_destroy(_event_list_player);
    sb_event_list_destroy(_event_list);

    if (value)
    {
        *_event_list = *value;
        _event_list_valid = true;
    }
    else
    {
        sb_event_list_init(_event_list, 0);
        _event_list_valid = false;
    }

    sb_event_list_player_init(_event_list_player, _event_list);
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
