#include <sys/stat.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

#include <skybrush/skybrush.h>

#include "AC_DroneShowManager.h"
#include "DroneShow_Constants.h"

extern const AP_HAL::HAL &hal;

bool AC_DroneShowManager::loaded_show_data_successfully() const
{
    return !sb_screenplay_is_empty(&_screenplay);
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
    int fd = -1;
    int retval;
    struct stat stat_data;
    uint8_t *show_data, *write_ptr, *end_ptr;
    ssize_t to_read, actually_read;
    bool success = false;
    
    // Clear any previously loaded show, rewind the clock to zero and free up
    // any memory used by the previously loaded show
    sb_screenplay_clear(&_screenplay);
    sb_screenplay_scene_clear_contents(&_main_show_scene);
    sb_show_controller_notify_screenplay_changed(&_show_controller);
    sb_show_controller_update_time_msec(&_show_controller, 0);
    _invalidate_projected_wall_clock_time_at_takeoff();
    if (_show_data)
    {
        free(_show_data);
        _show_data = nullptr;
    }

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
    
    // From this point onwards, show_data is owned by us and must be freed
    // so do not return from the function without freeing it. If you need an
    // early return, jump to the "exit" label.

    // Read the entire show file into memory
    fd = AP::FS().open(SHOW_FILE, O_RDONLY);
    if (fd < 0)
    {
        goto exit;
    }
    
    write_ptr = show_data;
    end_ptr = show_data + stat_data.st_size;

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
            goto exit;
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

    // Parse the show file and find the trajectory, light program, yaw control data
    // and event list in it.
    retval = static_cast<int>(
        sb_screenplay_update_from_binary_file_in_memory(&_screenplay, show_data, stat_data.st_size)
    );
    success = retval == SB_SUCCESS;
    
    if (success)
    {
        // Since the screenplay was updated, we need to let the show controller know
        // that any cached outputs are now invalid
        sb_show_controller_notify_screenplay_changed(&_show_controller);
        _invalidate_projected_wall_clock_time_at_takeoff();

        if (!_recalculate_trajectory_properties())
        {
            hal.console->printf("Error while calculating trajectory properties\n");
            success = false;
        }
        else if (!is_trajectory_plausible())
        {
            hal.console->printf("Takeoff or landing time is invalid\n");
            success = false;
        }
        else
        {
            hal.console->printf(
                "Loaded show: %.1fs, takeoff at %.1fs, landing at %.1fs\n",
                _trajectory_stats.duration_sec, 
                _trajectory_stats.takeoff_time_sec, 
                _trajectory_stats.landing_time_sec
            );
        }
    }
    else
    {
        hal.console->printf("Error while parsing show file, code: %d\n", retval);
    }
    
    if (success)
    {
        // Adjust the timestamps of pyro events if needed. Also set the duration of
        // each scene to the duration of its trajectory to ensure that we consider the
        // show as finished when the trajectory ends. The last (currently the only)
        // scene is also marked as dynamic so we know that we are free to adjust its
        // duration later if the trajectory changes (e.g., after takeoff, when the
        // trajectory is circular and we need to compensate for the actual takeoff
        // position).
        size_t i, num_scenes = sb_screenplay_size(&_screenplay);
        for (i = 0; i < num_scenes; i++)
        {
            sb_screenplay_scene_t* scene = sb_screenplay_get_scene_ptr(&_screenplay, i);
            sb_event_list_t* event_list = scene ? sb_screenplay_scene_get_events(scene) : nullptr;
            sb_trajectory_t* trajectory = scene ? sb_screenplay_scene_get_trajectory(scene) : nullptr;
            bool is_last = (i == num_scenes - 1);

            if (trajectory) {
                sb_screenplay_scene_set_duration_msec(scene,
                    is_last
                        ? static_cast<uint32_t>(_trajectory_stats.landing_time_sec * 1000.0f)
                        : _trajectory_stats.duration_msec
                );
            }

            if (event_list && _params.pyro_spec.time_compensation_msec != 0)
            {
                sb_event_list_adjust_timestamps_by_type(
                    event_list, SB_EVENT_TYPE_PYRO,
                    -_params.pyro_spec.time_compensation_msec
                );
            }

            if (is_last) {
                sb_screenplay_scene_set_flag(scene, SB_SCREENPLAY_SCENE_FLAG_DYNAMIC_DURATION);
            }
        }
    }
    
    if (success)
    {
        // move ownership of the show data to the class member
        _show_data = show_data;
        show_data = nullptr;
        
        // Update the main show scene from the first (and only) scene of the screenplay
        sb_screenplay_scene_t* first_scene = sb_screenplay_get_scene_ptr(&_screenplay, 0);
        if (first_scene)
        {
            sb_screenplay_scene_set_tag(first_scene, SceneTag_MainShow);
            sb_screenplay_scene_update_contents_from(&_main_show_scene, first_scene);
        }
    }
    else
    {
        sb_screenplay_clear(&_screenplay);
    }
    
    // Now that the screenplay is updated, forget the previous sequence number of
    // time axis configuration packets so we will process it again (with the new
    // screenplay) when the GCS sends one
    _last_time_axis_config_seq_no = 0xFFFF;    // 0xFFFF is never a valid sequence number

exit:
    // if we still have the show data here and its ownership was not moved to the
    // class member, free it now
    if (show_data)
    {
        free(show_data);
    }
    
    // Also clean up any open file descriptors
    if (fd >= 0)
    {
        AP::FS().close(fd);
    }
    
    return success;
}

bool AC_DroneShowManager::_recalculate_trajectory_properties()
{
    sb_trajectory_stats_calculator_t stats_calculator;
    sb_screenplay_scene_t* scene;
    sb_trajectory_t* trajectory;
    bool success = false;

    if (sb_trajectory_stats_calculator_init(&stats_calculator, 1000.0f /* [mm] */) != SB_SUCCESS)
    {
        return false;
    }

    _trajectory_stats.duration_sec = 0;
    _trajectory_stats.takeoff_time_sec = _trajectory_stats.landing_time_sec = -1;
    _trajectory_is_circular = false;

    stats_calculator.min_ascent = get_takeoff_altitude_cm() * 10.0f; /* [mm] */
    stats_calculator.preferred_descent = stats_calculator.min_ascent;
    stats_calculator.takeoff_speed = get_takeoff_speed_m_sec() * 1000.0f; /* [mm/s] */
    stats_calculator.acceleration = get_takeoff_acceleration_m_ss() * 1000.0f; /* [mm/s/s] */
    
    // Rewind the show controller to the start of the show
    if (sb_show_controller_update_time_msec(&_show_controller, 0) != SB_SUCCESS)
    {
        return false;
    }
    
    // Get the trajectory from the show controller
    scene = sb_show_controller_get_current_scene(&_show_controller);
    trajectory = scene ? sb_screenplay_scene_get_trajectory(scene) : nullptr;
    if (trajectory == nullptr)
    {
        return false;
    }

    if (sb_trajectory_stats_calculator_run(&stats_calculator, trajectory, &_trajectory_stats) == SB_SUCCESS)
    {
        success = true;
    }

    sb_trajectory_stats_calculator_destroy(&stats_calculator);

    if (success)
    {
        // Copy the first coordinate from the stats result
        _takeoff_position_mm.x = _trajectory_stats.initial_pos.x;
        _takeoff_position_mm.y = _trajectory_stats.initial_pos.y;
        _takeoff_position_mm.z = _trajectory_stats.initial_pos.z;
    
        // The trajectory is circular if the takeoff and landing positions are
        // sufficiently close in the XY plane
        _trajectory_is_circular = (
            _trajectory_stats.start_to_end_distance_xy <=
            DEFAULT_START_END_XY_DISTANCE_THRESHOLD_METERS * 1000.0f /* [mm] */
        );

        // Remember that we can modify the trajectory at takeoff to ensure
        // precise landing back at the takeoff position. This will be done only
        // if the trajectory is circular (i.e. we are meant to land at the
        // same position).
        _trajectory_modified_for_landing = false;

        // We need to takeoff earlier due to expected motor spool up time
        _trajectory_stats.takeoff_time_sec -= get_motor_spool_up_time_sec();

        // Make sure that we never take off before the scheduled start of the
        // show, even if we are going to be a bit late with the takeoff
        if (_trajectory_stats.takeoff_time_sec < 0)
        {
            _trajectory_stats.takeoff_time_sec = 0;
        }

        // Check whether the landing time is later than the takeoff time. If it is
        // earlier, it shows that there's something wrong with the trajectory so
        // let's not take off at all.
        success = _trajectory_stats.landing_time_sec >= _trajectory_stats.takeoff_time_sec;
    }

    if (!success)
    {
        // Clear the takeoff position
        // Copy the first coordinate from the stats result
        _takeoff_position_mm.x = 0;
        _takeoff_position_mm.y = 0;
        _takeoff_position_mm.z = 0;
    
        // This should ensure that is_trajectory_plausible() returns false
        _trajectory_stats.landing_time_sec = _trajectory_stats.takeoff_time_sec = -1;
    }
    
    // Invalidate the projected takeoff time because it depends on the trajectory
    // stats. It will be realculated later if needed.
    _invalidate_projected_wall_clock_time_at_takeoff();

    return true;
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
