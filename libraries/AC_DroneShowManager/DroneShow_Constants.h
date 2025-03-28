#pragma once

#include <AP_BoardConfig/AP_BoardConfig.h>

/// @file   DroneShow_Constants.h
/// @brief  Internal constants used in the drone show state management library

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

// Default horizontal bubble fence drift tolerance level, in meters.
#define DEFAULT_BUBBLE_FENCE_MAX_XY_DRIFT_METERS 10.0f

// Default vertical bubble fence drift tolerance level, in meters.
#define DEFAULT_BUBBLE_FENCE_MAX_Z_DRIFT_METERS 10.0f

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// UDP port that the drone show manager uses to broadcast the status of the RGB light
// when compiled with the SITL simulator. Uncomment if you need it.
// #  define RGB_SOCKET_PORT 4245
#endif

#ifndef HAL_BOARD_COLLMOT_DIRECTORY
#  if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#    define HAL_BOARD_COLLMOT_DIRECTORY "./collmot"
#  else
#    define HAL_BOARD_COLLMOT_DIRECTORY "/COLLMOT"
#  endif
#endif

#define SHOW_FILE (HAL_BOARD_COLLMOT_DIRECTORY "/show.skyb")
