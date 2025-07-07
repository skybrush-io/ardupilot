#pragma once

#include <AP_Common/AP_Common.h>
#include <cstdint>

#include "DroneShow_Enums.h"

namespace CustomPackets {
    static const uint8_t START_CONFIG = 1;
    static const uint8_t CRTL_TRIGGER = 2;
    static const uint8_t SIMPLE_GEOFENCE_SETUP = 3;
    static const uint8_t ACKNOWLEDGMENT = 4;

    static const uint8_t DRONE_TO_GCS_STATUS = 0x5b;
    static const uint8_t GCS_TO_DRONE = 0x5c;
    static const uint8_t DRONE_TO_GCS = 0x5d;

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

    typedef struct PACKED {
        // Number of points in the polygon geofence; zero means that the polygon
        // fence is off.
        uint8_t num_points;

        // Maximum altitude of the geofence, in decimeters above ground level.
        // Zero means that the altitude fence is off.
        uint16_t max_altitude_dm;

        // Radius of the circle geofence, in decimeters. Zero means that the
        // circular fence is off.
        uint16_t radius_dm;

        // Acknowledgment token to return to the sender
        uint16_t ack_token;

        // Flags that currently encode the geofence action in the four least
        // significant bits; the remaining bits must be zero.
        uint8_t flags;

        // Reserved bytes; must be zero
        uint8_t reserved[3];

        // The rest of the packet contains the geofence points in the show
        // coordinate system as (x, y) pairs, where x and y are in decimeters
        // and are encoded as 16-bit signed integers.
        //
        // The largest packet in which we can store this structure is DATA96,
        // where the payload is 96 bytes long. The first byte of the payload is
        // reserved for the packet type. The fields above take up an additional
        // 11 bytes, so we have 84 bytes for the points themselves. We need
        // 4 bytes per point, so we can store at most 21 points.
    } simple_geofence_setup_header_t;

    typedef struct PACKED {
        // Acknowledgment token to return to the sender, originally defined in
        // the packet that the acknowledgment responds to.
        uint16_t ack_token;

        // Result of the acknowledgment.
        MAV_RESULT result;
    } acknowledgment_t;
};

