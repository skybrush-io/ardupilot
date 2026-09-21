#pragma once

#include <cstdint>

/// @file   DroneShow_FenceConfig.h
/// @brief  Structure used to hold the full geofence configuration of a show

// This is currently experimental and it might never be adopted officially.
// The idea is that we want to have a single structure that holds the
// entire geofence configuration of a show. The ground station should be able
// to submit a single packet containing the preferred geofence configuration
// instead of MAVLink's standard (but complex) geofence upload protocol.

typedef struct PACKED {
    // X coordinate of the point in the polygon geofence, in decimeters
    // relative to the origin of the show coordinate system.
    int16_t x_dm;

    // Y coordinate of the point in the polygon geofence, in decimeters
    // relative to the origin of the show coordinate system.
    int16_t y_dm;
} DroneShow_FencePoint;

typedef struct {
    // Number of points in the polygon geofence; zero means that the polygon
    // fence is off.
    uint8_t num_points;

    // Maximum altitude of the geofence, in decimeters above ground level.
    // Zero means that the altitude fence is off.
    uint16_t max_altitude_dm;

    // Radius of the circle geofence, in decimeters. Zero means that the
    // circular fence is off.
    uint16_t radius_dm;

    // Action to take when the geofence is breached. Use one of the
    // AC_FENCE_ACTION_* constants defined in AC_Fence.h.
    uint8_t action;

    // Pointer to a (not owned) memory area holding the polygon geofence.
    DroneShow_FencePoint* points;
} DroneShow_FenceConfig;
