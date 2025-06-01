#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_DRONE_SHOW \
    LOG_DRONE_SHOW_MSG, \
    LOG_FENCE_STATUS_MSG, \
    LOG_DRONE_SHOW_EVENT_MSG

// @LoggerMessage: SHOW
// @Description: Drone show mode information
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time on the show clock
// @Field: Stage: Current stage of the show
// @Field: R: Red component of current color in show
// @Field: G: Green component of current color in show
// @Field: B: Blue component of current color in show
// @Field: HDist: Horizontal distance from desired position
// @Field: VDist: Vertical distance from desired position

// drone show mode logging
struct PACKED log_DroneShowStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t show_clock_ms;
    uint8_t stage;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    float h_dist;
    float v_dist;
};

// @LoggerMessage: FNCS
// @Description: Geofence status information
// @Field: TimeUS: Time since system startup
// @Field: GeoEn: Bitmask of enabled geofences
// @Field: GeoB: Bitmask of current geofence breaches
// @Field: GeoCnt: Number of geofence breaches
// @Field: HardB: Status of hard geofence (0: OK, 1: breached, 2: action taken)
// @Field: BubbleB: Status of bubble geofence (0: OK, 1: breached, 2: action taken)
// @Field: BubbleCnt: Number of bubble geofence actions taken

// fence status logging
struct PACKED log_FenceStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t geo_enabled;
    uint8_t geo_breaches;
    uint16_t geo_breach_count;
    uint8_t hard_breach_state;
    uint8_t bubble_breach_state;
    uint16_t bubble_breach_count;
};

// @LoggerMessage: SBEV
// @Description: Skybrush show file event execution log
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time on the show clock
// @Field: Type: Type of the event from the show file
// @Field: Subtype: Subtype of the event from the show file
// @Field: Payload: Payload of the event from the show file as uint32_t
// @Field: Result: Result of the event execution

// drone show events
struct PACKED log_DroneShowEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t show_clock_ms;
    uint8_t type;
    uint8_t subtype;
    uint32_t payload;
    uint8_t result;
};

#define LOG_STRUCTURE_FROM_DRONE_SHOW \
    { LOG_DRONE_SHOW_MSG, sizeof(log_DroneShowStatus),                  \
      "SHOW", "QiBBBBff", "TimeUS,ClockMS,Stage,R,G,B,HDist,VDist", "ss----mm", "FC----BB" }, \
    { LOG_FENCE_STATUS_MSG, sizeof(log_FenceStatus),                    \
      "FNCS", "QBBHBBH", "TimeUS,GeoEn,GeoB,GeoCnt,HardB,BubbleB,BubbleCnt", "s------", "F------" }, \
    { LOG_DRONE_SHOW_EVENT_MSG, sizeof(log_DroneShowEvent),              \
      "SBEV", "QiBBIB", "TimeUS,ClockMS,Type,Subtype,Payload,Result", "ss----", "FC----" }
