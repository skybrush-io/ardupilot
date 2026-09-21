#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_DRONE_SHOW \
    LOG_DRONE_SHOW_MSG, \
    LOG_FENCE_STATUS_MSG, \
    LOG_DRONE_SHOW_EVENT_MSG, \
    LOG_SCREENPLAY_ENTRY_MSG, \
    LOG_CRTH_TRIGGER_MSG

// @LoggerMessage: SHOW
// @Description: Drone show mode information
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: Stage: Current stage of the show
// @Field: Scene: Scene index in the current show
// @Field; ShowClockMS: Time on the show clock in the current scene
// @Field: R: Red component of current color in show
// @Field: G: Green component of current color in show
// @Field: B: Blue component of current color in show
// @Field: HDist: Horizontal distance from desired position
// @Field: VDist: Vertical distance from desired position

// drone show mode logging
struct PACKED log_DroneShowStatus {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint8_t stage;
    uint8_t scene;
    int32_t show_clock_ms;
    float x;
    float y;
    float z;
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
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: Scene: Scene index in the current show
// @Field; ShowClockMS: Time on the show clock in the current scene
// @Field: Type: Type of the event from the show file
// @Field: Subtype: Subtype of the event from the show file
// @Field: Payload: Payload of the event from the show file as uint32_t
// @Field: Result: Result of the event execution

// drone show events
struct PACKED log_DroneShowEvent {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint8_t scene;
    int32_t show_clock_ms;
    uint8_t type;
    uint8_t subtype;
    uint32_t payload;
    uint8_t result;
};

// @LoggerMessage: SBSP
// @Description: Skybrush screenplay log
// @Field: TimeUS: Time since system startup
// @Field: Seq: Sequence number of the time axis configuration
// @Field: Scene: Index of the scene that this entry refers to
// @Field: Index: Index of the time axis entry in this scene
// @Field: Origin: Number of milliseconds elapsed since the UNIX epoch at the time when the show clock is at 00:00,
// @Field: Duration: Duration of the time axis entry in milliseconds
// @Field: IR: Initial rate of the time axis entry (1 = real time, 0.5 = half speed, 0 = standstill, etc.)
// @Field: FR: Final rate of the time axis entry (1 = real time, 0.5 = half speed, 0 = standstill, etc.)
struct PACKED log_ScreenplayEntry {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t seq_no;
    uint8_t scene;
    uint8_t index;
    uint64_t origin_ms;
    uint32_t duration_ms;
    float initial_rate;
    float final_rate;
};

// @LoggerMessage: CRTH
// @Description: Collective RTH trigger log
// @Field: TimeUS: Time since system startup
// @Field: ClockMS: Time since the start of the show in wall clock time
// @Field: StartMS: Start time of the chosen collective RTH plan
// @Field: StartX: X coordinate of the start of the RTH trajectory
// @Field: StartY: Y coordinate of the start of the RTH trajectory
// @Field: StartZ: Z coordinate of the start of the RTH trajectory
struct PACKED log_CollectiveRTHTrigger {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int32_t wall_clock_ms;
    uint32_t rth_start_time_ms;
    float start_x;
    float start_y;
    float start_z;
};

#define LOG_STRUCTURE_FROM_DRONE_SHOW \
    { LOG_DRONE_SHOW_MSG, sizeof(log_DroneShowStatus),                  \
      "SHOW", "QiBBifffBBBff", "TimeUS,ClockMS,Stage,Scene,SceneMS,X,Y,Z,R,G,B,HDist,VDist", "ss--smmm---mm", "FC--C--------" }, \
    { LOG_FENCE_STATUS_MSG, sizeof(log_FenceStatus),                    \
      "FNCS", "QBBHBBH", "TimeUS,GeoEn,GeoB,GeoCnt,HardB,BubbleB,BubbleCnt", "s------", "F------" }, \
    { LOG_DRONE_SHOW_EVENT_MSG, sizeof(log_DroneShowEvent),              \
      "SBEV", "QiBiBBIB", "TimeUS,ClockMS,Scene,SceneMS,Type,Subtype,Payload,Result", "ss-s----", "FC-C----" }, \
    { LOG_SCREENPLAY_ENTRY_MSG, sizeof(log_ScreenplayEntry),             \
      "SBSP", "QBBBQIff", "TimeUS,Seq,Scene,Index,Origin,Duration,IR,FR", "s---ss--", "F---CC--" }, \
    { LOG_CRTH_TRIGGER_MSG, sizeof(log_CollectiveRTHTrigger),            \
      "CRTH", "QiIfff", "TimeUS,ClockMS,StartMS,StartX,StartY,StartZ", "sssmmm", "FCC---" }
