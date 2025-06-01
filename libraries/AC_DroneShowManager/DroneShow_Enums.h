#pragma once

#include <cstdint>

/// @file   DroneShow_Enums.h
/// @brief  Enumerations used in the drone show state management library

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
    DroneShow_TestingLights,
};

// Enum representing the flags in the control mode bitmasp
enum DroneShowControlModeFlag {
    DroneShowControl_VelocityControlEnabled = 1,
    DroneShowControl_AccelerationControlEnabled = 2,
};

// Enum representing the authorization scopes for the start of the show
enum DroneShowAuthorization : int8_t {
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

// Result codes for executing drone show events
enum DroneShowEventResult : uint8_t {
    DroneShowEventResult_Success = 0, // Event executed successfully
    DroneShowEventResult_Failure = 1, // Event execution failed, unspecified reason
    DroneShowEventResult_NotSupported = 2, // Event type not supported
    DroneShowEventResult_Unsafe = 3, // Event not executed due to safety reasons
    DroneShowEventResult_TimeMissed = 4, // Event not executed because the time was missed
    DroneShowEventResult_SkipLogging = 5, // Event executed, nothing to log
};
