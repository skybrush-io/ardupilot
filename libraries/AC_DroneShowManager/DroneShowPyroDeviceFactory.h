#pragma once

/// @file   DroneShowPyroDeviceFactory.h
/// @brief  Pyrotechnic device factory class that creates pyrotechnic device instances for the drone show manager module

#include "DroneShowPyroDevice.h"

// Supported pyrotechnic device types for the drone show manager
enum DroneShowPyroDeviceType {
    // No pyrotechnic device
    DroneShowPyroDeviceType_None = 0,

    // Debug output; pyro events are sent as MAVLink STATUSTEXT messages
    DroneShowPyroDeviceType_Debug = 1,

    // Pyrotechnic device is triggered via a single servo channel
    DroneShowPyroDeviceType_SingleServo = 2,
};

class DroneShowPyroDeviceFactory
{
public:
    DroneShowPyroDeviceFactory();

    /* Do not allow copies */
    DroneShowPyroDeviceFactory(const DroneShowPyroDeviceFactory &other) = delete;
    DroneShowPyroDeviceFactory &operator=(const DroneShowPyroDeviceFactory&) = delete;

    /**
     * Creates a new DroneShowPyro instance, given the pyrotechnic device type.
     */
    DroneShowPyroDevice* new_pyro_device_by_type(
        DroneShowPyroDeviceType type
    );
};
