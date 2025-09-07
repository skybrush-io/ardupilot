#include "DroneShowPyroDeviceFactory.h"

#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowPyroDevice_Debug.h"
#include "DroneShowPyroDevice_SingleServo.h"
#include "DroneShowPyroDevice_MultipleServos.h"

/// Default constructor.
DroneShowPyroDeviceFactory::DroneShowPyroDeviceFactory()
{
}

DroneShowPyroDevice* DroneShowPyroDeviceFactory::new_pyro_device_by_type(
    DroneShowPyroDeviceType type
) {
    DroneShowPyroDevice* result = NULL;
    uint8_t chan;
    uint32_t chans;

    switch (type) {
        case DroneShowPyroDeviceType_Debug:
            result = new DroneShowPyroDevice_Debug();
            break;

        case DroneShowPyroDeviceType_SingleServo:
            if (SRV_Channels::find_channel(SRV_Channel::k_scripting12, chan)) {
                result = new DroneShowPyroDevice_SingleServo(chan);
            }
            break;

        case DroneShowPyroDeviceType_MultipleServos:
            if (SRV_Channels::find_channels(SRV_Channel::k_scripting12, chans)) {
                result = new DroneShowPyroDevice_MultipleServos(chans);
            }
            break;

        default:
            break;
    }

    if (result && !result->init()) {
        // Initialization failed
        delete result;
        result = NULL;
    }

    return result;
}
