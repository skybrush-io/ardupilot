#include "DroneShowPyroDeviceFactory.h"

#include <AP_Relay/AP_Relay.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "DroneShowPyroDevice_Cobra.h"
#include "DroneShowPyroDevice_Debug.h"
#include "DroneShowPyroDevice_Relay.h"
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
    uint32_t mask;

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
            mask = SRV_Channels::get_output_channel_mask(SRV_Channel::k_scripting12);
            if (mask) {
                result = new DroneShowPyroDevice_MultipleServos(mask);
            }
            break;

        case DroneShowPyroDeviceType_Cobra:
            result = new DroneShowPyroDevice_Cobra();
            break;

        case DroneShowPyroDeviceType_Relay:
            mask = AP::relay()->get_index_mask(AP_Relay_Params::FUNCTION::RELAY);
            if (mask) {
                result = new DroneShowPyroDevice_Relay(mask);
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
