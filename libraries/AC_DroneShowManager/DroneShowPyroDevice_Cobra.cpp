#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include "DroneShowPyroDevice_Cobra.h"

#define CMD_FIRE_CUES 0x46

uint8_t DroneShowPyroDevice_Cobra::num_channels() const
{
    // Cobra 6M firing module supports 6 channels
    return 6;
}

bool DroneShowPyroDevice_Cobra::init_impl()
{
    auto &sm = AP::serialmanager();

    _serial = sm.find_serial(AP_SerialManager::SerialProtocol_Volz, 0);
    if (_serial == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Cobra: set SERIALx_PROTOCOL to Volz (14)");
        return false;
    }

    uint32_t baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_Volz, 0);
    if (baudrate == 0) {
        baudrate = 115200; // default baudrate for Cobra 6M
    }

    // 0 bytes RX buffer,16 bytes TX buffer (enough for our 4-byte commands)
    _serial->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _serial->begin(baudrate, 0, 16);

    return true;
}

DroneShowEventResult DroneShowPyroDevice_Cobra::fire_impl(uint8_t channel)
{
    // Cobra channels are mapped to bits as follows: 123456xx, i.e. channel 1
    // is the MSB, channel 2 is the next bit, etc.
    //
    // However, we use zero-based channel numbering in the API, so we need to
    // convert to 1-based channels.
    send_command(CMD_FIRE_CUES, (128 >> channel) & 0xFC);
    return DroneShowEventResult_Success;
}

DroneShowEventResult DroneShowPyroDevice_Cobra::off_impl(uint8_t channel)
{
    // Not needed for Cobra 6M, as it automatically turns off the channel after
    // ignition period
    return DroneShowEventResult_Success;
}

void DroneShowPyroDevice_Cobra::send_command(uint8_t command, uint8_t data) {
    // assert(_serial != nullptr);

    uint8_t buf[4];
    buf[0] = command;
    buf[1] = data;
    buf[2] = static_cast<uint8_t>(buf[0] + buf[1]); // Checksum
    buf[3] = 0x0d;   // CR, end of command

    _serial->write(buf, sizeof(buf));
    _serial->flush();
}
