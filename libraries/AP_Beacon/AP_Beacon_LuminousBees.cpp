#include "AP_Beacon_LuminousBees.h"
#include <ctype.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>

//for fake gps
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_GPS/AP_GPS.h>

//logs
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define TAG_UPDATE_INTERVAL 16 // millis


// constructor
AP_Beacon_LuminousBees::AP_Beacon_LuminousBees(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    _tag = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Beacon, 0);

    if (_tag == nullptr) {
        hal.console->printf("\r\n Cannot access TAG serial port! \r\n");
    }
}

bool AP_Beacon_LuminousBees::healthy()
{
    // Healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

struct tag_packet_t {
    float estPos[3];
    float posErr;
};

// update the state of the sensor
void AP_Beacon_LuminousBees::update(void)
{
    if (_tag != nullptr) {
        uint32_t nbytes = MIN(_tag->available(), 64);

        if (nbytes > 0) {
            struct tag_packet_t tagData;
            uint32_t curr_estimation_update = AP_HAL::millis();
            while (nbytes-- > 0) {
                // Search for start of packet byte
                int16_t ret = _tag->read();
                if (ret >= 0) {
                    if (((uint8_t) ret) == 0xBC) {
                        ret = _tag->read();
                        if ((ret >= 0) && (((uint8_t) ret) == 0x01)) {
                            // Now we are sure that the data is a position update packet
                            ret = _tag->read((uint8_t *) &tagData, 16);
                            if (ret > 0) {
                                if ((this->_last_estimation_update + TAG_UPDATE_INTERVAL) < curr_estimation_update) {
                                    this->_last_estimation_update = curr_estimation_update;
                                    
                                    uint64_t pkt0_time = AP_HAL::micros64();
                                    Vector3f estimated_position;
                                    float position_error = tagData.posErr;

                                    estimated_position.x = tagData.estPos[0];
                                    estimated_position.y = tagData.estPos[1];
                                    estimated_position.z = tagData.estPos[2];

                                    Vector3f position_ned = this->correct_for_orient_yaw(estimated_position);

                                    Location origin_loc;
                                    this->_frontend.get_origin(origin_loc);

                                    origin_loc.offset(position_ned.x, position_ned.y);

                                    this->send_externalNav(position_ned, position_error, pkt0_time, estimated_position);
                                }
                            }
                            last_update_ms = AP_HAL::millis();
                        }
                    }
                } else {
                    return;
                }
            }
        }

    }
}

void AP_Beacon_LuminousBees::send_externalNav(Vector3f position_ned, float position_error, uint64_t pkt0_time, Vector3f estimated_position) {

    // Correct microsec timestamp to millisec
    uint32_t timestamp_ms = pkt0_time / 1000; // System time the measurement was taken (not the time it was received)

    Quaternion attitude;
    attitude.from_euler(0, 0, 0);
    const float posErr = 0.15; // (m) TODO: this is a rough estimate
    const float angErr = M_PI; // (rad) TODO: this is a rough estimate
    const uint32_t reset_timestamp_ms = 0; // TODO: system time of the last position reset request
    
    const uint32_t delay_ms = 0; // TODO: average delay of external nav system measurements relative to inertial measurements

    AP::ahrs().writeExtNavData(position_ned,
                                attitude,
                                posErr,
                                angErr,
                                timestamp_ms,
                                delay_ms,
                                reset_timestamp_ms
                            );

}
