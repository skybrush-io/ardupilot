#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_GPS_RTK                    \
    LOG_GPS_RTK_PACKET_MSG

// @LoggerMessage: RTK
// @Description: GPS RTK injection log
// @Field: TimeUS: Time since system startup
// @Field: Type: RTCM3 message type
// @Field: Length: Length of injected RTCM3 message

struct PACKED log_GPSRTKPacket {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t type;
    uint16_t length;
};

#define LOG_STRUCTURE_FROM_GPS_RTK \
    { LOG_GPS_RTK_PACKET_MSG, sizeof(log_GPSRTKPacket), \
      "RTK", "QHH", "TimeUS,Type,Length", "s-b", "F--" }
