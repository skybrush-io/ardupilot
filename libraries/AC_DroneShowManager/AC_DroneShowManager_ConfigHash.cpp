#include <string.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>

#include "AC_DroneShowManager.h"

// Version of the layout of the data that the show configuration hash is
// calculated from. Increase it whenever the layout changes.
#define SHOW_CONFIG_HASH_VERSION 1

// Updates a running CRC-32 with a 32-bit value in little-endian byte order
static uint32_t crc32_update_le32(uint32_t crc, uint32_t value)
{
    uint8_t buf[4];
    put_le32_ptr(buf, value);
    return crc_crc32(crc, buf, sizeof(buf));
}

void AC_DroneShowManager::_clear_show_config_hash()
{
    _show_config_hash = 0;
}

void AC_DroneShowManager::_update_show_config_hash()
{
    // The hash is the standard CRC-32 (as in zlib) of the following data.
    // Multi-byte values are little-endian.
    //
    // - version of the layout (1 byte)
    // - show ID from the metadata block of the show file (4 bytes)
    // - SHOW_ORIGIN_LAT, SHOW_ORIGIN_LNG and SHOW_ORIGIN_AMSL (4 bytes each)
    // - bit pattern of SHOW_ORIENTATION as a 32-bit float (4 bytes)
    //
    // The parameters are hashed as they are, not the tentative coordinate
    // system derived from them. This makes the hash easy to reproduce on the
    // ground station, which knows the parameter values it has sent.
    //
    // crc_crc32() leaves out the initial value and the final inversion of the
    // standard CRC-32, so we apply them here.
    const uint8_t version = SHOW_CONFIG_HASH_VERSION;
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];
    const float orientation_deg = _params.orientation_deg;
    uint32_t orientation_bits;
    uint32_t crc = 0xFFFFFFFF;

    get_show_id(show_id);
    memcpy(&orientation_bits, &orientation_deg, sizeof(orientation_bits));

    crc = crc_crc32(crc, &version, sizeof(version));
    crc = crc_crc32(crc, show_id, sizeof(show_id));
    crc = crc32_update_le32(crc, _params.origin_lat);
    crc = crc32_update_le32(crc, _params.origin_lng);
    crc = crc32_update_le32(crc, _params.origin_amsl_mm);
    crc = crc32_update_le32(crc, orientation_bits);
    crc ^= 0xFFFFFFFF;

    // Zero is reserved to mean "not known" in the status packets
    _show_config_hash = crc ? crc : 1;
}
