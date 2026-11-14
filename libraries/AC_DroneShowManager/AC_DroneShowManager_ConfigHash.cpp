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
    _set_show_config_hash_value(0);
    _show_config_hash.fence_loaded_at_msec = 0;
    _show_config_hash.polygon_fence_enabled = false;
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
    // - number of items in the fence storage (2 bytes); zero if the polygon
    //   fence is disabled
    // - for each item in the fence storage: its type (1 byte), latitude and
    //   longitude in 1e-7 degrees (4 bytes each), vertex count (1 byte) and
    //   the bit pattern of its radius as a 32-bit float (4 bytes). Fields that
    //   do not apply to the type of the item are zero.
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

    // The fence is read from the storage and not from the loaded fence,
    // because the fence is loaded only when the EKF has an origin. Disabling
    // the polygon fence keeps its items in the storage; we skip them to make
    // all drones with a disabled polygon fence look the same.
    AC_Fence* fence = AP::fence();
    const bool polygon_fence_enabled = _is_polygon_fence_enabled();
    const uint16_t num_items = (fence && polygon_fence_enabled) ? fence->polyfence().num_stored_items() : 0;
    uint8_t buf[2];

    put_le16_ptr(buf, num_items);
    crc = crc_crc32(crc, buf, sizeof(buf));

    for (uint16_t i = 0; i < num_items; i++) {
        AC_PolyFenceItem item {};
        uint32_t radius_bits;

        if (!fence->polyfence().get_item(i, item)) {
            _clear_show_config_hash();
            return;
        }

        const uint8_t type = static_cast<uint8_t>(item.type);
        memcpy(&radius_bits, &item.radius, sizeof(radius_bits));

        crc = crc_crc32(crc, &type, sizeof(type));
        crc = crc32_update_le32(crc, item.loc.x);
        crc = crc32_update_le32(crc, item.loc.y);
        crc = crc_crc32(crc, &item.vertex_count, sizeof(item.vertex_count));
        crc = crc32_update_le32(crc, radius_bits);
    }

    _show_config_hash.fence_loaded_at_msec = fence ? fence->polyfence().get_inclusion_polygon_update_ms() : 0;
    _show_config_hash.polygon_fence_enabled = polygon_fence_enabled;

    crc ^= 0xFFFFFFFF;

    // Zero is reserved to mean "not known" in the status packets
    _set_show_config_hash_value(crc ? crc : 1);
}

void AC_DroneShowManager::_set_show_config_hash_value(uint32_t value)
{
    _show_config_hash.value = value;

    _update_readonly_param(_params.config_hash_hi, "SHOW_CFG_HASH_HI", value >> 16);
    _update_readonly_param(_params.config_hash_lo, "SHOW_CFG_HASH_LO", value & 0xFFFF);
}
