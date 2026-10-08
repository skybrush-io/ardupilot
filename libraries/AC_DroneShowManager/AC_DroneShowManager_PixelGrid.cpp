#include "AC_DroneShowManager.h"
#include "skybrush/colors.h"

bool AC_DroneShowManager::_handle_tunnel_message(const mavlink_message_t& msg) {
    mavlink_tunnel_t packet;

    mavlink_msg_tunnel_decode(&msg, &packet);

    switch (packet.payload_type) {
        case 42421: // pixel grid update
            return _handle_pixel_grid_update_message(packet.payload, packet.payload_length);
        default:
            return false;
    }
}

static const uint8_t PACKET_HEADER_LENGTH = 5; // bytes

// The payload of a pixel grid update message is laid out as follows:
//
// * Bytes 0-1: 2 unused version bits (00 at the moment), 2 bits for the color
//   encoding, followed by 12 bits representing the width of the block being
//   updated, big endian. The height is not encoded; it follows implicitly from
//   the payload length and the width.
// * Bytes 2-4: X and Y coordinate of the upper left corner of the block being
//   updated, 12 bits per coordinate, big endian.
// * Bytes 5-end: the pixel data, row-major order, big endian, in the format
//   indicated by the color encoding bits.
//
// Color encoding bits:
//
// * 00: black-and-white (1 bit per pixel, zero is black, nonzero is last entry of palette)
// * 01: 16-color palette (4 bits per pixel, 0-15 are palette indices)
// * 10: 256-color palette (8 bits per pixel, 0-255 are palette indices)
// * 11: RGB565 (2 bytes per pixel)
//
// Lights are not updated here but in the main loop; we only store the color extracted
// from the payload in _pixel_grid.color.
bool AC_DroneShowManager::_handle_pixel_grid_update_message(void* data, uint8_t length) {
    const uint8_t* payload = static_cast<const uint8_t*>(data);

    // The payload must be long enough to contain the header of the message
    if (length < PACKET_HEADER_LENGTH) {
        return false;
    }

    // Header: two version bits (currently unused, must be zero), two bits for
    // the color encoding, followed by the width of the block in 12 bits
    const uint8_t version = payload[0] >> 6;
    const uint8_t encoding = (payload[0] >> 4) & 0x03;
    const uint16_t width = (static_cast<uint16_t>(payload[0] & 0x0F) << 8) | payload[1];

    // X and Y coordinates of the upper left corner of the block, 12 bits each,
    // big endian
    const uint16_t x = (static_cast<uint16_t>(payload[2]) << 4) | (payload[3] >> 4);
    const uint16_t y = (static_cast<uint16_t>(payload[3] & 0x0F) << 8) | payload[4];

    // Unknown versions are ignored
    if (version != 0) {
        return false;
    }

    if (width == 0) {
        // Zero width; the message is malformed
        return false;
    }

    // Number of bits per pixel in the pixel data of the message
    const uint16_t bits_per_pixel = (encoding == 0)
        ? 1
        : (encoding == 1)
        ? 4 
        : (encoding == 2)
        ? 8 : 16;

    // The height of the block follows implicitly from the payload length, the
    // width and the color encoding. The payload must contain a whole number
    // of rows, otherwise the message is malformed.
    const uint32_t num_bits = static_cast<uint32_t>(length - PACKET_HEADER_LENGTH) * 8;
    const uint32_t bits_per_row = static_cast<uint32_t>(width) * bits_per_pixel;
    if (num_bits == 0 || (num_bits % bits_per_row) != 0) {
        return false;
    }

    const uint32_t height = num_bits / bits_per_row;

    // Check whether the block being updated contains the pixel that this
    // drone represents. If not, the message is handled but it does not affect
    // the color of the LED of this drone.
    uint16_t row, column;
    _pixel_grid.get_position(row, column);

    if (row < y || column < x) {
        return true;
    }

    const uint32_t local_row = row - y;
    const uint32_t local_column = column - x;

    if (local_row >= height || local_column >= width) {
        return true;
    }

    // Index of the pixel of this drone in the pixel data of the message,
    // row-major order
    const uint32_t pixel_index = local_row * width + local_column;

    switch (encoding) {
        case 0: {
            // Black-and-white, 1 bit per pixel, most significant bit first
            const uint8_t byte = payload[PACKET_HEADER_LENGTH + pixel_index / 8];
            const uint8_t bit = 7 - (pixel_index % 8);
            _pixel_grid.set_color_by_bit((byte >> bit) & 0x01);
            break;
        }

        case 1: {
            // 16-color palette
            const uint8_t byte = payload[PACKET_HEADER_LENGTH + pixel_index / 2];
            const uint8_t index = (pixel_index % 2 == 0) ? (byte >> 4) : (byte & 0x0F);
            _pixel_grid.set_color_by_palette_index(index);
            break;
        }

        case 2: {
            // 256-color palette
            const uint8_t index = payload[PACKET_HEADER_LENGTH + pixel_index];
            _pixel_grid.set_color_by_palette_index(index);
            break;
        }

        default: {
            // RGB565, 2 bytes per pixel, big endian
            const uint16_t color565 = (
                (static_cast<uint16_t>(payload[PACKET_HEADER_LENGTH + pixel_index * 2]) << 8)
                | payload[PACKET_HEADER_LENGTH + pixel_index * 2 + 1]
            );
            _pixel_grid.set_color(sb_rgb_color_decode_rgb565(color565));
            break;
        }
    }

    return true;
}

AC_DroneShowManager::PixelGridState::PixelGridState() :
    color(SB_COLOR_BLACK), _row(0), _column(0)
{
    sb_color_palette_init(&_palette);
    reset();
}

AC_DroneShowManager::PixelGridState::~PixelGridState() {
    sb_color_palette_destroy(&_palette);
}

void AC_DroneShowManager::PixelGridState::get_position(uint16_t& row_out, uint16_t& column_out) const {
    row_out = _row;
    column_out = _column;
}

bool AC_DroneShowManager::PixelGridState::set_row(uint32_t new_row) {
    if (new_row < 4096) {
        _row = static_cast<uint16_t>(new_row);
        return true;
    } else {
        return false;
    }
}

bool AC_DroneShowManager::PixelGridState::set_column(uint32_t new_column) {
    if (new_column < 4096) {
        _column = static_cast<uint16_t>(new_column);
        return true;
    } else {
        return false;
    }
}

bool AC_DroneShowManager::PixelGridState::set_position(uint32_t new_row, uint32_t new_column) {
    return set_row(new_row) && set_column(new_column);
}

void AC_DroneShowManager::PixelGridState::set_color_by_bit(bool bit) {
    size_t size = sb_color_palette_size(&_palette);
    switch (size) {
        case 0:
            color = bit ? SB_COLOR_WHITE : SB_COLOR_BLACK;
            break;
        case 1:
            color = bit ? sb_color_palette_get_color(&_palette, 0) : SB_COLOR_BLACK;
            break;
        default:
            color = sb_color_palette_get_color(&_palette, bit ? 1 : 0);
            break;
    }
}

void AC_DroneShowManager::PixelGridState::set_color_by_palette_index(uint8_t index) {
    color = sb_color_palette_get_color(&_palette, index);
}

void AC_DroneShowManager::PixelGridState::reset() {
    if (!set_position(0, 0)) {
        // should not happen
    }
    color = SB_COLOR_BLACK;
    sb_color_palette_clear(&_palette);
}

bool AC_DroneShowManager::PixelGridState::update_from_gcs_light_control_block(
    const sb_gcs_light_control_setup_t& spec) {
    // Copy the palette. No actual copy if the palette was only a view
    if (sb_color_palette_update(&_palette, &spec.palette) != SB_SUCCESS) {
        return false;
    }

    // Note the order - we need row/column so we use Y/X
    if (!set_position(spec.coords.y, spec.coords.x)) {
        return false;
    }

    return true;
}

bool AC_DroneShowManager::PixelGridState::update_from_binary_file_in_memory(
    uint8_t* show_data, size_t length) {
    sb_error_t retval;
    sb_gcs_light_control_setup_t spec;
    bool success = false;

    retval = sb_gcs_light_control_setup_init(&spec);
    if (retval != SB_SUCCESS) {
        return false;
    }

    retval = sb_gcs_light_control_setup_update_from_binary_file_in_memory(&spec, show_data, length);
    if (retval == SB_ENOENT) {
        // This is okay, no such block
    } else if (retval != SB_SUCCESS) {
        goto cleanup;
    }

    // Ownership of the palette is transferred to the pixel grid state here
    success = update_from_gcs_light_control_block(spec);

cleanup:
    sb_gcs_light_control_setup_destroy(&spec);

    return success;
}
