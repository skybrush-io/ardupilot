#include <string.h>

#include "AC_DroneShowManager.h"

void AC_DroneShowManager::get_show_id(uint8_t* show_id) const
{
    memcpy(show_id, _show_metadata.show_id, SB_SHOW_METADATA_SHOW_ID_LENGTH);
}

uint16_t AC_DroneShowManager::get_drone_index_in_show() const
{
    return _show_metadata.drone_index;
}

void AC_DroneShowManager::_update_show_metadata_params()
{
    uint8_t show_id[SB_SHOW_METADATA_SHOW_ID_LENGTH];

    get_show_id(show_id);

    _update_readonly_param(_params.drone_index_in_show, "SHOW_IDX", get_drone_index_in_show());
    _update_readonly_param(_params.show_id_hi, "SHOW_ID_HI", (show_id[0] << 8) | show_id[1]);
    _update_readonly_param(_params.show_id_lo, "SHOW_ID_LO", (show_id[2] << 8) | show_id[3]);
}
