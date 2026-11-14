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
