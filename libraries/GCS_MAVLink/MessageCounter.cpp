/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 Message counter for RTCMv3 messages so we can track how many RTCMv3
 correction messages were received recently.
*/

#include <AP_HAL/AP_HAL.h>
#include "MessageCounter.h"

static uint32_t get_timestamp_in_seconds();

MessageCounter::MessageCounter() : _sum(0), _hadMessage(false), _lastIndex(0) {
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        _messages[i] = 0;
    }
}

int16_t MessageCounter::get_count() const {
    return _hadMessage ? (_sum > 32767 ? 32767 : _sum) : -1;
}

void MessageCounter::notify() {
    uint8_t index = get_timestamp_in_seconds() % WINDOW_SIZE;

    while (_lastIndex != index) {
        _lastIndex = (_lastIndex + 1) % WINDOW_SIZE;
        _sum -= _messages[_lastIndex];
        _messages[_lastIndex] = 0;
    }

    _messages[index]++;
    _sum++;

    _hadMessage = true;
}

/// @brief Retrieves the current timestamp in seconds
static uint32_t get_timestamp_in_seconds()
{
    return static_cast<uint32_t>(AP_HAL::millis() / 1000);
}
