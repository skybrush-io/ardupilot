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

MessageCounter::MessageCounter() : _sum(0), _had_message(false), _last_updated_at(0), _last_index(0) {
    _clear();
}

int16_t MessageCounter::get_count() {
    if (!_had_message) {
        return -1;
    }
    
    if (_sum > 0) {
        _flush_counters();
    }

    return _sum > 32767 ? 32767 : _sum;
}

void MessageCounter::notify() {
    _flush_counters();

    _messages[_last_index]++;
    _sum++;

    _had_message = true;
}

void MessageCounter::_clear()
{
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        _messages[i] = 0;
    }
    _last_index = 0;
    _sum = 0;
}

void MessageCounter::_flush_counters()
{
    uint32_t now = get_timestamp_in_seconds();
    uint32_t diff = now - _last_updated_at;

    if (diff >= WINDOW_SIZE) {
        _clear();
    } else {
        while (diff > 0) {
            _last_index = (_last_index + 1) % WINDOW_SIZE;
            _sum -= _messages[_last_index];
            _messages[_last_index] = 0;
            diff--;
        }
    }

    _last_updated_at = now;
}

/// @brief Retrieves the current timestamp in seconds
static uint32_t get_timestamp_in_seconds()
{
    return static_cast<uint32_t>(AP_HAL::millis() / 1000);
}
