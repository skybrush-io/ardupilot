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
#pragma once

#include <cstdint>

class MessageCounter {
public:
    /// @brief  Creates a new RTCM3 message counter
    MessageCounter();

    /// @brief  Notifies the counter that a message was received
    void notify();

    /**
     * @brief  Returns the number of messages received recently.
     * 
     * The counter saturates at 32767 messages in the time window being monitored.
     * 
     * @return The number of messages received recently, or -1 if no messages
     *         have been received ever. This allows us to distinguish between
     *         no messages received and no messages received _recently_.
     */
    int16_t get_count();

private:
    /// @brief The size of the time window being monitored
    static constexpr uint8_t WINDOW_SIZE = 5;

    /// @brief Stores the number of messages received in the last seconds.
    uint16_t _messages[WINDOW_SIZE] = {0};

    /// @brief Stores the sum of all items in the messages array.
    uint16_t _sum;

    /// @brief Stores whether at least one event has been logged.
    bool _had_message;

    /// @brief Timestamp when the counters were updated for the last time.
    uint32_t _last_updated_at;

    /// @brief Index of the last cell that was written to in _messages.
    uint8_t _last_index;

    void _clear();
    void _flush_counters();
};
