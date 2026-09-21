/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "AP_RCProtocol_PPMSum.h"

#if AP_RCPROTOCOL_PPMSUM_ENABLED

/*
  process a PPM-sum pulse of the given width
 */
void AP_RCProtocol_PPMSum::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    // Stardard PPM details:
    //
    //   A single PPMSum frame comes in short, fixed length impulses (~100 us)
    //   and variable length gaps (~1000-2000 us) between them, coding the
    //   actual value of each channel. Impulses come before first channel gap,
    //   between channel gaps and after last channel gap as well (so 8 channels
    //   have 9 impulses in a frame).
    //
    //   To increase temporal resolution, the channel value is coded in the sum
    //   of the impulse length and the gap before or after it, depending on
    //   whether the first detected signal is a falling or rising edge.
    //
    //   If it is falling, width_s0 will contain actual gap and width_s1 will
    //   contain the impulse length after it. If it is rising, width_s0 will
    //   contain the impulse length and width_s1 will contain the gap after it.
    //
    //   This function should work equally for both cases.
    //
    //   A major problem with PPMSum is when all channel values are high, the
    //   typically larger gap (~>2300 us) between frames becomes so small that
    //   it gets indistinguishable from further channel values.
    //
    // Notes for Collmot's hacked FrSky v8R7II -> PPMSum receivers:
    //
    //   In our receivers impulse length is around 90-100 us before all channels
    //   and 32 us after the 8th channel. This shows the end of the frame.
    //
    //   frames come at 18ms, so main frequency is 55.5 Hz
    //
    //   max data length within a frame is thus approximately
    //   8 * 2000us + 8 * 100us + 32us = 16832 us,
    //   so only 1168 us is left until next frame if channel values are high,
    //   So detection of 32us pulse is critical!!!


#ifdef COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
    // we need to get the short ~32us impulse that signals the end of the frame,
    // so we need a static variable and we also need to assume that
    // process_pulse gets called only once for every pulse.
    static uint32_t last_width_s1 = 100;
#endif  // COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE

    if (width_s0 == 0 || width_s1 == 0) {
        //invalid data: reset frame
        ppm_state._channel_counter = -1;

#ifdef COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
        last_width_s1 = 100;
#endif  // COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE

        return;
    }

    uint32_t width_usec = width_s0 + width_s1;

#ifdef COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
    if (width_usec >= 2700 || width_s0 < 50 || last_width_s1 < 50) {
        // a long gap+pulse or a short last impulse indicates frame ending
        // Reset the channel counter so next pulse is channel 0
#else
    if (width_usec >= 2700) {
        // a long pulse indicates the end of a frame. Reset the
        // channel counter so next pulse is channel 0
#endif  // COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
        if (ppm_state._channel_counter >= MIN_RCIN_CHANNELS) {
            add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        }
        ppm_state._channel_counter = 0;

#ifdef COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
        last_width_s1 = width_s1;
#endif  // COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE

        return;
    }

#ifdef COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE
    last_width_s1 = width_s1;
#endif  // COLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE

    if (ppm_state._channel_counter == -1) {
        // we are not synchronised
        return;
    }

    /*
      we limit inputs to between 700usec and 2300usec. This allows us
      to decode SBUS on the same pin, as SBUS will have a maximum
      pulse width of 100usec
     */
    if (width_usec > 700 && width_usec < 2300) {
        // take a reading for the current channel
        // buffer these
        ppm_state._pulse_capt[ppm_state._channel_counter] = width_usec;

        // move to next channel
        ppm_state._channel_counter++;
    }

    // if we have reached the maximum supported channels then
    // mark as unsynchronised, so we wait for a wide pulse
    if (ppm_state._channel_counter >= MAX_RCIN_CHANNELS) {
        add_input(ppm_state._channel_counter, ppm_state._pulse_capt, false);
        ppm_state._channel_counter = -1;
    }
}

#endif  // AP_RCPROTOCOL_PPMSUM_ENABLED
