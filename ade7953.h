/*

ADE7953 energy metering IC driver. Version 2.0.

Dual-SDK: works under both esp-open-rtos (ESP8266) and the ESP-IDF family
(ESP-IDF v5.x for ESP32, and ESP8266_RTOS_SDK). The register/protocol logic is
SDK-independent; only the low-level I2C transport differs and is abstracted in
ade7953_i2c.c.

The I2C bus is initialised by the caller; this driver only performs register
transactions on the given bus/port (same contract as the original driver).

Copyright (C) 2026 by Aram Vartanian <https://github.com/AramVartanyan/>

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

#ifndef ade7953_h
#define ade7953_h

#include <stdint.h>
#include <stdbool.h>

/* Library version. */
#define ESP_ADE7953_VERSION "2.0"

#ifdef __cplusplus
extern "C"
{
#endif

/* Supported hardware. The board determines how the two current sensors map to
 * the chip's A/B channels: Shelly 2.5 has them swapped relative to the others.
 * EM is a net-metering device, so its power direction (import/export) is read
 * from the chip — see ade7953_get_apower_sign(). The gen3/gen4 2PM entries
 * assume the same (direct) mapping as the Plus 2PM; verify if it differs.
 *
 * HARDWARE NOTE: the "direct" mapping for SHELLY_PLUS_2PM was taken from the
 * ESPHome device data. On a real Shelly Plus 2PM PCB v0.1.5, however, the
 * channels were measured to be SWAPPED — i.e. wired like the old Shelly 2.5
 * (relay 1 / O1 -> chip channel B, relay 2 / O2 -> chip channel A). It may have
 * changed on the newer v0.1.9 (unverified). Until confirmed, firmware for the
 * v0.1.5 board should select ADE7953_MODEL_SHELLY_25 to get the correct
 * channel↔relay mapping, even though the device is a 2PM. */
typedef enum {
    ADE7953_MODEL_SHELLY_25 = 0,      // channels A/B swapped vs the others
    ADE7953_MODEL_SHELLY_PLUS_2PM,    // current development target
    ADE7953_MODEL_SHELLY_2PM_GEN3,
    ADE7953_MODEL_SHELLY_2PM_GEN4,
    ADE7953_MODEL_SHELLY_EM,          // net-metering (signed power direction)
} ade7953_model_t;

/* PGA (programmable gain amplifier) settings for the voltage and current
 * channels (datasheet Table 6). */
typedef enum {
    ADE7953_PGA_GAIN_1  = 0x00,
    ADE7953_PGA_GAIN_2  = 0x01,
    ADE7953_PGA_GAIN_4  = 0x02,
    ADE7953_PGA_GAIN_8  = 0x03,
    ADE7953_PGA_GAIN_16 = 0x04,
    ADE7953_PGA_GAIN_22 = 0x05,
} ade7953_pga_gain_t;

/* Optional chip-side calibration applied at init. Pass NULL to ade7953_init()
 * to keep the power-up defaults (gain 1, no RMS offset). A gain of
 * ADE7953_PGA_GAIN_1 (0) or an offset of 0 is left at the chip default.
 * These fields address the chip's A/B channels directly (not the logical
 * channel — see the model note above for the mapping).
 * Note: changing a PGA gain scales the raw readings, so the software scaling
 * (ADE7953_IREF / ADE7953_UREF) must be retuned accordingly. */
typedef struct {
    ade7953_pga_gain_t v_gain;    // voltage channel PGA gain
    ade7953_pga_gain_t ia_gain;   // current Channel A PGA gain
    ade7953_pga_gain_t ib_gain;   // current Channel B PGA gain
    int32_t v_offset;             // VRMSOS  (0x388) register value, signed
    int32_t ia_offset;            // AIRMSOS (0x386) register value, signed
    int32_t ib_offset;            // BIRMSOS (0x392) register value, signed
    uint16_t current_ref;         // IREF override for ade7953_getcurrent() scaling
                                  // (raw -> A x100); 0 = keep the library default
} ade7953_config_t;

/* Setup the ADE7953's address and required registers. Call first!
 * Performs a software reset, waits for the chip's reset flag, verifies
 * communication (VERSION register), applies the required register settings,
 * enables hardware no-load detection, programs any calibration from cfg, and
 * clears the energy accumulators.
 *
 * The I2C bus must already be initialised by the caller:
 *   - ESP-IDF / ESP8266_RTOS_SDK: i2c_param_config() + i2c_driver_install()
 *     on the given port (legacy "driver/i2c.h").
 *   - esp-open-rtos: i2c_init() on the given bus number.
 *
 *   bus   - I2C port / bus number the ADE7953 is wired to (I2C_NUM_0 -> 0).
 *   addr  - 7-bit I2C address (0x38 by default).
 *   model - the Shelly hardware (sets the channel mapping).
 *   cfg   - optional calibration (PGA gains + RMS offsets), or NULL for defaults.
 * Returns true on success. */
bool ade7953_init(uint8_t bus, uint8_t addr, ade7953_model_t model, const ade7953_config_t *cfg);

/* All getters take a LOGICAL channel: 1 = relay 1, 2 = relay 2. The library
 * maps it to the correct chip channel (A/B) based on the configured model. */

/* Refresh the cached active power for both channels. Call this before
 * ade7953_getactivepower(); the cached value is current as of this call. */
void ade7953getdata(void);

/* Instantaneous getters (each reads the chip directly; self-contained). */
uint16_t ade7953_getcurrent(uint8_t channel);   // A x100 (divide by 100 for A)
uint16_t ade7953_getvoltage(void);              // V x100 (divide by 100 for V)

/* Active power magnitude (abs — independent of sensor polarity). The chip's
 * no-load detection zeroes it at idle. Needs a prior ade7953getdata(). */
uint32_t ade7953_getactivepower(uint8_t channel);

/* Each energy reading resets its register: the value is the energy accumulated
 * since the previous ade7953_getenergy() call (read-with-reset). */
uint32_t ade7953_getenergy(uint8_t channel);    // Ws (divide by 3600 for Wh)

/* Additional parameters (untested on hardware). Apparent/reactive power return
 * a magnitude and share the active-power calibration (PREF), so they are
 * proportional but uncalibrated. Power factor and frequency are formula-exact. */
uint32_t ade7953_getapparentpower(uint8_t channel); // VA  (uncalibrated)
uint32_t ade7953_getreactivepower(uint8_t channel); // VAR magnitude (uncalibrated)
int16_t  ade7953_getpowerfactor(uint8_t channel);   // PF x1000 (-1000..+1000)
uint16_t ade7953_getfrequency(void);                // line frequency, Hz x100

/* Active-power direction from the chip's ACCMODE register: +1 = import
 * (consuming), -1 = export. Only meaningful for net-metering hardware
 * (ADE7953_MODEL_SHELLY_EM); other models read power as a magnitude. */
int8_t   ade7953_get_apower_sign(uint8_t channel);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif //ade7953_h
