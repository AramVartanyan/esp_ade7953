/*

ADE7953 energy metering IC driver — SDK-independent core. Version 2.4.

Supports several Shelly hardware variants (channel mapping differs per board),
the chip's hardware no-load detection, and magnitude-based power readings that
are independent of current-sensor polarity. Only the platform-specific I2C code
lives in ade7953_i2c.c.

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

#include "ade7953.h"
#include "ade7953_i2c.h"

/* Pull in CONFIG_IDF_TARGET_* under the ESP-IDF family (ESP-IDF / ESP8266_RTOS_SDK).
 * esp-open-rtos has no sdkconfig.h, so the include is skipped there. */
#ifdef __has_include
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP8266) || defined(CONFIG_IDF_TARGET_ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char *TAG = "ade7953";
#define ADE_LOGE(fmt, ...) ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#define ADE_LOGI(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
//esp-open-rtos
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#define ADE_LOGE(fmt, ...) printf("[ade7953] " fmt "\n", ##__VA_ARGS__)
#define ADE_LOGI(fmt, ...) printf("[ade7953] " fmt "\n", ##__VA_ARGS__)
#endif

/* A short settle delay after each transaction, as in the original driver. */
#define ADE_SETTLE()       vTaskDelay(1 / portTICK_PERIOD_MS)
#define ADE_DELAY_MS(ms)   vTaskDelay((ms) / portTICK_PERIOD_MS)

/* --- Per-channel measurement registers, indexed by ADE channel (0 = A, 1 = B).
 * The SIGNED quantities use the 0x3xx (32-bit) variants: the chip sign-extends
 * them, so a plain (int32_t) cast gives the correct signed value. */
static const uint16_t ADE7953_R_IRMS[2]   = { 0x31A, 0x31B }; // IRMS  A / B (unsigned)
static const uint16_t ADE7953_R_WATT[2]   = { 0x312, 0x313 }; // active   power A / B (signed)
static const uint16_t ADE7953_R_VA[2]     = { 0x310, 0x311 }; // apparent power A / B (signed)
static const uint16_t ADE7953_R_VAR[2]    = { 0x314, 0x315 }; // reactive power A / B (signed)
static const uint16_t ADE7953_R_ENERGY[2] = { 0x31E, 0x31F }; // active energy A / B (signed)
static const uint16_t ADE7953_R_PF[2]     = { 0x10A, 0x10B }; // power factor  A / B (signed, 16-bit)

/* --- Common measurement registers --- */
#define ADE7953_U_RMS  0x31C //RMS voltage  - unsigned, 32-bit
#define ADE7953_PERIOD 0x10E //line period  - unsigned, 16-bit

/* --- Control / configuration registers --- */
#define ADE7953_CONFIG       0x102 //Configuration register
#define ADE7953_CONFIG_SWRST (1u << 7) //CONFIG: software reset
#define ADE7953_UNLOCK       0x0FE //Unlock key for register 0x120
#define ADE7953_OPTIMUM      0x120 //"Reserved" — must be set to 0x30 (datasheet)
#define ADE7953_LCYCMODE     0x004 //Line-cycle accumulation mode (RSTREAD = bit 6)
#define ADE7953_DISNOLOAD    0x001 //No-load detection disable bits [2:0]
#define ADE7953_AP_NOLOAD    0x303 //Active-power no-load threshold
#define ADE7953_VAR_NOLOAD   0x304 //Reactive-power no-load threshold
#define ADE7953_ACCMODE      0x301 //Accumulation mode (APSIGN/VARSIGN bits)
#define ADE7953_IRQSTATA     0x32D //Interrupt status A
#define ADE7953_IRQSTATA_RST (1u << 20) //IRQSTATA: reset-complete flag
#define ADE7953_VERSION      0x702 //Silicon version (8-bit) — comms check

/* --- Calibration registers --- */
#define ADE7953_PGA_V        0x007 //Voltage channel PGA gain
#define ADE7953_PGA_IA       0x008 //Current Channel A PGA gain
#define ADE7953_PGA_IB       0x009 //Current Channel B PGA gain
#define ADE7953_VRMSOS       0x388 //Voltage RMS offset
#define ADE7953_AIRMSOS      0x386 //Current Channel A RMS offset
#define ADE7953_BIRMSOS      0x392 //Current Channel B RMS offset

/* Empirical software-scaling references (uncalibrated defaults; tune for accuracy). */
#define ADE7953_PREF   1540    //1540 (0,066W)
#define ADE7953_UREF   0x10B   //0x10B, 267 (Vx100)
#define ADE7953_IREF   0xAAA   //(0x181, 385) 2730 (Ax1000)
/* No-load cutoff for getcurrent(), in CALIBRATED units (A x100): idle IRMS noise
 * below this reads as exactly 0. Keep it SMALL vs any real load. (The former 75
 * lived in the pre-calibration inflated scale — once IREF was calibrated it
 * zeroed real currents, e.g. 730 mA -> 72 < 75. See getcurrent.) 2 = ~20 mA:
 * just above the measured reading noise (~+/-3 mA p-p) and the old ~16 mA cutoff. */
#define ADE7953_NO_LOAD_AX100  2

/* Period register clock: 223.75 kHz (datasheet, Period Measurement). */
#define ADE7953_PERIOD_CLK_HZ  223750UL

/* Hardware no-load threshold. The datasheet default is 58393; half of it lets
 * the chip report down to lower (~5 W) loads (value from Tasmota). */
#define ADE7953_NO_LOAD_THRESHOLD  29196

/* Reset-flag poll budget at init (10 ms per step). */
#define ADE7953_RESET_POLL_STEPS  20

static ade7953_model_t s_model = ADE7953_MODEL_SHELLY_25;

/* Active scaling references for getcurrent()/getvoltage() (raw -> A x100 / V x100).
 * Set at init from the per-model table below, or overridden via
 * ade7953_config_t.current_ref / .voltage_ref. */
static uint16_t s_iref = ADE7953_IREF;
static uint16_t s_uref = ADE7953_UREF;

/* Per-model calibration references, found by the calibration procedure (known
 * load / known mains voltage, read back over HomeKit). The application selects
 * ONLY the hardware model (ade7953_init's `model`); the matching iref/uref are
 * applied automatically. A non-zero cfg->current_ref / cfg->voltage_ref still
 * overrides (e.g. during a calibration run). */
static const struct { uint16_t iref; uint16_t uref; } s_model_cal[] = {
    [ADE7953_MODEL_SHELLY_25]       = { 60,   240 },  /* I & U calibrated on unit 1, 2026-07-15 */
    [ADE7953_MODEL_SHELLY_PLUS_2PM_V015] = { 2730, 267 },  /* v0.1.5 TODO (channels like 2.5) */
    [ADE7953_MODEL_SHELLY_PLUS_2PM_V019] = { 2730, 267 },  /* v0.1.9 TODO (mapping unverified) */
    [ADE7953_MODEL_SHELLY_2PM_GEN3] = { 2730, 267 },  /* TODO */
    [ADE7953_MODEL_SHELLY_2PM_GEN4] = { 2730, 267 },  /* TODO */
    [ADE7953_MODEL_SHELLY_EM]       = { 2730, 267 },  /* TODO */
};

/* Cached, abs() active power magnitude. Refreshed by ade7953getdata(). */
static uint32_t ade7953_active_power1 = 0;
static uint32_t ade7953_active_power2 = 0;

/* The data width (in bytes) is encoded in the register address:
 *   0x0xx -> 1 byte, 0x1xx -> 2, 0x2xx -> 3, 0x3xx -> 4.
 * Registers at/above 0x700 (VERSION 0x702, EX_REF 0x800) break this rule and
 * are 8-bit. This returns (byte_count - 1).
 * https://www.microchip.com/forums/m1011488.aspx */
static uint8_t ade7953regsize(uint16_t reg_a) {
    if (reg_a >= 0x700) return 0;   // VERSION / EX_REF: 8-bit (1 byte)
    return (uint8_t)((reg_a >> 8) & 0x0F);
}

static uint32_t ade7953read(uint16_t reg_a) {
    uint8_t len = ade7953regsize(reg_a) + 1;
    uint8_t byte[4] = {0, 0, 0, 0};

    if (!ade7953_i2c_read_reg(reg_a, byte, len)) {
        ADE_LOGE("read register 0x%03X failed", reg_a);
        return 0;
    }

    /* Bytes arrive MSB first: byte[0] is the most significant. */
    uint32_t response = 0;
    for (uint8_t i = 0; i < len; i++) {
        response = (response << 8) | byte[i];
    }

    ADE_SETTLE();
    return response;
}

static bool ade7953write(uint16_t reg_a, uint32_t data) {
    uint8_t len = ade7953regsize(reg_a) + 1;
    uint8_t byte[4] = {0, 0, 0, 0};

    /* MSB first: byte[0] holds the most significant byte of the value. */
    for (uint8_t i = 0; i < len; i++) {
        byte[i] = (uint8_t)((data >> (8 * (len - 1 - i))) & 0xFF);
    }

    bool ok = ade7953_i2c_write_reg(reg_a, byte, len);
    if (!ok) {
        ADE_LOGE("write register 0x%03X failed", reg_a);
    }

    ADE_SETTLE();
    return ok;
}

/* Map a logical channel (1 or 2) to the ADE channel index (0 = A, 1 = B).
 * Shelly 2.5 has its two current sensors wired to the opposite ADE channels
 * compared to the 2PM / EM / Pro family, so 2.5 is swapped. The Shelly Plus 2PM
 * PCB v0.1.5 was measured to be wired the SAME (swapped) way as the 2.5, so
 * _V015 swaps too. v0.1.9 (_V019) mapping is unverified (left direct). */
static int ade7953_ade_channel(uint8_t channel) {
    int is_b = (channel == 2);                 // direct: ch1 -> A, ch2 -> B
    if (s_model == ADE7953_MODEL_SHELLY_25 ||
        s_model == ADE7953_MODEL_SHELLY_PLUS_2PM_V015) {
        is_b = !is_b;                          // 2.5 & Plus 2PM v0.1.5: ch1 -> B, ch2 -> A
    }
    return is_b ? 1 : 0;
}

/* |v| as an unsigned value, safe for INT32_MIN. */
static uint32_t ade7953_abs32(int32_t v) {
    return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

bool ade7953_init(uint8_t bus, uint8_t addr, ade7953_model_t model, const ade7953_config_t *cfg) {
    s_model = model;
    {   /* Per-model calibration; a non-zero cfg override wins. */
        int n  = (int)(sizeof(s_model_cal) / sizeof(s_model_cal[0]));
        int mi = ((int)model >= 0 && (int)model < n) ? (int)model
                                                     : (int)ADE7953_MODEL_SHELLY_25;
        s_iref = (cfg && cfg->current_ref) ? cfg->current_ref : s_model_cal[mi].iref;
        s_uref = (cfg && cfg->voltage_ref) ? cfg->voltage_ref : s_model_cal[mi].uref;
    }

    if (!ade7953_i2c_setup(bus, addr)) {
        ADE_LOGE("I2C device setup failed");
        return false;
    }

    /* Allow the power-up sequence to finish (datasheet recommends >=100 ms when
     * the reset IRQ pin is not used), then verify communication by reading the
     * silicon version register. */
    ADE_DELAY_MS(100);
    uint32_t version = ade7953read(ADE7953_VERSION);
    if (version == 0 || version == 0xFF) {
        ADE_LOGE("ADE7953 not responding (version=0x%02X)", (unsigned)version);
        return false;
    }
    ADE_LOGI("ADE7953 silicon version 0x%02X, model %d", (unsigned)version, (int)model);

    /* Software reset, then wait for the reset-complete flag in IRQSTATA. The
     * SWRST write is not acknowledged (the chip resets immediately), so its
     * return value is ignored. */
    ade7953write(ADE7953_CONFIG, ADE7953_CONFIG_SWRST);
    ADE_DELAY_MS(10);
    bool reset_done = false;
    for (int i = 0; i < ADE7953_RESET_POLL_STEPS; i++) {
        if (ade7953read(ADE7953_IRQSTATA) & ADE7953_IRQSTATA_RST) {
            reset_done = true;
            break;
        }
        ADE_DELAY_MS(10);
    }
    if (!reset_done) {
        ADE_LOGE("reset flag not seen; continuing anyway");
    }

    /* Clear COMM_LOCK (bit 15) to lock the interface to I2C; HPFEN (bit 2) stays
     * enabled. CONFIG (0x102) default is 0x8004. */
    ade7953write(ADE7953_CONFIG, 0x0004);
    /* Required register setting (datasheet): unlock 0x120, then write 0x30. */
    ade7953write(ADE7953_UNLOCK, 0x00AD);
    ade7953write(ADE7953_OPTIMUM, 0x0030);

    /* Ensure RSTREAD (LCYCMODE bit 6) is set so an energy read resets the
     * accumulator. This is the power-up default; written explicitly for safety. */
    ade7953write(ADE7953_LCYCMODE, 0x40);

    /* Hardware no-load detection: below the threshold the chip zeroes the
     * power/energy registers. Disable first to change the thresholds, then
     * re-enable (datasheet / Tasmota procedure). */
    ade7953write(ADE7953_DISNOLOAD, 0x07);
    ade7953write(ADE7953_AP_NOLOAD, ADE7953_NO_LOAD_THRESHOLD);
    ade7953write(ADE7953_VAR_NOLOAD, ADE7953_NO_LOAD_THRESHOLD);
    ade7953write(ADE7953_DISNOLOAD, 0x00);

    /* Optional chip-side calibration: PGA gains and RMS offsets. A gain of
     * GAIN_1 (0) or an offset of 0 is left at the chip default. */
    if (cfg) {
        if (cfg->v_gain  != ADE7953_PGA_GAIN_1) ade7953write(ADE7953_PGA_V,  cfg->v_gain);
        if (cfg->ia_gain != ADE7953_PGA_GAIN_1) ade7953write(ADE7953_PGA_IA, cfg->ia_gain);
        if (cfg->ib_gain != ADE7953_PGA_GAIN_1) ade7953write(ADE7953_PGA_IB, cfg->ib_gain);
        if (cfg->v_offset)  ade7953write(ADE7953_VRMSOS,  (uint32_t)cfg->v_offset);
        if (cfg->ia_offset) ade7953write(ADE7953_AIRMSOS, (uint32_t)cfg->ia_offset);
        if (cfg->ib_offset) ade7953write(ADE7953_BIRMSOS, (uint32_t)cfg->ib_offset);
    }

    /* Read the energy registers once to clear them; the first post-init reading
     * may be inaccurate, so it is discarded here. */
    ade7953read(ADE7953_R_ENERGY[0]);
    ade7953read(ADE7953_R_ENERGY[1]);

    return true;
}

/* Refresh the cached active power for both channels (magnitude). */
void ade7953getdata(void) {
    ade7953_active_power1 = ade7953_abs32((int32_t)ade7953read(ADE7953_R_WATT[ade7953_ade_channel(1)]));
    ade7953_active_power2 = ade7953_abs32((int32_t)ade7953read(ADE7953_R_WATT[ade7953_ade_channel(2)]));
}

uint16_t ade7953_getvoltage(void) {
    uint32_t voltage_rms_readout = ade7953read(ADE7953_U_RMS);
    return (uint16_t)((voltage_rms_readout * s_uref) >> 16);
}

uint16_t ade7953_getcurrent(uint8_t channel) {
    if (channel < 1 || channel > 2) return 0;
    int ch = ade7953_ade_channel(channel);
    uint16_t c_rms_return = (uint16_t)(((ade7953read(ADE7953_R_IRMS[ch]) * s_iref) & 0xFFFF0000) >> 16);
    if (c_rms_return < ADE7953_NO_LOAD_AX100) {   // no-load: report exactly 0 when idle
        c_rms_return = 0;
    }
    return c_rms_return;
}
//the expected reading on the IRMSA and IRMSB register is 9032007d.
//current value (without calibration) = (34.0 / 9032007) * IRMS-register-reading;

uint32_t ade7953_getactivepower(uint8_t channel) {
    /* Magnitude (abs); the chip's no-load detection zeroes it at idle. */
    return (channel < 2 ? ade7953_active_power1 : ade7953_active_power2) / ADE7953_PREF;
}

uint32_t ade7953_getenergy(uint8_t channel) { // Read-with-reset: count restarts after this read.
    if (channel < 1 || channel > 2) return 0;
    int ch = ade7953_ade_channel(channel);
    /* Read fresh: the register resets on every read, so the value is the energy
     * accumulated since the previous call. 64-bit avoids overflow. */
    uint32_t mag = ade7953_abs32((int32_t)ade7953read(ADE7953_R_ENERGY[ch]));
    uint64_t ws = ((uint64_t)mag * ADE7953_PREF) / 1000;   // Ws (/3600 for Wh)
    return (uint32_t)ws;
}

uint32_t ade7953_getapparentpower(uint8_t channel) {
    if (channel < 1 || channel > 2) return 0;
    int ch = ade7953_ade_channel(channel);
    return ade7953_abs32((int32_t)ade7953read(ADE7953_R_VA[ch])) / ADE7953_PREF;
}

uint32_t ade7953_getreactivepower(uint8_t channel) {
    if (channel < 1 || channel > 2) return 0;
    int ch = ade7953_ade_channel(channel);
    return ade7953_abs32((int32_t)ade7953read(ADE7953_R_VAR[ch])) / ADE7953_PREF;
}

int16_t ade7953_getpowerfactor(uint8_t channel) {
    if (channel < 1 || channel > 2) return 0;
    int ch = ade7953_ade_channel(channel);
    /* PF register is signed 16-bit, full scale +-32768 == +-1.0.
     * Returned as PF x1000 (range -1000..+1000). Formula-exact, no calibration. */
    int16_t pf = (int16_t)ade7953read(ADE7953_R_PF[ch]);
    return (int16_t)(((int32_t)pf * 1000) / 32768);
}

uint16_t ade7953_getfrequency(void) {
    /* Line frequency from the period register: f = 223.75 kHz / (PERIOD + 1).
     * Returned as Hz x100 (format Hz*100). Formula-exact, no calibration. */
    uint32_t period = ade7953read(ADE7953_PERIOD) & 0xFFFF;
    return (uint16_t)((ADE7953_PERIOD_CLK_HZ * 100UL) / (period + 1));
}

int8_t ade7953_get_apower_sign(uint8_t channel) {
    /* Active-power direction from ACCMODE (APSIGN_A = bit 10, APSIGN_B = bit 11).
     * +1 = import (consuming), -1 = export. Meaningful for net-metering (EM). */
    if (channel < 1 || channel > 2) return 1;
    int ch = ade7953_ade_channel(channel);
    uint32_t accmode = ade7953read(ADE7953_ACCMODE);
    return (accmode & (1u << (10 + ch))) ? -1 : 1;
}
