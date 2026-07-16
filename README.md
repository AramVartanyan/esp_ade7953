# esp_ade7953 — v2.5

Driver for the **ADE7953** single-phase energy metering IC (voltage, current,
active power and energy over I²C).

It is a restructured version of the original ESP8266 / esp-open-rtos driver,
verified against the ADE7953 datasheet and cross-checked against the Tasmota,
mongoose-os and MacWyznawca implementations. The low-level I²C transport is
abstracted so the same code builds on multiple SDKs, and the channel mapping is
selectable per Shelly hardware model.

## Supported hardware

Pass the model to `ade7953_init()`; it sets how the two current sensors map to
the chip's A/B channels (Shelly 2.5 is swapped relative to the rest):

| `ade7953_model_t` | Mapping | Notes |
|---|---|---|
| `ADE7953_MODEL_SHELLY_25`        | ch1→B, ch2→A | swapped |
| `ADE7953_MODEL_SHELLY_PLUS_2PM`  | ch1→A, ch2→B | current target |
| `ADE7953_MODEL_SHELLY_2PM_GEN3`  | ch1→A, ch2→B | verify if it differs |
| `ADE7953_MODEL_SHELLY_2PM_GEN4`  | ch1→A, ch2→B | verify if it differs |
| `ADE7953_MODEL_SHELLY_EM`        | ch1→A, ch2→B | net-metering; signed direction via `ade7953_get_apower_sign()` |

## Layout

| File | Purpose |
|------|---------|
| `ade7953.h` | Public API (SDK-independent) |
| `ade7953.c` | Core logic: register encode/decode, init sequence, getters |
| `ade7953_i2c.h` / `ade7953_i2c.c` | I²C transport HAL — the **only** SDK-specific code |
| `CMakeLists.txt` | ESP-IDF component registration |
| `component.mk` | ESP8266_RTOS_SDK / legacy make component |

The platform is detected with `#if defined(CONFIG_IDF_TARGET_ESP8266) ||
defined(CONFIG_IDF_TARGET_ESP32)` — the same convention used in `esp_tm1637`.
These macros come from `sdkconfig.h` (the ESP-IDF family); esp-open-rtos has no
such file, so the `#else` branch is taken. Each `.c` pulls in `sdkconfig.h`
first (guarded with `__has_include`), since the check sits above the first
ESP-IDF header.

* **ESP-IDF v5.x (ESP32) and ESP8266_RTOS_SDK** → the **legacy** `driver/i2c.h`
  API (`i2c_cmd_link_*`), the same I²C framework used by the esp-homekit platform
  (`esp_mfi_i2c.c`) and by `esp_aht`. This keeps the whole project on one I²C
  driver until esp-homekit migrates to the new `i2c_master` driver — at which
  point only `ade7953_i2c.c` needs to change.
* **esp-open-rtos** → the original `i2c_start/write/read/stop` sequence, kept 1:1.

## Bus ownership

The driver does **not** install the I²C bus — the caller initialises it once and
this driver runs register transactions on the given port. This matches the
original driver's contract and lets the ADE7953 share the bus (e.g. with the HAP
MFi co-processor) and keeps the pin choice in the application.

## Usage — ESP-IDF v5.x (legacy I²C driver)

```c
#include "driver/i2c.h"
#include "ade7953.h"

#define ADE_PORT   I2C_NUM_0

i2c_config_t conf = {
    .mode             = I2C_MODE_MASTER,
    .sda_io_num       = CONFIG_SDA_GPIO,
    .scl_io_num       = CONFIG_SCL_GPIO,
    .sda_pullup_en    = GPIO_PULLUP_ENABLE,
    .scl_pullup_en    = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000,          // ADE7953 max is 400 kHz
};
i2c_param_config(ADE_PORT, &conf);
i2c_driver_install(ADE_PORT, conf.mode, 0, 0, 0);

// NULL = power-up defaults (gain 1, no offset). ade7953_init() itself does a
// software reset, waits for the reset flag, verifies the VERSION register and
// enables hardware no-load detection.
if (!ade7953_init(ADE_PORT, 0x38, ADE7953_MODEL_SHELLY_PLUS_2PM, NULL)) {
    ESP_LOGE("app", "ADE7953 init failed");
}

uint16_t i1 = ade7953_getcurrent(1);     // A x100 on relay 1
uint16_t i2 = ade7953_getcurrent(2);     // A x100 on relay 2
```

### Optional calibration

```c
const ade7953_config_t cal = {
    .ia_gain = ADE7953_PGA_GAIN_8,       // current channels often need more gain
    .ib_gain = ADE7953_PGA_GAIN_8,
    .v_offset = 0, .ia_offset = -17, .ib_offset = -17,
};
ade7953_init(ADE_PORT, 0x38, ADE7953_MODEL_SHELLY_PLUS_2PM, &cal);
```
PGA gains and RMS offsets are programmed into the chip. Remember that raising a
PGA gain scales the raw reading, so the software constant (`ADE7953_IREF` /
`ADE7953_UREF`) must be retuned to match.

## Usage — esp-open-rtos

```c
#include <i2c/i2c.h>
#include <ade7953.h>

i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
ade7953_init(I2C_BUS, 0x38, ADE7953_MODEL_SHELLY_25, NULL);
```

## Measurements

| Getter | Unit | Calibration | Notes |
|--------|------|-------------|-------|
| `ade7953_getcurrent(ch)`       | A ×100   | `IREF` (tested) | 0 below no-load threshold |
| `ade7953_getvoltage()`         | V ×100   | `UREF` | |
| `ade7953_getactivepower(ch)`   | W (PREF) | `PREF` | magnitude; needs `ade7953getdata()` first |
| `ade7953_getenergy(ch)`        | Ws       | `PREF` | read-with-reset (value since last call) |
| `ade7953_getapparentpower(ch)` | VA       | `PREF` (uncalibrated) | magnitude |
| `ade7953_getreactivepower(ch)` | VAR      | `PREF` (uncalibrated) | magnitude |
| `ade7953_getpowerfactor(ch)`   | PF ×1000 | exact (`/32768`) | −1000…+1000 |
| `ade7953_getfrequency()`       | Hz ×100  | exact (`223.75 kHz / (period+1)`) | |
| `ade7953_get_apower_sign(ch)`  | +1 / −1  | — | power direction; EM only |

`ch`: logical channel 1 = relay 1, 2 = relay 2 (mapped to chip A/B per model).

## Notes

* **Init** does a software reset, polls the chip's reset-complete flag
  (`IRQSTATA`), verifies communication via the `VERSION` register, applies the
  required register settings, sets `RSTREAD` (energy reads reset), enables
  hardware no-load detection, programs any `cfg` calibration and clears the
  energy accumulators. No external start-up delay is needed.
* **Power is read as a magnitude** (`abs`). This makes the reading independent of
  current-sensor polarity, so no per-board sign tweak is needed (the approach
  Tasmota uses for all non-EM Shelly models). For net-metering (EM), the
  import/export direction is available via `ade7953_get_apower_sign()`.
* **Hardware no-load detection** (`AP_NOLOAD` / `VAR_NOLOAD`) lets the chip zero
  the power/energy registers at idle. `getcurrent()` keeps its own software
  threshold (the IRMS register is not affected by no-load detection).
* **Signed quantities** use the chip's sign-extended 32-bit register variants
  cast to a signed type (no manual sign extension).
* **Channel↔relay mapping** is set by the model. Still confirm against the real
  board which relay corresponds to which chip channel (e.g. energise relay 1 and
  check which channel reads current).
* Default I²C address is `0x38` (7-bit). Bus clock is whatever the caller
  configured (chip max 400 kHz; 100 kHz matches the original setup).
* The `PREF`/`UREF`/`IREF` constants are uncalibrated defaults; only
  `getpowerfactor`/`getfrequency` are formula-exact. The guaranteed behaviour is
  **0 with no load / >0 with load** on `getcurrent`.
* I²C read/write timeout defaults to 100 ms; override with
  `-DADE7953_I2C_TIMEOUT_MS=...`.
