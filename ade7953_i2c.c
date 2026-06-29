/*

ADE7953 — I2C transport HAL implementation.

Two backends, selected automatically by CONFIG_IDF_TARGET_* (defined in
sdkconfig.h under the ESP-IDF family; absent under esp-open-rtos):
  - ESP-IDF / ESP8266_RTOS_SDK: the legacy "driver/i2c.h" API (i2c_cmd_link_*),
    the same I2C framework used by the esp-homekit platform (esp_mfi_i2c.c) and
    the esp_aht sensor driver. The read path writes the 16-bit register address,
    issues a repeated start, then reads the data bytes (datasheet Figure 70/71).
    The caller owns the bus.
  - esp-open-rtos: the original bit-level i2c_start/write/read/stop sequence,
    preserved 1:1 from the tested ESP8266 driver.

Copyright (C) 2026 by Aram Vartanian <https://github.com/AramVartanyan/>
Licensed under the GNU General Public License v3 or later.

*/

#include "ade7953_i2c.h"

/* Pull in CONFIG_IDF_TARGET_* under the ESP-IDF family. esp-open-rtos has no
 * sdkconfig.h, so the include is skipped there (mirrors esp_mfi_i2c.c, which
 * includes sdkconfig.h before testing CONFIG_IDF_TARGET_*). */
#ifdef __has_include
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP8266) || defined(CONFIG_IDF_TARGET_ESP32)
/* ============ ESP-IDF / ESP8266_RTOS_SDK backend (legacy driver) ============ */

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADE7953_ACK_CHECK_EN   0x1   /* master checks the slave ACK */

#ifndef ADE7953_I2C_TIMEOUT_MS
#define ADE7953_I2C_TIMEOUT_MS 100
#endif

static i2c_port_t s_port = I2C_NUM_0;
static uint8_t    s_addr = 0x38;      /* 7-bit address */

bool ade7953_i2c_setup(uint8_t bus, uint8_t addr) {
    s_port = (i2c_port_t)bus;
    s_addr = addr;
    return true;
}

bool ade7953_i2c_read_reg(uint16_t reg, uint8_t *buf, uint8_t len) {
    if (len == 0) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    /* Stage 1: set the register pointer (write the 16-bit address). */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, ADE7953_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, ADE7953_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg & 0xFF, ADE7953_ACK_CHECK_EN);
    /* Stage 2: repeated start, then read `len` bytes (MSB first). */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, ADE7953_ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(ADE7953_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

bool ade7953_i2c_write_reg(uint16_t reg, const uint8_t *buf, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, ADE7953_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, ADE7953_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg & 0xFF, ADE7953_ACK_CHECK_EN);
    if (len > 0) {
        i2c_master_write(cmd, (uint8_t *)buf, len, ADE7953_ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(ADE7953_I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

#else
/* ===================== esp-open-rtos backend ===================== */

#include <i2c/i2c.h>

static uint8_t s_bus;
static uint8_t s_w_addr;   /* address with write bit  */
static uint8_t s_r_addr;   /* address with read bit   */

bool ade7953_i2c_setup(uint8_t bus, uint8_t addr) {
    s_bus    = bus;
    s_w_addr = (uint8_t)(addr << 1);
    s_r_addr = (uint8_t)((addr << 1) | 0x1);
    return true;
}

bool ade7953_i2c_read_reg(uint16_t reg, uint8_t *buf, uint8_t len) {
    /* First stage: set the register pointer (write the 16-bit address). */
    i2c_start(s_bus);
    if (!i2c_write(s_bus, s_w_addr))           { i2c_stop(s_bus); return false; }
    if (!i2c_write(s_bus, (reg >> 8) & 0xFF))  { i2c_stop(s_bus); return false; }
    if (!i2c_write(s_bus, reg & 0xFF))         { i2c_stop(s_bus); return false; }

    /* Second stage: repeated start, then read `len` bytes (MSB first).
     * ACK on every byte but the last, NACK on the last (original semantics). */
    i2c_start(s_bus);
    if (!i2c_write(s_bus, s_r_addr))           { i2c_stop(s_bus); return false; }
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = i2c_read(s_bus, i == (len - 1));
    }
    i2c_stop(s_bus);
    return true;
}

bool ade7953_i2c_write_reg(uint16_t reg, const uint8_t *buf, uint8_t len) {
    bool ack;
    i2c_start(s_bus);
    ack = i2c_write(s_bus, s_w_addr);
    if (ack) ack = i2c_write(s_bus, (reg >> 8) & 0xFF);
    if (ack) ack = i2c_write(s_bus, reg & 0xFF);
    for (uint8_t i = 0; i < len && ack; i++) {
        ack = i2c_write(s_bus, buf[i]);
    }
    i2c_stop(s_bus);
    return ack;
}

#endif
