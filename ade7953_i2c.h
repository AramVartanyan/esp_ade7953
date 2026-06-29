/*

ADE7953 — I2C transport HAL (hardware abstraction layer).

This is the only part of the driver that depends on the SDK. The core
(ade7953.c) talks to the chip exclusively through these primitives, so porting
to another platform (or to the new ESP-IDF i2c_master driver, once the
esp-homekit library migrates) means re-implementing only this file.

Copyright (C) 2026 by Aram Vartanian <https://github.com/AramVartanyan/>
Licensed under the GNU General Public License v3 or later.

*/

#ifndef ade7953_i2c_h
#define ade7953_i2c_h

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Bind the driver to an already-initialised I2C bus / port and the device
 * address. See ade7953_init() for details. Returns true on success. */
bool ade7953_i2c_setup(uint8_t bus, uint8_t addr);

/* Write the 16-bit register address (MSB first), then read `len` bytes
 * (MSB first) into buf[0..len-1]. Returns true on success. */
bool ade7953_i2c_read_reg(uint16_t reg, uint8_t *buf, uint8_t len);

/* Write the 16-bit register address (MSB first), followed by `len` data
 * bytes from buf[0..len-1] (MSB first). Returns true on success. */
bool ade7953_i2c_write_reg(uint16_t reg, const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif
#endif //ade7953_i2c_h
