/*
ADE7953 ESP8266; ESP8285
Copyright (C) 2019 by Aram Vartanian <https://github.com/AramVartanyan/>
*/

#ifndef ade7953_h
#define ade7953_h

#include "FreeRTOS.h"
#include <etstimer.h>
#include "espressif/osapi.h"
#include "esp/gpio.h"
#include "espressif/esp8266/ets_sys.h"

#ifdef __cplusplus
extern "C"
{
#endif

bool ade7953_init(uint8_t bus, uint8_t addr); // Setup I2C bus, ade7953's address and registers.

void ade7953getdata(void); // You must called this function before any „get”
uint32_t ade7953_getenergy(uint8_t channel); // Ws (watt * secound divide by 3600 for Wh)
uint16_t ade7953_getcurrent(uint8_t channel); // mA (divide by 1000 for Amper).
//uint16_t ade7953_getcurrent_peak(uint8_t channel); // Current peak and reset
uint16_t ade7953_getvoltage(); // V x100 (divide by 100 for Volts). Data
uint32_t ade7953_getactivepower(uint8_t channel);

#ifdef __cplusplus
}
#endif 
#endif
