/*
ADE7953 ESP8266; ESP8285
Copyright (C) 2019 by Aram Vartanyan <https://github.com/AramVartanyan/>
*/

#include "ade7953.h"
#include "esp/gpio.h"
#include "mem.h"
#include <etstimer.h>
#include <i2c/i2c.h>

#define ADE7953_U_RMS  0x31C //RMS voltage - unsigned integer 24bit (0x31C 32bit)
#define ADE7953_C1_RMS 0x31B //RMS current B - unsigned integer 24bit (0x31B 32bit)
#define ADE7953_C1_P_R 0x32B //Read channel B current Peak with reset
#define ADE7953_C2_RMS 0x31A //RMS current A - unsigned integer 24bit (0x31A 32bit)
#define ADE7953_C2_P_R 0x329 //Read channel A current Peak with reset
#define ADE7953_AP1    0x213 //active power B - signed integer 24bit (0x313 32bit)
#define ADE7953_AP2    0x212 //active power A - signed integer 24bit (0x312 32bit)
#define ADE7953_E1     0x21F //active energy B - signed integer 24bit (0x31F 32bit)
#define ADE7953_E2     0x21E //active energy A - signed integer 24bit (0x31E 32bit)

uint8_t ade7953_w_addr;
uint8_t ade7953_r_addr;

#define ADE7953_PREF   1540
#define ADE7953_UREF   0x10B
#define ADE7953_IREF   0xAAA

uint8_t i2c_bus;
uint8_t ade7953_addr;
static uint32_t ade7953_active_power1 = 0;
static uint32_t ade7953_active_power2 = 0;
static uint32_t ade7953_current_rms1 = 0;
static uint32_t ade7953_current_rms2 = 0;
static uint32_t ade7953_voltage_rms = 0;
static uint32_t ade7953_energy1 = 0;
static uint32_t ade7953_energy2 = 0;

static uint8_t ade7953regsize(uint16_t reg_a) {
  uint8_t size = ((reg_a >> 8) & 0x0F);
  return size;
}

static uint32_t ade7953read(uint16_t reg_a) {
  uint32_t response = 0;
  uint8_t size = ade7953regsize(reg_a);
  printf("Register %d\n", reg_a);

  uint8_t reg[2];
  uint8_t byte[4] = {0, 0, 0, 0};

  reg[1] = ((reg_a >> 8) & 0xFF);
  reg[0] = (reg_a & 0xFF);

  i2c_start(i2c_bus);
  uint8_t acnlgmt = i2c_write(i2c_bus, ade7953_w_addr);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("Slave address error\n");
  }
  acnlgmt = i2c_write(i2c_bus, reg[1]);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("MSB of register error\n");
  }
  acnlgmt = i2c_write(i2c_bus, reg[0]);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("LSB of register error\n");
  }
  i2c_start(i2c_bus);
  acnlgmt = i2c_write(i2c_bus, ade7953_r_addr);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("2nd step Slave address error\n");
  }

  acnlgmt = false; //ACK
  for (uint8_t n=size; n>0; n=n-1) {
    byte[n] = i2c_read(i2c_bus, acnlgmt);
  }
  acnlgmt = true; //no ACK
  byte[0] = i2c_read(i2c_bus, acnlgmt);

  i2c_stop(i2c_bus);
  vTaskDelay(1 / portTICK_PERIOD_MS);

  switch (size) {
    case 0:
    response = byte[0];

    case 1:
    response = (byte[1] << 8) | byte[0];
 
    case 2:
    response = (byte[2] << 16) | (byte[1] << 8) | byte[0];
   
    case 3:
    response = (byte[3] << 24) | (byte[2] << 16) | (byte[1] << 8) | byte[0];
    
  }

  printf("Reading %d\n", response);
  return response;
}

static bool ade7953write(uint16_t reg_a, uint32_t data) {
  uint8_t size = ade7953regsize(reg_a);
  uint8_t n = size + 1;
  printf("Register %d\n", reg_a);
  printf("Writing %d\n", data);

  uint8_t reg[2];
  uint8_t byte[4] = {0, 0, 0, 0};

  reg[1] = ((reg_a >> 8) & 0xFF);
  reg[0] = (reg_a & 0xFF);

  i2c_start(i2c_bus);
  uint8_t acnlgmt = i2c_write(i2c_bus, ade7953_w_addr);
 
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("Slave address error\n");
  }
  acnlgmt = i2c_write(i2c_bus, reg[1]);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("MSB of register error\n");
  }
  
  acnlgmt = i2c_write(i2c_bus, reg[0]);
  if(!acnlgmt) {
    i2c_stop(i2c_bus);
    printf("LSB of register error\n");
  }
  
  while (n) {
    n = n-1;
    byte[n] = (data >> (8*n) & 0xFF);
    acnlgmt = i2c_write(i2c_bus, byte[n]);
    if(!acnlgmt) {
      i2c_stop(i2c_bus);
    }
  }
  i2c_stop(i2c_bus);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  return acnlgmt;
  }


bool ade7953_init(uint8_t bus, uint8_t addr) {
  i2c_bus = bus;
  ade7953_addr = addr;
  ade7953_w_addr = ade7953_addr << 1;
  ade7953_r_addr = ade7953_w_addr + 0x1;
  bool init;

  ade7953write(0x102, 0x0004);    // Locking the communication interface (Clear bit COMM_LOCK), Enable HPF
  ade7953write(0x0FE, 0x00AD);    // Unlock register 0x120
  init = ade7953write(0x120, 0x0030);    // Configure optimum setting
 return init;
}

void ade7953getdata(void){
    ade7953_voltage_rms = ade7953read(ADE7953_U_RMS);      // Both relays
    ade7953_current_rms1 = ade7953read(ADE7953_C1_RMS);     // Relay 1
    if (ade7953_current_rms1 < 2000) {             // No load threshold (20mA)
        ade7953_current_rms1 = 0;
        ade7953_active_power1 = 0;
    } else {
        ade7953_active_power1 = ade7953read(ADE7953_AP1) * -1;//ade7953read(0x213);  // Relay 1
    }

    ade7953_current_rms2 = ade7953read(ADE7953_C2_RMS);     // Relay 2
    if (ade7953_current_rms2 < 2000) {             // No load threshold (20mA)
        ade7953_current_rms2 = 0;
        ade7953_active_power2 = 0;
    } else {
        ade7953_active_power2 = ade7953read(ADE7953_AP2);  // Relay 2
    }

    ade7953_energy1 = ade7953read(ADE7953_E1);
    ade7953_energy2 = ade7953read(ADE7953_E2);
}

uint16_t ade7953_getvoltage(){
  uint32_t voltage_rms_readout;
  uint16_t v_rms_return;
  voltage_rms_readout = ade7953read(ADE7953_U_RMS);
  v_rms_return = (((voltage_rms_readout*ADE7953_UREF) & 0xFFFF0000) >> 16);
  return v_rms_return;
}

uint16_t ade7953_getcurrent(uint8_t channel){
  uint16_t c_rms_return = 0;
  if (channel == 2) {
    c_rms_return = (((ade7953read(ADE7953_C2_RMS)*ADE7953_IREF) & 0xFFFF0000) >> 16);
  }
  if (channel == 1) {
    c_rms_return = (((ade7953read(ADE7953_C1_RMS)*ADE7953_IREF) & 0xFFFF0000) >> 16);
  }
  if (c_rms_return < 75) {
    c_rms_return = 0;
  }

  return c_rms_return;
}

uint32_t ade7953_getactivepower(uint8_t channel){
    return (channel < 2 ? ade7953_active_power1 : ade7953_active_power2 ) / ADE7953_PREF;
}

uint32_t ade7953_getenergy(uint8_t channel){ // Any read reset register. Energy count from zero after read.
    return ((channel < 2 ? ade7953_energy1 : ade7953_energy2) * ADE7953_PREF ) / 1000; // Ws (watt * secound divide by 3600 for Wh)
}
