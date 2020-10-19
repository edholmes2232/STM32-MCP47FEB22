#ifndef __mcp_h
#define __mcp_h

#include "stm32l0xx.h"
#include "i2c.h"

// REG ADDRESSES ALREADY << 3
#define defaultVDD 5000
#define BASE_ADDR 0x60
#define RESET 0B00000110
#define WAKE 0B00001010
#define UPDATE 0B00001000
#define GENERALCALL 0B0000000
#define READ 0B00000110
#define WRITE 0B00000000
#define DAC0_REG 0B00000000
#define DAC1_REG 0B00001000
#define VREF_REG 0B01000000
#define PD_REG 0B01001000
#define GAIN_REG 0B01010000
#define WL_REG 0B000001011
#define DAC0_EP_REG 0B10000000
#define DAC1_EP_REG 0B10001000
#define VREF_EP_REG 0B11000000
#define PD_EP_REG 0B11001000
#define GAIN_EP_REG 0B11010000

#define UNLOCK_SALCK 0B11010010
#define LOCK_SALCK 0b11010100


unsigned char 	readBuffer[5];

unsigned char _buffer[5];
uint8_t      _dev_address;
uint8_t      _deviceID;
uint8_t      _intVref[2];
uint8_t      _gain[2];
uint8_t      _powerDown[2];
uint16_t     _values[2];
uint16_t     _valuesEp[2];
uint8_t      _intVrefEp[2];
uint8_t      _gainEp[2];
uint8_t      _powerDownEp[2];
uint8_t      _wiperLock[2];
uint16_t     _vdd;

 typedef struct {
	uint8_t deviceID;
	uint8_t devAddr;
	uint8_t vdd;
	I2C_HandleTypeDef hi2c;
} DAC_TypeDef;



#endif
