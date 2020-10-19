#include "mcp47feb22.h"
#include "stm32l0xx.h"
#include "stm32l0xx_hal_i2c.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_def.h"


#define word(x,y) (((x) << 8) | (y))
#define lowByte(x) ((x) & 0b00001111)


//DEVICE ID DEFAULT = 0
void MCP47FEB22_Init(DAC_TypeDef* dac, uint8_t deviceID, I2C_HandleTypeDef hi2c){//uint8_t deviceID = 0x00) {
	dac->deviceID = deviceID;
	dac->devAddr = (BASE_ADDR | (dac->deviceID));
    dac->vdd = defaultVDD;
    dac->hi2c = hi2c;
}

void ReadEpAddr(DAC_TypeDef dac, uint8_t REG, unsigned char buffer[5]) {
	uint8_t readReg = 0x80 | (READ | REG);// << 3);
	readReg = 0;
	
	HAL_I2C_Master_Transmit(&dac.hi2c, dac.devAddr<<1, &readReg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&dac.hi2c, dac.devAddr<<1, buffer, 2, HAL_MAX_DELAY);
}

static void ReadAddr(DAC_TypeDef dac, uint8_t REG, unsigned char buffer[5]) {
	uint8_t readReg = (READ | REG);// << 3);
	HAL_I2C_Master_Transmit(&dac.hi2c, dac.devAddr<<1, &readReg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&dac.hi2c, dac.devAddr<<1, buffer, 2, HAL_MAX_DELAY);
}


void FastWrite(DAC_TypeDef dac, uint8_t REG, uint16_t DATA) {
	uint8_t payload[8];
	payload[0] = REG | WRITE;
	payload[1] = (DATA >> 8) & 0xFF;
	payload[2] = (DATA & 0xFF);
	HAL_I2C_Master_Transmit(&dac.hi2c, dac.devAddr<<1, &payload, 3, HAL_MAX_DELAY);
}


static void WriteAddr(DAC_TypeDef dac, uint8_t REG, uint8_t data) {
	uint8_t payload[3];
	payload[0] = REG | WRITE;
	if (REG == GAIN_REG) {
		payload[2] = 0;
		payload[1] = data;
	} else {
		payload[1] = 0;
		payload[2] = data;
	}
	HAL_I2C_Master_Transmit(&dac.hi2c, dac.devAddr<<1, &payload, 3, HAL_MAX_DELAY);

}


uint8_t MCP47FEB22_UnlockSALCK(DAC_TypeDef dac) {
	//SET HVC PIN HIGH
	WriteAddr(dac, UNLOCK_SALCK, 0);
	// SET HVC PIN LOW
}

uint8_t MCP47FEB22_LockSALCK(DAC_TypeDef dac, uint8_t addr) {
	//SET HVC PIN HIGH
	DAC_TypeDef dac2 = {
		addr,
		BASE_ADDR | addr,
		dac.vdd,
		dac.hi2c,
	};

	WriteAddr(dac2, LOCK_SALCK, 0);
	//SET HVC PIN LOW
}

void MCP47FEB22_ChangeAddr(DAC_TypeDef dac, uint8_t addr) {
	MCP47FEB22_UnlockSALCK(dac);
	WriteAddr(dac, 0b11010000, BASE_ADDR|addr);
	MCP47FEB22_LockSALCK(dac, addr);

}

uint8_t MCP47FEB22_GetPowerDown(DAC_TypeDef dac, int channel) {
	ReadAddr(dac, PD_REG, readBuffer);
	_powerDown[0] = (readBuffer[1] & 0B00000011);
	_powerDown[1] = (readBuffer[1] & 0B00001100) >> 2;
	return (channel == 0) ? _powerDown[0] : _powerDown[1];
}

void MCP47FEB22_SetPowerDown(DAC_TypeDef dac, int val0, int val1) {
	WriteAddr(dac, PD_REG, (val0 | val1<<2));
}


uint8_t MCP47FEB22_GetPowerDownEp(DAC_TypeDef dac, int channel) {
	ReadEpAddr(dac, PD_REG, readBuffer);
	_powerDownEp[0] = (readBuffer[1] & 0B00000011);
	_powerDownEp[1] = (readBuffer[1] & 0B00001100) >> 2;
	return (channel == 0) ? _powerDownEp[0] : _powerDownEp[1];
}


uint8_t MCP47FEB22_GetGain(DAC_TypeDef dac, int channel) {
	uint8_t buff[5] = {0};
	ReadAddr(dac, GAIN_REG, buff);
	_gain[0] = (buff[0] & 0B00000001);
	_gain[1] = (buff[0] & 0B00000010)>>1;
	return (channel == 0) ? _gain[0] : _gain[1];
}


void MCP47FEB22_SetGain(DAC_TypeDef dac, int val0, int val1) {
	WriteAddr(dac, GAIN_REG, (val0 | (val1<<1)));
}

uint8_t MCP47FEB22_GetGainEp(DAC_TypeDef dac, int channel) {
	ReadEpAddr(dac, GAIN_REG, readBuffer);
	_gainEp[0] = (readBuffer[0] & 0B00000001);
	_gainEp[1] = (readBuffer[0] & 0B00000010)>>1;
	return (channel == 0) ? _gainEp[0] : _gainEp[1];
}




uint8_t MCP47FEB22_GetVref(DAC_TypeDef dac, uint8_t channel) {//uint8_t channel) {
	ReadAddr(dac,VREF_REG, readBuffer);
	_intVref[0] = (readBuffer[1] & 0b00000011);
	_intVref[1] = (readBuffer[1] & 0b00001100) >> 2;
	return (channel == 0) ? _intVref[0] : _intVref[1];
}

void MCP47FEB22_SetVref(DAC_TypeDef dac, uint8_t val0, uint8_t val1) {
	WriteAddr(dac, VREF_REG, (val0 | (val1<<2)));
}

uint8_t MCP47FEB22_GetVrefEp(DAC_TypeDef dac, uint8_t channel) {//uint8_t channel) {
	ReadEpAddr(dac, VREF_REG, readBuffer);
	_intVrefEp[0] = (readBuffer[1] & 0b00000011);
	_intVrefEp[1] = (readBuffer[1] & 0b00001100) >> 2;
	return (channel == 0) ? _intVrefEp[0] : _intVrefEp[1];
}





uint16_t MCP47FEB22_GetValue(DAC_TypeDef dac, uint8_t channel) {
	ReadAddr(dac, (channel << 3), readBuffer);
	return word((readBuffer[0] & 0B00001111), readBuffer[1]);
	//return ((readBuffer[0] & 0b00001111)<<8) + readBuffer[1];

//	ReadAddr();
}

void MCP47FEB22_AnalogWrite(DAC_TypeDef dac, uint16_t val0, uint16_t val1) {
	val0 &= 0xFFF;
	val1 &= 0xFFF; //Prevent going over 4095
	FastWrite(dac, DAC0_REG, val0);
	FastWrite(dac, DAC1_REG, val1);
}

void MCP47FEB22_EEPROMWrite(DAC_TypeDef dac) {
	//FastWrite(dac, DAC0_EP_REG, MCP47FEB22_GetValue(dac, 0));
	//FastWrite(dac, DAC1_EP_REG, MCP47FEB22_GetValue(dac, 1));
	volatile uint8_t high = MCP47FEB22_GetPowerDown(dac, 0);
	volatile uint8_t low = MCP47FEB22_GetPowerDown(dac, 1);
	volatile uint16_t data = (high | low<<2);
	//FastWrite(dac, VREF_EP_REG, (MCP47FEB22_GetVref(dac,0) | MCP47FEB22_GetVref(dac,1)<<2));
	
	
	//FastWrite(dac, GAIN_EP_REG, (MCP47FEB22_GetGain(dac, 0) | MCP47FEB22_GetGain(dac, 1)<<1)<<8);
	FastWrite(dac, PD_EP_REG, (MCP47FEB22_GetPowerDown(dac, 0) | MCP47FEB22_GetPowerDown(dac, 1)<<2));
}
