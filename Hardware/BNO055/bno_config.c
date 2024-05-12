/*
 * bno_config.c
 *
 *  Created on: Feb 1, 2024
 *      Author: nhakh
 */
#include "bno_config.h"

I2C_OS_HandlerStruct *_bno055_i2c_port;
uint8_t I2C_Address = BNO055_I2C_ADDR;

void bno055_assignI2C(I2C_OS_HandlerStruct *hi2c_device) {
	_bno055_i2c_port = hi2c_device;
}

void bno055_setAddress(uint8_t Addr){
	I2C_Address = Addr;
}

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
	osDelay(time);
#else
  HAL_Delay(time);
#endif
}

int bno055_writeData(uint8_t reg, uint8_t data) {
	uint8_t txdata[2] = { reg, data };
	uint8_t status;
	status = I2C_OS_Master_Transmit_DMA(_bno055_i2c_port, I2C_Address << 1,
			txdata, sizeof(txdata), 10);
//  status = I2C_OS_MEM_Write_DMA(bno055->i2c, bno055->Address << 1, reg, 1, &data, 1, BNO055_WRITE_TIMEOUT);
	return status;
// while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
// return;
}

int bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t status = I2C_OS_Master_Transmit_DMA(_bno055_i2c_port,
	I2C_Address << 1, &reg, 1, 100);
	if (status != HAL_OK) {
		return status;
	}
	status = I2C_OS_Master_Receive_DMA(_bno055_i2c_port, I2C_Address << 1,
			data, len, 100);
	return status;
	// HAL_I2C_Mem_Read(_bno055_i2c_port, I2C_Address_LO<<1, reg,
	// I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

