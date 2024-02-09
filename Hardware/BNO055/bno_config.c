/*
 * bno_config.c
 *
 *  Created on: Feb 1, 2024
 *      Author: nhakh
 */
#include "bno_config.h"

I2C_OS_HandlerStruct *_bno055_i2c_port;

void bno055_assignI2C(I2C_OS_HandlerStruct *hi2c_device) {
  _bno055_i2c_port = hi2c_device;
}

void bno055_delay(int time) {
#ifdef FREERTOS_ENABLED
  osDelay(time);
#else
  HAL_Delay(time);
#endif
}

void bno055_writeData(uint8_t reg, uint8_t data) {
  uint8_t txdata[2] = {reg, data};
  uint8_t status;
  status = I2C_OS_Master_Transmit_DMA(_bno055_i2c_port, BNO055_I2C_ADDR << 1,
                                   txdata, sizeof(txdata), 10);
  if (status == HAL_OK) {
    return;
  }
  // while (HAL_I2C_GetState(_bno055_i2c_port) != HAL_I2C_STATE_READY) {}
  // return;
}

void bno055_readData(uint8_t reg, uint8_t *data, uint8_t len) {
  I2C_OS_Master_Transmit_DMA(_bno055_i2c_port, BNO055_I2C_ADDR << 1, &reg, 1,
                          100);
  I2C_OS_Master_Receive_DMA(_bno055_i2c_port, BNO055_I2C_ADDR << 1, data, len,
                         100);
  // HAL_I2C_Mem_Read(_bno055_i2c_port, BNO055_I2C_ADDR_LO<<1, reg,
  // I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

