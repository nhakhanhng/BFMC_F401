#ifndef BNO055_STM32_H_
#define BNO055_STM32_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include <stm32f4xx.h>

//#define BNO055_I2C_ADDR    BNO055_I2C_ADDR_LO    // For 0x28
//#define BNO055_I2C_ADDR    BNO055_I2C_ADDR_HI    // For 0x29
#define FREERTOS_ENABLED 	1

#ifdef FREERTOS_ENABLED
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "I2C_Handler/I2C_Handler.h"
#include "GPIO/GPIOHandler.h"
#endif

#include "bno055.h"

typedef struct BNO_HandlerStruct {
	I2C_OS_HandlerStruct *hi2c_dev;
	GPIO_HandlerStruct Addr_pin;
	uint8_t Address;
} BNO_HandlerStruct;

void bno055_assignI2C(I2C_OS_HandlerStruct *hi2c_device);
void bno055_setAddress(uint8_t Addr);

#ifdef __cplusplus
  }
#endif

#endif  // BNO055_STM32_H_
