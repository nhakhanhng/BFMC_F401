/*
 * VL53_Handler.h
 *
 *  Created on: Dec 28, 2023
 *      Author: nhakh
 */

#ifndef VL53_HANDLER_H_
#define VL53_HANDLER_H_

#include "VL53L0X/core/inc/vl53l0x_api.h"
#include "GPIO/GPIOHandler.h"

#define VL53L0X_DEFAULT_ADDRESS 0x29


typedef struct Vl53l0x_HandlerStruct
{
	GPIO_HandlerStruct XshutPin;
	uint8_t Address;
	VL53L0X_RangingMeasurementData_t RangingData;
	VL53L0X_Dev_t*    Dev;
	uint8_t isInit;
} Vl53l0x_HandlerStruct;

VL53L0X_Error Vl53l0x_Init(Vl53l0x_HandlerStruct* Dev_t, I2C_OS_HandlerStruct * i2c);
uint16_t Vl53l0x_ReadMeasure(Vl53l0x_HandlerStruct *);

#endif /* VL53_HANDLER_H_ */
