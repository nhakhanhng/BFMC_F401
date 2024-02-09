/*
 * Vl53l1xHandler.h
 *
 *  Created on: Feb 3, 2024
 *      Author: nhakh
 */

#ifndef VL53L1X_VL53L1XHANDLER_VL53L1XHANDLER_H_
#define VL53L1X_VL53L1XHANDLER_VL53L1XHANDLER_H_

#include "vl53l1_api.h"
#include "GPIO/GPIOHandler.h"

typedef struct
{
	GPIO_HandlerStruct XshutPin;
	uint8_t Address;
	VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_Dev_t    Dev;
	uint8_t isInit;
} Vl53l1_HandleStruct;

VL53L1_Error Vl53l1_Init(Vl53l1_HandleStruct *, I2C_OS_HandlerStruct *);
int16_t Vl53l1_ReadMeasure(Vl53l1_HandleStruct *);

#endif /* VL53L1X_VL53L1XHANDLER_VL53L1XHANDLER_H_ */
