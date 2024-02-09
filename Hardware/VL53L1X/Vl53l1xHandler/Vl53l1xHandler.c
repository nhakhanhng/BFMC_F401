/*
 * Vl53l1xHandler.c
 *
 *  Created on: Feb 3, 2024
 *      Author: nhakh
 */

#include "VL53L1X/Vl53l1xHandler/Vl53l1xHandler.h"
#define VL53L0X_DEFAULT_ADDRESS 0x52

VL53L1_Error Vl53l1_Init(Vl53l1_HandleStruct *Dev_t, I2C_OS_HandlerStruct *i2c) {

	Dev_t->Dev.I2cHandle = i2c;
	Dev_t->Dev.I2cDevAddr = VL53L0X_DEFAULT_ADDRESS;

	GPIO_WritePin(Dev_t->XshutPin, GPIO_PIN_RESET); // Disable XSHUT
	osDelay(10);
	GPIO_WritePin(Dev_t->XshutPin, GPIO_PIN_SET); // Enable XSHUT
	osDelay(10);

	VL53L1_Error Status = VL53L1_ERROR_NONE;

	Status = VL53L1_WaitDeviceBooted(&Dev_t->Dev);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_DataInit(&Dev_t->Dev);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_StaticInit(&Dev_t->Dev);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_SetDistanceMode(&Dev_t->Dev, VL53L1_DISTANCEMODE_LONG);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&Dev_t->Dev, 50000);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&Dev_t->Dev, 50000);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_StartMeasurement(&Dev_t->Dev);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Status = VL53L1_SetDeviceAddress(&Dev_t->Dev, Dev_t->Address);
	if (Status != VL53L1_ERROR_NONE)
		return Status;
	Dev_t->Dev.I2cDevAddr = Dev_t->Address;
	osDelay(10);

	Dev_t->isInit = 1;
	return Status;
}
int16_t Vl53l1_ReadMeasure(Vl53l1_HandleStruct* Dev_t) {
	VL53L1_ClearInterruptAndStartMeasurement(&Dev_t->Dev);
	VL53L1_WaitMeasurementDataReady(&Dev_t->Dev);
	VL53L1_GetRangingMeasurementData(&Dev_t->Dev,&Dev_t->RangingData);
	return Dev_t->RangingData.RangeMilliMeter;
}

