/*
 * SpeedControlDuration.h
 *
 *  Created on: Jan 25, 2024
 *      Author: nhakh
 */

#ifndef SPEEDCONTROLDURATION_H_
#define SPEEDCONTROLDURATION_H_

#include "Motor/MotorControl.h"
#include "SteeringMotor/SteeringMotor.h"
#include "Servo/Servo.h"
#include "UART_Handler/UART_Handler.h"
#include "cmsis_os.h"

typedef struct {
	Motor_HandlerStruct *Motor;
	Steering_HandlerStruct *Steer;
	UARTHandler_Struct *UART;
	uint32_t durationTime;
	uint32_t startTime;
	uint8_t isEnable;
	osThreadId_t runThr;
} SpeedControlDuration_HandleStruct;

void SpeedControlDuration_Init(SpeedControlDuration_HandleStruct *Controller);
void SpeedControlDuration_setVCDCMD(void *Handler, char *InMsg, char *Resp);

#endif /* SPEEDCONTROLDURATION_H_ */
