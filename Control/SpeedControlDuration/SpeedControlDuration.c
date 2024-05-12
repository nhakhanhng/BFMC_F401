/*
 * SpeedControlDuration.c
 *
 *  Created on: Jan 25, 2024
 *      Author: nhakh
 */
#include "SpeedControlDuration.h"
#include <stdio.h>

const osThreadAttr_t SpeedControlAttr = { .priority =
		(osPriority_t) osPriorityNormal7, .name = "SpeedControl_Task",
		.stack_size = 256 * 4 };

void SpeedControlDuration_ThreadHandler(void *arg) {
	SpeedControlDuration_HandleStruct *Controller =
			(SpeedControlDuration_HandleStruct*) arg;
//	uint32_t tick = 0;
	while (1) {
		if (Controller->isEnable == 0) {
			osThreadSuspend(Controller->runThr);
		}
//		tick = osKernelGetTickCount();
		if ((Controller->startTime + Controller->durationTime)
				<= osKernelGetTickCount()) {
//			Motor_setSpeed(Controller->Motor, 0);
//			Steering_setAngle(Controller->Steer, 0);
			if (Controller->isEnable) UART_printf(Controller->UART, "@9:0;;\r\n");
			Controller->isEnable = 0;
//			tick = osKernelGetTickCount();
		}
		osDelay(100);
	}
}

void SpeedControlDuration_Init(SpeedControlDuration_HandleStruct *Controller) {
	Controller->isEnable = 0;
	Controller->runThr = osThreadNew(SpeedControlDuration_ThreadHandler,
			(void*) Controller, &SpeedControlAttr);
}
void SpeedControlDuration_setVCDCMD(void *Handler, char *InMsg, char *Resp) {
	SpeedControlDuration_HandleStruct *Controller =
			(SpeedControlDuration_HandleStruct*) Handler;
	float Angle = 0;
	float Speed = 0;
	float time = 0;
	uint32_t res = sscanf(InMsg, "%f;%f;%f", &Speed, &Angle, &time);
	if (res != 3) {
		sprintf(Resp, "-1;");
		return;
	}
//	osDelay(5000);
	Controller->durationTime = time * 1000;
	Steering_setAngle(Controller->Steer, Angle);
	Motor_setSpeed(Controller->Motor, -Speed);
//	Motor_setDutyCycle(Controller->Motor, Speed);
	Controller->isEnable = 1;
	Controller->startTime = osKernelGetTickCount();
	osThreadResume(Controller->runThr);
	sprintf(Resp, "0;");
}

