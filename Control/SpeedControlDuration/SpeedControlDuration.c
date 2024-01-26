/*
 * SpeedControlDuration.c
 *
 *  Created on: Jan 25, 2024
 *      Author: nhakh
 */
#include "SpeedControlDuration.h"
#include <stdio.h>
const osThreadAttr_t SpeedControlAttr = { .priority =
		(osPriority_t) osPriorityBelowNormal, .name = "SpeedControl_Task",
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
			Motor_setSpeed(Controller->Motor, 0);
			Servo_setAngle(Controller->Servo, 0);
			Controller->isEnable = 0;
			osDelay(100);
//			tick = osKernelGetTickCount();
		}
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
	uint32_t res = sscanf(InMsg, "%f;%f;%f", &Speed, &time, &Angle);
	if (res != 3) {
		sprintf(Resp, "-1;");
		return;
	}
	Controller->durationTime = time * 1000;
	Servo_setAngle(Controller->Servo, Angle);
	Motor_setSpeed(Controller->Motor, Speed);
	Controller->isEnable = 1;
	Controller->startTime = osKernelGetTickCount();
	osThreadResume(Controller->runThr);
	sprintf(Resp, "0;");
}

